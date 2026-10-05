//! Microphone streaming through the `ROS-Driver-Audio` client.

use super::{Capability, CapabilityId, Configuration, ConfigurationResult, Context};
use crate::Result;
use crate::domain::{AudioBuffer, Message, MicConfig};
use crate::qi::events::RemoteAudio;
use crate::qi::keys;
use async_trait::async_trait;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

/// Channel map of robots reporting a non-zero microphone config code.
const FOUR_CHANNEL_MAP: [u8; 4] = [3, 5, 0, 2];
const STANDARD_CHANNEL_MAP: [u8; 4] = [0, 2, 1, 4];

/// Publishes the audio chunks Pepper pushes via `processRemote`.
pub struct Mic {
    config: Mutex<MicConfig>,
    streaming: AtomicBool,
}

impl Mic {
    pub fn new() -> Self {
        Self {
            config: Mutex::new(MicConfig::DEFAULT),
            streaming: AtomicBool::new(false),
        }
    }
}

impl Default for Mic {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for Mic {
    fn id(&self) -> CapabilityId {
        CapabilityId::Mic
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let config = *self.config.lock().unwrap_or_else(|err| err.into_inner());
        config.validate()?;
        let map = channel_map(ctx.robot.robot_model.microphone_config().await?);
        let audio = &ctx.robot.audio;
        audio
            .set_client_preferences(
                keys::AUDIO_CALLBACK_SERVICE,
                i32::from(config.frequency),
                i32::from(config.channels),
            )
            .await?;
        audio.enable_energy_computation().await?;
        audio.subscribe(keys::AUDIO_CALLBACK_SERVICE).await?;
        self.streaming.store(true, Ordering::Relaxed);

        let transport = ctx.transport.clone();
        ctx.toolkit.audio().set(move |chunk: RemoteAudio| {
            let buffer = AudioBuffer {
                stamp: transport.now(),
                frequency: u32::from(config.frequency),
                channel_map: map.clone(),
                data: chunk.data,
            };
            if let Err(err) = transport.publish(CapabilityId::Mic.as_str(), Message::Audio(buffer))
            {
                tracing::warn!(error = %err, "audio publish failed");
            }
        });
        Ok(())
    }

    async fn disable(&self, ctx: &Context) -> Result<()> {
        ctx.toolkit.audio().clear();
        if self.streaming.swap(false, Ordering::Relaxed) {
            ctx.robot
                .audio
                .unsubscribe(keys::AUDIO_CALLBACK_SERVICE)
                .await?;
        }
        Ok(())
    }

    async fn configure(
        &self,
        _ctx: &Context,
        configuration: Configuration,
    ) -> Result<ConfigurationResult> {
        if let Configuration::Mic(config) = configuration {
            config.validate()?;
            *self.config.lock().unwrap_or_else(|err| err.into_inner()) = config;
        }
        Ok(ConfigurationResult::None)
    }
}

/// Channel map selected by the robot model's microphone config code.
fn channel_map(config_code: i32) -> Vec<u8> {
    if config_code != 0 {
        FOUR_CHANNEL_MAP.to_vec()
    } else {
        STANDARD_CHANNEL_MAP.to_vec()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use crate::qi::value::Raw;
    use qi::object::Object;
    use qi::value::{IntoValue, Value};

    fn harness_with_model_config(config_code: i32) -> Harness {
        let harness = Harness::default();
        harness
            .fakes
            .service("ALRobotModel")
            .script("_getMicrophoneConfig", config_code.into_value());
        harness
    }

    async fn process_remote(ctx: &crate::capabilities::Context, samples: Vec<u8>) {
        ctx.toolkit
            .meta_call(
                "processRemote".into(),
                (
                    2i32,
                    (samples.len() / 2) as i32,
                    Raw::new(Value::Unit),
                    Raw::new(samples.into_value()),
                )
                    .into_value(),
            )
            .await
            .expect("processRemote");
    }

    #[tokio::test]
    async fn enable_sets_up_the_microphone_client() {
        let harness = harness_with_model_config(0);
        Mic::new().enable(&harness.ctx).await.expect("enable");

        let audio = harness.fakes.service("ALAudioDevice");
        assert_eq!(
            audio.calls_to("setClientPreferences"),
            vec![("ROS-Driver-Audio".to_owned(), 48_000i32, 0i32, 0i32).into_value()]
        );
        assert_eq!(audio.calls_to("enableEnergyComputation").len(), 1);
        assert_eq!(
            audio.calls_to("subscribe"),
            vec!["ROS-Driver-Audio".to_owned().into_value()]
        );
        assert_eq!(
            harness
                .fakes
                .service("ALRobotModel")
                .calls_to("_getMicrophoneConfig")
                .len(),
            1
        );
    }

    #[tokio::test]
    async fn chunks_publish_with_the_standard_channel_map() {
        let harness = harness_with_model_config(0);
        Mic::new().enable(&harness.ctx).await.expect("enable");
        process_remote(&harness.ctx, vec![0x01, 0x02, 0xff, 0x7f]).await;

        let published = harness.transport.published_on("mic");
        let Message::Audio(buffer) = &published[0] else {
            panic!("expected audio");
        };
        assert_eq!(buffer.frequency, 48_000);
        assert_eq!(buffer.channel_map, vec![0, 2, 1, 4]);
        assert_eq!(buffer.data, vec![0x0201, 0x7fff]);
    }

    #[tokio::test]
    async fn chunks_publish_with_the_four_channel_map() {
        let harness = harness_with_model_config(7);
        Mic::new().enable(&harness.ctx).await.expect("enable");
        process_remote(&harness.ctx, vec![0x00, 0x01]).await;

        let published = harness.transport.published_on("mic");
        let Message::Audio(buffer) = &published[0] else {
            panic!("expected audio");
        };
        assert_eq!(buffer.channel_map, vec![3, 5, 0, 2]);
    }

    #[tokio::test]
    async fn configure_applies_on_the_next_enable() {
        let harness = harness_with_model_config(0);
        let mic = Mic::new();
        mic.configure(
            &harness.ctx,
            Configuration::Mic(MicConfig {
                frequency: 16_000,
                channels: 2,
            }),
        )
        .await
        .expect("configure");
        mic.enable(&harness.ctx).await.expect("enable");

        let audio = harness.fakes.service("ALAudioDevice");
        assert_eq!(
            audio.calls_to("setClientPreferences"),
            vec![("ROS-Driver-Audio".to_owned(), 16_000i32, 2i32, 0i32).into_value()]
        );
    }

    #[tokio::test]
    async fn invalid_config_is_rejected() {
        let harness = Harness::default();
        let mic = Mic::new();
        let bad = Configuration::Mic(MicConfig {
            frequency: 22_050,
            channels: 0,
        });
        assert!(mic.configure(&harness.ctx, bad).await.is_err());

        *mic.config.lock().unwrap_or_else(|err| err.into_inner()) = MicConfig {
            frequency: 22_050,
            channels: 0,
        };
        assert!(mic.enable(&harness.ctx).await.is_err());
        assert!(
            harness
                .fakes
                .service("ALAudioDevice")
                .calls_to("subscribe")
                .is_empty()
        );
    }

    #[tokio::test]
    async fn disable_unsubscribes_once_and_stops_publishing() {
        let harness = harness_with_model_config(0);
        let mic = Mic::new();
        mic.enable(&harness.ctx).await.expect("enable");
        mic.disable(&harness.ctx).await.expect("disable");
        mic.disable(&harness.ctx).await.expect("disable again");
        process_remote(&harness.ctx, vec![0x00, 0x01]).await;

        let audio = harness.fakes.service("ALAudioDevice");
        assert_eq!(audio.calls_to("unsubscribe").len(), 1);
        assert!(harness.transport.published_on("mic").is_empty());
    }
}
