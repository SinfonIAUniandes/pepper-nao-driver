//! Sound source localization from `ALSoundLocalization/SoundLocated`.

use super::{Capability, CapabilityId, Context};
use crate::domain::{Message, SoundBearing};
use crate::qi::keys;
use crate::Result;
use async_trait::async_trait;

/// Publishes the bearing of localized sounds.
pub struct MicLocalization;

#[async_trait]
impl Capability for MicLocalization {
    fn id(&self) -> CapabilityId {
        CapabilityId::MicLocalization
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let transport = ctx.transport.clone();
        ctx.toolkit.sound_located().set(move |mut bearing: SoundBearing| {
            bearing.stamp = transport.now();
            if let Err(err) = transport.publish(CapabilityId::MicLocalization.as_str(), Message::SoundBearing(bearing))
            {
                tracing::warn!(error = %err, "sound bearing publish failed");
            }
        });
        ctx.robot
            .memory
            .subscribe_to_event(
                keys::SOUND_LOCATED,
                &keys::callback_service_name(keys::SOUND_LOCATED),
                "soundLocatedCallback",
            )
            .await
    }

    async fn disable(&self, ctx: &Context) -> Result<()> {
        ctx.toolkit.sound_located().clear();
        ctx.robot
            .memory
            .unsubscribe_to_event(
                keys::SOUND_LOCATED,
                &keys::callback_service_name(keys::SOUND_LOCATED),
            )
            .await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::support::Harness;
    use crate::capabilities::Capability;
    use crate::qi::value::Raw;
    use qi::object::Object;
    use qi::value::IntoValue;

    fn event_payload() -> qi::value::Value<'static> {
        vec![
            0.0f32.into_value(),
            vec![0.5f32, -0.25, 0.9, 0.1].into_value(),
            vec![1.0f32, 2.0, 3.0].into_value(),
            vec![4.0f32, 5.0].into_value(),
        ]
        .into_value()
    }

    async fn sound_located(harness: &Harness) {
        harness
            .ctx
            .toolkit
            .meta_call(
                "soundLocatedCallback".into(),
                (
                    keys::SOUND_LOCATED.to_owned(),
                    Raw::new(event_payload()),
                    "subscriber".to_owned(),
                )
                    .into_value(),
            )
            .await
            .expect("event");
    }

    #[tokio::test]
    async fn enable_subscribes_the_event_and_bearings_are_published() {
        let harness = Harness::default();
        MicLocalization.enable(&harness.ctx).await.expect("enable");
        sound_located(&harness).await;

        let memory = harness.fakes.service("ALMemory");
        assert_eq!(
            memory.calls_to("subscribeToEvent"),
            vec![(
                keys::SOUND_LOCATED.to_owned(),
                "ROS-DriverALSoundLocalization/SoundLocated".to_owned(),
                "soundLocatedCallback".to_owned()
            )
                .into_value()]
        );

        let published = harness.transport.published_on("miclocalization");
        let Message::SoundBearing(bearing) = &published[0] else {
            panic!("expected a sound bearing");
        };
        assert_eq!(bearing.azimuth, 0.5);
        assert_eq!(bearing.elevation, -0.25);
        assert_eq!(bearing.confidence, 0.9);
        assert_eq!(bearing.energy, 0.1);
        assert_eq!(bearing.head_in_torso, vec![1.0, 2.0, 3.0]);
        assert_eq!(bearing.head_in_robot, vec![4.0, 5.0]);
    }

    #[tokio::test]
    async fn disable_unsubscribes_the_event() {
        let harness = Harness::default();
        let capability = MicLocalization;
        capability.enable(&harness.ctx).await.expect("enable");
        capability.disable(&harness.ctx).await.expect("disable");
        sound_located(&harness).await;

        let memory = harness.fakes.service("ALMemory");
        assert_eq!(
            memory.calls_to("unsubscribeToEvent"),
            vec![(
                keys::SOUND_LOCATED.to_owned(),
                "ROS-DriverALSoundLocalization/SoundLocated".to_owned()
            )
                .into_value()]
        );
        assert!(harness
            .transport
            .published_on("miclocalization")
            .is_empty());
    }
}
