// Copyright 2026 SinfonIA Uniandes <sinfonia@uniandes.edu.co>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Sound source localization from `ALSoundLocalization/SoundLocated`.

use super::{Capability, CapabilityId, Context};
use crate::Result;
use crate::domain::{Message, SoundBearing};
use crate::qi::keys;
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
        ctx.toolkit
            .sound_located()
            .set(move |mut bearing: SoundBearing| {
                bearing.stamp = transport.now();
                if let Err(err) = transport.publish(
                    CapabilityId::MicLocalization.as_str(),
                    Message::SoundBearing(bearing),
                ) {
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
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
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
            vec![
                (
                    keys::SOUND_LOCATED.to_owned(),
                    "ROS-DriverALSoundLocalization/SoundLocated".to_owned(),
                    "soundLocatedCallback".to_owned()
                )
                    .into_value()
            ]
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
            vec![
                (
                    keys::SOUND_LOCATED.to_owned(),
                    "ROS-DriverALSoundLocalization/SoundLocated".to_owned()
                )
                    .into_value()
            ]
        );
        assert!(harness.transport.published_on("miclocalization").is_empty());
    }
}
