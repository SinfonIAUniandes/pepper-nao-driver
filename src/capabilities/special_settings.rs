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

//! Robot-wide special settings; the only capability enabled at boot.

use super::{Capability, CapabilityId, Context, message_handler};
use crate::Result;
use crate::domain::{Message, SpecialSetting};
use crate::transport::Subscription;
use async_trait::async_trait;
use std::sync::Mutex;

/// Applies rest, collision protection, security distance and awareness.
pub struct SpecialSettings {
    subscription: Mutex<Option<Subscription>>,
}

impl SpecialSettings {
    pub fn new() -> Self {
        Self {
            subscription: Mutex::new(None),
        }
    }
}

impl Default for SpecialSettings {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for SpecialSettings {
    fn id(&self) -> CapabilityId {
        CapabilityId::SpecialSettings
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let robot = ctx.robot.clone();
        let subscription = ctx.transport.subscribe(
            self.id().as_str(),
            message_handler(
                |message| match message {
                    Message::SpecialSetting(setting) => Some(*setting),
                    _ => None,
                },
                move |setting| {
                    let robot = robot.clone();
                    Box::pin(async move { apply(&robot, setting).await })
                },
            ),
        );
        *self
            .subscription
            .lock()
            .unwrap_or_else(|err| err.into_inner()) = Some(subscription);
        Ok(())
    }

    async fn disable(&self, _ctx: &Context) -> Result<()> {
        self.subscription
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .take();
        Ok(())
    }
}

async fn apply(robot: &crate::qi::Robot, setting: SpecialSetting) -> Result<()> {
    match setting {
        SpecialSetting::Rest(true) => robot.motion.rest().await,
        SpecialSetting::Rest(false) => robot.motion.wake_up().await,
        SpecialSetting::ExternalCollisionProtection(enabled) => {
            robot
                .motion
                .set_external_collision_protection_enabled(enabled)
                .await
        }
        SpecialSetting::SecurityDistance(distance) => {
            robot
                .motion
                .set_orthogonal_security_distance(distance)
                .await
        }
        SpecialSetting::Awareness(enabled) => robot.basic_awareness.set_enabled(enabled).await,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use std::time::Duration;

    async fn settle() {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    #[tokio::test]
    async fn settings_map_to_their_qi_calls() {
        let harness = Harness::default();
        let capability = SpecialSettings::new();
        capability.enable(&harness.ctx).await.expect("enable");

        for setting in [
            SpecialSetting::Rest(true),
            SpecialSetting::Rest(false),
            SpecialSetting::ExternalCollisionProtection(false),
            SpecialSetting::SecurityDistance(0.4),
            SpecialSetting::Awareness(true),
        ] {
            harness
                .transport
                .inject("special_settings", Message::SpecialSetting(setting));
        }
        settle().await;

        let motion = harness.fakes.service("ALMotion");
        assert_eq!(motion.calls_to("rest").len(), 1);
        assert_eq!(motion.calls_to("wakeUp").len(), 1);
        assert_eq!(
            motion.calls_to("setExternalCollisionProtectionEnabled")[0],
            ("Move".to_owned(), false).into_value()
        );
        assert_eq!(
            motion.calls_to("setOrthogonalSecurityDistance")[0],
            0.4f32.into_value()
        );
        assert_eq!(
            harness
                .fakes
                .service("ALBasicAwareness")
                .calls_to("setEnabled")[0],
            true.into_value()
        );
    }

    #[tokio::test]
    async fn disable_unsubscribes() {
        let harness = Harness::default();
        let capability = SpecialSettings::new();
        capability.enable(&harness.ctx).await.expect("enable");
        capability.disable(&harness.ctx).await.expect("disable");
        harness.transport.inject(
            "special_settings",
            Message::SpecialSetting(SpecialSetting::Rest(true)),
        );
        settle().await;
        assert!(
            harness
                .fakes
                .service("ALMotion")
                .calls_to("rest")
                .is_empty()
        );
    }

    use qi::value::IntoValue;
}
