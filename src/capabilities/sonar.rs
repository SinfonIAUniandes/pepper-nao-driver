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

//! Sonar ranges, one message per head.

use super::{Capability, CapabilityId, Context};
use crate::Result;
use crate::domain::{Message, Range};
use crate::qi::keys;
use crate::qi::value::as_f32s;
use async_trait::async_trait;
use std::sync::atomic::{AtomicBool, Ordering};

/// Publishing rate in Hz.
pub const DEFAULT_HZ: f32 = 50.0;

/// Client name that enables the sonars.
const SONAR_CLIENT: &str = "ROS";

/// Opening angle of the sonar cone (pi / 6).
const FIELD_OF_VIEW: f32 = std::f32::consts::FRAC_PI_6;
const MIN: f32 = 0.25;
const MAX: f32 = 2.55;

const HEADS: [(&str, &str); 2] = [
    (keys::SONAR_FRONT, "SonarFront_frame"),
    (keys::SONAR_BACK, "SonarBack_frame"),
];

/// Publishes the front and back sonar readings.
pub struct Sonar {
    subscribed: AtomicBool,
}

impl Sonar {
    pub fn new() -> Self {
        Self {
            subscribed: AtomicBool::new(false),
        }
    }
}

impl Default for Sonar {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for Sonar {
    fn id(&self) -> CapabilityId {
        CapabilityId::Sonar
    }

    fn period(&self) -> Option<f32> {
        Some(DEFAULT_HZ)
    }

    async fn enable(&self, _ctx: &Context) -> Result<()> {
        Ok(())
    }

    async fn disable(&self, ctx: &Context) -> Result<()> {
        if self.subscribed.swap(false, Ordering::Relaxed) {
            ctx.robot.sonar.unsubscribe(SONAR_CLIENT).await?;
        }
        Ok(())
    }

    async fn tick(&self, ctx: &Context) -> Result<()> {
        if !self.subscribed.load(Ordering::Relaxed) {
            ctx.robot.sonar.subscribe(SONAR_CLIENT).await?;
            self.subscribed.store(true, Ordering::Relaxed);
        }
        let keys: Vec<String> = HEADS.iter().map(|(key, _)| (*key).to_owned()).collect();
        let raw = ctx.robot.memory.get_list_data(&keys).await?;
        let Some(values) = as_f32s(&raw) else {
            return Ok(());
        };
        let stamp = ctx.transport.now();
        for (index, (_, frame)) in HEADS.iter().enumerate() {
            let value = values.get(index).copied().unwrap_or(f32::NAN);
            ctx.transport.publish(
                self.id().as_str(),
                Message::Range(Range {
                    stamp,
                    frame: (*frame).to_owned(),
                    field_of_view: FIELD_OF_VIEW,
                    min: MIN,
                    max: MAX,
                    value,
                }),
            )?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use qi::value::IntoValue;

    #[tokio::test]
    async fn tick_subscribes_once_and_publishes_both_heads() {
        let harness = Harness::default();
        harness
            .fakes
            .service("ALMemory")
            .script("getListData", vec![0.5f32, 1.5].into_value());
        let sonar = Sonar::new();

        sonar.tick(&harness.ctx).await.expect("tick");
        sonar.tick(&harness.ctx).await.expect("tick");

        let calls = harness.fakes.service("ALSonar").calls_to("subscribe");
        assert_eq!(calls, vec!["ROS".to_owned().into_value()]);

        let published = harness.transport.published_on("sonar");
        assert_eq!(published.len(), 4);
        let Message::Range(front) = &published[0] else {
            panic!("expected a range");
        };
        assert_eq!(front.frame, "SonarFront_frame");
        assert_eq!(front.value, 0.5);
        assert_eq!(front.field_of_view, FIELD_OF_VIEW);
        assert_eq!((front.min, front.max), (MIN, MAX));
        let Message::Range(back) = &published[1] else {
            panic!("expected a range");
        };
        assert_eq!(back.frame, "SonarBack_frame");
        assert_eq!(back.value, 1.5);
    }

    #[tokio::test]
    async fn missing_readings_skip_the_cycle() {
        let harness = Harness::default();
        let sonar = Sonar::new();
        sonar.tick(&harness.ctx).await.expect("tick");
        assert!(harness.transport.published_on("sonar").is_empty());
    }

    #[tokio::test]
    async fn disable_unsubscribes_and_resets() {
        let harness = Harness::default();
        harness
            .fakes
            .service("ALMemory")
            .script("getListData", vec![0.5f32, 1.5].into_value());
        let sonar = Sonar::new();
        sonar.tick(&harness.ctx).await.expect("tick");
        sonar.disable(&harness.ctx).await.expect("disable");
        sonar.disable(&harness.ctx).await.expect("idempotent");

        assert_eq!(
            harness.fakes.service("ALSonar").calls_to("unsubscribe"),
            vec!["ROS".to_owned().into_value()]
        );

        sonar.tick(&harness.ctx).await.expect("tick");
        assert_eq!(
            harness.fakes.service("ALSonar").calls_to("subscribe").len(),
            2
        );
    }
}
