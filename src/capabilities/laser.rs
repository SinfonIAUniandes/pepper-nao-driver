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

//! Physical laser scan built from the three on-board laser segments.

use super::{Capability, CapabilityId, Context};
use crate::Result;
use crate::domain::{LaserScan, Message, Timestamp};
use crate::qi::keys;
use crate::qi::value::as_f32s_lossy;
use async_trait::async_trait;

/// Default publishing rate in Hz.
pub const DEFAULT_HZ: f32 = 10.0;

const ANGLE_SPAN: f32 = 2.0944;
const SEGMENTS: usize = 15;
const CLUSTER: usize = 2 * SEGMENTS;
const BEAMS: usize = 61;
const HOLE: f32 = -1.0;

const RIGHT_ROTATION: f32 = -1.757;
const LEFT_ROTATION: f32 = 1.757;
const OFFSET_X: f32 = -0.018;
const OFFSET_RIGHT_Y: f32 = -0.090;
const OFFSET_LEFT_Y: f32 = 0.090;
const FRONT_OFFSET_X: f32 = 0.056;

/// Converts the three laser clusters into one scan.
///
/// Each cluster contributes 15 beams in reversed segment order, separated by
/// eight empty beams, matching the on-board laser layout.
pub struct Laser;

#[async_trait]
impl Capability for Laser {
    fn id(&self) -> CapabilityId {
        CapabilityId::Laser
    }

    fn period(&self) -> Option<f32> {
        Some(DEFAULT_HZ)
    }

    async fn enable(&self, _ctx: &Context) -> Result<()> {
        Ok(())
    }

    async fn disable(&self, _ctx: &Context) -> Result<()> {
        Ok(())
    }

    async fn tick(&self, ctx: &Context) -> Result<()> {
        let keys = keys::laser_keys();
        let raw = ctx.robot.memory.get_list_data(&keys).await?;
        let xy = as_f32s_lossy(&raw);
        let scan = build_scan(&xy, ctx.transport.now());
        ctx.transport
            .publish(self.id().as_str(), Message::LaserScan(scan))
    }
}

/// Builds the scan from the 90 XY readings, in key order.
pub fn build_scan(xy: &[f32], stamp: Timestamp) -> LaserScan {
    let mut ranges = vec![HOLE; BEAMS];
    let mut beam = 0;
    for (cluster, rotation, offset_y) in [
        (28, RIGHT_ROTATION, OFFSET_RIGHT_Y),
        (58, 0.0, 0.0),
        (88, LEFT_ROTATION, OFFSET_LEFT_Y),
    ] {
        for step in (0..CLUSTER).step_by(2) {
            let (lx, ly) = (
                xy.get(cluster - step).copied().unwrap_or(HOLE),
                xy.get(cluster - step + 1).copied().unwrap_or(HOLE),
            );
            let offset_x = if rotation == 0.0 {
                FRONT_OFFSET_X
            } else {
                OFFSET_X
            };
            let (sin, cos) = rotation.sin_cos();
            let bx = lx * cos - ly * sin + offset_x;
            let by = lx * sin + ly * cos + offset_y;
            ranges[beam] = bx.hypot(by);
            beam += 1;
        }
        beam += 8;
    }
    LaserScan {
        stamp,
        frame: "base_link".to_owned(),
        angle_min: -ANGLE_SPAN,
        angle_max: ANGLE_SPAN,
        angle_increment: (2.0 * ANGLE_SPAN) / BEAMS as f32,
        scan_time: 0.0,
        time_increment: 0.0,
        range_min: 0.1,
        range_max: 1.5,
        ranges,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clusters_are_separated_by_holes() {
        let scan = build_scan(&vec![0.0; 90], Timestamp::default());
        assert_eq!(scan.ranges.len(), BEAMS);
        for beam in [15..23, 38..46] {
            for index in beam {
                assert_eq!(scan.ranges[index], HOLE, "beam {index}");
            }
        }
    }

    #[test]
    fn zero_readings_give_the_sensor_offsets() {
        let scan = build_scan(&vec![0.0; 90], Timestamp::default());
        let right = OFFSET_X.hypot(OFFSET_RIGHT_Y);
        let left = OFFSET_X.hypot(OFFSET_LEFT_Y);
        assert!((scan.ranges[0] - right).abs() < 1e-5);
        assert!((scan.ranges[23] - FRONT_OFFSET_X).abs() < 1e-5);
        assert!((scan.ranges[46] - left).abs() < 1e-5);
    }

    #[test]
    fn segments_are_read_in_reversed_order() {
        // Only the first segment pair of the right cluster is far away; it is
        // the last beam of that cluster.
        let mut xy = vec![0.0; 90];
        xy[0] = 10.0;
        xy[1] = 10.0;
        let scan = build_scan(&xy, Timestamp::default());
        assert!(scan.ranges[14] > 5.0, "seg01 lands on the last beam");
        assert!(scan.ranges[0] < 1.0, "seg15 stays close");
    }

    #[test]
    fn angle_layout_matches_the_spec() {
        let scan = build_scan(&[], Timestamp::default());
        assert_eq!(scan.angle_min, -2.0944);
        assert_eq!(scan.angle_max, 2.0944);
        assert!((scan.angle_increment - (2.0 * 2.0944) / 61.0).abs() < 1e-6);
        assert_eq!(scan.frame, "base_link");
    }

    #[tokio::test]
    async fn tick_reads_the_ninety_keys_and_publishes() {
        use crate::capabilities::support::Harness;
        use qi::value::IntoValue;

        let harness = Harness::default();
        harness
            .fakes
            .service("ALMemory")
            .script("getListData", vec![0.0f32; 90].into_value());
        Laser.tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("laser");
        assert_eq!(published.len(), 1);
        let Message::LaserScan(scan) = &published[0] else {
            panic!("expected a laser scan");
        };
        assert_eq!(scan.ranges.len(), BEAMS);
    }
}
