//! Merged laser scan: the physical beams plus the external depth-to-laser module.

use super::{Capability, CapabilityId, Context};
use crate::domain::{LaserScan, Message, Timestamp};
use crate::qi::keys;
use crate::qi::services::AlMemory;
use crate::qi::value::{as_f32, as_f32s};
use crate::Result;
use async_trait::async_trait;
use qi::value::Value;

/// Default publishing rate in Hz.
pub const DEFAULT_HZ: f32 = 10.0;

/// Samples of the merged scan; the physical beams are spread over it.
const SAMPLES: usize = 512;
/// Physical beams of the three on-board lasers.
const BEAMS: usize = 61;
/// Physical beams replaced by the depth reading in the merged scan.
const SKIPPED_FIRST: usize = 23;
const SKIPPED_LAST: usize = 38;
/// Physical beam index that splits the scan into its two halves and bounds the
/// hole fill on the near side of the depth sector.
const HALF_SPLIT: usize = 14;
/// Physical beam index bounding the hole fill on the far side of the depth
/// sector. Both bounds come from the reference toolkit.
const RIGHT_FILL_BOUND: usize = 47;

const ANGLE_SPAN: f32 = 2.0944;
const SCAN_TIME: f32 = 0.01;
/// Fill for samples with no reading from either sensor.
const FREE: f32 = 80.0;
/// Fill for the gap sectors between the sensors; `-1` marks a hole.
const HOLE: f32 = -1.0;
/// Physical beams farther than this count as free space.
const PHYSICAL_FREE_LIMIT: f32 = 1.0;

/// One reading of the external `NAOqiDepth2Laser` module.
#[derive(Clone, Debug, PartialEq)]
pub struct DepthSample {
    pub ranges: Vec<f32>,
    pub min_angle: f32,
    pub max_angle: f32,
    pub num_ranges: f32,
    pub max_range: f32,
}

/// Publishes the merged scan.
pub struct MergedLaser;

#[async_trait]
impl Capability for MergedLaser {
    fn id(&self) -> CapabilityId {
        CapabilityId::MergedLaser
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
        let stamp = ctx.transport.now();
        let (Ok(readings), Some(depth)) = (
            ctx.robot.memory.get_list_data(&keys::laser_keys()).await,
            read_depth_sample(&ctx.robot.memory).await,
        ) else {
            tracing::debug!("NAOqiDepth2Laser not available");
            return Ok(());
        };
        let xy = floats_or_holes(&readings);
        let scan = super::laser::build_scan(&xy, stamp);
        let merged = merge(&scan.ranges, &depth, stamp);
        ctx.transport
            .publish(self.id().as_str(), Message::LaserScan(merged))
    }
}

/// Merges the 61 physical beams with a depth sample into one wide scan.
///
/// The depth sensor covers the front sector and the physical beams are spread
/// over the scan in between; the sectors between the two sensors are holes, as
/// in the reference toolkit.
pub fn merge(physical: &[f32], depth: &DepthSample, stamp: Timestamp) -> LaserScan {
    let step = (2.0 * ANGLE_SPAN) / SAMPLES as f32;
    let count = depth.ranges.len().min(depth.num_ranges.max(0.0) as usize);
    let depth_first = ((depth.min_angle + ANGLE_SPAN) / step) as i32;
    let beta = (depth.max_angle - depth.min_angle) / depth.num_ranges;
    let beta_p = depth.max_angle / (SAMPLES as f32 / 2.0 - depth_first as f32);

    let mut ranges = vec![FREE; SAMPLES];
    for (index, &range) in depth.ranges.iter().take(count).enumerate() {
        let slot = ((index as f32 * beta) / beta_p).round() as i32 + depth_first;
        if let Some(slot) = usize::try_from(slot).ok().filter(|slot| *slot < SAMPLES) {
            ranges[slot] = range;
        }
    }

    let depth_last = ((count.saturating_sub(1) as f32 * beta) / beta_p).round() as i32 + depth_first;
    let mut halves = [Vec::new(), Vec::new()];
    for (index, &range) in physical.iter().take(BEAMS).enumerate() {
        let range = if range > PHYSICAL_FREE_LIMIT {
            FREE
        } else {
            range
        };
        if !(SKIPPED_FIRST..=SKIPPED_LAST).contains(&index) {
            let slot = beam_slot(index);
            ranges[slot] = range;
            halves[usize::from(index > HALF_SPLIT)].push(slot);
        }
    }

    fill(
        &mut ranges,
        beam_slot(HALF_SPLIT) as i32 + 1,
        depth_first,
        HOLE,
    );
    fill(
        &mut ranges,
        depth_last + 1,
        beam_slot(RIGHT_FILL_BOUND) as i32,
        HOLE,
    );

    for slots in &halves {
        for pair in slots.windows(2) {
            let middle = (pair[0] + pair[1]) / 2;
            ranges[middle] = (ranges[pair[0]] + ranges[pair[1]]) / 2.0;
        }
    }

    LaserScan {
        stamp,
        frame: "base_footprint".to_owned(),
        angle_min: -ANGLE_SPAN,
        angle_max: ANGLE_SPAN,
        angle_increment: step,
        scan_time: SCAN_TIME,
        time_increment: SCAN_TIME / depth.num_ranges,
        range_min: 0.1,
        range_max: depth.max_range,
        ranges,
    }
}

/// Scan slot fed by physical beam `index`.
fn beam_slot(index: usize) -> usize {
    index * (SAMPLES - 1) / BEAMS
}

fn fill(ranges: &mut [f32], from: i32, to: i32, value: f32) {
    let limit = ranges.len() as i32;
    let from = from.clamp(0, limit) as usize;
    let to = to.clamp(0, limit) as usize;
    if from < to {
        ranges[from..to].fill(value);
    }
}

async fn read_depth_sample(memory: &AlMemory) -> Option<DepthSample> {
    let (Ok(ranges), Ok(min_angle), Ok(max_angle), Ok(num_ranges), Ok(max_range)) = (
        memory.get_data(keys::DEPTH2LASER_RANGES).await,
        memory.get_data(keys::DEPTH2LASER_MIN_ANGLE).await,
        memory.get_data(keys::DEPTH2LASER_MAX_ANGLE).await,
        memory.get_data(keys::DEPTH2LASER_NUM_RANGES).await,
        memory.get_data(keys::DEPTH2LASER_MAX_RANGE).await,
    ) else {
        return None;
    };
    Some(DepthSample {
        ranges: as_f32s(&ranges)?,
        min_angle: as_f32(&min_angle)?,
        max_angle: as_f32(&max_angle)?,
        num_ranges: as_f32(&num_ranges)?,
        max_range: as_f32(&max_range)?,
    })
}

/// Non-numeric readings are holes, as on the wire.
fn floats_or_holes(value: &Value<'_>) -> Vec<f32> {
    match value {
        Value::List(elements) | Value::Tuple(elements) => elements
            .iter()
            .map(|element| as_f32(element).unwrap_or(HOLE))
            .collect(),
        _ => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::support::Harness;
    use crate::capabilities::Capability;
    use qi::value::IntoValue;

    /// Depth sample whose four readings land on scan slots 133, 195, 256, 318.
    fn depth_sample() -> DepthSample {
        DepthSample {
            ranges: vec![1.0, 2.0, 3.0, 4.0],
            min_angle: -1.0,
            max_angle: 1.0,
            num_ranges: 4.0,
            max_range: 5.0,
        }
    }

    fn holes() -> Vec<f32> {
        vec![HOLE; BEAMS]
    }

    #[test]
    fn merge_places_depth_samples_and_fills_the_gaps() {
        let scan = merge(&holes(), &depth_sample(), Timestamp::default());

        assert_eq!(scan.ranges.len(), SAMPLES);
        assert_eq!(scan.frame, "base_footprint");
        assert_eq!(scan.angle_min, -2.0944);
        assert_eq!(scan.angle_max, 2.0944);
        assert!((scan.angle_increment - (2.0 * ANGLE_SPAN) / SAMPLES as f32).abs() < 1e-9);
        assert_eq!(scan.scan_time, 0.01);
        assert_eq!(scan.time_increment, 0.0025);
        assert_eq!(scan.range_min, 0.1);
        assert_eq!(scan.range_max, 5.0);

        assert_eq!(scan.ranges[133], 1.0);
        assert_eq!(scan.ranges[195], 2.0);
        assert_eq!(scan.ranges[256], 3.0);
        assert_eq!(scan.ranges[318], 4.0);

        assert!(scan.ranges[118..133].iter().all(|&range| range == HOLE));
        assert!(scan.ranges[319..393].iter().all(|&range| range == HOLE));
    }

    #[test]
    fn merge_keeps_close_beams_and_marks_distant_ones_free() {
        let mut physical = holes();
        physical.fill(0.5);
        physical[1] = 2.0;

        let scan = merge(&physical, &depth_sample(), Timestamp::default());

        assert_eq!(scan.ranges[0], 0.5);
        assert_eq!(scan.ranges[beam_slot(1)], FREE);
    }

    #[test]
    fn merge_leaves_the_skipped_sector_to_the_depth_sensor() {
        let scan = merge(&holes(), &depth_sample(), Timestamp::default());
        assert_eq!(scan.ranges[beam_slot(25)], FREE);
    }

    #[tokio::test]
    async fn tick_publishes_the_merged_scan() {
        let harness = Harness::default();
        let memory = harness.fakes.service("ALMemory");
        memory.script("getListData", vec![0.5f32; 90].into_value());
        memory.script_for("getData", keys::DEPTH2LASER_RANGES, vec![1.0f32, 2.0].into_value());
        memory.script_for("getData", keys::DEPTH2LASER_MIN_ANGLE, (-1.0f32).into_value());
        memory.script_for("getData", keys::DEPTH2LASER_MAX_ANGLE, 1.0f32.into_value());
        memory.script_for("getData", keys::DEPTH2LASER_NUM_RANGES, 2.0f32.into_value());
        memory.script_for("getData", keys::DEPTH2LASER_MAX_RANGE, 5.0f32.into_value());

        MergedLaser.tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("merged_laser");
        assert_eq!(published.len(), 1);
        let Message::LaserScan(scan) = &published[0] else {
            panic!("expected a scan");
        };
        assert_eq!(scan.ranges.len(), SAMPLES);
    }

    #[tokio::test]
    async fn missing_depth_module_skips_the_cycle() {
        let harness = Harness::default();
        harness
            .fakes
            .service("ALMemory")
            .script("getListData", vec![0.5f32; 90].into_value());

        MergedLaser.tick(&harness.ctx).await.expect("tick");

        assert!(harness.transport.published_on("merged_laser").is_empty());
    }
}
