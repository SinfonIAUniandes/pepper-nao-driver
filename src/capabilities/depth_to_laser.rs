//! Depth-to-laser scan from the external `NAOqiDepth2Laser` module.
//!
//! The driver only reads the module keys; range computation lives outside.

use super::{Capability, CapabilityId, Configuration, ConfigurationResult, Context};
use crate::Result;
use crate::domain::{DepthToLaserParams, LaserScan, Message};
use crate::qi::keys;
use crate::qi::value::{as_f32, as_f32s};
use async_trait::async_trait;
use std::sync::Mutex;

/// Default publishing rate in Hz.
pub const DEFAULT_HZ: f32 = 10.0;

/// Laser scan published from the external module's keys.
pub struct DepthToLaser {
    params: Mutex<DepthToLaserParams>,
}

impl DepthToLaser {
    pub fn new() -> Self {
        Self {
            params: Mutex::new(DepthToLaserParams::default()),
        }
    }
}

impl Default for DepthToLaser {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for DepthToLaser {
    fn id(&self) -> CapabilityId {
        CapabilityId::DepthToLaser
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
        let memory = &ctx.robot.memory;
        // The external module may be absent: skip the cycle quietly.
        let (Ok(ranges), Ok(min_angle), Ok(max_angle), Ok(num_ranges), Ok(max_range)) = (
            memory.get_data(keys::DEPTH2LASER_RANGES).await,
            memory.get_data(keys::DEPTH2LASER_MIN_ANGLE).await,
            memory.get_data(keys::DEPTH2LASER_MAX_ANGLE).await,
            memory.get_data(keys::DEPTH2LASER_NUM_RANGES).await,
            memory.get_data(keys::DEPTH2LASER_MAX_RANGE).await,
        ) else {
            tracing::debug!("NAOqiDepth2Laser not available");
            return Ok(());
        };
        let Some(ranges) = as_f32s(&ranges) else {
            return Ok(());
        };
        let (Some(min_angle), Some(max_angle), Some(num_ranges), Some(max_range)) = (
            as_f32(&min_angle),
            as_f32(&max_angle),
            as_f32(&num_ranges),
            as_f32(&max_range),
        ) else {
            return Ok(());
        };
        let params = *self.params.lock().unwrap_or_else(|err| err.into_inner());
        let scan = LaserScan {
            stamp: ctx.transport.now(),
            frame: "base_footprint".to_owned(),
            angle_min: min_angle,
            angle_max: max_angle,
            angle_increment: (max_angle - min_angle) / num_ranges,
            scan_time: params.scan_time,
            time_increment: params.scan_time / num_ranges,
            range_min: params.range_min,
            range_max: max_range,
            ranges,
        };
        ctx.transport
            .publish(self.id().as_str(), Message::LaserScan(scan))
    }

    async fn configure(
        &self,
        _ctx: &Context,
        configuration: Configuration,
    ) -> Result<ConfigurationResult> {
        match configuration {
            Configuration::DepthToLaser(params) => {
                *self.params.lock().unwrap_or_else(|err| err.into_inner()) = params;
                Ok(ConfigurationResult::None)
            }
            _ => Ok(ConfigurationResult::None),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use qi::value::IntoValue;

    fn script_module(harness: &Harness) {
        let memory = harness.fakes.service("ALMemory");
        memory.script_for(
            "getData",
            keys::DEPTH2LASER_RANGES,
            vec![1.0f32, 2.0].into_value(),
        );
        memory.script_for(
            "getData",
            keys::DEPTH2LASER_MIN_ANGLE,
            (-1.0f32).into_value(),
        );
        memory.script_for("getData", keys::DEPTH2LASER_MAX_ANGLE, 1.0f32.into_value());
        memory.script_for("getData", keys::DEPTH2LASER_NUM_RANGES, 2.0f32.into_value());
        memory.script_for("getData", keys::DEPTH2LASER_MAX_RANGE, 5.0f32.into_value());
    }

    #[tokio::test]
    async fn tick_publishes_the_module_scan() {
        let harness = Harness::default();
        script_module(&harness);
        DepthToLaser::new().tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("depth_to_laser");
        let Message::LaserScan(scan) = &published[0] else {
            panic!("expected a scan");
        };
        assert_eq!(scan.frame, "base_footprint");
        assert_eq!(scan.ranges, vec![1.0, 2.0]);
        assert_eq!(scan.angle_increment, 1.0);
        assert_eq!(scan.time_increment, 0.005);
        assert_eq!(scan.range_max, 5.0);
    }

    #[tokio::test]
    async fn missing_module_skips_the_cycle() {
        let harness = Harness::default();
        DepthToLaser::new().tick(&harness.ctx).await.expect("tick");
        assert!(harness.transport.published_on("depth_to_laser").is_empty());
    }

    #[tokio::test]
    async fn configure_updates_the_published_metadata() {
        let harness = Harness::default();
        script_module(&harness);
        let capability = DepthToLaser::new();
        capability
            .configure(
                &harness.ctx,
                Configuration::DepthToLaser(DepthToLaserParams {
                    scan_time: 0.02,
                    range_min: 0.2,
                    ..DepthToLaserParams::default()
                }),
            )
            .await
            .expect("configure");
        capability.tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("depth_to_laser");
        let Message::LaserScan(scan) = &published[0] else {
            panic!("expected a scan");
        };
        assert_eq!(scan.scan_time, 0.02);
        assert_eq!(scan.range_min, 0.2);
    }
}
