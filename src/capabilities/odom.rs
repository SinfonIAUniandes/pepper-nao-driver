//! Odometry of the robot base.

use super::{Capability, CapabilityId, Context};
use crate::Result;
use crate::domain::{Message, Odometry, Pose3, Quaternion, Twist, Vector3};
use crate::qi::services::ReferenceFrame;
use async_trait::async_trait;

/// Default publishing rate in Hz.
pub const DEFAULT_HZ: f32 = 10.0;

/// Publishes the world pose and the measured base velocity.
pub struct Odom;

#[async_trait]
impl Capability for Odom {
    fn id(&self) -> CapabilityId {
        CapabilityId::Odom
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
        let motion = &ctx.robot.motion;
        let position = motion
            .get_position("Torso", ReferenceFrame::World, true)
            .await?;
        let velocity = motion.get_robot_velocity().await?;
        let odometry = Odometry {
            stamp: ctx.transport.now(),
            frame: if ctx.publish_odom {
                "odom".to_owned()
            } else {
                "odom_wheels".to_owned()
            },
            child_frame: "base_link".to_owned(),
            pose: pose_from_position(&position)?,
            twist: twist_from_velocity(&velocity)?,
        };
        ctx.transport
            .publish(self.id().as_str(), Message::Odometry(odometry))
    }
}

/// Converts `[x, y, z, roll, pitch, yaw]` into a spatial pose.
pub fn pose_from_position(position: &[f32]) -> Result<Pose3> {
    let value = |index: usize| {
        position
            .get(index)
            .copied()
            .ok_or_else(|| crate::Error::qi_value("ALMotion.getPosition", "fewer than 6 values"))
    };
    Ok(Pose3 {
        position: Vector3::new(value(0)?, value(1)?, value(2)?),
        orientation: Quaternion::from_euler(value(3)?, value(4)?, value(5)?),
    })
}

/// Converts `[vx, vy, vz, wx, wy, wz]` into a planar twist.
pub fn twist_from_velocity(velocity: &[f32]) -> Result<Twist> {
    let value = |index: usize| {
        velocity.get(index).copied().ok_or_else(|| {
            crate::Error::qi_value("ALMotion.getRobotVelocity", "fewer than 6 values")
        })
    };
    Ok(Twist {
        vx: value(0)?,
        vy: value(1)?,
        wz: value(5)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use qi::value::IntoValue;

    #[test]
    fn pose_conversion_reads_rpy() {
        let pose = pose_from_position(&[1.0, 2.0, 3.0, 0.0, 0.0, 0.5]).expect("pose");
        assert_eq!(pose.position, Vector3::new(1.0, 2.0, 3.0));
        let rotated = pose.orientation.rotate(Vector3::new(1.0, 0.0, 0.0));
        assert!(rotated.y > 0.4);
        assert!(pose_from_position(&[1.0]).is_err());
    }

    #[test]
    fn twist_conversion_keeps_vx_vy_wz() {
        let twist = twist_from_velocity(&[1.0, 2.0, 0.0, 0.0, 0.0, 0.3]).expect("twist");
        assert_eq!(
            twist,
            Twist {
                vx: 1.0,
                vy: 2.0,
                wz: 0.3
            }
        );
        assert!(twist_from_velocity(&[1.0]).is_err());
    }

    #[tokio::test]
    async fn tick_publishes_pose_and_twist() {
        let harness = Harness::default();
        let motion = harness.fakes.service("ALMotion");
        motion.script(
            "getPosition",
            vec![1.0f32, 0.0, 0.0, 0.0, 0.0, 0.0].into_value(),
        );
        motion.script(
            "getRobotVelocity",
            vec![0.1f32, 0.0, 0.0, 0.0, 0.0, 0.2].into_value(),
        );
        Odom.tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("odom");
        let Message::Odometry(odometry) = &published[0] else {
            panic!("expected odometry");
        };
        assert_eq!(odometry.frame, "odom_wheels");
        assert_eq!(odometry.child_frame, "base_link");
        assert_eq!(odometry.pose.position.x, 1.0);
        assert_eq!(odometry.twist.wz, 0.2);
    }

    #[tokio::test]
    async fn publish_odom_renames_the_frame() {
        let harness = Harness::default();
        let mut ctx = harness.ctx;
        ctx.publish_odom = true;
        let motion = harness.fakes.service("ALMotion");
        motion.script("getPosition", vec![0.0f32; 6].into_value());
        motion.script("getRobotVelocity", vec![0.0f32; 6].into_value());
        Odom.tick(&ctx).await.expect("tick");
        let published = harness.transport.published_on("odom");
        let Message::Odometry(odometry) = &published[0] else {
            panic!("expected odometry");
        };
        assert_eq!(odometry.frame, "odom");
    }
}
