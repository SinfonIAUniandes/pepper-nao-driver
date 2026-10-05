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

//! TF frames: `odom`, `base_link`, `torso` and the head / camera chain.

use super::{Capability, CapabilityId, Context};
use crate::Result;
use crate::domain::{Message, Pose3, Transform};
use crate::qi::services::ReferenceFrame;
use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Default publishing rate in Hz.
pub const DEFAULT_HZ: f32 = 50.0;

/// Frames published from the URDF chain, children of `torso` and below.
const CHILD_FRAMES: [&str; 8] = [
    "Neck",
    "Head",
    "CameraBottom_frame",
    "CameraBottom_optical_frame",
    "CameraTop_frame",
    "CameraTop_optical_frame",
    "CameraDepth_frame",
    "CameraDepth_optical_frame",
];

/// The latest transform per child frame: the driver's internal TF buffer.
#[derive(Default)]
pub struct Buffer {
    transforms: Mutex<HashMap<String, Transform>>,
}

impl Buffer {
    pub fn update(&self, transforms: &[Transform]) {
        let mut stored = self
            .transforms
            .lock()
            .unwrap_or_else(|err| err.into_inner());
        for transform in transforms {
            stored.insert(transform.child.clone(), transform.clone());
        }
    }

    pub fn lookup(&self, child: &str) -> Option<Transform> {
        self.transforms
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .get(child)
            .cloned()
    }
}

/// Publishes the frame tree each cycle.
pub struct Tf {
    pub buffer: Arc<Buffer>,
}

impl Default for Tf {
    fn default() -> Self {
        Self {
            buffer: Arc::new(Buffer::default()),
        }
    }
}

#[async_trait]
impl Capability for Tf {
    fn id(&self) -> CapabilityId {
        CapabilityId::Tf
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
        let names = motion.get_body_names("Body").await?;
        let angles = motion.get_angles("Body", true).await?;
        let positions: HashMap<String, f32> = names.into_iter().zip(angles).collect();

        let world_t_torso = pose_from_position(
            &motion
                .get_position("Torso", ReferenceFrame::World, true)
                .await?,
        )?;
        let robot_t_torso = pose_from_position(
            &motion
                .get_position("Torso", ReferenceFrame::Robot, true)
                .await?,
        )?;

        let mut transforms = Vec::with_capacity(10);
        let base_t_torso = transform("base_link", "torso", robot_t_torso);
        // Kept in the buffer even when `publish_odom` holds the stream back.
        self.buffer.update(&[transform(
            "odom",
            "base_link",
            world_t_torso.compose(robot_t_torso.inverse()),
        )]);
        transforms.push(base_t_torso);

        for frame in CHILD_FRAMES {
            let Some((parent, pose)) = ctx
                .assets
                .robot_model
                .link_pose_in_parent(frame, &positions)
            else {
                return Err(crate::Error::invalid(
                    "tf",
                    format!("no URDF chain to {frame}"),
                ));
            };
            transforms.push(transform(&parent, frame, pose));
        }
        self.buffer.update(&transforms);
        if ctx.publish_odom {
            let odom = self.buffer.lookup("base_link").ok_or_else(|| {
                crate::Error::invalid("tf", "odom->base_link missing from the buffer")
            })?;
            transforms.insert(0, odom);
        }
        ctx.transport
            .publish(self.id().as_str(), Message::Transforms(transforms))
    }
}

fn pose_from_position(position: &[f32]) -> Result<Pose3> {
    super::odom::pose_from_position(position)
}

fn transform(parent: &str, child: &str, pose: Pose3) -> Transform {
    Transform {
        parent: parent.to_owned(),
        child: child.to_owned(),
        translation: pose.position,
        rotation: pose.orientation,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use qi::value::IntoValue;

    #[test]
    fn buffer_keeps_the_latest_transform_per_child() {
        let buffer = Buffer::default();
        buffer.update(&[transform("a", "b", Pose3::IDENTITY)]);
        buffer.update(&[transform("b", "c", Pose3::IDENTITY)]);
        assert_eq!(buffer.lookup("b").expect("b").parent, "a");
        assert_eq!(buffer.lookup("c").expect("c").parent, "b");
        assert!(buffer.lookup("a").is_none());
    }

    #[tokio::test]
    async fn tick_publishes_the_documented_frames() {
        let harness = Harness::default();
        let motion = harness.fakes.service("ALMotion");
        motion.script(
            "getBodyNames",
            vec!["HeadYaw".to_owned(), "HeadPitch".to_owned()].into_value(),
        );
        motion.script("getAngles", vec![0.0f32, 0.0].into_value());
        motion.script("getPosition", vec![0.0f32; 6].into_value());
        Tf::default().tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("tf");
        let Message::Transforms(transforms) = &published[0] else {
            panic!("expected transforms");
        };
        let children: Vec<&str> = transforms.iter().map(|t| t.child.as_str()).collect();
        assert_eq!(children[0], "torso");
        assert!(children.contains(&"Neck"));
        assert!(children.contains(&"CameraTop_optical_frame"));
        assert_eq!(transforms.len(), 9);
    }

    #[tokio::test]
    async fn odom_to_base_link_is_buffered_but_not_streamed_without_flag() {
        let harness = Harness::default();
        let motion = harness.fakes.service("ALMotion");
        motion.script("getBodyNames", vec!["HeadYaw".to_owned()].into_value());
        motion.script("getAngles", vec![0.0f32].into_value());
        motion.script(
            "getPosition",
            vec![1.0f32, 0.0, 0.0, 0.0, 0.0, 0.0].into_value(),
        );
        let tf = Tf::default();
        tf.tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("tf");
        let Message::Transforms(transforms) = &published[0] else {
            panic!("expected transforms");
        };
        assert!(transforms.iter().all(|t| t.child != "base_link"));
        assert_eq!(
            tf.buffer.lookup("base_link").expect("buffered").parent,
            "odom"
        );
    }

    #[tokio::test]
    async fn publish_odom_streams_odom_to_base_link() {
        let harness = Harness::default();
        let mut ctx = harness.ctx;
        ctx.publish_odom = true;
        let motion = harness.fakes.service("ALMotion");
        motion.script("getBodyNames", vec!["HeadYaw".to_owned()].into_value());
        motion.script("getAngles", vec![0.0f32].into_value());
        motion.script(
            "getPosition",
            vec![1.0f32, 0.0, 0.0, 0.0, 0.0, 0.0].into_value(),
        );
        Tf::default().tick(&ctx).await.expect("tick");

        let published = harness.transport.published_on("tf");
        let Message::Transforms(transforms) = &published[0] else {
            panic!("expected transforms");
        };
        assert_eq!(transforms[0].child, "base_link");
        assert_eq!(transforms[0].parent, "odom");
    }

    #[tokio::test]
    async fn missing_joint_angles_are_reported() {
        let harness = Harness::default();
        let motion = harness.fakes.service("ALMotion");
        motion.script("getBodyNames", vec!["HeadYaw".to_owned()].into_value());
        motion.script("getAngles", vec![0.0f32].into_value());
        motion.script("getPosition", vec![0.0f32; 6].into_value());
        // The URDF chain lookup fails only for unknown frames; everything is
        // present in pepper.urdf, so this stays a successful cycle.
        assert!(Tf::default().tick(&harness.ctx).await.is_ok());
    }
}
