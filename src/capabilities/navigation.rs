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

//! Planner and localizer bridge: goals and poses in, paths and results out.

use super::{Capability, CapabilityId, Context, message_handler};
use crate::Result;
use crate::domain::{Message, Path, Pose2, Vector3};
use crate::qi::keys;
use crate::qi::services::Robot;
use crate::qi::value::as_f32s;
use crate::transport::Subscription;
use async_trait::async_trait;
use qi::value::{IntoValue, Value};
use std::sync::Mutex;

/// Default publishing rate in Hz.
pub const DEFAULT_HZ: f32 = 10.0;

/// Localizer pose to publish while the key is missing.
const UNKNOWN_POSE: Vector3 = Vector3::new(f32::NAN, f32::NAN, f32::NAN);

/// Raises `NAOqiPlanner/Goal` with navigation goals.
pub struct NavigationGoal {
    subscription: Mutex<Option<Subscription>>,
}

impl NavigationGoal {
    pub fn new() -> Self {
        Self {
            subscription: Mutex::new(None),
        }
    }
}

impl Default for NavigationGoal {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for NavigationGoal {
    fn id(&self) -> CapabilityId {
        CapabilityId::NavigationGoal
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let subscription =
            subscribe_pose(
                ctx,
                self.id(),
                keys::PLANNER_GOAL,
                |message| match message {
                    Message::NavigationGoal(pose) => Some(*pose),
                    _ => None,
                },
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

/// Raises `NAOqiLocalizer/SetPose` to move the localizer.
pub struct PoseSet {
    subscription: Mutex<Option<Subscription>>,
}

impl PoseSet {
    pub fn new() -> Self {
        Self {
            subscription: Mutex::new(None),
        }
    }
}

impl Default for PoseSet {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for PoseSet {
    fn id(&self) -> CapabilityId {
        CapabilityId::PoseSet
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let subscription =
            subscribe_pose(
                ctx,
                self.id(),
                keys::LOCALIZER_SET_POSE,
                |message| match message {
                    Message::PoseSet(pose) => Some(*pose),
                    _ => None,
                },
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

/// Publishes the path computed by the external planner.
pub struct NavigationPath;

#[async_trait]
impl Capability for NavigationPath {
    fn id(&self) -> CapabilityId {
        CapabilityId::NavigationPath
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
        let points = match ctx.robot.memory.get_data(keys::PLANNER_PATH).await {
            Ok(raw) => path_points(&raw),
            Err(_) => Vec::new(),
        };
        ctx.transport.publish(
            self.id().as_str(),
            Message::Path(Path {
                stamp: ctx.transport.now(),
                frame: "odom".to_owned(),
                points,
            }),
        )
    }
}

/// Publishes the pose reported by the external localizer.
pub struct PosePub;

#[async_trait]
impl Capability for PosePub {
    fn id(&self) -> CapabilityId {
        CapabilityId::PosePub
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
        let pose = match ctx.robot.memory.get_data(keys::LOCALIZER_ROBOT_POSE).await {
            Ok(raw) => localizer_pose(&raw).unwrap_or(UNKNOWN_POSE),
            Err(_) => UNKNOWN_POSE,
        };
        ctx.transport
            .publish(self.id().as_str(), Message::Vector3(pose))
    }
}

/// Publishes the planner outcome event.
pub struct NavigationResult;

#[async_trait]
impl Capability for NavigationResult {
    fn id(&self) -> CapabilityId {
        CapabilityId::NavigationResult
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let transport = ctx.transport.clone();
        let topic = self.id().as_str();
        ctx.toolkit.result().set(move |result| {
            if let Err(err) = transport.publish(topic, Message::Text(result)) {
                tracing::warn!(error = %err, "publish failed");
            }
        });
        if let Err(err) = ctx
            .robot
            .memory
            .subscribe_to_event(
                keys::PLANNER_RESULT,
                &keys::callback_service_name(keys::PLANNER_RESULT),
                "onResultCallback",
            )
            .await
        {
            ctx.toolkit.result().clear();
            return Err(err);
        }
        Ok(())
    }

    async fn disable(&self, ctx: &Context) -> Result<()> {
        ctx.toolkit.result().clear();
        ctx.robot
            .memory
            .unsubscribe_to_event(
                keys::PLANNER_RESULT,
                &keys::callback_service_name(keys::PLANNER_RESULT),
            )
            .await
    }
}

/// Subscribes a `Pose2` command topic that raises an ALMemory event.
fn subscribe_pose(
    ctx: &Context,
    id: CapabilityId,
    key: &'static str,
    extract: fn(&Message) -> Option<Pose2>,
) -> Subscription {
    let robot = ctx.robot.clone();
    ctx.transport.subscribe(
        id.as_str(),
        message_handler(extract, move |pose| {
            let robot = robot.clone();
            Box::pin(async move { raise_pose(&robot, key, pose).await })
        }),
    )
}

async fn raise_pose(robot: &Robot, key: &'static str, pose: Pose2) -> Result<()> {
    let values = vec![pose.position.x, pose.position.y, pose.theta];
    robot.memory.raise_event(key, values.into_value()).await
}

/// Path points stored as floats in groups of three.
fn path_points(raw: &Value<'_>) -> Vec<Vector3> {
    let Some(floats) = as_f32s(raw) else {
        return Vec::new();
    };
    floats
        .chunks_exact(3)
        .map(|point| Vector3::new(point[0], point[1], point[2]))
        .collect()
}

/// The localizer pose `[x, y, theta]`, carried by a `Vec3`.
fn localizer_pose(raw: &Value<'_>) -> Option<Vector3> {
    match as_f32s(raw)?.as_slice() {
        [x, y, theta, ..] => Some(Vector3::new(*x, *y, *theta)),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use crate::qi::value::Raw;
    use qi::object::Object;
    use std::time::Duration;

    async fn settle() {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    fn raised_events(harness: &Harness) -> Vec<Value<'static>> {
        harness.fakes.service("ALMemory").calls_to("raiseEvent")
    }

    fn result_event(result: &str) -> Value<'static> {
        (
            keys::PLANNER_RESULT.to_owned(),
            Raw::new(result.to_owned().into_value()),
            "subscriber".to_owned(),
        )
            .into_value()
    }

    #[test]
    fn path_points_group_the_floats_by_three() {
        let raw = vec![0.0f32, 1.0, 2.0, 3.0, 4.0, 5.0].into_value();
        assert_eq!(
            path_points(&raw),
            vec![Vector3::new(0.0, 1.0, 2.0), Vector3::new(3.0, 4.0, 5.0)]
        );
        assert!(path_points(&"nope".to_owned().into_value()).is_empty());
        // A trailing incomplete group is dropped.
        assert_eq!(path_points(&vec![0.0f32, 1.0].into_value()).len(), 0);
    }

    #[test]
    fn localizer_pose_reads_x_y_theta() {
        let raw = vec![1.0f32, 2.0, 0.5].into_value();
        assert_eq!(localizer_pose(&raw), Some(Vector3::new(1.0, 2.0, 0.5)));
        assert_eq!(localizer_pose(&vec![1.0f32].into_value()), None);
    }

    #[tokio::test]
    async fn goal_commands_raise_the_planner_event() {
        let harness = Harness::default();
        let capability = NavigationGoal::new();
        capability.enable(&harness.ctx).await.expect("enable");

        harness.transport.inject(
            "navigation_goal",
            Message::NavigationGoal(Pose2::new(1.0, 2.0, 0.5)),
        );
        settle().await;

        assert_eq!(
            raised_events(&harness),
            vec![
                (
                    keys::PLANNER_GOAL.to_owned(),
                    vec![1.0f32, 2.0, 0.5].into_value()
                )
                    .into_value()
            ]
        );
    }

    #[tokio::test]
    async fn goal_disable_unsubscribes() {
        let harness = Harness::default();
        let capability = NavigationGoal::new();
        capability.enable(&harness.ctx).await.expect("enable");
        capability.disable(&harness.ctx).await.expect("disable");

        harness.transport.inject(
            "navigation_goal",
            Message::NavigationGoal(Pose2::new(1.0, 2.0, 0.5)),
        );
        settle().await;

        assert!(raised_events(&harness).is_empty());
    }

    #[tokio::test]
    async fn pose_set_commands_raise_the_localizer_event() {
        let harness = Harness::default();
        let capability = PoseSet::new();
        capability.enable(&harness.ctx).await.expect("enable");

        harness
            .transport
            .inject("pose-set", Message::PoseSet(Pose2::new(3.0, 4.0, 1.5)));
        settle().await;

        assert_eq!(
            raised_events(&harness),
            vec![
                (
                    keys::LOCALIZER_SET_POSE.to_owned(),
                    vec![3.0f32, 4.0, 1.5].into_value()
                )
                    .into_value()
            ]
        );
    }

    #[tokio::test]
    async fn path_ticks_publish_the_planner_path() {
        let harness = Harness::default();
        harness.fakes.service("ALMemory").script_for(
            "getData",
            keys::PLANNER_PATH,
            vec![0.0f32, 1.0, 2.0, 3.0, 4.0, 5.0].into_value(),
        );

        NavigationPath.tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("navigation_path");
        let Message::Path(path) = &published[0] else {
            panic!("expected a path");
        };
        assert_eq!(path.frame, "odom");
        assert_eq!(
            path.points,
            vec![Vector3::new(0.0, 1.0, 2.0), Vector3::new(3.0, 4.0, 5.0)]
        );
    }

    #[tokio::test]
    async fn missing_path_key_publishes_an_empty_path() {
        let harness = Harness::default();
        NavigationPath.tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("navigation_path");
        let Message::Path(path) = &published[0] else {
            panic!("expected a path");
        };
        assert!(path.points.is_empty());
    }

    #[tokio::test]
    async fn pose_ticks_publish_the_localizer_pose() {
        let harness = Harness::default();
        harness.fakes.service("ALMemory").script_for(
            "getData",
            keys::LOCALIZER_ROBOT_POSE,
            vec![1.0f32, 2.0, 0.5].into_value(),
        );

        PosePub.tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("pose-pub");
        let Message::Vector3(pose) = &published[0] else {
            panic!("expected a pose");
        };
        assert_eq!(*pose, Vector3::new(1.0, 2.0, 0.5));
    }

    #[tokio::test]
    async fn missing_pose_key_publishes_unknown_pose() {
        let harness = Harness::default();
        PosePub.tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("pose-pub");
        let Message::Vector3(pose) = &published[0] else {
            panic!("expected a pose");
        };
        assert!(pose.x.is_nan() && pose.y.is_nan() && pose.z.is_nan());
    }

    #[tokio::test]
    async fn result_events_reach_the_transport() {
        let harness = Harness::default();
        let capability = NavigationResult;
        capability.enable(&harness.ctx).await.expect("enable");

        harness
            .ctx
            .toolkit
            .meta_call("onResultCallback".into(), result_event("done"))
            .await
            .expect("event");

        let published = harness.transport.published_on("navigation_result");
        assert_eq!(published, vec![Message::Text("done".to_owned())]);

        let subscriptions = harness
            .fakes
            .service("ALMemory")
            .calls_to("subscribeToEvent");
        assert_eq!(
            subscriptions,
            vec![
                (
                    keys::PLANNER_RESULT.to_owned(),
                    keys::callback_service_name(keys::PLANNER_RESULT),
                    "onResultCallback".to_owned()
                )
                    .into_value()
            ]
        );
    }

    #[tokio::test]
    async fn result_disable_unsubscribes_the_event() {
        let harness = Harness::default();
        let capability = NavigationResult;
        capability.enable(&harness.ctx).await.expect("enable");
        capability.disable(&harness.ctx).await.expect("disable");

        harness
            .ctx
            .toolkit
            .meta_call("onResultCallback".into(), result_event("done"))
            .await
            .expect("event");

        assert!(
            harness
                .transport
                .published_on("navigation_result")
                .is_empty()
        );
        assert_eq!(
            harness
                .fakes
                .service("ALMemory")
                .calls_to("unsubscribeToEvent")
                .len(),
            1
        );
    }
}
