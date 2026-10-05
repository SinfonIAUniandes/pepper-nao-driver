//! Free-zone walk: find reachable free space and walk to it.

use super::{Capability, CapabilityId, Context, message_handler};
use crate::Result;
use crate::domain::{FreeZoneRequest, Message, Pose2};
use crate::qi::Robot;
use crate::qi::value::{as_f32s, plain};
use crate::transport::{Subscription, Transport};
use async_trait::async_trait;
use qi::value::Value;
use std::sync::Mutex;

/// Topic carrying the walk result.
const RESULT_TOPIC: &str = "free_zone_result";

/// Asks the robot for a reachable free circle and walks to its centre.
pub struct FreeZone {
    subscription: Mutex<Option<Subscription>>,
}

impl FreeZone {
    pub fn new() -> Self {
        Self {
            subscription: Mutex::new(None),
        }
    }
}

impl Default for FreeZone {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for FreeZone {
    fn id(&self) -> CapabilityId {
        CapabilityId::FreeZone
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let robot = ctx.robot.clone();
        let transport = ctx.transport.clone();
        let subscription = ctx.transport.subscribe(
            self.id().as_str(),
            message_handler(
                |message| match message {
                    Message::FreeZone(request) => Some(*request),
                    _ => None,
                },
                move |request| {
                    let robot = robot.clone();
                    let transport = transport.clone();
                    Box::pin(async move { walk(&robot, transport.as_ref(), request).await })
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

/// Walks to the free-zone centre expressed in the robot frame and publishes
/// the reached position.
async fn walk(robot: &Robot, transport: &dyn Transport, request: FreeZoneRequest) -> Result<()> {
    let raw = robot
        .navigation
        .get_free_zone(request.desired_radius, request.displacement)
        .await?;
    let centre = world_centre(&raw)?;
    let robot_world = robot_pose(&robot.motion.get_robot_position(true).await?)?;
    let target = robot_world.inverse().compose(centre);

    robot
        .motion
        .move_to(target.position.x, target.position.y, 0.0)
        .await?;
    robot.motion.wait_until_move_is_finished().await?;

    transport.publish(
        RESULT_TOPIC,
        Message::Pose2(Pose2 {
            position: target.position,
            theta: 0.0,
        }),
    )
}

/// The free-zone centre, element `[2]` of the `getFreeZone` result.
fn world_centre(result: &Value<'_>) -> Result<Pose2> {
    let centre = match plain(result.clone()) {
        Value::List(elements) | Value::Tuple(elements) => elements.into_iter().nth(2),
        _ => None,
    };
    let xyz = centre.as_ref().and_then(|value| as_f32s(value));
    match xyz.as_deref() {
        Some([x, y, ..]) => Ok(Pose2::new(*x, *y, 0.0)),
        _ => Err(crate::Error::qi_value(
            "ALNavigation.getFreeZone",
            "missing world centre",
        )),
    }
}

/// The robot pose `[x, y, theta]`.
fn robot_pose(values: &[f32]) -> Result<Pose2> {
    match values {
        [x, y, theta, ..] => Ok(Pose2::new(*x, *y, *theta)),
        _ => Err(crate::Error::qi_value(
            "ALMotion.getRobotPosition",
            "fewer than 3 values",
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use crate::qi::value::as_f32;
    use qi::value::IntoValue;
    use std::time::Duration;

    async fn settle() {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    /// `getFreeZone` result whose element `[2]` is the world centre.
    fn free_zone_result(x: f32, y: f32) -> Value<'static> {
        vec![
            0.0f32.into_value(),
            0.0f32.into_value(),
            vec![x, y, 0.0f32].into_value(),
        ]
        .into_value()
    }

    fn script_walk(harness: &Harness, centre: (f32, f32), robot: (f32, f32, f32)) {
        let navigation = harness.fakes.service("ALNavigation");
        navigation.script("getFreeZone", free_zone_result(centre.0, centre.1));
        harness.fakes.service("ALMotion").script(
            "getRobotPosition",
            vec![robot.0, robot.1, robot.2].into_value(),
        );
    }

    fn move_calls(harness: &Harness) -> Vec<Value<'static>> {
        harness.fakes.service("ALMotion").calls_to("moveTo")
    }

    fn assert_close(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < 1e-5, "{actual} != {expected}");
    }

    #[test]
    fn world_centre_reads_the_third_element() {
        assert_eq!(
            world_centre(&free_zone_result(3.0, 1.0)).expect("centre"),
            Pose2::new(3.0, 1.0, 0.0)
        );
        assert!(world_centre(&0.0f32.into_value()).is_err());
        assert!(world_centre(&vec![0.0f32].into_value()).is_err());
    }

    #[test]
    fn robot_pose_reads_x_y_theta() {
        assert_eq!(
            robot_pose(&[1.0, 2.0, 0.5]).expect("pose"),
            Pose2::new(1.0, 2.0, 0.5)
        );
        assert!(robot_pose(&[1.0]).is_err());
    }

    #[tokio::test]
    async fn walk_targets_the_centre_in_the_robot_frame() {
        let harness = Harness::default();
        script_walk(&harness, (3.0, 1.0), (1.0, 0.0, 0.0));
        let capability = FreeZone::new();
        capability.enable(&harness.ctx).await.expect("enable");

        harness.transport.inject(
            "free_zone",
            Message::FreeZone(FreeZoneRequest {
                desired_radius: 0.5,
                displacement: 1.0,
            }),
        );
        settle().await;

        assert_eq!(
            move_calls(&harness),
            vec![(2.0f32, 1.0f32, 0.0f32).into_value()]
        );
        let waits = harness
            .fakes
            .service("ALMotion")
            .calls_to("waitUntilMoveIsFinished");
        assert_eq!(waits.len(), 1);
        let published = harness.transport.published_on("free_zone_result");
        assert_eq!(published.len(), 1);
        let Message::Pose2(result) = &published[0] else {
            panic!("expected a pose");
        };
        assert_eq!(*result, Pose2::new(2.0, 1.0, 0.0));
    }

    #[tokio::test]
    async fn rotated_robot_targets_the_centre_relative_to_itself() {
        let harness = Harness::default();
        // Robot at (1, 0) turned 90°: the centre (2, 0) lies one metre to its
        // left.
        script_walk(
            &harness,
            (2.0, 0.0),
            (1.0, 0.0, std::f32::consts::FRAC_PI_2),
        );
        let capability = FreeZone::new();
        capability.enable(&harness.ctx).await.expect("enable");

        harness.transport.inject(
            "free_zone",
            Message::FreeZone(FreeZoneRequest {
                desired_radius: 0.5,
                displacement: 1.0,
            }),
        );
        settle().await;

        let calls = move_calls(&harness);
        let Value::Tuple(values) = &calls[0] else {
            panic!("expected a tuple");
        };
        assert_close(as_f32(&values[0]).expect("x"), 0.0);
        assert_close(as_f32(&values[1]).expect("y"), -1.0);
        assert_close(as_f32(&values[2]).expect("theta"), 0.0);

        let published = harness.transport.published_on("free_zone_result");
        let Message::Pose2(result) = &published[0] else {
            panic!("expected a pose");
        };
        assert_close(result.position.y, -1.0);
        assert_eq!(result.theta, 0.0);
    }

    #[tokio::test]
    async fn disable_unsubscribes() {
        let harness = Harness::default();
        let capability = FreeZone::new();
        capability.enable(&harness.ctx).await.expect("enable");
        capability.disable(&harness.ctx).await.expect("disable");

        harness.transport.inject(
            "free_zone",
            Message::FreeZone(FreeZoneRequest {
                desired_radius: 0.5,
                displacement: 1.0,
            }),
        );
        settle().await;

        assert!(
            harness
                .transport
                .published_on("free_zone_result")
                .is_empty()
        );
    }
}
