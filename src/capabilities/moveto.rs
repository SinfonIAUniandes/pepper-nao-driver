//! Relative walk commands.

use super::{Capability, CapabilityId, Context, message_handler};
use crate::Result;
use crate::domain::{Message, Pose2};
use crate::transport::Subscription;
use async_trait::async_trait;
use std::sync::Mutex;

/// Walks to goals expressed in `base_footprint`, the frame `ALMotion.moveTo`
/// works in; the frameless `Pose2` command needs no TF lookup.
pub struct MoveTo {
    subscription: Mutex<Option<Subscription>>,
}

impl MoveTo {
    pub fn new() -> Self {
        Self {
            subscription: Mutex::new(None),
        }
    }
}

impl Default for MoveTo {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for MoveTo {
    fn id(&self) -> CapabilityId {
        CapabilityId::MoveTo
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let robot = ctx.robot.clone();
        let subscription = ctx.transport.subscribe(
            self.id().as_str(),
            message_handler(
                |message| match message {
                    Message::MoveTo(goal) => Some(*goal),
                    _ => None,
                },
                move |goal| {
                    let robot = robot.clone();
                    Box::pin(async move { walk(&robot, goal).await })
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

async fn walk(robot: &crate::qi::Robot, goal: Pose2) -> Result<()> {
    robot
        .motion
        .move_to(goal.position.x, goal.position.y, goal.theta)
        .await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use qi::value::IntoValue;
    use std::time::Duration;

    async fn settle() {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    #[tokio::test]
    async fn commands_reach_move_to() {
        let harness = Harness::default();
        let capability = MoveTo::new();
        capability.enable(&harness.ctx).await.expect("enable");

        harness
            .transport
            .inject("moveto", Message::MoveTo(Pose2::new(1.0, -2.0, 0.5)));
        settle().await;

        let calls = harness.fakes.service("ALMotion").calls_to("moveTo");
        assert_eq!(calls, vec![(1.0f32, -2.0f32, 0.5f32).into_value()]);
    }

    #[tokio::test]
    async fn disable_unsubscribes() {
        let harness = Harness::default();
        let capability = MoveTo::new();
        capability.enable(&harness.ctx).await.expect("enable");
        capability.disable(&harness.ctx).await.expect("disable");

        harness
            .transport
            .inject("moveto", Message::MoveTo(Pose2::new(1.0, -2.0, 0.5)));
        settle().await;

        assert!(
            harness
                .fakes
                .service("ALMotion")
                .calls_to("moveTo")
                .is_empty()
        );
    }
}
