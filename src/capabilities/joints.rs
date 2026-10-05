//! Joint angle commands with the allow-list of movable joints.

use super::{Capability, CapabilityId, Context, message_handler};
use crate::Result;
use crate::domain::{JointCommand, Message};
use crate::transport::Subscription;
use async_trait::async_trait;
use std::sync::Mutex;

/// The joints the driver is allowed to move (spec 4.17).
const ALLOWED_JOINTS: [&str; 17] = [
    "HeadYaw",
    "HeadPitch",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
    "LHand",
    "HipRoll",
    "HipPitch",
    "KneePitch",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw",
    "RHand",
];

/// Moves joints to requested angles, one asynchronous call per joint.
pub struct SetAngles {
    subscription: Mutex<Option<Subscription>>,
}

impl SetAngles {
    pub fn new() -> Self {
        Self {
            subscription: Mutex::new(None),
        }
    }
}

impl Default for SetAngles {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for SetAngles {
    fn id(&self) -> CapabilityId {
        CapabilityId::SetAngles
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let robot = ctx.robot.clone();
        let subscription = ctx.transport.subscribe(
            self.id().as_str(),
            message_handler(
                |message| match message {
                    Message::SetAngles(command) => Some(command.clone()),
                    _ => None,
                },
                move |command| {
                    let robot = robot.clone();
                    Box::pin(async move { apply(&robot, &command).await })
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

/// Rejects inconsistent arrays and joints outside the allow-list.
pub fn validate(command: &JointCommand) -> Result<()> {
    if command.names.len() != command.angles.len()
        || command.names.len() != command.fraction_max_speed.len()
    {
        return Err(crate::Error::invalid(
            "joint command",
            "names, angles and fraction_max_speed must have equal lengths",
        ));
    }
    if let Some(name) = command
        .names
        .iter()
        .find(|name| !ALLOWED_JOINTS.contains(&name.as_str()))
    {
        return Err(crate::Error::invalid(
            "joint command",
            format!("{name} cannot be moved through set_angles"),
        ));
    }
    Ok(())
}

async fn apply(robot: &crate::qi::Robot, command: &JointCommand) -> Result<()> {
    validate(command)?;
    for ((name, angle), speed) in command
        .names
        .iter()
        .zip(&command.angles)
        .zip(&command.fraction_max_speed)
    {
        robot.motion.set_angle(name, *angle, *speed).await?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use qi::value::IntoValue;
    use std::time::Duration;

    fn command(names: &[&str], angles: &[f32], speeds: &[f32]) -> JointCommand {
        JointCommand {
            names: names.iter().map(|name| (*name).to_owned()).collect(),
            angles: angles.to_vec(),
            fraction_max_speed: speeds.to_vec(),
        }
    }

    #[test]
    fn validation_checks_lengths_and_joint_names() {
        assert!(validate(&command(&["HeadYaw"], &[0.1], &[0.5])).is_ok());
        assert!(validate(&JointCommand::default()).is_ok());
        assert!(validate(&command(&["HeadYaw"], &[0.1, 0.2], &[0.5])).is_err());
        assert!(validate(&command(&["Finger"], &[0.1], &[0.5])).is_err());
    }

    #[tokio::test]
    async fn valid_commands_post_one_call_per_joint() {
        let harness = Harness::default();
        let set_angles = SetAngles::new();
        set_angles.enable(&harness.ctx).await.expect("enable");
        harness.transport.inject(
            "set_angles",
            Message::SetAngles(command(&["HeadYaw", "RHand"], &[0.1, 0.2], &[0.5, 1.0])),
        );
        tokio::time::sleep(Duration::from_millis(10)).await;

        let calls = harness.fakes.service("ALMotion").calls_to("setAngles");
        assert_eq!(
            calls,
            vec![
                ("HeadYaw".to_owned(), 0.1f32, 0.5f32).into_value(),
                ("RHand".to_owned(), 0.2f32, 1.0f32).into_value(),
            ]
        );
    }

    #[tokio::test]
    async fn invalid_commands_are_rejected() {
        let harness = Harness::default();
        let set_angles = SetAngles::new();
        set_angles.enable(&harness.ctx).await.expect("enable");
        harness.transport.inject(
            "set_angles",
            Message::SetAngles(command(&["Finger"], &[0.1], &[0.5])),
        );
        tokio::time::sleep(Duration::from_millis(10)).await;

        assert!(
            harness
                .fakes
                .service("ALMotion")
                .calls_to("setAngles")
                .is_empty()
        );
    }

    #[tokio::test]
    async fn disable_stops_accepting_commands() {
        let harness = Harness::default();
        let set_angles = SetAngles::new();
        set_angles.enable(&harness.ctx).await.expect("enable");
        set_angles.disable(&harness.ctx).await.expect("disable");
        harness.transport.inject(
            "set_angles",
            Message::SetAngles(command(&["HeadYaw"], &[0.1], &[0.5])),
        );
        tokio::time::sleep(Duration::from_millis(10)).await;

        assert!(
            harness
                .fakes
                .service("ALMotion")
                .calls_to("setAngles")
                .is_empty()
        );
    }
}
