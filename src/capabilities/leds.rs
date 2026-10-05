//! LED color commands.

use super::{Capability, CapabilityId, Context, message_handler};
use crate::Result;
use crate::domain::{LedCommand, Message};
use crate::transport::Subscription;
use async_trait::async_trait;
use std::sync::Mutex;

/// Fades LEDs to the requested colors.
pub struct Leds {
    subscription: Mutex<Option<Subscription>>,
}

impl Leds {
    pub fn new() -> Self {
        Self {
            subscription: Mutex::new(None),
        }
    }
}

impl Default for Leds {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for Leds {
    fn id(&self) -> CapabilityId {
        CapabilityId::Leds
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let robot = ctx.robot.clone();
        let subscription = ctx.transport.subscribe(
            self.id().as_str(),
            message_handler(
                |message| match message {
                    Message::Leds(command) => Some(command.clone()),
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

/// Scales the 0-255 channels to 0.0-1.0; the ear LEDs are blue only.
fn fade_values(command: &LedCommand) -> (f32, f32, f32) {
    let scale = |channel: u8| f32::from(channel) / 255.0;
    if command.name.contains("Ear") {
        (0.0, 0.0, scale(command.blue))
    } else {
        (
            scale(command.red),
            scale(command.green),
            scale(command.blue),
        )
    }
}

async fn apply(robot: &crate::qi::Robot, command: &LedCommand) -> Result<()> {
    let (red, green, blue) = fade_values(command);
    robot
        .leds
        .fade_rgb(&command.name, red, green, blue, command.duration)
        .await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use qi::value::IntoValue;
    use std::time::Duration;

    fn command(name: &str, red: u8, green: u8, blue: u8) -> LedCommand {
        LedCommand {
            name: name.to_owned(),
            red,
            green,
            blue,
            duration: 1.0,
        }
    }

    #[test]
    fn channels_are_scaled_to_unit_range() {
        assert_eq!(
            fade_values(&command("FaceLeds", 255, 128, 0)),
            (1.0, 128.0 / 255.0, 0.0)
        );
    }

    #[test]
    fn ear_leds_ignore_red_and_green() {
        assert_eq!(
            fade_values(&command("EarLeds", 255, 255, 255)),
            (0.0, 0.0, 1.0)
        );
        assert_eq!(
            fade_values(&command("LEar", 10, 20, 30)),
            (0.0, 0.0, 30.0 / 255.0)
        );
    }

    #[tokio::test]
    async fn commands_reach_the_led_service() {
        let harness = Harness::default();
        let leds = Leds::new();
        leds.enable(&harness.ctx).await.expect("enable");
        harness
            .transport
            .inject("leds", Message::Leds(command("EarLeds", 255, 0, 255)));
        tokio::time::sleep(Duration::from_millis(10)).await;

        assert_eq!(
            harness.fakes.service("ALLeds").calls_to("fadeRGB"),
            vec![("EarLeds".to_owned(), 0.0f32, 0.0f32, 1.0f32, 1.0f32).into_value()]
        );
    }

    #[tokio::test]
    async fn disable_stops_accepting_commands() {
        let harness = Harness::default();
        let leds = Leds::new();
        leds.enable(&harness.ctx).await.expect("enable");
        leds.disable(&harness.ctx).await.expect("disable");
        harness
            .transport
            .inject("leds", Message::Leds(command("FaceLeds", 1, 2, 3)));
        tokio::time::sleep(Duration::from_millis(10)).await;

        assert!(
            harness
                .fakes
                .service("ALLeds")
                .calls_to("fadeRGB")
                .is_empty()
        );
    }
}
