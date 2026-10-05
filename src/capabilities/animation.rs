//! Animations played through ALBehaviorManager.

use super::{Capability, CapabilityId, Context, message_handler};
use crate::Result;
use crate::domain::{AnimationCommand, AnimationFamily, Message};
use crate::transport::Subscription;
use async_trait::async_trait;
use std::path::Path;
use std::sync::Mutex;

/// Where the robot keeps its installed behavior packages.
pub const DEFAULT_APP_ROOT: &str = "/home/nao/.local/share/PackageManager/apps";

/// Plays installed animations requested on the bus.
pub struct Animation {
    subscription: Mutex<Option<Subscription>>,
}

impl Animation {
    pub fn new() -> Self {
        Self {
            subscription: Mutex::new(None),
        }
    }
}

impl Default for Animation {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for Animation {
    fn id(&self) -> CapabilityId {
        CapabilityId::Animation
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let robot = ctx.robot.clone();
        let subscription = ctx.transport.subscribe(
            self.id().as_str(),
            message_handler(
                |message| match message {
                    Message::Animation(command) => Some(command.clone()),
                    _ => None,
                },
                move |command| {
                    let robot = robot.clone();
                    Box::pin(
                        async move { play(&robot, Path::new(DEFAULT_APP_ROOT), &command).await },
                    )
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

/// Behavior path of `command` inside its animation family.
pub fn behavior_path(command: &AnimationCommand) -> String {
    match command.family {
        AnimationFamily::Animations => format!("animations/Stand/{}", command.name),
        AnimationFamily::AnimationsSinfonia => {
            format!("animations_sinfonia/animations/{}", command.name)
        }
    }
}

/// Whether the behavior package is installed under `root`.
pub fn app_exists(root: &Path, path: &str) -> bool {
    root.join(path).exists()
}

async fn play(robot: &crate::qi::Robot, root: &Path, command: &AnimationCommand) -> Result<()> {
    let path = behavior_path(command);
    if !app_exists(root, &path) {
        tracing::warn!(path, "animation is not installed on the robot");
        return Ok(());
    }
    robot.behavior_manager.start_behavior(&path).await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use qi::value::IntoValue;
    use std::time::Duration;

    fn command(family: AnimationFamily, name: &str) -> AnimationCommand {
        AnimationCommand {
            family,
            name: name.to_owned(),
        }
    }

    fn temp_root() -> std::path::PathBuf {
        let root = std::env::temp_dir().join(format!("al-driver-anim-{}", std::process::id()));
        std::fs::create_dir_all(root.join("animations/Stand")).expect("temp dir");
        std::fs::create_dir_all(root.join("animations_sinfonia/animations")).expect("temp dir");
        root
    }

    #[test]
    fn behavior_paths_follow_the_family_layout() {
        assert_eq!(
            behavior_path(&command(AnimationFamily::Animations, "Bow")),
            "animations/Stand/Bow"
        );
        assert_eq!(
            behavior_path(&command(AnimationFamily::AnimationsSinfonia, "Wave")),
            "animations_sinfonia/animations/Wave"
        );
    }

    #[test]
    fn app_exists_checks_the_package_root() {
        let root = temp_root();
        std::fs::create_dir_all(root.join("animations/Stand/Bow")).expect("behavior");
        assert!(app_exists(&root, "animations/Stand/Bow"));
        assert!(!app_exists(&root, "animations/Stand/Missing"));
    }

    #[tokio::test]
    async fn installed_animations_are_started() {
        let harness = Harness::default();
        let root = temp_root();
        std::fs::create_dir_all(root.join("animations/Stand/Bow")).expect("behavior");
        play(
            &harness.ctx.robot,
            &root,
            &command(AnimationFamily::Animations, "Bow"),
        )
        .await
        .expect("play");
        assert_eq!(
            harness
                .fakes
                .service("ALBehaviorManager")
                .calls_to("startBehavior"),
            vec!["animations/Stand/Bow".to_owned().into_value()]
        );
    }

    #[tokio::test]
    async fn missing_animations_are_skipped() {
        let harness = Harness::default();
        play(
            &harness.ctx.robot,
            &temp_root(),
            &command(AnimationFamily::AnimationsSinfonia, "Missing"),
        )
        .await
        .expect("play");
        assert!(
            harness
                .fakes
                .service("ALBehaviorManager")
                .calls_to("startBehavior")
                .is_empty()
        );

        let animation = Animation::new();
        animation.enable(&harness.ctx).await.expect("enable");
        harness.transport.inject(
            "animation",
            Message::Animation(command(AnimationFamily::Animations, "NoSuchAnimation")),
        );
        tokio::time::sleep(Duration::from_millis(10)).await;
        assert!(
            harness
                .fakes
                .service("ALBehaviorManager")
                .calls_to("startBehavior")
                .is_empty()
        );
    }

    #[tokio::test]
    async fn disable_stops_accepting_commands() {
        let harness = Harness::default();
        let animation = Animation::new();
        animation.enable(&harness.ctx).await.expect("enable");
        animation.disable(&harness.ctx).await.expect("disable");
        harness.transport.inject(
            "animation",
            Message::Animation(command(AnimationFamily::Animations, "Bow")),
        );
        tokio::time::sleep(Duration::from_millis(10)).await;
        assert!(
            harness
                .fakes
                .service("ALBehaviorManager")
                .calls_to("startBehavior")
                .is_empty()
        );
    }
}
