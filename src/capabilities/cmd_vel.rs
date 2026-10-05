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

//! Velocity commands with a stop watchdog.

use super::{Capability, CapabilityId, Configuration, ConfigurationResult, Context};
use crate::Result;
use crate::domain::{Message, Twist};
use async_trait::async_trait;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::mpsc;

/// Default watchdog timeout in seconds.
pub const DEFAULT_WATCHDOG_SECS: f32 = 0.5;

/// Watchdog tick rate; fast enough to stop the robot near the timeout.
const WATCHDOG_TICK: Duration = Duration::from_millis(50);

struct State {
    security_timer: f32,
}

/// Forwards `cmd_vel` to `ALMotion.move` and stops the robot when commands
/// dry up or the capability is disabled.
pub struct CmdVel {
    state: Arc<Mutex<State>>,
    task: Mutex<Option<tokio::task::JoinHandle<()>>>,
}

impl CmdVel {
    pub fn new() -> Self {
        Self {
            state: Arc::new(Mutex::new(State {
                security_timer: DEFAULT_WATCHDOG_SECS,
            })),
            task: Mutex::new(None),
        }
    }
}

impl Default for CmdVel {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for CmdVel {
    fn id(&self) -> CapabilityId {
        CapabilityId::CmdVel
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let (sender, mut commands) = mpsc::channel::<Twist>(16);
        let subscription = ctx.transport.subscribe(
            self.id().as_str(),
            Arc::new(move |message| {
                if let Message::CmdVel(twist) = message {
                    // A full queue means the driver is behind: drop the oldest.
                    if sender.try_send(twist).is_err() {
                        tracing::warn!("cmd_vel queue overflow");
                    }
                }
            }),
        );
        let robot = Arc::clone(&ctx.robot);
        let state = Arc::clone(&self.state);
        let task = tokio::spawn(async move {
            let _subscription = subscription;
            let mut last_command: Option<tokio::time::Instant> = None;
            let mut moving = false;
            let mut watchdog = tokio::time::interval(WATCHDOG_TICK);
            loop {
                tokio::select! {
                    command = commands.recv() => {
                        let Some(twist) = command else { return; };
                        if let Err(err) = robot.motion.move_(twist.vx, twist.vy, twist.wz).await {
                            tracing::warn!(error = %err, "ALMotion.move failed");
                        }
                        moving = twist != Twist::default();
                        last_command = Some(tokio::time::Instant::now());
                    }
                    _ = watchdog.tick() => {
                        let security_timer = state.lock().unwrap_or_else(|err| err.into_inner()).security_timer;
                        let expired = security_timer > 0.0
                            && moving
                            && last_command
                                .is_some_and(|last| {
                                    last.elapsed() > Duration::from_secs_f64(f64::from(security_timer))
                                });
                        if expired {
                            if let Err(err) = robot.motion.move_(0.0, 0.0, 0.0).await {
                                tracing::warn!(error = %err, "ALMotion.move failed");
                            }
                            moving = false;
                        }
                    }
                }
            }
        });
        *self.task.lock().unwrap_or_else(|err| err.into_inner()) = Some(task);
        Ok(())
    }

    async fn disable(&self, ctx: &Context) -> Result<()> {
        if let Some(task) = self
            .task
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .take()
        {
            task.abort();
        }
        // Disabling must leave the robot stopped.
        ctx.robot.motion.move_(0.0, 0.0, 0.0).await
    }

    async fn configure(
        &self,
        _ctx: &Context,
        configuration: Configuration,
    ) -> Result<ConfigurationResult> {
        if let Configuration::CmdVel(security_timer) = configuration {
            self.state
                .lock()
                .unwrap_or_else(|err| err.into_inner())
                .security_timer = security_timer;
        }
        Ok(ConfigurationResult::None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use qi::value::IntoValue;

    fn moves(harness: &Harness) -> Vec<Value<'static>> {
        harness
            .fakes
            .service("ALMotion")
            .calls_to("move")
            .into_iter()
            .collect()
    }

    use qi::value::Value;

    async fn wait_for_moves(harness: &Harness, count: usize) {
        for _ in 0..200 {
            if moves(harness).len() >= count {
                return;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        panic!("expected {count} move calls");
    }

    #[tokio::test]
    async fn commands_reach_al_motion() {
        let harness = Harness::default();
        let cmd_vel = CmdVel::new();
        cmd_vel.enable(&harness.ctx).await.expect("enable");
        harness.transport.inject(
            "cmd_vel",
            Message::CmdVel(Twist {
                vx: 0.5,
                vy: 0.0,
                wz: 0.1,
            }),
        );
        wait_for_moves(&harness, 1).await;
        assert_eq!(moves(&harness)[0], (0.5f32, 0.0f32, 0.1f32).into_value());
    }

    #[tokio::test(start_paused = true)]
    async fn watchdog_stops_the_robot_after_the_timeout() {
        let harness = Harness::default();
        let cmd_vel = CmdVel::new();
        cmd_vel.enable(&harness.ctx).await.expect("enable");
        harness.transport.inject(
            "cmd_vel",
            Message::CmdVel(Twist {
                vx: 0.5,
                vy: 0.0,
                wz: 0.0,
            }),
        );
        wait_for_moves(&harness, 1).await;

        tokio::time::sleep(Duration::from_secs(1)).await;
        wait_for_moves(&harness, 2).await;
        assert_eq!(moves(&harness)[1], (0.0f32, 0.0f32, 0.0f32).into_value());
    }

    #[tokio::test(start_paused = true)]
    async fn non_positive_timeout_disables_the_watchdog() {
        let harness = Harness::default();
        let cmd_vel = CmdVel::new();
        cmd_vel
            .configure(&harness.ctx, Configuration::CmdVel(-1.0))
            .await
            .expect("configure");
        cmd_vel.enable(&harness.ctx).await.expect("enable");
        harness.transport.inject(
            "cmd_vel",
            Message::CmdVel(Twist {
                vx: 0.5,
                vy: 0.0,
                wz: 0.0,
            }),
        );
        wait_for_moves(&harness, 1).await;

        tokio::time::sleep(Duration::from_secs(5)).await;
        assert_eq!(moves(&harness).len(), 1);
    }

    #[tokio::test]
    async fn disable_stops_the_robot() {
        let harness = Harness::default();
        let cmd_vel = CmdVel::new();
        cmd_vel.enable(&harness.ctx).await.expect("enable");
        cmd_vel.disable(&harness.ctx).await.expect("disable");
        assert_eq!(moves(&harness)[0], (0.0f32, 0.0f32, 0.0f32).into_value());
    }
}
