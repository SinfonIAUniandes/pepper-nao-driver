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

//! Pepper robot driver.
//!
//! The driver binds Pepper's QI surface (AL* services, ALMemory keys and events)
//! to domain capabilities that flow over a pluggable [`transport::Transport`].
//! All QI functionality comes from the [`qi`] crate; this crate never touches the
//! QI wire format itself.
//!
//! Layout:
//! - [`domain`]: robot-agnostic data types exchanged with the transport adapter.
//! - [`transport`]: the adapter seam (publish / subscribe / RPC / timers / clock).
//! - [`qi`]: thin typed proxies over the NAOqi services, plus the objects the
//!   driver serves back to the robot.
//! - [`capabilities`]: one module per capability (`tf`, `odom`, `laser`, ...).
//! - [`control`]: the control RPCs (`navigation_tools`, `vision_tools`, ...).
//! - [`driver`]: lifecycle, capability registry and scheduler.

// Driver code must not panic outside of tests; `unimplemented!` markers for
// missing libqi-rs support are the documented exception.
#![cfg_attr(not(test), warn(clippy::unwrap_used, clippy::panic))]

pub mod assets;
pub mod capabilities;
pub mod control;
pub mod domain;
pub mod driver;
pub mod error;
pub mod kinematics;
pub mod qi;
pub mod scheduler;
pub mod shm;
pub mod transport;

pub use error::{Error, Result};
