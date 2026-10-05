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
//! - [`driver`]: lifecycle, capability registry and scheduler.

// Driver code must not panic outside of tests; `unimplemented!` markers for
// missing libqi-rs support are the documented exception.
#![cfg_attr(not(test), warn(clippy::unwrap_used, clippy::panic))]

pub mod domain;
pub mod error;
pub mod transport;
pub mod transport;

pub use error::{Error, Result};
