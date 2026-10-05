//! Pepper robot driver.
//!
//! The driver binds Pepper's QI surface (AL* services, ALMemory keys and events)
//! to domain capabilities that flow over a pluggable [`transport::Transport`].
//! All QI functionality comes from the [`qi`] crate; this crate never touches the
//! QI wire format itself.

// Driver code must not panic outside of tests; `unimplemented!` markers for
// missing libqi-rs support are the documented exception.
#![cfg_attr(not(test), warn(clippy::unwrap_used, clippy::panic))]

pub mod assets;
pub mod assets;
pub mod capabilities;
pub mod domain;
pub mod error;
pub mod kinematics;
pub mod qi;
pub mod scheduler;
pub mod shm;
pub mod transport;

pub use error::{Error, Result};
