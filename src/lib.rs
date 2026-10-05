//! Pepper robot driver.
//!
//! The driver binds Pepper's QI surface (AL* services, ALMemory keys and events)
//! to domain capabilities that flow over a pluggable transport. All QI
//! functionality comes from the `qi` crate.

// Driver code must not panic outside of tests; `unimplemented!` markers for
// missing libqi-rs support are the documented exception.
#![cfg_attr(not(test), warn(clippy::unwrap_used, clippy::panic))]

pub mod domain;
pub mod error;

pub use error::{Error, Result};
