//! The QI layer: the only place that speaks NAOqi.
//!
//! Everything here delegates to the `qi` crate. [`services`] holds typed
//! proxies over the AL* modules, [`object`] serves the `robot_toolkit` object
//! and the event callbacks back to the robot, and [`value`] translates between
//! QI dynamic values and plain values.

pub mod events;
pub mod keys;
pub mod object;
pub mod service;
pub mod services;
pub mod value;

pub use service::{ObjectService, Service};
pub use services::Robot;
