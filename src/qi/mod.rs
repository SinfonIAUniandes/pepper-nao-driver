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
