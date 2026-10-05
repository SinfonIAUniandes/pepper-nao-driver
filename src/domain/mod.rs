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

//! Domain types exchanged with the transport adapter.
//!
//! Nothing in this module knows about QI, NAOqi or any particular bus.

mod command;
mod control;
mod geometry;
mod message;
mod params;
mod sensor;
mod time;

pub use command::*;
pub use control::*;
pub use geometry::*;
pub use message::*;
pub use params::*;
pub use sensor::*;
pub use time::*;
