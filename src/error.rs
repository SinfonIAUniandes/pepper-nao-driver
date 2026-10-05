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

//! Driver error type.

/// Result alias for driver operations.
pub type Result<T> = std::result::Result<T, Error>;

/// Errors produced by the driver.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// A call to a NAOqi service failed.
    #[error("QI error: {0}")]
    Qi(#[from] qi::Error),

    /// A value coming from QI could not be decoded into a domain type.
    #[error("invalid QI value for {context}: {detail}")]
    QiValue {
        context: &'static str,
        detail: String,
    },

    /// A command or configuration value is out of range or inconsistent.
    #[error("invalid {context}: {detail}")]
    Invalid {
        context: &'static str,
        detail: String,
    },

    /// A required asset (URDF, camera info) is missing or malformed.
    #[error("asset error: {0}")]
    Asset(String),

    /// A POSIX shared memory segment could not be opened or written.
    #[error("shared memory {name}: {detail}")]
    SharedMemory { name: &'static str, detail: String },

    /// The transport adapter rejected a message or registration.
    #[error("transport error: {0}")]
    Transport(String),
}

impl Error {
    pub(crate) fn qi_value(context: &'static str, detail: impl std::fmt::Display) -> Self {
        Self::QiValue {
            context,
            detail: detail.to_string(),
        }
    }

    pub(crate) fn invalid(context: &'static str, detail: impl std::fmt::Display) -> Self {
        Self::Invalid {
            context,
            detail: detail.to_string(),
        }
    }
}
