//! Asset loading: robot description and camera calibration.
//!
//! Assets are plain files under `share/` next to the binary (or under a given
//! base directory, e.g. a QI package prefix). Nothing is looked up on a bus.

use crate::domain::{CameraInfo, Resolution};
use crate::kinematics::RobotModel;
use crate::{Error, Result};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Camera calibration tables of one camera, keyed by resolution.
pub struct CameraInfoFile {
    entries: HashMap<String, CameraInfo>,
}

impl CameraInfoFile {
    pub fn get(&self, resolution: Resolution) -> Option<&CameraInfo> {
        self.entries.get(resolution.camera_info_key())
    }
}

/// Everything the driver needs from disk.
pub struct Assets {
    pub robot_model: RobotModel,
    camera_info: HashMap<crate::domain::CameraId, CameraInfoFile>,
}

impl Assets {
    /// Loads assets from `<base>/share`; `base` defaults to the directory of the
    /// executable, then the working directory.
    pub fn load(base: Option<&Path>) -> Result<Self> {
        let root = find_share(base)?;
        let urdf = std::fs::read_to_string(root.join("urdf/pepper.urdf"))
            .map_err(|err| Error::Asset(format!("cannot read pepper.urdf: {err}")))?;
        let mut camera_info = HashMap::new();
        for (camera, file, frame) in [
            (crate::domain::CameraId::Front, "top_camera_info.json", "CameraTop_optical_frame"),
            (crate::domain::CameraId::Bottom, "bottom_camera_info.json", "CameraBottom_optical_frame"),
            // The depth camera publishes the top camera calibration, matching
            // the frame used by the original toolkit.
            (crate::domain::CameraId::Depth, "depth_camera_info.json", "CameraTop_optical_frame"),
        ] {
            let path = root.join("camera_info").join(file);
            let raw = std::fs::read_to_string(&path)
                .map_err(|err| Error::Asset(format!("cannot read {}: {err}", path.display())))?;
            camera_info.insert(camera, parse_camera_info(&raw, frame)?);
        }
        Ok(Self {
            robot_model: RobotModel::from_urdf(&urdf)?,
            camera_info,
        })
    }

    pub fn camera_info(&self, camera: crate::domain::CameraId) -> Option<&CameraInfoFile> {
        self.camera_info.get(&camera)
    }
}

fn find_share(base: Option<&Path>) -> Result<PathBuf> {
    let mut candidates: Vec<PathBuf> = Vec::new();
    match base {
        // An explicit base (e.g. a QI package prefix) is authoritative.
        Some(base) => candidates.push(base.join("share")),
        None => {
            if let Ok(executable) = std::env::current_exe() {
                if let Some(dir) = executable.parent() {
                    candidates.push(dir.join("share"));
                }
            }
            candidates.push(PathBuf::from("share"));
        }
    }
    for candidate in candidates {
        if candidate.join("urdf/pepper.urdf").is_file() {
            return Ok(candidate);
        }
    }
    Err(Error::Asset(
        "share/ directory with pepper.urdf not found (beside the binary, in the working directory or under the given base)"
            .to_owned(),
    ))
}

/// Parses one camera info JSON file; the `frame` is fixed per camera.
fn parse_camera_info(raw: &str, frame: &str) -> Result<CameraInfoFile> {
    let document: serde_json::Value = serde_json::from_str(raw)
        .map_err(|err| Error::Asset(format!("camera info JSON error: {err}")))?;
    let document = document
        .as_object()
        .ok_or_else(|| Error::Asset("camera info root must be an object".to_owned()))?;
    let mut entries = HashMap::new();
    for (key, entry) in document {
        entries.insert(key.clone(), parse_entry(entry, frame)?);
    }
    Ok(CameraInfoFile { entries })
}

fn parse_entry(entry: &serde_json::Value, frame: &str) -> Result<CameraInfo> {
    let number = |name: &str| -> Result<f32> {
        entry
            .get(name)
            .and_then(serde_json::Value::as_f64)
            .map(|value| value as f32)
            .ok_or_else(|| Error::Asset(format!("camera info entry misses {name}")))
    };
    let numbered = |name: &str, len: usize| -> Result<Vec<f32>> {
        let map = entry
            .get(name)
            .and_then(serde_json::Value::as_object)
            .ok_or_else(|| Error::Asset(format!("camera info entry misses {name}")))?;
        (0..len)
            .map(|index| {
                map.get(&index.to_string())
                    .and_then(serde_json::Value::as_f64)
                    .map(|value| value as f32)
                    .ok_or_else(|| Error::Asset(format!("camera info {name} misses index {index}")))
            })
            .collect()
    };
    Ok(CameraInfo {
        frame: frame.to_owned(),
        width: number("width")? as u32,
        height: number("height")? as u32,
        distortion_model: entry
            .get("distortion_model")
            .and_then(serde_json::Value::as_str)
            .unwrap_or("none")
            .to_owned(),
        d: numbered("D", 5)?,
        k: fixed(&numbered("K", 9)?),
        r: fixed(&numbered("R", 9)?),
        p: fixed(&numbered("P", 12)?),
        binning_x: number("binning_x")? as u32,
        binning_y: number("binning_y")? as u32,
    })
}

fn fixed<const N: usize>(values: &[f32]) -> [f32; N] {
    let mut result = [0.0; N];
    for (slot, value) in result.iter_mut().zip(values) {
        *slot = *value;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::CameraId;

    fn assets() -> Assets {
        Assets::load(None).expect("assets load from the repository layout")
    }

    #[test]
    fn loads_robot_model_and_camera_calibration() {
        let assets = assets();
        assert!(assets.robot_model.joint("HeadYaw").is_some());
        let top = assets.camera_info(CameraId::Front).expect("top info");
        let info = top.get(Resolution::QVGA).expect("kQVGA");
        assert_eq!(info.frame, "CameraTop_optical_frame");
        assert_eq!(info.width, 320);
        assert_eq!(info.height, 240);
        assert_eq!(info.k.len(), 9);
        assert!(top.get(Resolution::SIXTEEN_VGA).is_some());
    }

    #[test]
    fn depth_calibration_uses_the_top_optical_frame() {
        let assets = assets();
        let depth = assets.camera_info(CameraId::Depth).expect("depth info");
        assert_eq!(
            depth.get(Resolution::QVGA).expect("kQVGA").frame,
            "CameraTop_optical_frame"
        );
    }

    #[test]
    fn missing_share_directory_is_an_error() {
        let error = Assets::load(Some(Path::new("/nonexistent")));
        assert!(error.is_err());
    }

    #[test]
    fn malformed_camera_json_is_an_error() {
        assert!(parse_camera_info("{ not json", "frame").is_err());
        assert!(parse_camera_info("{\"kQVGA\": {}}", "frame").is_err());
    }
}
