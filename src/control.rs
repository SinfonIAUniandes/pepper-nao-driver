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

//! Control RPCs: the tool commands and the word-spotting RPC.
//!
//! Commands always succeed on the channel; failures are reported in the result
//! string of the response.

use crate::Result;
use crate::capabilities::{CapabilityId, Configuration, ConfigurationResult};
use crate::domain::{
    AudioCommand, ControlParams, ControlRequest, ControlResponse, DepthToLaserSetting,
    FrequencySetting, MiscCommand, MotionCommand, NavigationCommand, NavigationCustom, Switch,
    VisionCommand, VisionTools,
};
use crate::driver::Driver;
use crate::shm::Segment;

/// The control RPC names, in tool order.
pub const RPC_NAMES: [&str; 6] = [
    "navigation_tools",
    "vision_tools",
    "audio_tools",
    "motion_tools",
    "misc_tools",
    "speech_recognition",
];

/// Runs one control command; errors land in the result string.
pub async fn dispatch(driver: &Driver, request: ControlRequest) -> ControlResponse {
    let outcome = match request {
        ControlRequest::Navigation(command) => navigation(driver, command).await,
        ControlRequest::Vision(command) => vision(driver, command).await,
        ControlRequest::Audio(command) => audio(driver, command).await,
        ControlRequest::Motion(command) => motion(driver, command).await,
        ControlRequest::Misc(command) => misc(driver, command).await,
        ControlRequest::SpeechRecognition(request) => {
            return ControlResponse {
                result: driver.recognize_speech(request).await,
                params: None,
            };
        }
    };
    match outcome {
        Ok(params) => ControlResponse {
            result: "ok".to_owned(),
            params,
        },
        Err(err) => ControlResponse::error(err),
    }
}

type Params = Option<ControlParams>;

async fn navigation(driver: &Driver, command: NavigationCommand) -> Result<Params> {
    /// Capabilities of the mapper group and their default rates.
    const MAPPER: [(CapabilityId, f32); 6] = [
        (CapabilityId::Tf, 50.0),
        (CapabilityId::Laser, 10.0),
        (CapabilityId::DepthToLaser, 10.0),
        (CapabilityId::Odom, 10.0),
        (CapabilityId::MergedLaser, 10.0),
        (CapabilityId::CmdVel, 0.0),
    ];
    /// Capabilities of the navigate group and their default rates.
    const NAVIGATE: [(CapabilityId, f32); 6] = [
        (CapabilityId::NavigationGoal, 0.0),
        (CapabilityId::PoseSet, 0.0),
        (CapabilityId::FreeZone, 0.0),
        (CapabilityId::NavigationResult, 0.0),
        (CapabilityId::NavigationPath, 10.0),
        (CapabilityId::PosePub, 10.0),
    ];
    /// Capabilities of `enable_all`, excluding the mapper-only laser merger.
    const ALL: [(CapabilityId, f32); 12] = [
        (CapabilityId::Tf, 50.0),
        (CapabilityId::Odom, 10.0),
        (CapabilityId::Laser, 10.0),
        (CapabilityId::DepthToLaser, 10.0),
        (CapabilityId::NavigationPath, 10.0),
        (CapabilityId::PosePub, 10.0),
        (CapabilityId::NavigationResult, 0.0),
        (CapabilityId::NavigationGoal, 0.0),
        (CapabilityId::PoseSet, 0.0),
        (CapabilityId::FreeZone, 0.0),
        (CapabilityId::CmdVel, 0.0),
        (CapabilityId::MoveTo, 0.0),
    ];

    match command {
        NavigationCommand::EnableMapper => {
            set_group(driver, &MAPPER, true).await?;
            driver
                .context()
                .shm
                .set_enabled(Segment::PepperHead, true)?;
            driver
                .context()
                .shm
                .set_enabled(Segment::Depth2Laser, true)?;
        }
        NavigationCommand::DisableMapper => {
            set_group(driver, &MAPPER, false).await?;
            driver
                .context()
                .shm
                .set_enabled(Segment::PepperHead, false)?;
            driver
                .context()
                .shm
                .set_enabled(Segment::Depth2Laser, false)?;
        }
        NavigationCommand::EnableNavigate => {
            for segment in [
                Segment::PepperHead,
                Segment::Depth2Laser,
                Segment::Localizer,
                Segment::Planner,
            ] {
                driver.context().shm.set_enabled(segment, true)?;
            }
            set_group(driver, &NAVIGATE, true).await?;
        }
        NavigationCommand::DisableNavigate => {
            set_group(driver, &NAVIGATE, false).await?;
            for segment in [
                Segment::PepperHead,
                Segment::Depth2Laser,
                Segment::Localizer,
                Segment::Planner,
            ] {
                driver.context().shm.set_enabled(segment, false)?;
            }
        }
        NavigationCommand::EnableAll => {
            set_group(driver, &ALL, true).await?;
            driver
                .context()
                .shm
                .set_enabled(Segment::Depth2Laser, true)?;
        }
        NavigationCommand::DisableAll => {
            set_group(driver, &ALL, false).await?;
            driver
                .context()
                .shm
                .set_enabled(Segment::Depth2Laser, false)?;
        }
        NavigationCommand::Custom(custom) => custom_navigation(driver, custom).await?,
    }
    Ok(None)
}

/// Applies a `custom` command field by field.
async fn custom_navigation(driver: &Driver, custom: NavigationCustom) -> Result<()> {
    frequency_setting(driver, CapabilityId::Tf, &custom.tf).await?;
    frequency_setting(driver, CapabilityId::Odom, &custom.odom).await?;
    frequency_setting(driver, CapabilityId::Laser, &custom.laser).await?;
    frequency_setting(driver, CapabilityId::NavigationPath, &custom.path).await?;
    frequency_setting(driver, CapabilityId::PosePub, &custom.pose_pub).await?;

    toggle(driver, CapabilityId::CmdVel, custom.cmd_vel.enable).await?;
    if custom.cmd_vel.enable {
        driver
            .configure(
                CapabilityId::CmdVel,
                Configuration::CmdVel(custom.cmd_vel.security_timer),
            )
            .await?;
    }
    toggle(driver, CapabilityId::MoveTo, custom.moveto).await?;
    toggle(driver, CapabilityId::NavigationGoal, custom.goal).await?;
    toggle(driver, CapabilityId::PoseSet, custom.pose_set).await?;
    toggle(driver, CapabilityId::NavigationResult, custom.result).await?;
    toggle(driver, CapabilityId::FreeZone, custom.free_zone).await?;

    let DepthToLaserSetting { enable, params } = custom.depth_to_laser;
    toggle(driver, CapabilityId::DepthToLaser, enable).await?;
    if enable {
        driver
            .configure(
                CapabilityId::DepthToLaser,
                Configuration::DepthToLaser(params),
            )
            .await?;
    }
    Ok(())
}

async fn frequency_setting(
    driver: &Driver,
    id: CapabilityId,
    setting: &FrequencySetting,
) -> Result<()> {
    toggle(driver, id, setting.enable).await?;
    if setting.enable {
        driver.set_frequency(id, setting.frequency);
    }
    Ok(())
}

async fn toggle(driver: &Driver, id: CapabilityId, enable: bool) -> Result<()> {
    if enable {
        driver.enable(id).await
    } else {
        driver.disable(id).await
    }
}

async fn set_group(driver: &Driver, group: &[(CapabilityId, f32)], enable: bool) -> Result<()> {
    for (id, hz) in group {
        if *hz > 0.0 {
            driver.set_frequency(*id, *hz);
        }
        toggle(driver, *id, enable).await?;
    }
    Ok(())
}

async fn vision(driver: &Driver, command: VisionTools) -> Result<Params> {
    let VisionTools { camera, command } = command;
    let id = match camera {
        crate::domain::CameraTarget::FrontCamera => CapabilityId::FrontCamera,
        crate::domain::CameraTarget::BottomCamera => CapabilityId::BottomCamera,
        crate::domain::CameraTarget::DepthCamera => CapabilityId::DepthCamera,
        crate::domain::CameraTarget::FrontCameraFaceDetector => {
            CapabilityId::FrontCameraFaceDetector
        }
        crate::domain::CameraTarget::BottomCameraFaceDetector => {
            CapabilityId::BottomCameraFaceDetector
        }
    };
    if camera.is_face_detector() {
        return match command {
            VisionCommand::Enable => {
                driver.enable(id).await?;
                Ok(None)
            }
            VisionCommand::Disable => {
                driver.disable(id).await?;
                Ok(None)
            }
            _ => Err(crate::Error::invalid(
                "vision command",
                "face detectors only take enable and disable",
            )),
        };
    }
    match command {
        VisionCommand::Enable => {
            driver.enable(id).await?;
            Ok(None)
        }
        VisionCommand::Disable => {
            driver.disable(id).await?;
            Ok(None)
        }
        VisionCommand::Custom(config, params) => {
            driver
                .configure(id, Configuration::Camera(config, params))
                .await?;
            driver.set_frequency(id, f32::from(config.fps));
            driver.enable(id).await?;
            Ok(None)
        }
        VisionCommand::SetParameters(params) => {
            driver
                .configure(id, Configuration::CameraParams(params))
                .await?;
            Ok(None)
        }
        VisionCommand::GetParameters(request) => {
            let result = driver
                .configure(id, Configuration::ReadCameraParams(request))
                .await?;
            Ok(match result {
                ConfigurationResult::CameraParams(params) => Some(ControlParams::Camera(params)),
                _ => None,
            })
        }
    }
}

async fn audio(driver: &Driver, command: AudioCommand) -> Result<Params> {
    use AudioCommand::*;
    match command {
        Enable => {
            for id in [
                CapabilityId::Mic,
                CapabilityId::Speech,
                CapabilityId::MicLocalization,
            ] {
                driver.enable(id).await?;
            }
            Ok(None)
        }
        Disable => {
            for id in [
                CapabilityId::Mic,
                CapabilityId::Speech,
                CapabilityId::MicLocalization,
            ] {
                driver.disable(id).await?;
            }
            Ok(None)
        }
        EnableMic => {
            driver.enable(CapabilityId::Mic).await?;
            Ok(None)
        }
        DisableMic => {
            driver.disable(CapabilityId::Mic).await?;
            Ok(None)
        }
        Custom(config) => {
            driver
                .configure(CapabilityId::Mic, Configuration::Mic(config))
                .await?;
            driver.enable(CapabilityId::Mic).await?;
            Ok(None)
        }
        EnableTts => {
            driver.enable(CapabilityId::Speech).await?;
            Ok(None)
        }
        DisableTts => {
            driver.disable(CapabilityId::Speech).await?;
            Ok(None)
        }
        EnableLocalization => {
            driver.enable(CapabilityId::MicLocalization).await?;
            Ok(None)
        }
        DisableLocalization => {
            driver.disable(CapabilityId::MicLocalization).await?;
            Ok(None)
        }
        GetSpeechParams => {
            let result = driver
                .configure(CapabilityId::Speech, Configuration::ReadSpeechParams)
                .await?;
            Ok(match result {
                ConfigurationResult::SpeechParams(params) => Some(ControlParams::Speech(params)),
                _ => None,
            })
        }
        SetSpeechParams(params) => {
            driver
                .configure(CapabilityId::Speech, Configuration::Speech(params))
                .await?;
            Ok(None)
        }
        ResetSpeechParams => {
            driver
                .configure(CapabilityId::Speech, Configuration::ResetSpeechParams)
                .await?;
            Ok(None)
        }
    }
}

async fn motion(driver: &Driver, command: MotionCommand) -> Result<Params> {
    match command {
        MotionCommand::EnableAll => {
            for id in [CapabilityId::Animation, CapabilityId::SetAngles] {
                driver.enable(id).await?;
            }
        }
        MotionCommand::DisableAll => {
            for id in [CapabilityId::Animation, CapabilityId::SetAngles] {
                driver.disable(id).await?;
            }
        }
        MotionCommand::Custom {
            animation,
            set_angles,
        } => {
            switch(driver, CapabilityId::Animation, animation).await?;
            switch(driver, CapabilityId::SetAngles, set_angles).await?;
        }
    }
    Ok(None)
}

async fn misc(driver: &Driver, command: MiscCommand) -> Result<Params> {
    match command {
        MiscCommand::EnableAll => {
            for id in [CapabilityId::Leds, CapabilityId::Sonar, CapabilityId::Touch] {
                driver.enable(id).await?;
            }
        }
        MiscCommand::DisableAll => {
            for id in [CapabilityId::Leds, CapabilityId::Sonar, CapabilityId::Touch] {
                driver.disable(id).await?;
            }
        }
        MiscCommand::Custom {
            leds,
            sonars,
            touch,
        } => {
            switch(driver, CapabilityId::Leds, leds).await?;
            switch(driver, CapabilityId::Sonar, sonars).await?;
            switch(driver, CapabilityId::Touch, touch).await?;
        }
    }
    Ok(None)
}

async fn switch(driver: &Driver, id: CapabilityId, setting: Switch) -> Result<()> {
    match setting {
        Switch::Enable => driver.enable(id).await,
        Switch::Disable => driver.disable(id).await,
        Switch::Keep => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::ConfigurationResult;
    use crate::capabilities::support::{Harness, Recording, shm_lock};
    use crate::domain::{CameraParams, CameraTarget, SpeechParams};

    /// A driver wired to recording doubles for every capability it addresses.
    fn driver(harness: &Harness) -> (Driver, Vec<Arc<Recording>>) {
        let doubles: Vec<Arc<Recording>> = CapabilityId::all()
            .iter()
            .map(|id| Recording::new(*id, None))
            .collect();
        let driver = Driver::new(
            harness.ctx.clone(),
            doubles
                .iter()
                .map(|double| double.clone() as Arc<dyn Capability>)
                .collect(),
        );
        (driver, doubles)
    }

    use crate::capabilities::Capability;
    use std::sync::Arc;

    #[tokio::test]
    async fn enable_mapper_sets_its_group_and_shm_flags() {
        let _guard = shm_lock().await;
        let harness = Harness::default();
        let (driver, _) = driver(&harness);
        let response = dispatch(
            &driver,
            ControlRequest::Navigation(NavigationCommand::EnableMapper),
        )
        .await;
        assert_eq!(response.result, "ok");

        for id in [
            CapabilityId::Tf,
            CapabilityId::Laser,
            CapabilityId::DepthToLaser,
            CapabilityId::Odom,
            CapabilityId::MergedLaser,
            CapabilityId::CmdVel,
        ] {
            assert!(driver.is_enabled(id), "{}", id.as_str());
        }
        assert!(!driver.is_enabled(CapabilityId::MoveTo));
        assert!(harness.ctx.shm.enabled(Segment::PepperHead));
        assert!(harness.ctx.shm.enabled(Segment::Depth2Laser));
        assert!(!harness.ctx.shm.enabled(Segment::Planner));
    }

    #[tokio::test]
    async fn enable_all_skips_the_laser_merger() {
        let _guard = shm_lock().await;
        let harness = Harness::default();
        let (driver, _) = driver(&harness);
        dispatch(
            &driver,
            ControlRequest::Navigation(NavigationCommand::EnableAll),
        )
        .await;

        assert!(driver.is_enabled(CapabilityId::MoveTo));
        assert!(driver.is_enabled(CapabilityId::FreeZone));
        assert!(!driver.is_enabled(CapabilityId::MergedLaser));
        assert!(harness.ctx.shm.enabled(Segment::Depth2Laser));
        assert!(!harness.ctx.shm.enabled(Segment::PepperHead));
    }

    #[tokio::test]
    async fn disable_commands_reverse_their_group() {
        let _guard = shm_lock().await;
        let harness = Harness::default();
        let (driver, _) = driver(&harness);
        dispatch(
            &driver,
            ControlRequest::Navigation(NavigationCommand::EnableNavigate),
        )
        .await;
        assert!(driver.is_enabled(CapabilityId::NavigationPath));
        assert!(harness.ctx.shm.enabled(Segment::Planner));

        dispatch(
            &driver,
            ControlRequest::Navigation(NavigationCommand::DisableNavigate),
        )
        .await;
        assert!(!driver.is_enabled(CapabilityId::NavigationPath));
        assert!(!harness.ctx.shm.enabled(Segment::Planner));
    }

    #[tokio::test]
    async fn custom_navigation_applies_fields_independently() {
        let _guard = shm_lock().await;
        let harness = Harness::default();
        let (driver, _) = driver(&harness);
        let custom = NavigationCustom {
            tf: FrequencySetting {
                enable: true,
                frequency: 25.0,
            },
            odom: FrequencySetting {
                enable: false,
                frequency: 10.0,
            },
            laser: FrequencySetting {
                enable: false,
                frequency: 10.0,
            },
            path: FrequencySetting {
                enable: false,
                frequency: 10.0,
            },
            pose_pub: FrequencySetting {
                enable: false,
                frequency: 10.0,
            },
            cmd_vel: crate::domain::CmdVelSetting {
                enable: true,
                security_timer: 0.25,
            },
            depth_to_laser: DepthToLaserSetting {
                enable: true,
                params: crate::domain::DepthToLaserParams::default(),
            },
            moveto: true,
            goal: false,
            pose_set: true,
            result: false,
            free_zone: true,
        };
        dispatch(
            &driver,
            ControlRequest::Navigation(NavigationCommand::Custom(custom)),
        )
        .await;

        assert!(driver.is_enabled(CapabilityId::Tf));
        assert!(!driver.is_enabled(CapabilityId::Odom));
        assert!(driver.is_enabled(CapabilityId::CmdVel));
        assert!(driver.is_enabled(CapabilityId::DepthToLaser));
        assert!(driver.is_enabled(CapabilityId::MoveTo));
        assert!(!driver.is_enabled(CapabilityId::NavigationGoal));
        assert!(driver.is_enabled(CapabilityId::PoseSet));
        assert!(driver.is_enabled(CapabilityId::FreeZone));
        assert_eq!(driver.frequency(CapabilityId::Tf), Some(25.0));
    }

    #[tokio::test]
    async fn face_detectors_only_take_enable_and_disable() {
        let harness = Harness::default();
        let (driver, _) = driver(&harness);
        let response = dispatch(
            &driver,
            ControlRequest::Vision(VisionTools {
                camera: CameraTarget::FrontCameraFaceDetector,
                command: VisionCommand::GetParameters(CameraParams::default()),
            }),
        )
        .await;
        assert!(response.result.contains("face detectors"));
    }

    #[tokio::test]
    async fn get_commands_report_their_parameters() {
        let harness = Harness::default();
        let params = SpeechParams::defaults(crate::domain::Language::English);
        let speech = Recording::with_configure_result(
            CapabilityId::Speech,
            ConfigurationResult::SpeechParams(params),
        );
        let driver = Driver::new(harness.ctx.clone(), vec![speech]);
        let response = dispatch(
            &driver,
            ControlRequest::Audio(AudioCommand::GetSpeechParams),
        )
        .await;
        assert_eq!(response.result, "ok");
        assert_eq!(response.params, Some(ControlParams::Speech(params)));
    }

    #[tokio::test]
    async fn misc_and_motion_commands_follow_the_switches() {
        let harness = Harness::default();
        let (driver, _) = driver(&harness);
        dispatch(
            &driver,
            ControlRequest::Misc(MiscCommand::Custom {
                leds: Switch::Enable,
                sonars: Switch::Keep,
                touch: Switch::Disable,
            }),
        )
        .await;
        assert!(driver.is_enabled(CapabilityId::Leds));
        assert!(!driver.is_enabled(CapabilityId::Sonar));
        assert!(!driver.is_enabled(CapabilityId::Touch));

        dispatch(
            &driver,
            ControlRequest::Motion(MotionCommand::Custom {
                animation: Switch::Enable,
                set_angles: Switch::Enable,
            }),
        )
        .await;
        assert!(driver.is_enabled(CapabilityId::Animation));
        assert!(driver.is_enabled(CapabilityId::SetAngles));
    }

    #[tokio::test]
    async fn vision_commands_drive_the_camera_capabilities() {
        let harness = Harness::default();
        let camera = Recording::with_configure_result(
            CapabilityId::FrontCamera,
            ConfigurationResult::CameraParams(CameraParams::default()),
        );
        let driver = Driver::new(harness.ctx.clone(), vec![camera.clone()]);
        let config = crate::domain::CameraConfig::color_default();

        dispatch(
            &driver,
            ControlRequest::Vision(VisionTools {
                camera: CameraTarget::FrontCamera,
                command: VisionCommand::Custom(config, CameraParams::default()),
            }),
        )
        .await;
        assert!(driver.is_enabled(CapabilityId::FrontCamera));
        assert_eq!(driver.frequency(CapabilityId::FrontCamera), Some(10.0));
        assert_eq!(
            camera.configures.load(std::sync::atomic::Ordering::Relaxed),
            1
        );

        let response = dispatch(
            &driver,
            ControlRequest::Vision(VisionTools {
                camera: CameraTarget::FrontCamera,
                command: VisionCommand::GetParameters(CameraParams::default()),
            }),
        )
        .await;
        assert_eq!(
            response.params,
            Some(ControlParams::Camera(CameraParams::default()))
        );

        dispatch(
            &driver,
            ControlRequest::Vision(VisionTools {
                camera: CameraTarget::FrontCamera,
                command: VisionCommand::Disable,
            }),
        )
        .await;
        assert!(!driver.is_enabled(CapabilityId::FrontCamera));
    }

    #[tokio::test]
    async fn face_detectors_enable_and_disable() {
        let harness = Harness::default();
        let (driver, _) = driver(&harness);
        for command in [VisionCommand::Enable, VisionCommand::Disable] {
            let response = dispatch(
                &driver,
                ControlRequest::Vision(VisionTools {
                    camera: CameraTarget::BottomCameraFaceDetector,
                    command,
                }),
            )
            .await;
            assert_eq!(response.result, "ok");
        }
        assert!(!driver.is_enabled(CapabilityId::BottomCameraFaceDetector));
    }

    #[tokio::test]
    async fn audio_commands_drive_their_group() {
        let harness = Harness::default();
        let mic = Recording::new(CapabilityId::Mic, None);
        let driver = Driver::new(
            harness.ctx.clone(),
            vec![
                mic.clone(),
                Recording::new(CapabilityId::Speech, None),
                Recording::new(CapabilityId::MicLocalization, None),
            ],
        );

        dispatch(&driver, ControlRequest::Audio(AudioCommand::Enable)).await;
        assert!(driver.is_enabled(CapabilityId::Mic));
        assert!(driver.is_enabled(CapabilityId::Speech));
        assert!(driver.is_enabled(CapabilityId::MicLocalization));

        dispatch(&driver, ControlRequest::Audio(AudioCommand::Disable)).await;
        assert!(!driver.is_enabled(CapabilityId::Mic));

        dispatch(
            &driver,
            ControlRequest::Audio(AudioCommand::Custom(crate::domain::MicConfig::DEFAULT)),
        )
        .await;
        assert!(driver.is_enabled(CapabilityId::Mic));
        assert_eq!(mic.configures.load(std::sync::atomic::Ordering::Relaxed), 1);

        dispatch(&driver, ControlRequest::Audio(AudioCommand::DisableTts)).await;
        assert!(!driver.is_enabled(CapabilityId::Speech));
    }

    #[tokio::test]
    async fn motion_and_misc_disable_all_reverse_their_groups() {
        let harness = Harness::default();
        let (driver, _) = driver(&harness);
        dispatch(&driver, ControlRequest::Motion(MotionCommand::EnableAll)).await;
        dispatch(&driver, ControlRequest::Motion(MotionCommand::DisableAll)).await;
        assert!(!driver.is_enabled(CapabilityId::Animation));
        assert!(!driver.is_enabled(CapabilityId::SetAngles));

        dispatch(&driver, ControlRequest::Misc(MiscCommand::EnableAll)).await;
        dispatch(&driver, ControlRequest::Misc(MiscCommand::DisableAll)).await;
        assert!(!driver.is_enabled(CapabilityId::Leds));
        assert!(!driver.is_enabled(CapabilityId::Sonar));
        assert!(!driver.is_enabled(CapabilityId::Touch));
    }

    #[tokio::test]
    async fn failures_are_reported_in_the_result_string() {
        let harness = Harness::default();
        let driver = Driver::new(harness.ctx.clone(), vec![]);
        let response = dispatch(&driver, ControlRequest::Misc(MiscCommand::EnableAll)).await;
        assert!(
            response.result.contains("unknown id"),
            "{}",
            response.result
        );
    }
}
