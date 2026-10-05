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

//! Camera streaming: one capability per camera source.

use super::{Capability, CapabilityId, Configuration, ConfigurationResult, Context};
use crate::Result;
use crate::domain::{
    CameraConfig, CameraId, CameraParam, CameraParams, ColorSpace, ImageFrame, Message, Timestamp,
};
use crate::qi::value::{as_bytes, as_f32, as_i32, plain};
use async_trait::async_trait;
use qi::value::Value;
use std::sync::Mutex;

/// Default publishing rate in Hz; the control layer syncs it with `CameraConfig::fps`.
pub const DEFAULT_HZ: f32 = 10.0;

/// Pixels decoded out of the NAOqi image wire value.
pub(crate) struct WireImage {
    pub(crate) width: u32,
    pub(crate) height: u32,
    color_space: ColorSpace,
    stamp: Timestamp,
    pub(crate) data: Vec<u8>,
}

/// Streams one camera: frames plus calibration on the capability topic.
pub struct Camera {
    camera: CameraId,
    config: Mutex<CameraConfig>,
    handle: Mutex<Option<String>>,
}

impl Camera {
    pub fn front() -> Self {
        Self::new(CameraId::Front)
    }

    pub fn bottom() -> Self {
        Self::new(CameraId::Bottom)
    }

    pub fn depth() -> Self {
        Self::new(CameraId::Depth)
    }

    pub fn new(camera: CameraId) -> Self {
        let config = if camera.is_depth() {
            CameraConfig::depth_default()
        } else {
            CameraConfig::color_default()
        };
        Self {
            camera,
            config: Mutex::new(config),
            handle: Mutex::new(None),
        }
    }

    fn capability_id(&self) -> CapabilityId {
        match self.camera {
            CameraId::Front => CapabilityId::FrontCamera,
            CameraId::Bottom => CapabilityId::BottomCamera,
            CameraId::Depth => CapabilityId::DepthCamera,
        }
    }

    fn config(&self) -> CameraConfig {
        *self.config.lock().unwrap_or_else(|err| err.into_inner())
    }

    async fn subscribe(&self, ctx: &Context) -> Result<()> {
        let config = self.config();
        let handle = ctx
            .robot
            .video
            .subscribe_camera(
                self.capability_id().as_str(),
                self.camera.source(),
                i32::from(config.resolution.0),
                config.color_space.0,
                i32::from(config.fps),
            )
            .await?;
        *self.handle.lock().unwrap_or_else(|err| err.into_inner()) = Some(handle);
        Ok(())
    }

    async fn apply_params(&self, ctx: &Context, params: CameraParams) -> Result<()> {
        for (param, value) in params.as_pairs() {
            ctx.robot
                .video
                .set_camera_parameter(self.camera.source(), param.id(), value)
                .await?;
        }
        Ok(())
    }

    async fn read_params(&self, ctx: &Context, request: CameraParams) -> Result<CameraParams> {
        let mut read = CameraParams::default();
        for (param, _) in request.as_pairs() {
            let value = ctx
                .robot
                .video
                .get_camera_parameter(self.camera.source(), param.id())
                .await?;
            assign(&mut read, param, value);
        }
        Ok(read)
    }
}

#[async_trait]
impl Capability for Camera {
    fn id(&self) -> CapabilityId {
        self.capability_id()
    }

    fn period(&self) -> Option<f32> {
        Some(DEFAULT_HZ)
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        if self
            .handle
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .is_some()
        {
            return Ok(());
        }
        self.config().validate(self.camera)?;
        self.subscribe(ctx).await
    }

    async fn disable(&self, ctx: &Context) -> Result<()> {
        let handle = self
            .handle
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .take();
        if let Some(handle) = handle {
            ctx.robot.video.unsubscribe(&handle).await?;
        }
        Ok(())
    }

    async fn tick(&self, ctx: &Context) -> Result<()> {
        let Some(handle) = self
            .handle
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .clone()
        else {
            return Ok(());
        };
        let raw = ctx.robot.video.get_image_remote(&handle).await?;
        let image = decode_image(&raw).ok_or_else(|| {
            crate::Error::qi_value("ALVideoDevice.getImageRemote", "malformed image")
        })?;
        let config = self.config();
        let camera_info = ctx
            .assets
            .camera_info(self.camera)
            .and_then(|file| file.get(config.resolution))
            .cloned();
        let frame = ImageFrame {
            stamp: image.stamp,
            frame: camera_frame(self.camera).to_owned(),
            width: image.width,
            height: image.height,
            color_space: image.color_space,
            jpeg: match config.compress && image.color_space == ColorSpace::RGB {
                true => encode_jpeg(&image, config.jpeg_quality),
                false => None,
            },
            data: image.data,
            camera_info,
        };
        ctx.transport
            .publish(self.capability_id().as_str(), Message::Image(frame))
    }

    async fn configure(
        &self,
        ctx: &Context,
        configuration: Configuration,
    ) -> Result<ConfigurationResult> {
        match configuration {
            Configuration::Camera(config, params) => {
                config.validate(self.camera)?;
                if self.camera.is_depth() && !params.as_pairs().is_empty() {
                    return Err(depth_params_error());
                }
                *self.config.lock().unwrap_or_else(|err| err.into_inner()) = config;
                self.apply_params(ctx, params).await?;
                // The streaming setup changed: re-subscribe with the new config.
                if self
                    .handle
                    .lock()
                    .unwrap_or_else(|err| err.into_inner())
                    .is_some()
                {
                    self.disable(ctx).await?;
                    self.subscribe(ctx).await?;
                }
                Ok(ConfigurationResult::None)
            }
            Configuration::CameraParams(params) => {
                if self.camera.is_depth() {
                    return Err(depth_params_error());
                }
                self.apply_params(ctx, params).await?;
                Ok(ConfigurationResult::None)
            }
            Configuration::ReadCameraParams(request) => {
                if self.camera.is_depth() {
                    return Err(depth_params_error());
                }
                let read = self.read_params(ctx, request).await?;
                Ok(ConfigurationResult::CameraParams(read))
            }
            _ => Ok(ConfigurationResult::None),
        }
    }
}

fn depth_params_error() -> crate::Error {
    crate::Error::invalid("camera params", "the depth camera has no parameters")
}

/// Optical frame published with images of `camera`.
pub(crate) fn camera_frame(camera: CameraId) -> &'static str {
    match camera {
        CameraId::Front | CameraId::Depth => "CameraTop_optical_frame",
        CameraId::Bottom => "CameraBottom_optical_frame",
    }
}

/// Decodes the `getImageRemote` wire value:
/// `[width, height, layers, color_space, stamp_s, stamp_us, buffer, cam_id, fov...]`.
pub(crate) fn decode_image(value: &Value<'_>) -> Option<WireImage> {
    let elements = match plain(value.clone()) {
        Value::List(elements) | Value::Tuple(elements) => elements,
        _ => return None,
    };
    if elements.len() < 8 {
        return None;
    }
    let color_space = ColorSpace(as_i32(&elements[3])?);
    Some(WireImage {
        width: as_i32(&elements[0])? as u32,
        height: as_i32(&elements[1])? as u32,
        color_space,
        stamp: Timestamp::new(
            as_f32(&elements[4])? as u64,
            (as_f32(&elements[5])? as u32) * 1000,
        ),
        data: as_bytes(&elements[6])?,
    })
}

fn encode_jpeg(image: &WireImage, quality: u8) -> Option<Vec<u8>> {
    let mut encoded = Vec::new();
    let encoder = jpeg_encoder::Encoder::new(&mut encoded, quality);
    encoder
        .encode(
            &image.data,
            image.width as u16,
            image.height as u16,
            jpeg_encoder::ColorType::Rgb,
        )
        .ok()?;
    Some(encoded)
}

/// Sets one register value on a parameter payload.
fn assign(params: &mut CameraParams, param: CameraParam, value: i32) {
    match param {
        CameraParam::Brightness => params.brightness = Some(value),
        CameraParam::Contrast => params.contrast = Some(value),
        CameraParam::Saturation => params.saturation = Some(value),
        CameraParam::Hue => params.hue = Some(value),
        CameraParam::Gain => params.gain = Some(value),
        CameraParam::Exposure => params.exposure = Some(value),
        CameraParam::HorizontalFlip => params.horizontal_flip = Some(value != 0),
        CameraParam::VerticalFlip => params.vertical_flip = Some(value != 0),
        CameraParam::AutoExposure => params.auto_exposure = Some(value != 0),
        CameraParam::AutoWhiteBalance => params.auto_white_balance = Some(value != 0),
        CameraParam::AutoGain => params.auto_gain = Some(value != 0),
        CameraParam::AutoFocus => params.auto_focus = Some(value != 0),
        CameraParam::AverageLuminance => params.average_luminance = Some(value),
        CameraParam::BlcBlue => params.blc_blue = Some(value),
        CameraParam::BlcRed => params.blc_red = Some(value),
        CameraParam::BlcGreen => params.blc_green = Some(value),
        CameraParam::Resolution | CameraParam::Fps | CameraParam::SetDefault => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use crate::domain::Resolution;
    use qi::value::IntoValue;

    /// The `getImageRemote` wire value of a `width` x `height` RGB image.
    fn test_image(width: i32, height: i32) -> Value<'static> {
        let data: Vec<u8> = (0..width * height * 3).map(|index| index as u8).collect();
        vec![
            width.into_value(),
            height.into_value(),
            3i32.into_value(),
            11i32.into_value(),
            42i32.into_value(),
            500i32.into_value(),
            data.into_value(),
            0i32.into_value(),
            0.5f32.into_value(),
            0.5f32.into_value(),
            (-0.5f32).into_value(),
            (-0.5f32).into_value(),
        ]
        .into_value()
    }

    #[test]
    fn decode_image_reads_the_wire_layout() {
        let image = decode_image(&test_image(2, 2)).expect("image");
        assert_eq!(image.width, 2);
        assert_eq!(image.height, 2);
        assert_eq!(image.color_space, ColorSpace::RGB);
        assert_eq!(image.stamp, Timestamp::new(42, 500_000));
        assert_eq!(image.data.len(), 12);
    }

    #[test]
    fn decode_image_rejects_malformed_values() {
        assert!(decode_image(&Value::Unit).is_none());
        assert!(decode_image(&vec![1i32.into_value()].into_value()).is_none());
    }

    #[tokio::test]
    async fn enable_subscribes_with_the_default_config() {
        let harness = Harness::default();
        harness
            .fakes
            .service("ALVideoDevice")
            .script("subscribeCamera", "handle-1".to_owned().into_value());
        Camera::front().enable(&harness.ctx).await.expect("enable");

        let calls = harness
            .fakes
            .service("ALVideoDevice")
            .calls_to("subscribeCamera");
        assert_eq!(
            calls[0],
            ("front_camera".to_owned(), 0i32, 1i32, 11i32, 10i32).into_value()
        );
    }

    #[tokio::test]
    async fn tick_publishes_frames_with_calibration() {
        let harness = Harness::default();
        let video = harness.fakes.service("ALVideoDevice");
        video.script("subscribeCamera", "handle-1".to_owned().into_value());
        video.script("getImageRemote", test_image(2, 2));
        let camera = Camera::front();
        camera.enable(&harness.ctx).await.expect("enable");
        camera.tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("front_camera");
        let Message::Image(image) = &published[0] else {
            panic!("expected an image");
        };
        assert_eq!(image.width, 2);
        assert_eq!(image.frame, "CameraTop_optical_frame");
        assert_eq!(image.stamp, Timestamp::new(42, 500_000));
        assert_eq!(image.jpeg, None);
        let info = image.camera_info.as_ref().expect("calibration");
        assert_eq!(info.width, 320);
        assert_eq!(info.frame, "CameraTop_optical_frame");
    }

    #[tokio::test]
    async fn compressed_frames_carry_a_jpeg() {
        let harness = Harness::default();
        let video = harness.fakes.service("ALVideoDevice");
        video.script("subscribeCamera", "handle-1".to_owned().into_value());
        video.script("getImageRemote", test_image(2, 2));
        let camera = Camera::front();
        camera
            .configure(
                &harness.ctx,
                Configuration::Camera(
                    CameraConfig {
                        compress: true,
                        jpeg_quality: 97,
                        ..CameraConfig::color_default()
                    },
                    CameraParams::default(),
                ),
            )
            .await
            .expect("configure");
        camera.enable(&harness.ctx).await.expect("enable");
        camera.tick(&harness.ctx).await.expect("tick");

        let published = harness.transport.published_on("front_camera");
        let Message::Image(image) = &published[0] else {
            panic!("expected an image");
        };
        let jpeg = image.jpeg.as_ref().expect("jpeg");
        assert_eq!(&jpeg[..2], &[0xff, 0xd8]);
    }

    #[tokio::test]
    async fn invalid_configs_are_rejected() {
        let harness = Harness::default();
        let camera = Camera::front();
        let error = camera
            .configure(
                &harness.ctx,
                Configuration::Camera(
                    CameraConfig {
                        resolution: Resolution::VGA,
                        fps: 30,
                        ..CameraConfig::depth_default()
                    },
                    CameraParams::default(),
                ),
            )
            .await;
        assert!(error.is_err());
    }

    #[tokio::test]
    async fn parameters_are_set_and_read_back() {
        let harness = Harness::default();
        let video = harness.fakes.service("ALVideoDevice");
        video.script("subscribeCamera", "handle-1".to_owned().into_value());
        video.script("getCameraParameter", 11i32.into_value());
        let camera = Camera::front();

        let params = CameraParams {
            brightness: Some(10),
            gain: Some(32),
            ..Default::default()
        };
        camera
            .configure(&harness.ctx, Configuration::CameraParams(params))
            .await
            .expect("set");
        let calls = video.calls_to("setCameraParameter");
        assert_eq!(calls[0], (0i32, 0i32, 10i32).into_value());
        assert_eq!(calls[1], (0i32, 6i32, 32i32).into_value());

        let read = camera
            .configure(
                &harness.ctx,
                Configuration::ReadCameraParams(CameraParams {
                    brightness: Some(0),
                    ..Default::default()
                }),
            )
            .await
            .expect("get");
        let ConfigurationResult::CameraParams(read) = read else {
            panic!("expected camera params");
        };
        assert_eq!(read.brightness, Some(11));
    }

    #[tokio::test]
    async fn depth_camera_rejects_parameter_requests() {
        let harness = Harness::default();
        let camera = Camera::depth();
        assert!(
            camera
                .configure(
                    &harness.ctx,
                    Configuration::CameraParams(CameraParams::default())
                )
                .await
                .is_err()
        );
        assert!(
            camera
                .configure(
                    &harness.ctx,
                    Configuration::ReadCameraParams(CameraParams {
                        brightness: Some(0),
                        ..Default::default()
                    })
                )
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn disable_unsubscribes_once() {
        let harness = Harness::default();
        let video = harness.fakes.service("ALVideoDevice");
        video.script("subscribeCamera", "handle-1".to_owned().into_value());
        let camera = Camera::bottom();
        camera.enable(&harness.ctx).await.expect("enable");
        camera.disable(&harness.ctx).await.expect("disable");
        camera.disable(&harness.ctx).await.expect("disable again");

        let unsubscribes = video.calls_to("unsubscribe");
        assert_eq!(unsubscribes, vec!["handle-1".to_owned().into_value()]);
        assert_eq!(
            video.calls_to("subscribeCamera")[0],
            ("bottom_camera".to_owned(), 1i32, 1i32, 11i32, 10i32).into_value()
        );
    }

    #[tokio::test]
    async fn custom_config_takes_effect_on_an_enabled_camera() {
        let harness = Harness::default();
        let video = harness.fakes.service("ALVideoDevice");
        video.script("subscribeCamera", "handle".to_owned().into_value());
        let camera = Camera::front();
        camera.enable(&harness.ctx).await.expect("enable");
        camera
            .configure(
                &harness.ctx,
                Configuration::Camera(
                    CameraConfig {
                        resolution: Resolution::QQVGA,
                        fps: 5,
                        ..CameraConfig::color_default()
                    },
                    CameraParams::default(),
                ),
            )
            .await
            .expect("configure");

        let subscriptions = video.calls_to("subscribeCamera");
        assert_eq!(subscriptions.len(), 2);
        assert_eq!(
            subscriptions[1],
            ("front_camera".to_owned(), 0i32, 0i32, 11i32, 5i32).into_value()
        );
    }
}
