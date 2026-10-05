//! Face detection: crops detected faces out of a VGA camera stream.

use super::camera::{camera_frame, decode_image, WireImage};
use super::{Capability, CapabilityId, Context};
use crate::domain::{CameraId, ColorSpace, Face, FaceSet, ImageFrame, Message, Resolution, Timestamp};
use crate::qi::events::FaceEvent;
use crate::qi::keys;
use crate::Result;
use async_trait::async_trait;
use std::sync::{Arc, Mutex};

/// Camera subscription rate of the detector, in Hz.
const CAMERA_HZ: u8 = 30;

/// Relative margin added around each face box.
const MARGIN: f32 = 0.09;

struct Inner {
    camera: CameraId,
    topic: &'static str,
    handle: Mutex<Option<String>>,
}

/// Publishes face crops for one camera whenever `FaceDetected` fires.
pub struct FaceDetector {
    inner: Arc<Inner>,
}

impl FaceDetector {
    pub fn front() -> Self {
        Self::new(CameraId::Front)
    }

    pub fn bottom() -> Self {
        Self::new(CameraId::Bottom)
    }

    pub fn new(camera: CameraId) -> Self {
        let topic = match camera {
            CameraId::Front | CameraId::Depth => CapabilityId::FrontCameraFaceDetector,
            CameraId::Bottom => CapabilityId::BottomCameraFaceDetector,
        };
        Self {
            inner: Arc::new(Inner {
                camera,
                topic: topic.as_str(),
                handle: Mutex::new(None),
            }),
        }
    }

    fn capability_id(&self) -> CapabilityId {
        match self.inner.camera {
            CameraId::Front | CameraId::Depth => CapabilityId::FrontCameraFaceDetector,
            CameraId::Bottom => CapabilityId::BottomCameraFaceDetector,
        }
    }

    async fn on_faces(event: FaceEvent, shared: Arc<Shared>) -> Result<()> {
        if event.camera_id != shared.inner.camera.source() {
            return Ok(());
        }
        let Some(handle) = shared
            .inner
            .handle
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .clone()
        else {
            return Ok(());
        };
        let raw = shared.robot.video.get_image_remote(&handle).await?;
        let image = decode_image(&raw)
            .ok_or_else(|| crate::Error::qi_value("ALVideoDevice.getImageRemote", "malformed image"))?;
        let stamp = Timestamp::new(event.stamp[0] as u64, (event.stamp[1] as u32) * 1000);
        let mut faces = Vec::with_capacity(event.faces.len());
        for shape in &event.faces {
            if shape.len() < 5 {
                continue;
            }
            let corner = shared
                .robot
                .video
                .image_position_from_angular_position(
                    shared.inner.camera.source(),
                    (shape[1], shape[2]),
                )
                .await?;
            let Some([x, y, width, height]) = face_box(corner, shape[3], shape[4], &image) else {
                continue;
            };
            faces.push(Face {
                stamp,
                image: ImageFrame {
                    stamp,
                    frame: camera_frame(shared.inner.camera).to_owned(),
                    width,
                    height,
                    color_space: ColorSpace::RGB,
                    jpeg: None,
                    data: crop_rgb(&image.data, image.width, x, y, width, height),
                    camera_info: None,
                },
            });
        }
        let set = FaceSet {
            stamp,
            camera: shared.inner.topic.to_owned(),
            faces,
        };
        shared
            .transport
            .publish(shared.inner.topic, Message::Faces(set))
    }
}

/// The QI pieces the face handler needs after `enable` returns.
struct Shared {
    robot: Arc<crate::qi::Robot>,
    transport: Arc<dyn crate::transport::Transport>,
    inner: Arc<Inner>,
}

#[async_trait]
impl Capability for FaceDetector {
    fn id(&self) -> CapabilityId {
        self.capability_id()
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        if self
            .inner
            .handle
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .is_some()
        {
            return Ok(());
        }
        let handle = ctx
            .robot
            .video
            .subscribe_camera(
                self.inner.topic,
                self.inner.camera.source(),
                i32::from(Resolution::VGA.0),
                ColorSpace::RGB.0,
                i32::from(CAMERA_HZ),
            )
            .await?;
        *self.inner.handle.lock().unwrap_or_else(|err| err.into_inner()) = Some(handle);

        let shared = Arc::new(Shared {
            robot: Arc::clone(&ctx.robot),
            transport: Arc::clone(&ctx.transport),
            inner: Arc::clone(&self.inner),
        });
        let runtime = tokio::runtime::Handle::current();
        ctx.toolkit.face_detected().set(move |event| {
            let shared = Arc::clone(&shared);
            runtime.spawn(async move {
                if let Err(err) = Self::on_faces(event, shared).await {
                    tracing::warn!(error = %err, "face crop failed");
                }
            });
        });
        ctx.robot
            .memory
            .subscribe_to_event(
                keys::FACE_DETECTED,
                &keys::callback_service_name(keys::FACE_DETECTED),
                "faceDetectedCallback",
            )
            .await
    }

    async fn disable(&self, ctx: &Context) -> Result<()> {
        let handle = self
            .inner
            .handle
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .take();
        let Some(handle) = handle else {
            return Ok(());
        };
        ctx.toolkit.face_detected().clear();
        ctx.robot.video.unsubscribe(&handle).await?;
        ctx.robot
            .memory
            .unsubscribe_to_event(
                keys::FACE_DETECTED,
                &keys::callback_service_name(keys::FACE_DETECTED),
            )
            .await
    }
}

/// Pixel crop box of one face: `corner` in image coordinates (0..1) and the
/// angular extents, widened by [`MARGIN`] and clamped to the image.
fn face_box(
    corner: Vec<f32>,
    width_angle: f32,
    height_angle: f32,
    image: &WireImage,
) -> Option<[u32; 4]> {
    let [width, height] = [image.width as f32, image.height as f32];
    let mut x = width * (corner.first()? - width_angle / 2.0) - width * MARGIN;
    let mut y = height * (corner.get(1)? - height_angle / 2.0) - height * MARGIN;
    let mut box_width = width * (width_angle + 2.0 * MARGIN);
    let mut box_height = height * (height_angle + 2.0 * MARGIN);
    if x < 0.0 {
        x = 0.0;
    }
    if y < 0.0 {
        y = 0.0;
    }
    if x + box_width > width {
        box_width = width - x;
    }
    if y + box_height > height {
        box_height = height - y;
    }
    if box_width <= 0.0 || box_height <= 0.0 {
        return None;
    }
    Some([x as u32, y as u32, box_width as u32, box_height as u32])
}

/// Copies the `(x, y, w, h)` region out of RGB pixels.
fn crop_rgb(data: &[u8], stride_width: u32, x: u32, y: u32, width: u32, height: u32) -> Vec<u8> {
    let mut crop = Vec::with_capacity((width * height * 3) as usize);
    for row in y..y + height {
        let start = ((row * stride_width + x) * 3) as usize;
        let end = start + (width * 3) as usize;
        if end <= data.len() {
            crop.extend_from_slice(&data[start..end]);
        }
    }
    crop
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::support::Harness;
    use crate::capabilities::Capability;
    use crate::qi::value::Raw;
    use qi::value::IntoValue;
    use qi::{Object, Value};

    fn wire_image(width: i32, height: i32) -> WireImage {
        decode_image(&wire_value(width, height)).expect("wire image")
    }

    fn wire_value(width: i32, height: i32) -> Value<'static> {
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

    fn face_event(camera_id: i32) -> Value<'static> {
        let shape = vec![0.0f32, 0.1, 0.2, 0.3, 0.4].into_value();
        let face = vec![shape, 0.0f32.into_value()].into_value();
        vec![
            vec![100.0f32, 200.0].into_value(),
            vec![face].into_value(),
            vec![1.0f32, 2.0].into_value(),
            vec![3.0f32].into_value(),
            camera_id.into_value(),
        ]
        .into_value()
    }

    fn callback_args(payload: Value<'static>) -> Value<'static> {
        (
            keys::FACE_DETECTED.to_owned(),
            Raw::new(payload),
            "subscriber".to_owned(),
        )
            .into_value()
    }

    #[test]
    fn face_box_centers_and_margins_the_crop() {
        let image = wire_image(100, 100);
        assert_eq!(face_box(vec![0.5, 0.5], 0.2, 0.2, &image), Some([31, 31, 38, 38]));
    }

    #[test]
    fn face_box_clamps_to_the_image() {
        let image = wire_image(100, 100);
        assert_eq!(face_box(vec![0.0, 0.0], 0.2, 0.2, &image), Some([0, 0, 38, 38]));
        assert_eq!(face_box(vec![1.0, 1.0], 0.2, 0.2, &image), Some([81, 81, 19, 19]));
        assert_eq!(face_box(vec![2.0, 0.5], 0.2, 0.2, &image), None);
    }

    #[test]
    fn crop_rgb_extracts_the_region() {
        // 3x2 RGB image, pixels filled with their byte index.
        let data: Vec<u8> = (0..18).collect();
        assert_eq!(
            crop_rgb(&data, 3, 1, 0, 2, 2),
            vec![3, 4, 5, 6, 7, 8, 12, 13, 14, 15, 16, 17]
        );
    }

    #[tokio::test]
    async fn enable_subscribes_camera_and_event() {
        let harness = Harness::default();
        let video = harness.fakes.service("ALVideoDevice");
        video.script("subscribeCamera", "face-handle".to_owned().into_value());
        FaceDetector::front().enable(&harness.ctx).await.expect("enable");

        assert_eq!(
            video.calls_to("subscribeCamera")[0],
            ("front_camera_face_detector".to_owned(), 0i32, 2i32, 11i32, 30i32).into_value()
        );
        let memory = harness.fakes.service("ALMemory");
        assert_eq!(
            memory.calls_to("subscribeToEvent")[0],
            (
                keys::FACE_DETECTED.to_owned(),
                keys::callback_service_name(keys::FACE_DETECTED),
                "faceDetectedCallback".to_owned()
            )
                .into_value()
        );
    }

    #[tokio::test]
    async fn events_crop_faces_and_publish() {
        let harness = Harness::default();
        let video = harness.fakes.service("ALVideoDevice");
        video.script("subscribeCamera", "face-handle".to_owned().into_value());
        video.script("getImageRemote", wire_value(32, 24));
        video.script(
            "getImagePositionFromAngularPosition",
            vec![0.5f32, 0.5].into_value(),
        );
        let detector = FaceDetector::front();
        detector.enable(&harness.ctx).await.expect("enable");

        harness
            .ctx
            .toolkit
            .meta_call("faceDetectedCallback".into(), callback_args(face_event(0)))
            .await
            .expect("callback");
        tokio::time::sleep(std::time::Duration::from_millis(20)).await;

        let published = harness.transport.published_on("front_camera_face_detector");
        let Message::Faces(set) = &published[0] else {
            panic!("expected faces");
        };
        assert_eq!(set.camera, "front_camera_face_detector");
        assert_eq!(set.stamp, Timestamp::new(100, 200_000));
        assert_eq!(set.faces.len(), 1);
        let crop = &set.faces[0].image;
        assert_eq!(crop.camera_info, None);
        assert_eq!(crop.frame, "CameraTop_optical_frame");
        assert!(!crop.data.is_empty());
    }

    #[tokio::test]
    async fn events_from_other_cameras_are_ignored() {
        let harness = Harness::default();
        let video = harness.fakes.service("ALVideoDevice");
        video.script("subscribeCamera", "face-handle".to_owned().into_value());
        video.script("getImageRemote", wire_value(32, 24));
        let detector = FaceDetector::front();
        detector.enable(&harness.ctx).await.expect("enable");

        harness
            .ctx
            .toolkit
            .meta_call("faceDetectedCallback".into(), callback_args(face_event(1)))
            .await
            .expect("callback");
        tokio::time::sleep(std::time::Duration::from_millis(20)).await;
        assert!(harness
            .transport
            .published_on("front_camera_face_detector")
            .is_empty());
    }

    #[tokio::test]
    async fn disable_releases_everything() {
        let harness = Harness::default();
        let video = harness.fakes.service("ALVideoDevice");
        video.script("subscribeCamera", "face-handle".to_owned().into_value());
        let detector = FaceDetector::bottom();
        detector.enable(&harness.ctx).await.expect("enable");
        detector.disable(&harness.ctx).await.expect("disable");
        detector.disable(&harness.ctx).await.expect("disable again");

        assert_eq!(
            video.calls_to("unsubscribe"),
            vec!["face-handle".to_owned().into_value()]
        );
        assert_eq!(
            harness
                .fakes
                .service("ALMemory")
                .calls_to("unsubscribeToEvent")
                .len(),
            1
        );
    }
}
