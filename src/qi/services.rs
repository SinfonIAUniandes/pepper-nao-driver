//! Typed proxies over the NAOqi AL* services.
//!
//! Method names and argument shapes follow the NAOqi API; the proxies only
//! translate between domain values and QI values.

use super::service::Service;
use super::value::{as_f32, as_f32s, as_i32, as_text, as_texts, plain};
use crate::Result;
use qi::value::{IntoValue, Value};
use std::sync::Arc;

/// Frame in which ALMotion expresses poses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReferenceFrame {
    /// Frame 1: the world as tracked by the robot.
    World,
    /// Frame 2: the robot base.
    Robot,
}

impl ReferenceFrame {
    fn id(self) -> i32 {
        match self {
            Self::World => 1,
            Self::Robot => 2,
        }
    }
}

/// ALMemory: sensor keys, module keys and events.
pub struct AlMemory {
    service: Arc<dyn Service>,
}

impl AlMemory {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    pub async fn get_data(&self, key: &str) -> Result<Value<'static>> {
        let value = self.service.call("getData", key.to_owned().into_value()).await?;
        Ok(plain(value))
    }

    pub async fn get_list_data(&self, keys: &[String]) -> Result<Value<'static>> {
        let value = self
            .service
            .call("getListData", keys.to_vec().into_value())
            .await?;
        Ok(plain(value))
    }

    pub async fn insert_data(&self, key: &str, data: Value<'static>) -> Result<()> {
        self.service
            .post("insertData", (key.to_owned(), data).into_value())
            .await
    }

    /// Raises an event for the external planner / localizer bridge.
    pub async fn raise_event(&self, key: &str, event: Value<'static>) -> Result<()> {
        self.service
            .post("raiseEvent", (key.to_owned(), event).into_value())
            .await
    }

    /// Routes the event `key` to `callback` of the registered object `object`.
    pub async fn subscribe_to_event(
        &self,
        key: &str,
        object: &str,
        callback: &str,
    ) -> Result<()> {
        self.service
            .call(
                "subscribeToEvent",
                (key.to_owned(), object.to_owned(), callback.to_owned()).into_value(),
            )
            .await?;
        Ok(())
    }

    pub async fn unsubscribe_to_event(&self, key: &str, object: &str) -> Result<()> {
        self.service
            .call(
                "unsubscribeToEvent",
                (key.to_owned(), object.to_owned()).into_value(),
            )
            .await?;
        Ok(())
    }
}

/// ALMotion: walking, joints, poses and safety settings.
pub struct AlMotion {
    service: Arc<dyn Service>,
}

impl AlMotion {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    /// Velocity command `(vx, vy, wz)`, applied asynchronously.
    pub async fn move_(&self, vx: f32, vy: f32, wz: f32) -> Result<()> {
        self.service.post("move", (vx, vy, wz).into_value()).await
    }

    /// Relative walk to `(x, y, theta)`, applied asynchronously.
    pub async fn move_to(&self, x: f32, y: f32, theta: f32) -> Result<()> {
        self.service
            .post("moveTo", (x, y, theta).into_value())
            .await
    }

    /// Blocks until the current move completes; call on a dedicated thread.
    pub async fn wait_until_move_is_finished(&self) -> Result<()> {
        self.service
            .call("waitUntilMoveIsFinished", Value::Unit)
            .await?;
        Ok(())
    }

    pub async fn get_angles(&self, name: &str, use_sensors: bool) -> Result<Vec<f32>> {
        let value = self
            .service
            .call("getAngles", (name.to_owned(), use_sensors).into_value())
            .await?;
        as_f32s(&value)
            .ok_or_else(|| crate::Error::qi_value("ALMotion.getAngles", value))
    }

    pub async fn get_body_names(&self, name: &str) -> Result<Vec<String>> {
        let value = self
            .service
            .call("getBodyNames", name.to_owned().into_value())
            .await?;
        as_texts(&value)
            .ok_or_else(|| crate::Error::qi_value("ALMotion.getBodyNames", value))
    }

    /// Pose of `frame` in the given reference frame: `[x, y, z, roll, pitch, yaw]`.
    pub async fn get_position(
        &self,
        frame: &str,
        reference: ReferenceFrame,
        use_sensors: bool,
    ) -> Result<Vec<f32>> {
        let value = self
            .service
            .call(
                "getPosition",
                (frame.to_owned(), reference.id(), use_sensors).into_value(),
            )
            .await?;
        as_f32s(&value)
            .ok_or_else(|| crate::Error::qi_value("ALMotion.getPosition", value))
    }

    /// Base velocity `[vx, vy, vz, wx, wy, wz]`.
    pub async fn get_robot_velocity(&self) -> Result<Vec<f32>> {
        let value = self.service.call("getRobotVelocity", Value::Unit).await?;
        as_f32s(&value)
            .ok_or_else(|| crate::Error::qi_value("ALMotion.getRobotVelocity", value))
    }

    /// SE(2) robot pose `[x, y, theta]`.
    pub async fn get_robot_position(&self, use_sensors: bool) -> Result<Vec<f32>> {
        let value = self
            .service
            .call("getRobotPosition", use_sensors.into_value())
            .await?;
        as_f32s(&value)
            .ok_or_else(|| crate::Error::qi_value("ALMotion.getRobotPosition", value))
    }

    /// Moves one joint asynchronously.
    pub async fn set_angle(
        &self,
        name: &str,
        angle: f32,
        fraction_max_speed: f32,
    ) -> Result<()> {
        self.service
            .post(
                "setAngles",
                (name.to_owned(), angle, fraction_max_speed).into_value(),
            )
            .await
    }

    pub async fn rest(&self) -> Result<()> {
        self.service.call("rest", Value::Unit).await?;
        Ok(())
    }

    pub async fn wake_up(&self) -> Result<()> {
        self.service.call("wakeUp", Value::Unit).await?;
        Ok(())
    }

    /// Enables or disables external collision protection for walking ("Move").
    pub async fn set_external_collision_protection_enabled(&self, enabled: bool) -> Result<()> {
        self.service
            .call("setExternalCollisionProtectionEnabled", ("Move".to_owned(), enabled).into_value())
            .await?;
        Ok(())
    }

    pub async fn set_orthogonal_security_distance(&self, distance: f32) -> Result<()> {
        self.service
            .call("setOrthogonalSecurityDistance", distance.into_value())
            .await?;
        Ok(())
    }
}

/// ALVideoDevice: camera subscriptions and parameters.
pub struct AlVideoDevice {
    service: Arc<dyn Service>,
}

impl AlVideoDevice {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    /// Returns the subscription handle.
    pub async fn subscribe_camera(
        &self,
        name: &str,
        source: i32,
        resolution: i32,
        color_space: i32,
        fps: i32,
    ) -> Result<String> {
        let value = self
            .service
            .call(
                "subscribeCamera",
                (name.to_owned(), source, resolution, color_space, fps).into_value(),
            )
            .await?;
        as_text(&value)
            .ok_or_else(|| crate::Error::qi_value("ALVideoDevice.subscribeCamera", value))
    }

    /// Raw image wire value: width, height, layers, color space, timestamps,
    /// buffer, camera id and field of view.
    pub async fn get_image_remote(&self, handle: &str) -> Result<Value<'static>> {
        self.service
            .call("getImageRemote", handle.to_owned().into_value())
            .await
    }

    pub async fn unsubscribe(&self, handle: &str) -> Result<()> {
        self.service
            .call("unsubscribe", handle.to_owned().into_value())
            .await?;
        Ok(())
    }

    pub async fn set_camera_parameter(&self, source: i32, parameter: i32, value: i32) -> Result<()> {
        self.service
            .call(
                "setCameraParameter",
                (source, parameter, value).into_value(),
            )
            .await?;
        Ok(())
    }

    pub async fn get_camera_parameter(&self, source: i32, parameter: i32) -> Result<i32> {
        let value = self
            .service
            .call("getCameraParameter", (source, parameter).into_value())
            .await?;
        as_i32(&value)
            .ok_or_else(|| crate::Error::qi_value("ALVideoDevice.getCameraParameter", value))
    }

    pub async fn set_all_parameters_to_default(&self, source: i32) -> Result<()> {
        self.service
            .call("setAllParametersToDefault", source.into_value())
            .await?;
        Ok(())
    }

    /// Maps angular offsets from the optical axis to image coordinates.
    pub async fn image_position_from_angular_position(
        &self,
        source: i32,
        angles: (f32, f32),
    ) -> Result<Vec<f32>> {
        let value = self
            .service
            .call(
                "getImagePositionFromAngularPosition",
                (source, (angles.0, angles.1).into_value()).into_value(),
            )
            .await?;
        as_f32s(&value).ok_or_else(|| {
            crate::Error::qi_value("ALVideoDevice.getImagePositionFromAngularPosition", value)
        })
    }
}

/// ALAudioDevice: microphone streaming.
pub struct AlAudioDevice {
    service: Arc<dyn Service>,
}

impl AlAudioDevice {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    pub async fn set_client_preferences(
        &self,
        client_name: &str,
        rate: i32,
        channels: i32,
    ) -> Result<()> {
        self.service
            .call(
                "setClientPreferences",
                (client_name.to_owned(), rate, channels, 0).into_value(),
            )
            .await?;
        Ok(())
    }

    pub async fn enable_energy_computation(&self) -> Result<()> {
        self.service
            .call("enableEnergyComputation", Value::Unit)
            .await?;
        Ok(())
    }

    /// Starts the audio stream; samples arrive via `processRemote`.
    pub async fn subscribe(&self, client_name: &str) -> Result<()> {
        self.service
            .call("subscribe", client_name.to_owned().into_value())
            .await?;
        Ok(())
    }

    pub async fn unsubscribe(&self, client_name: &str) -> Result<()> {
        self.service
            .call("unsubscribe", client_name.to_owned().into_value())
            .await?;
        Ok(())
    }
}

/// ALRobotModel: robot variant details.
pub struct AlRobotModel {
    service: Arc<dyn Service>,
}

impl AlRobotModel {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    /// Microphone layout code; non-zero selects the 4-channel map.
    pub async fn microphone_config(&self) -> Result<i32> {
        let value = self
            .service
            .call("_getMicrophoneConfig", Value::Unit)
            .await?;
        as_i32(&value)
            .ok_or_else(|| crate::Error::qi_value("ALRobotModel._getMicrophoneConfig", value))
    }
}

/// ALSpeechRecognition: vocabulary-based word spotting.
pub struct AlSpeechRecognition {
    service: Arc<dyn Service>,
}

impl AlSpeechRecognition {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    pub async fn pause(&self, paused: bool) -> Result<()> {
        self.service
            .call("pause", paused.into_value())
            .await?;
        Ok(())
    }

    pub async fn set_language(&self, language: &str) -> Result<()> {
        self.service
            .call("setLanguage", language.to_owned().into_value())
            .await?;
        Ok(())
    }

    pub async fn set_vocabulary(&self, words: Vec<String>, word_spotting: bool) -> Result<()> {
        self.service
            .call("setVocabulary", (words, word_spotting).into_value())
            .await?;
        Ok(())
    }

    pub async fn subscribe(&self, name: &str) -> Result<()> {
        self.service
            .call("subscribe", name.to_owned().into_value())
            .await?;
        Ok(())
    }

    pub async fn unsubscribe(&self, name: &str) -> Result<()> {
        self.service
            .call("unsubscribe", name.to_owned().into_value())
            .await?;
        Ok(())
    }
}

/// ALTextToSpeech: plain speech synthesis.
pub struct AlTextToSpeech {
    service: Arc<dyn Service>,
}

impl AlTextToSpeech {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    pub async fn set_language(&self, language: &str) -> Result<()> {
        self.service
            .call("setLanguage", language.to_owned().into_value())
            .await?;
        Ok(())
    }

    /// Says `text` asynchronously.
    pub async fn say(&self, text: &str) -> Result<()> {
        self.service.post("say", text.to_owned().into_value()).await
    }

    pub async fn get_parameter(&self, name: &str) -> Result<f32> {
        let value = self
            .service
            .call("getParameter", name.to_owned().into_value())
            .await?;
        as_f32(&value).ok_or_else(|| crate::Error::qi_value("ALTextToSpeech.getParameter", value))
    }

    pub async fn set_parameter(&self, name: &str, value: f32) -> Result<()> {
        self.service
            .post("setParameter", (name.to_owned(), value).into_value())
            .await
    }
}

/// ALAnimatedSpeech: speech with gestures.
pub struct AlAnimatedSpeech {
    service: Arc<dyn Service>,
}

impl AlAnimatedSpeech {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    /// Says `text` asynchronously, animating when the text allows it.
    pub async fn say(&self, text: &str) -> Result<()> {
        self.service.post("say", text.to_owned().into_value()).await
    }
}

/// ALSonar: sonar activation.
pub struct AlSonar {
    service: Arc<dyn Service>,
}

impl AlSonar {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    /// Enables the sonars by subscribing a client.
    pub async fn subscribe(&self, client_name: &str) -> Result<()> {
        self.service
            .call("subscribe", client_name.to_owned().into_value())
            .await?;
        Ok(())
    }

    pub async fn unsubscribe(&self, client_name: &str) -> Result<()> {
        self.service
            .call("unsubscribe", client_name.to_owned().into_value())
            .await?;
        Ok(())
    }
}

/// ALNavigation: free-zone search.
pub struct AlNavigation {
    service: Arc<dyn Service>,
}

impl AlNavigation {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    /// Raw `getFreeZone` result; the world centre is element `[2]`.
    pub async fn get_free_zone(&self, desired_radius: f32, displacement: f32) -> Result<Value<'static>> {
        let value = self
            .service
            .call("getFreeZone", (desired_radius, displacement).into_value())
            .await?;
        Ok(plain(value))
    }
}

/// ALLeds: LED colors and fades.
pub struct AlLeds {
    service: Arc<dyn Service>,
}

impl AlLeds {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    /// Fades `name` to the given color (0.0–1.0 per channel) over `duration`.
    pub async fn fade_rgb(&self, name: &str, red: f32, green: f32, blue: f32, duration: f32) -> Result<()> {
        self.service
            .call(
                "fadeRGB",
                (name.to_owned(), red, green, blue, duration).into_value(),
            )
            .await?;
        Ok(())
    }
}

/// ALBehaviorManager: animations installed as behaviors.
pub struct AlBehaviorManager {
    service: Arc<dyn Service>,
}

impl AlBehaviorManager {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    /// Starts a behavior asynchronously.
    pub async fn start_behavior(&self, path: &str) -> Result<()> {
        self.service
            .post("startBehavior", path.to_owned().into_value())
            .await
    }
}

/// ALBasicAwareness: attention toggling.
pub struct AlBasicAwareness {
    service: Arc<dyn Service>,
}

impl AlBasicAwareness {
    pub fn new(service: Arc<dyn Service>) -> Self {
        Self { service }
    }

    pub async fn set_enabled(&self, enabled: bool) -> Result<()> {
        self.service
            .call("setEnabled", enabled.into_value())
            .await?;
        Ok(())
    }
}

/// All AL* proxies the driver talks to.
pub struct Robot {
    pub memory: AlMemory,
    pub motion: AlMotion,
    pub video: AlVideoDevice,
    pub audio: AlAudioDevice,
    pub robot_model: AlRobotModel,
    pub speech_recognition: AlSpeechRecognition,
    pub text_to_speech: AlTextToSpeech,
    pub animated_speech: AlAnimatedSpeech,
    pub sonar: AlSonar,
    pub navigation: AlNavigation,
    pub leds: AlLeds,
    pub behavior_manager: AlBehaviorManager,
    pub basic_awareness: AlBasicAwareness,
}

impl Robot {
    /// Builds the proxies over a service lookup (`ALMemory`, `ALMotion`, ...).
    pub fn new(lookup: impl Fn(&str) -> Arc<dyn Service>) -> Self {
        Self {
            memory: AlMemory::new(lookup("ALMemory")),
            motion: AlMotion::new(lookup("ALMotion")),
            video: AlVideoDevice::new(lookup("ALVideoDevice")),
            audio: AlAudioDevice::new(lookup("ALAudioDevice")),
            robot_model: AlRobotModel::new(lookup("ALRobotModel")),
            speech_recognition: AlSpeechRecognition::new(lookup("ALSpeechRecognition")),
            text_to_speech: AlTextToSpeech::new(lookup("ALTextToSpeech")),
            animated_speech: AlAnimatedSpeech::new(lookup("ALAnimatedSpeech")),
            sonar: AlSonar::new(lookup("ALSonar")),
            navigation: AlNavigation::new(lookup("ALNavigation")),
            leds: AlLeds::new(lookup("ALLeds")),
            behavior_manager: AlBehaviorManager::new(lookup("ALBehaviorManager")),
            basic_awareness: AlBasicAwareness::new(lookup("ALBasicAwareness")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use async_trait::async_trait;
    use qi::value::Value;
    use std::sync::Mutex;

    #[derive(Default)]
    struct Fake {
        calls: Mutex<Vec<(String, Value<'static>)>>,
        replies: Mutex<std::collections::HashMap<String, Value<'static>>>,
    }

    impl Fake {
        fn replies(&self, replies: &[(&str, Value<'static>)]) {
            let mut map = self.replies.lock().expect("lock");
            for (method, value) in replies {
                map.insert((*method).to_owned(), value.clone());
            }
        }

        fn last_call(&self) -> (String, Value<'static>) {
            self.calls.lock().expect("lock").last().expect("call").clone()
        }
    }

    #[async_trait]
    impl Service for Fake {
        async fn call(&self, method: &str, args: Value<'static>) -> Result<Value<'static>> {
            self.calls
                .lock()
                .expect("lock")
                .push((method.to_owned(), args.clone()));
            Ok(self
                .replies
                .lock()
                .expect("lock")
                .get(method)
                .cloned()
                .unwrap_or(Value::Unit))
        }

        async fn post(&self, method: &str, args: Value<'static>) -> Result<()> {
            self.calls
                .lock()
                .expect("lock")
                .push((method.to_owned(), args));
            Ok(())
        }
    }

    fn fake() -> Arc<Fake> {
        Arc::new(Fake::default())
    }

    #[tokio::test]
    async fn memory_event_subscription_shape() {
        let fake = fake();
        let memory = AlMemory::new(fake.clone());
        memory
            .subscribe_to_event("FrontTactilTouched", "ROS-DriverFrontTactilTouched", "touchCallback")
            .await
            .expect("subscribe");
        let (method, args) = fake.last_call();
        assert_eq!(method, "subscribeToEvent");
        assert_eq!(
            args,
            (
                "FrontTactilTouched".to_owned(),
                "ROS-DriverFrontTactilTouched".to_owned(),
                "touchCallback".to_owned()
            )
                .into_value()
        );
    }

    #[tokio::test]
    async fn motion_calls_carry_floats_and_frames() {
        let fake = fake();
        fake.replies(&[
            ("getAngles", vec![0.1f32, -0.2].into_value()),
            ("getPosition", vec![1.0f32, 2.0, 3.0, 0.0, 0.0, 0.5].into_value()),
        ]);
        let motion = AlMotion::new(fake.clone());

        motion.move_to(1.0, -1.0, 0.3).await.expect("moveTo");
        let (method, args) = fake.last_call();
        assert_eq!(method, "moveTo");
        assert_eq!(args, (1.0f32, -1.0f32, 0.3f32).into_value());

        let angles = motion.get_angles("Body", true).await.expect("getAngles");
        assert_eq!(angles, vec![0.1, -0.2]);

        let pose = motion
            .get_position("Torso", ReferenceFrame::World, true)
            .await
            .expect("getPosition");
        assert_eq!(pose.len(), 6);
        let (_, args) = fake.last_call();
        assert_eq!(args, ("Torso".to_owned(), 1i32, true).into_value());
    }

    #[tokio::test]
    async fn motion_rejects_non_numeric_replies() {
        let fake = fake();
        fake.replies(&[("getRobotVelocity", "nope".to_owned().into_value())]);
        let motion = AlMotion::new(fake);
        assert!(motion.get_robot_velocity().await.is_err());
    }

    #[tokio::test]
    async fn video_subscription_and_image_fetch() {
        let fake = fake();
        fake.replies(&[
            ("subscribeCamera", "handle-1".to_owned().into_value()),
            ("getImageRemote", Value::Unit),
        ]);
        let video = AlVideoDevice::new(fake.clone());

        let handle = video
            .subscribe_camera("front_camera", 0, 1, 11, 10)
            .await
            .expect("subscribeCamera");
        assert_eq!(handle, "handle-1");
        let (_, args) = fake.last_call();
        assert_eq!(
            args,
            ("front_camera".to_owned(), 0i32, 1i32, 11i32, 10i32).into_value()
        );

        video.get_image_remote(&handle).await.expect("getImageRemote");
        video.unsubscribe(&handle).await.expect("unsubscribe");
        assert_eq!(fake.last_call().0, "unsubscribe");
    }

    #[tokio::test]
    async fn speech_parameter_calls() {
        let fake = fake();
        fake.replies(&[("getParameter", 1.17f32.into_value())]);
        let tts = AlTextToSpeech::new(fake.clone());

        tts.set_parameter("pitchShift", 1.17).await.expect("setParameter");
        let (method, args) = fake.last_call();
        assert_eq!(method, "setParameter");
        assert_eq!(args, ("pitchShift".to_owned(), 1.17f32).into_value());

        assert_eq!(tts.get_parameter("pitchShift").await.expect("get"), 1.17);
    }

    #[tokio::test]
    async fn robot_lookup_resolves_every_service_by_name() {
        let fake = fake();
        let robot = Robot::new({
            let fake = Arc::clone(&fake);
            move |_name| fake.clone() as Arc<dyn Service>
        });
        robot.basic_awareness.set_enabled(true).await.expect("setEnabled");
        assert_eq!(fake.last_call().0, "setEnabled");
    }
}
