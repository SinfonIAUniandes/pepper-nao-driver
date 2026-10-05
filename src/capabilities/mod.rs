//! Capabilities: the driver's units of behaviour.
//!
//! Each capability binds one piece of the QI surface (a sensor, an actuator, a
//! bridge) to one or more transport topics. Capabilities are disabled at boot
//! except `special_settings`; control commands turn them on and off.

pub mod animation;
pub mod camera;
pub mod cmd_vel;
pub mod depth_to_laser;
pub mod faces;
pub mod free_zone;
pub mod joints;
pub mod laser;
pub mod leds;
pub mod merged_laser;
pub mod mic;
pub mod mic_localization;
pub mod moveto;
pub mod navigation;
pub mod odom;
pub mod sonar;
pub mod special_settings;
pub mod speech;
pub mod tf;
pub mod touch;

use crate::assets::Assets;
use crate::domain::{CameraConfig, CameraParams, DepthToLaserParams, Message, MicConfig, SpeechParams};
use crate::qi::object::Toolkit;
use crate::qi::Robot;
use crate::shm::SharedMemories;
use crate::transport::{MessageHandler, Transport};
use async_trait::async_trait;
use futures::future::BoxFuture;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Stable capability ids. They double as topic names on the transport.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CapabilityId {
    Tf,
    Odom,
    Laser,
    DepthToLaser,
    MergedLaser,
    FrontCamera,
    BottomCamera,
    DepthCamera,
    FrontCameraFaceDetector,
    BottomCameraFaceDetector,
    Mic,
    MicLocalization,
    Speech,
    CmdVel,
    MoveTo,
    FreeZone,
    NavigationGoal,
    PoseSet,
    NavigationPath,
    PosePub,
    NavigationResult,
    Animation,
    SetAngles,
    Leds,
    SpecialSettings,
    Sonar,
    Touch,
}

impl CapabilityId {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Tf => "tf",
            Self::Odom => "odom",
            Self::Laser => "laser",
            Self::DepthToLaser => "depth_to_laser",
            Self::MergedLaser => "merged_laser",
            Self::FrontCamera => "front_camera",
            Self::BottomCamera => "bottom_camera",
            Self::DepthCamera => "depth_camera",
            Self::FrontCameraFaceDetector => "front_camera_face_detector",
            Self::BottomCameraFaceDetector => "bottom_camera_face_detector",
            Self::Mic => "mic",
            Self::MicLocalization => "miclocalization",
            Self::Speech => "speech",
            Self::CmdVel => "cmd_vel",
            Self::MoveTo => "moveto",
            Self::FreeZone => "free_zone",
            Self::NavigationGoal => "navigation_goal",
            Self::PoseSet => "pose-set",
            Self::NavigationPath => "navigation_path",
            Self::PosePub => "pose-pub",
            Self::NavigationResult => "navigation_result",
            Self::Animation => "animation",
            Self::SetAngles => "set_angles",
            Self::Leds => "leds",
            Self::SpecialSettings => "special_settings",
            Self::Sonar => "sonar",
            Self::Touch => "touch",
        }
    }

    pub fn all() -> &'static [CapabilityId] {
        use CapabilityId::*;
        &[
            Tf, Odom, Laser, DepthToLaser, MergedLaser, FrontCamera, BottomCamera, DepthCamera,
            FrontCameraFaceDetector, BottomCameraFaceDetector, Mic, MicLocalization, Speech,
            CmdVel, MoveTo, FreeZone, NavigationGoal, PoseSet, NavigationPath, PosePub,
            NavigationResult, Animation, SetAngles, Leds, SpecialSettings, Sonar, Touch,
        ]
    }
}

/// Shared services a capability runs against.
#[derive(Clone)]
pub struct Context {
    pub robot: Arc<Robot>,
    pub transport: Arc<dyn Transport>,
    /// The served object: publish gate and event callback slots.
    pub toolkit: Arc<Toolkit>,
    pub shm: Arc<SharedMemories>,
    pub assets: Arc<Assets>,
    /// Streams `odom -> base_link` on `tf` when set.
    pub publish_odom: bool,
}

/// Wraps an async command handler into a transport message handler.
///
/// The handler runs on the driver's runtime; errors are logged and dropped.
/// `extract` picks the command out of the message and ignores unrelated ones.
pub fn message_handler<T, E, F>(extract: E, handle: F) -> MessageHandler
where
    T: Send + 'static,
    E: Fn(&Message) -> Option<T> + Send + Sync + 'static,
    F: Fn(T) -> BoxFuture<'static, crate::Result<()>> + Send + Sync + 'static,
{
    let runtime = tokio::runtime::Handle::current();
    let handle = Arc::new(handle);
    Arc::new(move |message| {
        if let Some(value) = extract(&message) {
            let handle = Arc::clone(&handle);
            runtime.spawn(async move {
                if let Err(err) = handle(value).await {
                    tracing::warn!(error = %err, "command failed");
                }
            });
        }
    })
}

/// Parameter changes pushed by control commands.
#[derive(Clone, Debug, PartialEq)]
pub enum Configuration {
    /// Streaming setup and register values of a camera.
    Camera(CameraConfig, CameraParams),
    /// Register values only (`set_parameters`).
    CameraParams(CameraParams),
    /// Request for the register values flagged in the payload.
    ReadCameraParams(CameraParams),
    Mic(MicConfig),
    Speech(SpeechParams),
    /// Watchdog timeout of `cmd_vel`, in seconds.
    CmdVel(f32),
    ReadSpeechParams,
    ResetSpeechParams,
    DepthToLaser(DepthToLaserParams),
}

/// Outcome of a [`Capability::configure`] call.
#[derive(Clone, Debug, PartialEq)]
pub enum ConfigurationResult {
    None,
    CameraParams(CameraParams),
    SpeechParams(SpeechParams),
}

/// One unit of driver behaviour.
#[async_trait]
pub trait Capability: Send + Sync {
    fn id(&self) -> CapabilityId;

    /// Ticking period in Hz; `None` for capabilities driven by bus inputs.
    fn period(&self) -> Option<f32> {
        None
    }

    /// Called when the capability is enabled: QI and bus subscriptions.
    async fn enable(&self, ctx: &Context) -> crate::Result<()>;

    /// Called when disabled; must undo [`Capability::enable`] completely.
    async fn disable(&self, ctx: &Context) -> crate::Result<()>;

    /// One periodic cycle; only called while enabled with bus consumers.
    async fn tick(&self, ctx: &Context) -> crate::Result<()> {
        let _ = ctx;
        Ok(())
    }

    /// Applies a parameter change.
    async fn configure(
        &self,
        ctx: &Context,
        configuration: Configuration,
    ) -> crate::Result<ConfigurationResult> {
        let _ = (ctx, configuration);
        Ok(ConfigurationResult::None)
    }
}

/// The full capability set, as assembled by the driver binary.
pub fn default_capabilities() -> Vec<Arc<dyn Capability>> {
    vec![
        Arc::new(tf::Tf::default()),
        Arc::new(odom::Odom),
        Arc::new(laser::Laser),
        Arc::new(depth_to_laser::DepthToLaser::new()),
        Arc::new(merged_laser::MergedLaser),
        Arc::new(camera::Camera::new(crate::domain::CameraId::Front)),
        Arc::new(camera::Camera::new(crate::domain::CameraId::Bottom)),
        Arc::new(camera::Camera::new(crate::domain::CameraId::Depth)),
        Arc::new(faces::FaceDetector::new(crate::domain::CameraId::Front)),
        Arc::new(faces::FaceDetector::new(crate::domain::CameraId::Bottom)),
        Arc::new(mic::Mic::new()),
        Arc::new(mic_localization::MicLocalization),
        Arc::new(speech::Speech::new()),
        Arc::new(cmd_vel::CmdVel::new()),
        Arc::new(moveto::MoveTo::new()),
        Arc::new(free_zone::FreeZone::new()),
        Arc::new(navigation::NavigationGoal::new()),
        Arc::new(navigation::PoseSet::new()),
        Arc::new(navigation::NavigationPath),
        Arc::new(navigation::PosePub),
        Arc::new(navigation::NavigationResult),
        Arc::new(animation::Animation::new()),
        Arc::new(joints::SetAngles::new()),
        Arc::new(leds::Leds::new()),
        Arc::new(special_settings::SpecialSettings::new()),
        Arc::new(sonar::Sonar::new()),
        Arc::new(touch::Touch),
    ]
}

/// Tracks which capabilities are enabled.
#[derive(Default)]
pub struct Registry {
    states: Mutex<HashMap<CapabilityId, bool>>,
}

impl Registry {
    pub fn is_enabled(&self, id: CapabilityId) -> bool {
        self.states
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .get(&id)
            .copied()
            .unwrap_or(false)
    }

    pub fn set(&self, id: CapabilityId, enabled: bool) {
        self.states
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .insert(id, enabled);
    }
}

#[cfg(test)]
pub(crate) mod support {
    //! Shared harness for capability tests.

    use super::{CapabilityId, Context};
    use crate::qi::object::Toolkit;
    use crate::qi::value::plain;
    use crate::qi::{Robot, Service};
    use crate::transport::MemoryTransport;
    use crate::Result;
    use async_trait::async_trait;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use qi::value::Value;
    use std::collections::HashMap;
    use std::sync::{Arc, Mutex};

    /// Service double recording every call and returning scripted replies.
    #[derive(Default)]
    pub struct FakeService {
        pub calls: Mutex<Vec<(String, Value<'static>)>>,
        replies: Mutex<HashMap<String, Value<'static>>>,
        keyed_replies: Mutex<HashMap<(String, String), Value<'static>>>,
    }

    impl FakeService {
        pub fn script(&self, method: &str, reply: Value<'static>) {
            self.replies
                .lock()
                .expect("lock")
                .insert(method.to_owned(), reply);
        }

        /// Scripts the reply of `method` when its first argument is `argument`.
        pub fn script_for(&self, method: &str, argument: &str, reply: Value<'static>) {
            self.keyed_replies
                .lock()
                .expect("lock")
                .insert((method.to_owned(), argument.to_owned()), reply);
        }

        /// All recorded calls to `method`.
        pub fn calls_to(&self, method: &str) -> Vec<Value<'static>> {
            self.calls
                .lock()
                .expect("lock")
                .iter()
                .filter(|(name, _)| name == method)
                .map(|(_, args)| args.clone())
                .collect()
        }

    }

    #[async_trait]
    impl Service for FakeService {
        async fn call(&self, method: &str, args: Value<'static>) -> Result<Value<'static>> {
            self.calls
                .lock()
                .expect("lock")
                .push((method.to_owned(), args.clone()));
            Ok(self.reply_for(method, &args))
        }

        async fn post(&self, method: &str, args: Value<'static>) -> Result<()> {
            self.calls
                .lock()
                .expect("lock")
                .push((method.to_owned(), args));
            Ok(())
        }
    }

    impl FakeService {
        fn reply_for(&self, method: &str, args: &Value<'static>) -> Value<'static> {
            let argument = match args {
                Value::String(text) => text.as_str().map(str::to_owned),
                Value::Tuple(elements) => elements.first().and_then(|value| match value {
                    Value::String(text) => text.as_str().map(str::to_owned),
                    _ => None,
                }),
                _ => None,
            };
            if let Some(argument) = argument
                && let Some(reply) = self
                    .keyed_replies
                    .lock()
                    .expect("lock")
                    .get(&(method.to_owned(), argument))
            {
                return plain(reply.clone());
            }
            self.replies
                .lock()
                .expect("lock")
                .get(method)
                .cloned()
                .map(plain)
                .unwrap_or(Value::Unit)
        }
    }

    /// A named collection of service doubles.
    #[derive(Clone, Default)]
    pub struct FakeRobot {
        services: Arc<Mutex<HashMap<String, Arc<FakeService>>>>,
    }

    impl FakeRobot {
        pub fn service(&self, name: &str) -> Arc<FakeService> {
            self.services
                .lock()
                .expect("lock")
                .entry(name.to_owned())
                .or_default()
                .clone()
        }

        pub fn lookup(&self, name: &str) -> Arc<dyn Service> {
            self.service(name)
        }
    }

    /// Capability double recording its lifecycle.
    pub struct Recording {
        id: CapabilityId,
        period: Option<f32>,
        configure_result: super::ConfigurationResult,
        pub enables: AtomicUsize,
        pub disables: AtomicUsize,
        pub ticks: AtomicUsize,
        pub configures: AtomicUsize,
    }

    impl Recording {
        pub fn new(id: CapabilityId, period: Option<f32>) -> Arc<Self> {
            Self::build(id, period, super::ConfigurationResult::None)
        }

        /// A double whose `configure` answers `result`.
        pub fn with_configure_result(
            id: CapabilityId,
            result: super::ConfigurationResult,
        ) -> Arc<Self> {
            Self::build(id, None, result)
        }

        fn build(
            id: CapabilityId,
            period: Option<f32>,
            configure_result: super::ConfigurationResult,
        ) -> Arc<Self> {
            Arc::new(Self {
                id,
                period,
                configure_result,
                enables: AtomicUsize::new(0),
                disables: AtomicUsize::new(0),
                ticks: AtomicUsize::new(0),
                configures: AtomicUsize::new(0),
            })
        }
    }

    #[async_trait]
    impl crate::capabilities::Capability for Recording {
        fn id(&self) -> CapabilityId {
            self.id
        }

        fn period(&self) -> Option<f32> {
            self.period
        }

        async fn enable(&self, _ctx: &Context) -> Result<()> {
            self.enables.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }

        async fn disable(&self, _ctx: &Context) -> Result<()> {
            self.disables.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }

        async fn tick(&self, ctx: &Context) -> Result<()> {
            self.ticks.fetch_add(1, Ordering::Relaxed);
            ctx.transport.publish(
                self.id.as_str(),
                crate::domain::Message::Text("tick".to_owned()),
            )
        }

        async fn configure(
            &self,
            _ctx: &Context,
            _configuration: super::Configuration,
        ) -> Result<super::ConfigurationResult> {
            self.configures.fetch_add(1, Ordering::Relaxed);
            Ok(self.configure_result.clone())
        }
    }

    /// Serializes tests that assert on the process-wide shared-memory flags.
    pub async fn shm_lock() -> tokio::sync::MutexGuard<'static, ()> {
        static LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());
        LOCK.lock().await
    }

    /// Capability test harness: a context wired to fakes.
    pub struct Harness {
        pub ctx: Context,
        pub fakes: FakeRobot,
        pub transport: Arc<MemoryTransport>,
    }

    impl Default for Harness {
        fn default() -> Self {
            let fakes = FakeRobot::default();
            let robot = Arc::new(Robot::new({
                let fakes = fakes.clone();
                move |name| fakes.lookup(name)
            }));
            let transport = Arc::new(MemoryTransport::new());
            let ctx = Context {
                robot,
                transport: transport.clone(),
                toolkit: Arc::new(Toolkit::new("test")),
                shm: Arc::new(crate::shm::SharedMemories::open().expect("shm")),
                assets: Arc::new(crate::assets::Assets::load(None).expect("assets")),
                publish_odom: false,
            };
            Self {
                ctx,
                fakes,
                transport,
            }
        }
    }
}
