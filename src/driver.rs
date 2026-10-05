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

//! Driver lifecycle: connect to the robot, run the capabilities, serve the
//! control RPCs.

use crate::Result;
use crate::assets::Assets;
use crate::capabilities::{
    Capability, CapabilityId, Configuration, ConfigurationResult, Context, Registry,
};
use crate::domain::{ControlRequest, ControlResponse, SpeechRecognitionRequest};
use crate::qi::keys::{self, AUDIO_CALLBACK_SERVICE, callback_service_name};
use crate::qi::object::Toolkit;
use crate::qi::services::Robot;
use crate::qi::{ObjectService, Service};
use crate::shm::SharedMemories;
use crate::transport::{Subscription, Transport};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// How long the word-spotting RPC waits for a match.
const RECOGNITION_TIMEOUT: Duration = Duration::from_secs(5);

/// How to reach the robot.
#[derive(Debug, Clone)]
pub struct Options {
    /// Address of the robot's QI space, e.g. `tcp://pepper.local:9559`.
    pub qi_address: qi::Address,
    /// Instance prefix reported by `_whoWillWin`; must not be empty.
    pub instance_prefix: String,
    /// Streams `odom -> base_link` on `tf`.
    pub publish_odom: bool,
    /// Base directory holding `share/`, e.g. a QI package prefix.
    pub assets_base: Option<PathBuf>,
}

/// Everything the driver needs once the QI side is up.
pub struct Connected {
    pub ctx: Context,
    /// The QI node kept alive so the registered objects stay reachable.
    _node: Box<dyn std::any::Any + Send>,
}

impl Connected {
    /// Unregisters the callback objects served to the robot.
    ///
    /// `unimplemented`: libqi-rs registers services only while starting a node,
    /// so callback objects live as long as the process does. The disable paths
    /// unsubscribe the ALMemory events instead, which stops all traffic.
    pub fn unregister_callback_objects(&self) {
        unimplemented!(
            "libqi-rs has no runtime service (un)registration; callback objects live for the process lifetime"
        )
    }
}

/// The names under which the served object is registered: the `robot_toolkit`
/// handshake plus one callback object per ALMemory event.
fn served_object_names() -> Vec<String> {
    let mut names = vec!["robot_toolkit".to_owned()];
    for (key, _) in keys::TOUCH_EVENTS {
        names.push(callback_service_name(key));
    }
    for key in [
        keys::WORD_RECOGNIZED,
        keys::SOUND_LOCATED,
        keys::FACE_DETECTED,
        keys::PLANNER_RESULT,
    ] {
        names.push(callback_service_name(key));
    }
    names.push(AUDIO_CALLBACK_SERVICE.to_owned());
    names
}

/// Connects to the robot's QI space and prepares the driver context.
pub async fn connect(options: &Options, transport: Arc<dyn Transport>) -> Result<Connected> {
    if options.instance_prefix.is_empty() {
        return Err(crate::Error::invalid(
            "instance prefix",
            "must not be empty",
        ));
    }
    let toolkit = Arc::new(Toolkit::new(options.instance_prefix.clone()));
    let mut builder = qi::node::Builder::new();
    for name in served_object_names() {
        builder = builder.add_service(name, Toolkit::clone(&toolkit));
    }
    let node = builder
        .connect_to_space(options.qi_address, None)
        .start()
        .await?;

    let mut services: HashMap<&str, Arc<dyn Service>> = HashMap::new();
    for name in Robot::SERVICE_NAMES {
        let client = node.service(name).await?;
        services.insert(name, Arc::new(ObjectService::new(client)));
    }
    let robot = Arc::new(Robot::new(|name| {
        services
            .get(name)
            .cloned()
            .unwrap_or_else(|| unreachable!("all robot services are resolved: {name}"))
    }));

    let ctx = Context {
        robot,
        transport,
        toolkit,
        shm: Arc::new(SharedMemories::open()?),
        assets: Arc::new(Assets::load(options.assets_base.as_deref())?),
        publish_odom: options.publish_odom,
    };
    Ok(Connected {
        ctx,
        _node: Box::new(node),
    })
}

struct Inner {
    ctx: Context,
    capabilities: HashMap<CapabilityId, Arc<dyn Capability>>,
    registry: Arc<Registry>,
    scheduler: crate::scheduler::Scheduler,
    rpc_subscriptions: Mutex<Vec<Subscription>>,
}

/// Runs the capability set against one robot and one transport.
#[derive(Clone)]
pub struct Driver(Arc<Inner>);

impl Driver {
    /// Builds a driver from a connected robot and the capability set.
    pub fn new(ctx: Context, capabilities: Vec<Arc<dyn Capability>>) -> Self {
        Self(Arc::new(Inner {
            capabilities: capabilities
                .into_iter()
                .map(|capability| (capability.id(), capability))
                .collect(),
            registry: Arc::new(Registry::default()),
            scheduler: crate::scheduler::Scheduler::default(),
            rpc_subscriptions: Mutex::new(Vec::new()),
            ctx,
        }))
    }

    /// Starts the scheduler and the control RPCs, then enables the only
    /// capability that is on at boot.
    pub async fn start(&self) -> Result<()> {
        for capability in self.0.capabilities.values() {
            let Some(hz) = capability.period() else {
                continue;
            };
            self.0.scheduler.spawn(
                self.0.ctx.transport.clone(),
                capability.id().as_str(),
                hz,
                self.gate(capability.id()),
                self.work(capability.clone()),
            );
        }
        self.register_rpcs();
        self.0.ctx.toolkit.start_publishing();
        self.enable(CapabilityId::SpecialSettings).await
    }

    /// Stops the periodic work and the control RPCs.
    pub fn stop(&self) {
        self.0.scheduler.shutdown();
        self.0
            .rpc_subscriptions
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .clear();
    }

    /// Full shutdown: nothing enabled, robot stopped, shared memory cleared.
    pub async fn shutdown(&self) -> Result<()> {
        self.stop();
        for id in CapabilityId::all() {
            if self.is_enabled(*id) {
                self.disable(*id).await?;
            }
        }
        self.0.ctx.robot.motion.move_(0.0, 0.0, 0.0).await?;
        self.0.ctx.shm.reset()
    }

    pub fn context(&self) -> &Context {
        &self.0.ctx
    }

    pub fn is_enabled(&self, id: CapabilityId) -> bool {
        self.0.registry.is_enabled(id)
    }

    pub async fn enable(&self, id: CapabilityId) -> Result<()> {
        self.capability(id)?.enable(&self.0.ctx).await?;
        self.0.registry.set(id, true);
        Ok(())
    }

    pub async fn disable(&self, id: CapabilityId) -> Result<()> {
        self.capability(id)?.disable(&self.0.ctx).await?;
        self.0.registry.set(id, false);
        Ok(())
    }

    pub fn set_frequency(&self, id: CapabilityId, hz: f32) {
        self.0.scheduler.set_frequency(id.as_str(), hz);
    }

    pub fn frequency(&self, id: CapabilityId) -> Option<f32> {
        self.0.scheduler.frequency(id.as_str())
    }

    pub async fn configure(
        &self,
        id: CapabilityId,
        configuration: Configuration,
    ) -> Result<ConfigurationResult> {
        self.capability(id)?
            .configure(&self.0.ctx, configuration)
            .await
    }

    /// Serves one control RPC.
    pub async fn handle_control(&self, request: ControlRequest) -> ControlResponse {
        crate::control::dispatch(self, request).await
    }

    /// Word spotting (spec 4.11): sets the vocabulary, waits for the best match
    /// above the threshold and tears everything down again.
    pub async fn recognize_speech(&self, request: SpeechRecognitionRequest) -> String {
        match self.recognize(&request).await {
            Ok(word) => word,
            Err(err) => {
                tracing::warn!(error = %err, "speech recognition failed");
                "NONE".to_owned()
            }
        }
    }

    async fn recognize(&self, request: &SpeechRecognitionRequest) -> Result<String> {
        let robot = &self.0.ctx.robot;
        let recognition = &robot.speech_recognition;
        recognition.pause(true).await?;
        recognition.set_language("English").await?;
        recognition
            .set_vocabulary(request.words.clone(), true)
            .await?;
        recognition
            .subscribe(keys::SPEECH_RECOGNITION_CLIENT)
            .await?;
        robot
            .memory
            .subscribe_to_event(
                keys::WORD_RECOGNIZED,
                &callback_service_name(keys::WORD_RECOGNIZED),
                "wordRecognizedCallback",
            )
            .await?;

        let (sender, receiver) = tokio::sync::oneshot::channel();
        let sender = Mutex::new(Some(sender));
        self.0.ctx.toolkit.word_recognized().set(move |words| {
            if let Some(sender) = sender.lock().unwrap_or_else(|err| err.into_inner()).take() {
                let _ = sender.send(words);
            }
        });
        let outcome = tokio::time::timeout(RECOGNITION_TIMEOUT, receiver).await;
        self.0.ctx.toolkit.word_recognized().clear();

        // Teardown runs whatever the outcome was.
        let _ = robot
            .memory
            .unsubscribe_to_event(
                keys::WORD_RECOGNIZED,
                &callback_service_name(keys::WORD_RECOGNIZED),
            )
            .await;
        let _ = recognition
            .unsubscribe(keys::SPEECH_RECOGNITION_CLIENT)
            .await;
        let _ = recognition.pause(false).await;

        Ok(match outcome {
            Ok(Ok(words)) => words
                .best_above(request.threshold)
                .unwrap_or("NONE")
                .to_owned(),
            _ => "NONE".to_owned(),
        })
    }

    fn capability(&self, id: CapabilityId) -> Result<&Arc<dyn Capability>> {
        self.0.capabilities.get(&id).ok_or_else(|| {
            crate::Error::invalid("capability", format!("unknown id {}", id.as_str()))
        })
    }

    /// Periodic work runs only when enabled, publishing is on and the bus has
    /// consumers.
    fn gate(&self, id: CapabilityId) -> crate::scheduler::Gate {
        let registry = Arc::clone(&self.0.registry);
        let transport = Arc::clone(&self.0.ctx.transport);
        let toolkit = self.0.ctx.toolkit.clone();
        Arc::new(move || {
            registry.is_enabled(id)
                && toolkit.publish_enabled()
                && transport.has_consumers(id.as_str())
        })
    }

    fn work(&self, capability: Arc<dyn Capability>) -> crate::scheduler::Work {
        let ctx = &self.0.ctx;
        let robot = Arc::clone(&ctx.robot);
        let transport = Arc::clone(&ctx.transport);
        let toolkit = ctx.toolkit.clone();
        let shm = Arc::clone(&ctx.shm);
        let assets = Arc::clone(&ctx.assets);
        let publish_odom = ctx.publish_odom;
        Arc::new(move || {
            let capability = Arc::clone(&capability);
            let ctx = Context {
                robot: Arc::clone(&robot),
                transport: Arc::clone(&transport),
                toolkit: toolkit.clone(),
                shm: Arc::clone(&shm),
                assets: Arc::clone(&assets),
                publish_odom,
            };
            Box::pin(async move { capability.tick(&ctx).await })
        })
    }

    fn register_rpcs(&self) {
        let mut subscriptions = self
            .0
            .rpc_subscriptions
            .lock()
            .unwrap_or_else(|err| err.into_inner());
        for name in crate::control::RPC_NAMES {
            let driver = self.clone();
            subscriptions.push(self.0.ctx.transport.register_rpc(
                name,
                Arc::new(move |request| {
                    let driver = driver.clone();
                    Box::pin(async move { driver.handle_control(request).await })
                }),
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::support::{Harness, Recording, shm_lock};
    use crate::capabilities::{Configuration, ConfigurationResult};
    use qi::object::Object;

    #[tokio::test(start_paused = true)]
    async fn start_enables_special_settings_and_serves_rpcs() {
        let harness = Harness::default();
        let special = Recording::new(CapabilityId::SpecialSettings, None);
        let laser = Recording::new(CapabilityId::Laser, Some(10.0));
        let driver = Driver::new(
            harness.ctx.clone(),
            vec![
                special.clone(),
                laser.clone(),
                Recording::new(CapabilityId::Leds, None),
                Recording::new(CapabilityId::Sonar, None),
                Recording::new(CapabilityId::Touch, None),
            ],
        );
        driver.start().await.expect("start");

        assert!(driver.is_enabled(CapabilityId::SpecialSettings));
        assert_eq!(
            special.enables.load(std::sync::atomic::Ordering::Relaxed),
            1
        );

        let response = harness
            .transport
            .call_rpc(
                "misc_tools",
                ControlRequest::Misc(crate::domain::MiscCommand::EnableAll),
            )
            .await;
        assert_eq!(response, Some(ControlResponse::ok()));

        tokio::time::sleep(Duration::from_millis(250)).await;
        assert!(
            laser.ticks.load(std::sync::atomic::Ordering::Relaxed) == 0,
            "laser stays disabled"
        );

        driver.enable(CapabilityId::Laser).await.expect("enable");
        tokio::time::sleep(Duration::from_millis(250)).await;
        assert!(laser.ticks.load(std::sync::atomic::Ordering::Relaxed) >= 2);
        assert!(!harness.transport.published_on("laser").is_empty());

        driver.stop();
    }

    #[tokio::test]
    async fn shutdown_disables_and_zeroes_the_velocity() {
        let _guard = shm_lock().await;
        let harness = Harness::default();
        let special = Recording::new(CapabilityId::SpecialSettings, None);
        let driver = Driver::new(harness.ctx.clone(), vec![special.clone()]);
        driver.start().await.expect("start");
        driver.shutdown().await.expect("shutdown");

        assert!(!driver.is_enabled(CapabilityId::SpecialSettings));
        assert_eq!(
            special.disables.load(std::sync::atomic::Ordering::Relaxed),
            1
        );
        let moves = harness.fakes.service("ALMotion").calls_to("move");
        assert_eq!(moves[0], (0.0f32, 0.0f32, 0.0f32).into_value());
        assert!(!harness.ctx.shm.enabled(crate::shm::Segment::Planner));
    }

    #[tokio::test]
    async fn configure_is_delegated_to_the_capability() {
        let harness = Harness::default();
        let speech = Recording::new(CapabilityId::Speech, None);
        let driver = Driver::new(harness.ctx.clone(), vec![speech.clone()]);
        assert_eq!(
            driver
                .configure(CapabilityId::Speech, Configuration::ReadSpeechParams)
                .await
                .expect("configure"),
            ConfigurationResult::None
        );
        assert_eq!(
            speech.configures.load(std::sync::atomic::Ordering::Relaxed),
            1
        );
        assert!(
            driver
                .configure(CapabilityId::Laser, Configuration::ReadSpeechParams)
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn speech_recognition_returns_the_best_word() {
        let harness = Harness::default();
        let driver = Driver::new(harness.ctx.clone(), vec![]);
        let recognition = harness.fakes.service("ALSpeechRecognition");
        recognition.script("setVocabulary", qi::value::Value::Unit);

        let result = driver.recognize_speech(SpeechRecognitionRequest {
            words: vec!["robot".to_owned(), "stop".to_owned()],
            threshold: 0.5,
        });
        // Deliver the event through the served object, like the robot does.
        let toolkit = harness.ctx.toolkit.clone();
        let deliver = async {
            tokio::time::sleep(Duration::from_millis(10)).await;
            toolkit
                .meta_call(
                    "wordRecognizedCallback".into(),
                    (
                        keys::WORD_RECOGNIZED.to_owned(),
                        crate::qi::value::Raw::new(
                            vec![
                                "<...> robot <...>".to_owned().into_value(),
                                0.8f32.into_value(),
                            ]
                            .into_value(),
                        ),
                        "subscriber".to_owned(),
                    )
                        .into_value(),
                )
                .await
                .expect("callback");
        };
        let (word, _) = tokio::join!(result, deliver);
        assert_eq!(word, "robot");

        // Teardown happened: the event is unsubscribed and the pause lifted.
        let memory = harness.fakes.service("ALMemory");
        assert_eq!(memory.calls_to("unsubscribeToEvent").len(), 1);
        assert_eq!(
            recognition.calls_to("pause").last().expect("pause"),
            &false.into_value()
        );
    }

    /// End-to-end smoke test over the full capability set: control commands
    /// enable their groups and the periodic sensors reach the transport.
    #[tokio::test(start_paused = true)]
    async fn default_set_runs_the_navigation_and_misc_flows() {
        let _guard = shm_lock().await;
        let harness = Harness::default();
        let memory = harness.fakes.service("ALMemory");
        memory.script("getListData", vec![0.0f32; 90].into_value());
        let motion = harness.fakes.service("ALMotion");
        motion.script("getBodyNames", vec!["HeadYaw".to_owned()].into_value());
        motion.script("getAngles", vec![0.0f32].into_value());
        motion.script("getPosition", vec![0.0f32; 6].into_value());
        motion.script("getRobotVelocity", vec![0.0f32; 6].into_value());

        let driver = Driver::new(
            harness.ctx.clone(),
            crate::capabilities::default_capabilities(),
        );
        driver.start().await.expect("start");

        for request in [
            ControlRequest::Navigation(crate::domain::NavigationCommand::EnableAll),
            ControlRequest::Motion(crate::domain::MotionCommand::EnableAll),
            ControlRequest::Misc(crate::domain::MiscCommand::EnableAll),
        ] {
            assert_eq!(driver.handle_control(request).await.result, "ok");
        }
        // A second of virtual time, advanced explicitly so the scheduler jobs
        // run regardless of runtime idleness.
        for _ in 0..50 {
            tokio::time::advance(Duration::from_millis(20)).await;
            tokio::task::yield_now().await;
        }
        for topic in ["tf", "odom", "laser"] {
            assert!(
                !harness.transport.published_on(topic).is_empty(),
                "{topic} never reached the transport"
            );
        }
        assert!(harness.ctx.shm.enabled(crate::shm::Segment::Depth2Laser));

        driver.shutdown().await.expect("shutdown");
    }

    #[tokio::test(start_paused = true)]
    async fn speech_recognition_times_out_with_none() {
        let harness = Harness::default();
        let driver = Driver::new(harness.ctx.clone(), vec![]);
        let word = driver
            .recognize_speech(SpeechRecognitionRequest {
                words: vec!["robot".to_owned()],
                threshold: 0.5,
            })
            .await;
        assert_eq!(word, "NONE");
    }

    use qi::value::IntoValue;
}
