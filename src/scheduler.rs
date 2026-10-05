//! Periodic jobs: one task per scheduled capability.
//!
//! A job ticks at its frequency while its gate passes; the gate combines the
//! capability enable state and the transport consumer report. Frequencies are
//! changeable at runtime (`custom` control commands).

use crate::transport::{Timer, Transport};
use futures::StreamExt;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Gate deciding whether a job does QI work on a tick.
pub type Gate = Arc<dyn Fn() -> bool + Send + Sync>;

/// The periodic work of one capability.
pub type Work =
    Arc<dyn Fn() -> futures::future::BoxFuture<'static, crate::Result<()>> + Send + Sync>;

#[derive(Clone)]
struct Frequency(Arc<Mutex<f32>>);

impl Frequency {
    fn new(hz: f32) -> Self {
        Self(Arc::new(Mutex::new(hz)))
    }

    fn set(&self, hz: f32) {
        *self.0.lock().unwrap_or_else(|err| err.into_inner()) = hz;
    }
}

struct Job {
    frequency: Frequency,
    task: tokio::task::JoinHandle<()>,
}

/// Runs the periodic work of all scheduled capabilities.
#[derive(Default)]
pub struct Scheduler {
    jobs: Mutex<HashMap<String, Job>>,
    frequencies: Mutex<HashMap<String, f32>>,
}

impl Scheduler {
    /// Spawns the periodic work of `id` at `hz` ticks per second, or at the
    /// last rate set for `id`.
    pub fn spawn(&self, transport: Arc<dyn Transport>, id: &str, hz: f32, gate: Gate, work: Work) {
        let hz = self.frequency(id).unwrap_or(hz);
        self.set_frequency(id, hz);
        let frequency = Frequency::new(hz);
        let task = tokio::spawn(run_job(transport, Arc::clone(&frequency.0), gate, work));
        self.jobs
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .insert(id.to_owned(), Job { frequency, task });
    }

    /// Changes the tick rate of `id`, now or when its job spawns.
    pub fn set_frequency(&self, id: &str, hz: f32) {
        self.frequencies
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .insert(id.to_owned(), hz);
        if let Some(job) = self
            .jobs
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .get(id)
        {
            job.frequency.set(hz);
        }
    }

    pub fn frequency(&self, id: &str) -> Option<f32> {
        self.frequencies
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .get(id)
            .copied()
    }

    /// Stops all periodic work.
    pub fn shutdown(&self) {
        for job in self
            .jobs
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .values()
        {
            job.task.abort();
        }
    }
}

async fn run_job(
    transport: Arc<dyn Transport>,
    frequency: Arc<Mutex<f32>>,
    gate: Gate,
    work: Work,
) {
    let mut rate = 0.0;
    let mut timer: Option<Timer> = None;
    loop {
        let wanted = *frequency.lock().unwrap_or_else(|err| err.into_inner());
        if timer.is_none() || wanted != rate {
            rate = wanted;
            timer = Some(transport.timer(rate));
        }
        let Some(mut stream) = timer.take() else {
            return;
        };
        if rate <= 0.0 {
            // Parked job: no work while the frequency is zero.
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            timer = Some(stream);
            continue;
        }
        if stream.next().await.is_none() {
            return;
        }
        timer = Some(stream);
        if gate()
            && let Err(err) = work().await
        {
            tracing::warn!(error = %err, "cycle skipped on error");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::MemoryTransport;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[tokio::test(start_paused = true)]
    async fn jobs_tick_according_to_their_frequency() {
        let scheduler = Scheduler::default();
        let transport = Arc::new(MemoryTransport::new());
        let ticks = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&ticks);
        scheduler.spawn(
            transport.clone(),
            "laser",
            10.0,
            Arc::new(|| true),
            Arc::new(move || {
                let counter = Arc::clone(&counter);
                Box::pin(async move {
                    counter.fetch_add(1, Ordering::Relaxed);
                    Ok(())
                })
            }),
        );

        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
        scheduler.shutdown();
        assert_eq!(ticks.load(Ordering::Relaxed), 10);
    }

    #[tokio::test(start_paused = true)]
    async fn gated_jobs_do_no_work() {
        let scheduler = Scheduler::default();
        let transport = Arc::new(MemoryTransport::new());
        let ticks = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&ticks);
        scheduler.spawn(
            transport.clone(),
            "laser",
            10.0,
            Arc::new(|| false),
            Arc::new(move || {
                let counter = Arc::clone(&counter);
                Box::pin(async move {
                    counter.fetch_add(1, Ordering::Relaxed);
                    Ok(())
                })
            }),
        );

        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        scheduler.shutdown();
        assert_eq!(ticks.load(Ordering::Relaxed), 0);
    }

    #[tokio::test(start_paused = true)]
    async fn frequency_changes_take_effect() {
        let scheduler = Scheduler::default();
        let transport = Arc::new(MemoryTransport::new());
        let ticks = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&ticks);
        scheduler.spawn(
            transport.clone(),
            "odom",
            1.0,
            Arc::new(|| true),
            Arc::new(move || {
                let counter = Arc::clone(&counter);
                Box::pin(async move {
                    counter.fetch_add(1, Ordering::Relaxed);
                    Ok(())
                })
            }),
        );
        assert_eq!(scheduler.frequency("odom"), Some(1.0));
        scheduler.set_frequency("odom", 100.0);

        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        scheduler.shutdown();
        assert!(ticks.load(Ordering::Relaxed) >= 5);
    }

    #[tokio::test(start_paused = true)]
    async fn work_errors_skip_the_cycle_but_not_the_job() {
        let scheduler = Scheduler::default();
        let transport = Arc::new(MemoryTransport::new());
        let ticks = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&ticks);
        scheduler.spawn(
            transport.clone(),
            "laser",
            10.0,
            Arc::new(|| true),
            Arc::new(move || {
                let counter = Arc::clone(&counter);
                Box::pin(async move {
                    counter.fetch_add(1, Ordering::Relaxed);
                    Err(crate::Error::invalid("test", "boom"))
                })
            }),
        );

        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
        scheduler.shutdown();
        assert!(ticks.load(Ordering::Relaxed) >= 2, "the job keeps ticking");
    }
}
