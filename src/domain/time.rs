//! Time stamps as carried by domain messages.

/// Point in time, seconds and nanoseconds since the epoch.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Timestamp {
    pub sec: u64,
    pub nanosec: u32,
}

impl Timestamp {
    pub fn new(sec: u64, nanosec: u32) -> Self {
        Self {
            sec,
            nanosec: nanosec.min(999_999_999),
        }
    }

    pub fn as_nanos(self) -> u128 {
        u128::from(self.sec) * 1_000_000_000 + u128::from(self.nanosec)
    }

    /// Duration in seconds since `earlier`, as a float.
    pub fn seconds_since(self, earlier: Timestamp) -> f32 {
        (self.as_nanos() - earlier.as_nanos()) as f32 / 1e9
    }
}

impl From<std::time::SystemTime> for Timestamp {
    fn from(time: std::time::SystemTime) -> Self {
        let since_epoch = time
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default();
        Self::new(since_epoch.as_secs(), since_epoch.subsec_nanos())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nanosec_is_clamped() {
        assert_eq!(Timestamp::new(1, 2_000_000_000).nanosec, 999_999_999);
    }

    #[test]
    fn seconds_since_precise_enough_for_watchdogs() {
        let a = Timestamp::new(10, 0);
        let b = Timestamp::new(10, 500_000_000);
        assert!((b.seconds_since(a) - 0.5).abs() < 1e-6);
    }
}
