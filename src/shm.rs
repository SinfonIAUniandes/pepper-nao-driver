//! POSIX shared-memory flags shared with the external SinfonIA stack.
//!
//! Each segment is a one-byte enable flag: the mapper, navigator, depth-to-laser
//! and localizer processes poll these and switch themselves on and off.

use crate::{Error, Result};
use std::ffi::CString;

/// The four shared-memory segments of the SinfonIA stack.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Segment {
    /// Enables head / laser related external processing.
    PepperHead,
    Depth2Laser,
    Localizer,
    Planner,
}

impl Segment {
    pub fn name(self) -> &'static str {
        match self {
            Self::PepperHead => "PepperHeadSharedMemory",
            Self::Depth2Laser => "Depth2LaserSharedMemory",
            Self::Localizer => "PepperLocalizerSharedMemory",
            Self::Planner => "PepperPlannerSharedMemory",
        }
    }
}

/// Handle to one shared-memory segment. Writes only happen after a successful
/// open; a failed open yields an error instead of a lying success.
pub struct SharedMemory {
    segment: Segment,
    raw: *mut u8,
    length: usize,
    descriptor: i32,
}

// The mapping is fixed at open time and only a single byte is written.
unsafe impl Send for SharedMemory {}
unsafe impl Sync for SharedMemory {}

impl SharedMemory {
    pub fn open(segment: Segment) -> Result<Self> {
        let name = CString::new(format!("/{}", segment.name()))
            .map_err(|err| Self::error(segment, err))?;
        // SAFETY: `name` is a valid NUL-terminated string; the returned
        // descriptor is checked before any further use.
        let descriptor = unsafe { libc::shm_open(name.as_ptr(), libc::O_CREAT | libc::O_RDWR, 0o666) };
        if descriptor < 0 {
            return Err(Self::error(segment, std::io::Error::last_os_error()));
        }
        // Fresh segments start empty and are given their one byte; existing
        // ones are left alone (ftruncate rejects a no-op on some platforms).
        let mut stat: libc::stat = unsafe { std::mem::zeroed() };
        let sized = unsafe { libc::fstat(descriptor, &mut stat) } == 0 && stat.st_size > 0;
        if !sized && unsafe { libc::ftruncate(descriptor, 1) } < 0 {
            let detail = std::io::Error::last_os_error();
            unsafe {
                libc::close(descriptor);
            }
            return Err(Self::error(segment, detail));
        }
        // SAFETY: the descriptor refers to a file of one byte, mapped read-write.
        let raw = unsafe { libc::mmap(std::ptr::null_mut(), 1, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, descriptor, 0) };
        if raw == libc::MAP_FAILED {
            let detail = std::io::Error::last_os_error();
            unsafe {
                libc::close(descriptor);
            }
            return Err(Self::error(segment, detail));
        }
        Ok(Self {
            segment,
            raw: raw.cast::<u8>(),
            length: 1,
            descriptor,
        })
    }

    /// Reads the one-byte enable flag back.
    pub fn enabled(&self) -> bool {
        // SAFETY: `raw` points to the single mapped byte.
        (unsafe { std::ptr::read_volatile(self.raw) }) != 0
    }

    /// Writes the one-byte enable flag.
    pub fn set_enabled(&self, enabled: bool) -> Result<()> {
        // SAFETY: `raw` points to the single mapped byte.
        unsafe { std::ptr::write_volatile(self.raw, u8::from(enabled)) };
        if unsafe { libc::msync(self.raw.cast(), self.length, libc::MS_SYNC) } < 0 {
            return Err(Self::error(self.segment, std::io::Error::last_os_error()));
        }
        Ok(())
    }

    fn error(segment: Segment, detail: impl std::fmt::Display) -> Error {
        Error::SharedMemory {
            name: segment.name(),
            detail: detail.to_string(),
        }
    }
}

impl Drop for SharedMemory {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.raw.cast(), self.length);
            libc::close(self.descriptor);
        }
    }
}

/// The four segments of one driver instance.
pub struct SharedMemories {
    head: SharedMemory,
    depth_to_laser: SharedMemory,
    localizer: SharedMemory,
    planner: SharedMemory,
}

impl SharedMemories {
    /// Opens all four segments; fails if any of them cannot be opened.
    pub fn open() -> Result<Self> {
        Ok(Self {
            head: SharedMemory::open(Segment::PepperHead)?,
            depth_to_laser: SharedMemory::open(Segment::Depth2Laser)?,
            localizer: SharedMemory::open(Segment::Localizer)?,
            planner: SharedMemory::open(Segment::Planner)?,
        })
    }

    pub fn set_enabled(&self, segment: Segment, enabled: bool) -> Result<()> {
        self.segment(segment).set_enabled(enabled)
    }

    pub fn enabled(&self, segment: Segment) -> bool {
        self.segment(segment).enabled()
    }

    fn segment(&self, segment: Segment) -> &SharedMemory {
        match segment {
            Segment::PepperHead => &self.head,
            Segment::Depth2Laser => &self.depth_to_laser,
            Segment::Localizer => &self.localizer,
            Segment::Planner => &self.planner,
        }
    }

    /// Clears every flag, e.g. on shutdown.
    pub fn reset(&self) -> Result<()> {
        for segment in [Segment::PepperHead, Segment::Depth2Laser, Segment::Localizer, Segment::Planner] {
            self.set_enabled(segment, false)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn open_write_and_read_back() {
        let memory = SharedMemory::open(Segment::Planner).expect("open");
        memory.set_enabled(true).expect("write");
        memory.set_enabled(false).expect("write");
    }

    #[test]
    fn shared_memories_reset_clears_every_flag() {
        let memories = SharedMemories::open().expect("open");
        memories.set_enabled(Segment::PepperHead, true).expect("write");
        assert!(memories.enabled(Segment::PepperHead));
        memories.reset().expect("reset");
        assert!(!memories.enabled(Segment::PepperHead));
    }
}
