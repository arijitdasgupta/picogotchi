//! Time-based switch debouncing.
//!
//! Mechanical switches bounce for a few milliseconds when they are pressed or
//! released, which the emulator would otherwise see as a burst of presses. A
//! [`Debouncer`] only accepts a new level once it has been stable for a
//! configured window.

/// Debounces a single switch.
///
/// Feed it raw samples with [`Debouncer::update`]; it returns the debounced
/// level. `true` means pressed.
#[derive(Clone, Copy, Debug)]
pub struct Debouncer {
    stable: bool,
    raw_last: bool,
    changed_at_us: u64,
    debounce_us: u64,
}

impl Debouncer {
    /// Create a debouncer that requires `debounce_us` microseconds of a stable
    /// level before accepting a change. The initial level is released.
    pub const fn new(debounce_us: u64) -> Self {
        Self {
            stable: false,
            raw_last: false,
            changed_at_us: 0,
            debounce_us,
        }
    }

    /// Feed a raw sample taken at `now_us` and return the debounced level.
    ///
    /// `now_us` only needs to be a monotonically increasing microsecond clock;
    /// it is compared using wrapping arithmetic so it may also wrap.
    pub fn update(&mut self, raw: bool, now_us: u64) -> bool {
        if raw != self.raw_last {
            // The raw level just changed: (re)start the settling window.
            self.raw_last = raw;
            self.changed_at_us = now_us;
        } else if raw != self.stable
            && now_us.wrapping_sub(self.changed_at_us) >= self.debounce_us
        {
            // The new level has been stable long enough: accept it.
            self.stable = raw;
        }
        self.stable
    }

    /// The current debounced level (`true` = pressed).
    pub const fn level(&self) -> bool {
        self.stable
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WINDOW: u64 = 20_000; // 20 ms

    #[test]
    fn ignores_short_glitches() {
        let mut d = Debouncer::new(WINDOW);
        // A 5 ms blip is not accepted.
        assert!(!d.update(true, 0));
        assert!(!d.update(true, 5_000));
        assert!(!d.update(false, 5_001));
        assert!(!d.level());
    }

    #[test]
    fn accepts_a_stable_press() {
        let mut d = Debouncer::new(WINDOW);
        assert!(!d.update(true, 1_000));
        assert!(!d.update(true, 10_000));
        assert!(d.update(true, 21_000)); // exactly one window after the change
        assert!(d.level());
    }

    #[test]
    fn accepts_a_stable_release() {
        let mut d = Debouncer::new(WINDOW);
        let _ = d.update(true, 0);
        assert!(d.update(true, WINDOW));
        assert!(d.update(false, WINDOW + 1_000)); // still reported pressed
        assert!(!d.update(false, WINDOW * 2 + 1_000));
        assert!(!d.level());
    }

    #[test]
    fn bouncing_does_not_flip_the_level() {
        let mut d = Debouncer::new(WINDOW);
        let _ = d.update(true, 0);
        assert!(d.update(true, WINDOW));
        // Chatter around a release that never settles long enough.
        for t in [WINDOW + 500, WINDOW + 900, WINDOW + 1_300, WINDOW + 1_600] {
            assert!(d.update(false, t));
        }
        // Back to pressed before the window elapses.
        assert!(d.update(true, WINDOW + 2_000));
        assert!(d.level());
    }

    #[test]
    fn holds_pressed_while_held() {
        let mut d = Debouncer::new(WINDOW);
        let _ = d.update(true, 0);
        assert!(d.update(true, WINDOW));
        // Still held much later.
        assert!(d.update(true, 5_000_000));
        assert!(d.level());
    }
}
