//! The K-line as a wired-AND of the tester and the ECU: a merged list of
//! intervals (in CPU cycles) during which someone pulls it low.

#[derive(Default)]
pub struct Line {
    lows: Vec<(u64, u64)>,
    /// Bumped on every change, so readers can cache derived state.
    pub version: u64,
}

impl Line {
    pub fn pull_low(&mut self, s: u64, e: u64) {
        if e <= s {
            return;
        }
        self.version += 1;
        let i = self.lows.partition_point(|&(_, ie)| ie < s);
        let mut ns = s;
        let mut ne = e;
        let mut j = i;
        while j < self.lows.len() && self.lows[j].0 <= ne {
            ns = ns.min(self.lows[j].0);
            ne = ne.max(self.lows[j].1);
            j += 1;
        }
        self.lows.splice(i..j, [(ns, ne)]);
    }

    /// Pull low for the zero bits of a UART frame (start, 8 data LSB
    /// first, `stops` stop bits) starting at `t` with bit time `bit`.
    pub fn frame(&mut self, t: u64, b: u8, bit: u64) {
        for i in 0..9u64 {
            let one = i > 0 && (b >> (i - 1)) & 1 == 1;
            if !one {
                self.pull_low(t + i * bit, t + (i + 1) * bit);
            }
        }
    }

    pub fn level(&self, t: u64) -> bool {
        let i = self.lows.partition_point(|&(_, e)| e <= t);
        !(i < self.lows.len() && self.lows[i].0 <= t)
    }

    /// First falling edge at or after `t` (an interval that starts at or
    /// after `t`; if `t` is inside a low interval the edge is a later one).
    pub fn next_fall(&self, t: u64) -> Option<u64> {
        let i = self.lows.partition_point(|&(s, _)| s < t);
        self.lows.get(i).map(|&(s, _)| s)
    }

    /// Drop intervals that ended before `t` (keeps lookups cheap).
    pub fn forget_before(&mut self, t: u64) {
        let i = self.lows.partition_point(|&(_, e)| e < t);
        if i > 64 {
            self.lows.drain(..i);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn merge_and_sample() {
        let mut l = Line::default();
        l.pull_low(10, 20);
        l.pull_low(30, 40);
        l.pull_low(18, 32);
        assert_eq!(l.lows, vec![(10, 40)]);
        assert!(l.level(9));
        assert!(!l.level(10));
        assert!(l.level(40));
        assert_eq!(l.next_fall(0), Some(10));
        assert_eq!(l.next_fall(11), None);
    }

    #[test]
    fn fast_init_at_200_baud_reads_f0() {
        // 25 ms low then high: at 200 baud (5 ms bits) the receiver sees a
        // start bit plus data bits 0-3 low, 4-7 high, stop high: 0xF0.
        let mut l = Line::default();
        l.pull_low(0, 25_000);
        let bit = 5_000;
        let mut v = 0u8;
        for k in 0..8 {
            if l.level(bit * (k + 1) + bit / 2) {
                v |= 1 << k;
            }
        }
        assert_eq!(v, 0xF0);
        assert!(l.level(9 * bit + bit / 2));
    }
}
