//! Bit-exact model of glibc's `rand()` with the default seed (1).
//!
//! The firmware reads I/O ports 0x201 and 0x236, which upstream answers with
//! `rand()`. The oracle never calls `srand()`, so its sequence is glibc's
//! TYPE_3 additive generator seeded with 1:
//! `r[i] = r[i-31] + r[i-3]`, output `r[i] >> 1`, first 310 values discarded.

pub struct GlibcRand {
    /// Last 34 values of `r`, oldest first.
    r: [i32; 34],
}

impl Default for GlibcRand {
    fn default() -> Self {
        Self::new(1)
    }
}

impl GlibcRand {
    pub fn new(seed: u32) -> Self {
        let mut r = [0i32; 34];
        r[0] = if seed == 0 { 1 } else { seed as i32 };
        for i in 1..31 {
            // (16807 * r[i-1]) % 2147483647 without overflow (Schrage)
            let hi = r[i - 1] / 127773;
            let lo = r[i - 1] % 127773;
            let mut word = 16807 * lo - 2836 * hi;
            if word < 0 {
                word += 2147483647;
            }
            r[i] = word;
        }
        for i in 31..34 {
            r[i] = r[i - 31];
        }
        let mut g = Self { r };
        for _ in 34..344 {
            g.next_raw();
        }
        g
    }

    fn next_raw(&mut self) -> i32 {
        let v = self.r[34 - 31].wrapping_add(self.r[34 - 3]);
        self.r.rotate_left(1);
        self.r[33] = v;
        v
    }

    /// Next value of `rand()`, in `0..=RAND_MAX` (2^31 - 1).
    pub fn rand(&mut self) -> i32 {
        ((self.next_raw() as u32) >> 1) as i32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_glibc_default_sequence() {
        let mut g = GlibcRand::default();
        let got: Vec<i32> = (0..5).map(|_| g.rand()).collect();
        assert_eq!(
            got,
            [1804289383, 846930886, 1681692777, 1714636915, 1957747793]
        );
    }
}
