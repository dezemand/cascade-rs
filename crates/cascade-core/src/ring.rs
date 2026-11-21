//! Port of upstream `ring.h`: fixed ring without overflow protection.
//! `add` on a full ring overwrites and makes it look empty, as upstream does.

#[derive(Clone)]
pub struct Ring<T: Copy + Default, const N: usize> {
    buf: [T; N],
    start: usize,
    end: usize,
}

impl<T: Copy + Default, const N: usize> Default for Ring<T, N> {
    fn default() -> Self {
        Self {
            buf: [T::default(); N],
            start: 0,
            end: 0,
        }
    }
}

impl<T: Copy + Default, const N: usize> Ring<T, N> {
    pub fn add(&mut self, v: T) {
        self.buf[self.end] = v;
        self.end = (self.end + 1) % N;
    }
    pub fn prepend(&mut self, v: T) {
        self.start = if self.start == 0 {
            N - 1
        } else {
            self.start - 1
        };
        self.buf[self.start] = v;
    }
    pub fn consume(&mut self) -> T {
        let v = self.buf[self.start];
        self.start = (self.start + 1) % N;
        v
    }
    pub fn snoop(&self) -> T {
        self.buf[self.start]
    }
    pub fn flush(&mut self) {
        self.start = self.end;
    }
    pub fn is_empty(&self) -> bool {
        self.start == self.end
    }
}
