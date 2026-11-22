//! An in-memory CAN bus: every frame one endpoint sends reaches all the
//! other endpoints (not itself), as on a real bus. For tests and for
//! simulated ECUs; frames arrive instantly and are never lost.

use std::io;
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::{CanBus, Frame};

type Nodes = Arc<Mutex<Vec<(usize, Sender<Frame>)>>>;

#[derive(Clone, Default)]
pub struct Hub {
    nodes: Nodes,
}

impl Hub {
    pub fn new() -> Self {
        Self::default()
    }

    /// A new node on the bus.
    pub fn endpoint(&self) -> Endpoint {
        let (tx, rx) = channel();
        let mut nodes = self.nodes.lock().expect("bus lock");
        let me = nodes.iter().map(|(i, _)| i + 1).max().unwrap_or(0);
        nodes.push((me, tx));
        Endpoint {
            me,
            rx,
            nodes: self.nodes.clone(),
        }
    }
}

pub struct Endpoint {
    me: usize,
    rx: Receiver<Frame>,
    nodes: Nodes,
}

impl CanBus for Endpoint {
    fn send(&mut self, frame: &Frame) -> io::Result<()> {
        let mut nodes = self.nodes.lock().expect("bus lock");
        // a node whose endpoint was dropped leaves the bus
        nodes.retain(|(i, tx)| *i == self.me || tx.send(frame.clone()).is_ok());
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> io::Result<Option<Frame>> {
        match self.rx.recv_timeout(timeout) {
            Ok(f) => Ok(Some(f)),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => Err(io::Error::other("bus gone")),
        }
    }
}

impl Drop for Endpoint {
    fn drop(&mut self) {
        if let Ok(mut nodes) = self.nodes.lock() {
            nodes.retain(|(i, _)| *i != self.me);
        }
    }
}
