use tokio::sync::{Mutex, Notify};
use std::sync::Arc;
use prost::bytes::Bytes;
use crate::communication::receive_cp::receive_cp;
use crate::communication::receive_onboard_vision::receive_onboard_vision;
use crate::communication::teensy_communication::teensy_communication;
use crate::config;
use crate::proto::CpRobot;

pub mod receive_cp;
pub mod send_cp;
pub mod receive_onboard_vision;
pub mod teensy_communication;

/// Raw Stream from the OnBoard Jetson Vision
#[derive(Default)]
pub struct VisionMsg {
  pub x: f32,
  pub y: f32,
  pub size: f32
}

/// Raw HID Msg from the Teensy
#[derive(Default)]
pub struct TeensyRecMSG {
  pub has_ball: bool,
  pub kick_ready: bool,
  pub batt_volt: f32,
  pub orientation: u16,
}

/// Raw HID Msg for the Teensy
pub struct TeensySendMsg {
  pub kick: bool,
  pub chip: bool,
  pub kick_pwr: u8,
  pub dir: u16,
  pub speed: u16,
  pub orient: u16
}

#[derive(Default)]
struct TeensyLastState {
  seq: u64,
  payload: Option<Bytes>,
}

/// Outbound Teensy handle (RobotCode -> Teensy).
///
/// This is intentionally implemented as an `Arc<Mutex<...>>` holding only the *latest* message.
/// If producers publish faster than a client can send, the client will skip intermediate updates
/// and only transmit the newest snapshot.
#[derive(Clone, Default)]
pub struct TeensyOut {
  state: Arc<Mutex<TeensyLastState>>,
  notify: Arc<Notify>,
}

impl TeensyOut {
  pub fn new() -> Self {
    Self {
      state: Arc::new(Mutex::new(TeensyLastState::default())),
      notify: Arc::new(Notify::new()),
    }
  }

  /// Publish a new binary payload.
  pub async fn publish(&self, payload: Bytes) {
    let mut lock = self.state.lock().await;
    lock.seq = lock.seq.wrapping_add(1);
    lock.payload = Some(payload);
    drop(lock);
    self.notify.notify_waiters();
  }

  /// Wait until a payload newer than `last_seq` is available and return it.
  ///
  /// This is implemented in a race-free way (won't miss notifications): it creates the
  /// notification future *before* checking the current sequence.
  pub async fn wait_latest_after(&self, last_seq: u64) -> (u64, Bytes) {
    loop {
      let notified = self.notify.notified();

      {
        let lock = self.state.lock().await;
        if lock.seq != last_seq && let Some(payload) = lock.payload.clone() {
          return (lock.seq, payload);
        }
      }

      notified.await;
    }
  }

  /// Return the latest payload if it is newer than `last_seq`, otherwise `None`.
  pub async fn try_latest_after(&self, last_seq: u64) -> Option<(u64, Bytes)> {
    let lock = self.state.lock().await;

    if lock.seq != last_seq {
      lock.payload.clone().map(|payload| (lock.seq, payload))
    } else {
      None
    }
  }
}

pub type EventShare = Arc<Mutex<(Option<CpRobot>, Option<VisionMsg>, Option<TeensyRecMSG>)>>;

pub struct CommunicationHandles {
  pub events: EventShare,
  pub teensy: TeensyOut,
}

pub async fn communication_receiver(cfg: &config::Config) -> anyhow::Result<CommunicationHandles> {
  let events = Arc::new(Mutex::new((None, None, None)));
  let teensy = TeensyOut::new();

  receive_cp(cfg, events.clone()).await;

  receive_onboard_vision(cfg, events.clone()).await;

  teensy_communication(cfg, events.clone(), teensy.clone()).await;

  Ok(CommunicationHandles { events, teensy })
}
