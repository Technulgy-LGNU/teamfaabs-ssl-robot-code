use tokio::sync::{Mutex, Notify};
use std::sync::Arc;
use bitflags::bitflags;
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

// Teensy data
bitflags! {
  #[derive(Debug, Clone, Copy)]
  /// Teensy send flags
  pub struct SendFlags: u16 {
      const ERROR    = 1 << 0;
      const KICK     = 1 << 1;
      const CHIP     = 1 << 2;
      const DRIBBLER = 1 << 3;
  }

  #[derive(Debug, Clone, Copy)]
    pub struct RecFlags: u32 {
        const ERROR       = 1 << 0;
        const HAS_BALL    = 1 << 1;
        const KICK_READY  = 1 << 2;
        const CHIP_READY  = 1 << 3;
    }
}

/// Raw HID Msg from the Teensy
#[repr(C)]
pub struct TeensyRecMSG {
  // Bitflags:
  // Bit 0: Error
  // Bit 1: Has Ball
  // Bit 2: Kick Ready
  // Bit 3: Chip Ready
  // Bit 4:
  // Bit 5:
  // Bit 6:
  // Bit 7:
  // Bit 8-31: Buttons followed by dip-switches
  pub flags: RecFlags,
  pub batt_level: u8,
  pub orientation: u8,
}
impl TeensyRecMSG {
  pub fn button(&self, idx: u8) -> bool {
    self.flags.bits() & (1 << (8 + idx)) != 0
  }
}
impl Default for TeensyRecMSG {
  fn default() -> Self {
    Self {
      flags: RecFlags(Default::default()),
      batt_level: 0,
      orientation: 0,
    }
  }
}

/// Raw HID Msg for the Teensy
#[repr(C)]
#[derive(Debug)]
pub struct TeensySendMsg {
  // Bitflags:
  // Bit 0: Error
  // Bit 1: Kick
  // Bit 2: Chip
  // Bit 3: Dribbler
  // Bit 4:
  // Bit 5:
  // Bit 6:
  // Bit 7:
  // Bit 8-15: LEDS
  pub flags: SendFlags,
  // The general GC State, so the robot follows the `HALT` and `STOP` command
  pub state: u8,
  // How strong to kick
  pub kick_pwr: u8,
  // How fast to run the dribbler, probably a config option later on,
  // because you don't need to vary it most of the times
  pub dribbler_pwr: u8,
  // Actual Direction stuff
  pub dir: u16,
  pub speed: u16,
  pub orient: u16,
}
impl TeensySendMsg {
  pub const SIZE: usize = 11;

  pub fn encode(&self) -> [u8; Self::SIZE] {
    let mut buf = [0u8; Self::SIZE];

    // flags
    buf[0..2].copy_from_slice(&self.flags.bits().to_le_bytes());

    // single-byte fields
    buf[2] = self.state;
    buf[3] = self.kick_pwr;
    buf[4] = self.dribbler_pwr;

    // u16 values
    buf[5..7].copy_from_slice(&self.dir.to_le_bytes());
    buf[7..9].copy_from_slice(&self.speed.to_le_bytes());
    buf[9..11].copy_from_slice(&self.orient.to_le_bytes());

    buf
  }
}
impl Default for TeensySendMsg {
  fn default() -> Self {
    Self {
      flags: SendFlags(Default::default()),
      state: 0,
      kick_pwr: 0,
      dribbler_pwr: 0,
      dir: 0,
      speed: 0,
      orient: 0,
    }
  }
}

#[derive(Default)]
struct TeensyLastState {
  seq: u64,
  payload: Option<[u8; 11]>,
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
  pub async fn publish(&self, payload: [u8; 11]) {
    let mut lock = self.state.lock().await;
    lock.seq = lock.seq.wrapping_add(1);
    lock.payload = Some(payload);
    drop(lock);
    self.notify.notify_waiters();
  }

  /// Return the latest payload if it is newer than `last_seq`, otherwise `None`.
  pub async fn try_latest_after(&self, last_seq: u64) -> Option<(u64, [u8; 11])> {
    let lock = self.state.lock().await;

    if lock.seq != last_seq {
      lock.payload.map(|payload| (lock.seq, payload))
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
