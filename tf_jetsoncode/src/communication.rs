use crate::communication::receive_cp::receive_cp;
use crate::communication::receive_onboard_vision::receive_onboard_vision;
use crate::communication::teensy_communication::teensy_communication;
use crate::proto::CpRobot;
use crate::{config, TEENSY_SEND_MSG_SIZE};
use std::sync::Arc;
use tokio::sync::{Mutex, Notify};

pub mod receive_cp;
pub mod receive_onboard_vision;
pub mod send_cp;
pub mod teensy_communication;

/// Raw Stream from the OnBoard Jetson Vision
#[derive(Default)]
pub struct VisionMsg {
  pub x: f32,
  pub y: f32,
  pub size: f32,
}

// Teensy data
/// Raw HID Msg from the Teensy
#[repr(C)]
#[derive(Debug, Default)]
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
  pub flags: u32,
  pub batt_level: u8,
  pub current: u16,
}
impl TeensyRecMSG {
  pub fn error(&self) -> bool {
    self.flags & (1 << 0) != 0
  }

  pub fn has_ball(&self) -> bool {
    self.flags & (1 << 1) != 0
  }

  pub fn kick_ready(&self) -> bool {
    self.flags & (1 << 2) != 0
  }

  pub fn chip_ready(&self) -> bool {
    self.flags & (1 << 3) != 0
  }

  pub fn button(&self, idx: u8) -> bool {
    self.flags & (1 << (8 + idx)) != 0
  }

  pub fn has_any_button(&self) -> bool {
    self.flags & 0xFF00 != 0
  }
}

/// Raw HID Msg for the Teensy
#[repr(C)]
#[derive(Debug, Default)]
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
  pub flags: u16,
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
  // Own direction and velocity as VecI2
  pub self_orient: u16,
  pub vel_x: i16,
  pub vel_y: i16,
}
impl TeensySendMsg {
  pub fn encode(&self) -> [u8; TEENSY_SEND_MSG_SIZE] {
    let mut buf = [0u8; TEENSY_SEND_MSG_SIZE];

    // flags (u16)

    buf[0..2].copy_from_slice(&self.flags.to_le_bytes());

    // u8 fields
    buf[2] = self.state;
    buf[3] = self.kick_pwr;
    buf[4] = self.dribbler_pwr;

    // u16 fields
    // Direction
    buf[5..7].copy_from_slice(&self.dir.to_le_bytes());
    buf[7..9].copy_from_slice(&self.speed.to_le_bytes());
    buf[9..11].copy_from_slice(&self.orient.to_le_bytes());
    // Own direction
    buf[11..13].copy_from_slice(&self.self_orient.to_le_bytes());

    // i16 fields
    // Velocity as VecI2
    buf[13..15].copy_from_slice(&self.vel_x.to_le_bytes());
    buf[15..17].copy_from_slice(&self.vel_y.to_le_bytes());

    buf
  }

  pub fn set_flag(&mut self, flag: u16) {
    self.flags |= flag;
  }

  // pub fn clear_flag(&mut self, flag: u16) {
  //   self.flags &= !flag;
  // }

  pub fn clear_all_flags(&mut self) {
    self.flags = 0;
  }
}
pub mod send_flags {
  pub const ERROR: u16 = 1 << 0;
  pub const KICK: u16 = 1 << 1;
  pub const CHIP: u16 = 1 << 2;
  pub const DRIBBLER: u16 = 1 << 3;
  // 8–15 reserved for LEDs etc.
}

#[derive(Default)]
struct TeensyLastState {
  seq: u64,
  payload: Option<[u8; TEENSY_SEND_MSG_SIZE]>,
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
  pub async fn publish(&self, payload: [u8; TEENSY_SEND_MSG_SIZE]) {
    let mut lock = self.state.lock().await;
    lock.seq = lock.seq.wrapping_add(1);
    lock.payload = Some(payload);
    drop(lock);
    self.notify.notify_waiters();
  }

  /// Return the latest payload if it is newer than `last_seq`, otherwise `None`.
  pub async fn try_latest_after(&self, last_seq: u64) -> Option<(u64, [u8; TEENSY_SEND_MSG_SIZE])> {
    let lock = self.state.lock().await;

    if lock.seq != last_seq {
      lock.payload.map(|payload| (lock.seq, payload))
    } else {
      None
    }
  }
}

#[derive(Default)]
pub struct Events {
  pub cp: Option<CpRobot>,
  pub vis: Option<VisionMsg>,
  pub teensy: Option<TeensyRecMSG>,
}

impl Events {
  pub fn clear(&mut self) {
    self.cp = None;
    self.vis = None;
    self.teensy = None;
  }

  pub fn take(&mut self) -> Self {
    Self {
      cp: self.cp.take(),
      vis: self.vis.take(),
      teensy: self.teensy.take(),
    }
  }
}

pub type EventShare = Arc<Mutex<Events>>;

pub struct CommunicationHandles {
  pub events: EventShare,
  pub teensy: TeensyOut,
}

pub async fn communication_receiver(cfg: &config::Config) -> anyhow::Result<CommunicationHandles> {
  let events = Arc::new(Mutex::new(Events::default()));
  let teensy = TeensyOut::new();

  receive_cp(cfg, events.clone()).await;

  receive_onboard_vision(cfg, events.clone()).await;

  teensy_communication(cfg, events.clone(), teensy.clone()).await;

  Ok(CommunicationHandles { events, teensy })
}
