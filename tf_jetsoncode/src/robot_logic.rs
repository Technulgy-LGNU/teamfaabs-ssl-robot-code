use crate::communication::{TeensySendMsg, VisionMsg, send_flags};
use crate::config;
use crate::proto::{CpRobot, CpTrackedRobot};
use crate::robot_logic::ball_logic::{get_ball, receive_ball};
use crate::robot_logic::goalie::goalie;
use crate::robot_logic::helpers::distance_cpv;
use crate::robot_logic::orca::OrcaOptions;
use tracing::info;

mod ball_logic;
pub mod goalie;
pub mod helpers;
pub mod orca;

#[inline]
pub async fn command(
  cfg: &config::Config, cp_data: &CpRobot, vision_data: &VisionMsg, mut msg: TeensySendMsg,
  stop: bool, robot_self: CpTrackedRobot,
) -> TeensySendMsg {
  match cp_data.cmd.task {
    0 => {
      // UNKNOWN
      println!("UNKNOWN");
      msg.set_flag(send_flags::ERROR);
    }
    1 => {
      // Speed check
      let max_speed_mm_s = if cp_data.cmd.speed > Some(1500) && stop {
        1500
      } else {
        cp_data.cmd.speed.unwrap_or_default()
      };

      // Check if near of pos, and then stop
      if distance_cpv(robot_self.pos, cp_data.cmd.pos.unwrap_or_default()) < 10.0 {
        msg.speed = 0;
      } else {
        let plan = orca::drive_to_target(
          cfg,
          cp_data,
          robot_self,
          cp_data.cmd.pos.unwrap_or_default(),
          OrcaOptions {
            max_speed_mm_s: max_speed_mm_s as f32,
            avoid_ball: stop,
            ..OrcaOptions::default()
          },
        );
        msg = orca::orca_to_teensy(msg, &plan, robot_self);
      }

      msg.orient = cp_data.cmd.orientation.unwrap_or_default() as u16;
    }
    2 => {
      // Kick in kick dir

      // First rotate robot
      // ToDo: Make more precise, when encoders arrive
      if (robot_self.orientation - cp_data.cmd.kick_orient.unwrap_or_default() as i32).abs() > 30 {
        // If we are facing the right direction (variance of two degrees)
        msg.orient = cp_data.cmd.kick_orient.unwrap_or_default() as u16;
      } else {
        msg.kick_pwr = cp_data.cmd.kick_speed.unwrap_or_default() as u8;
        msg.set_flag(send_flags::KICK);
      }
    }
    3 => {
      // Chip in kick dir

      // First rotate robot
      // ToDo: Make more precise, when encoders arrive
      if (robot_self.orientation - cp_data.cmd.kick_orient.unwrap_or_default() as i32).abs() > 30 {
        // If we are facing the right direction (variance of two degrees)
        msg.orient = cp_data.cmd.kick_orient.unwrap_or_default() as u16;
      } else {
        msg.kick_pwr = cp_data.cmd.kick_speed.unwrap_or_default() as u8;
        msg.set_flag(send_flags::CHIP);
      }
    }
    4 => {
      // Rec Kick
      msg = receive_ball(cp_data, robot_self, vision_data, msg);
    }
    5 => {
      // Steal Ball
      msg = get_ball(cfg, cp_data, vision_data, msg, robot_self).await;
    }
    6 => {
      // Dribble the Ball
    }
    7 => {
      // Position the Ball
    }
    9 => {
      // Kickoff
    }
    11 => {
      // Free kick
    }
    _ => {
      info!("Unknown task: {}", cp_data.cmd.task);
    }
  }

  msg
}
