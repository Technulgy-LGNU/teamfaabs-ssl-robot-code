use crate::communication::{TeensySendMsg, VisionMsg, send_flags};
use crate::config;
use crate::proto::{CpRobot, CpTrackedRobot};
use crate::robot_logic::ball_logic::get_ball;
use crate::robot_logic::orca::{
  OrcaHandle, OrcaRequest, Vec2i, WorldSnapshot, nav_command_to_teensy,
};

mod ball_logic;
pub mod goalie;
mod helpers;
pub mod orca;

pub async fn command(
  cfg: &config::Config, cp_data: &CpRobot, orca: &mut OrcaHandle, world: &WorldSnapshot,
  vision_data: &VisionMsg, mut msg: TeensySendMsg, stop: bool,
) -> TeensySendMsg {
  let mut robot_self: CpTrackedRobot = Default::default();
  if cfg.robot_team.as_str() == "yellow" {
    robot_self = *cp_data
      .robots_yellow
      .iter()
      .find(|r| r.robot_id == cfg.robot_id as u32)
      .unwrap_or(&robot_self);
  } else if cfg.robot_team.as_str() == "blue" {
    robot_self = *cp_data
      .robots_blue
      .iter()
      .find(|r| r.robot_id == cfg.robot_id as u32)
      .unwrap_or(&robot_self);
  } else {
    panic!("Unknown team: {}", cfg.robot_team);
  }

  match cp_data.cmd.task {
    0 => {
      // UNKNOWN
      println!("UNKNOWN");
      msg.set_flag(send_flags::ERROR);
    }
    1 => {
      // Speed check
      let speed = if cp_data.cmd.speed > Some(1500) && stop {
        1500
      } else {
        cp_data.cmd.speed.unwrap_or_default()
      };

      // Drive to pos
      let intent = orca::NavIntent::GoToPosition {
        target_pos_mm: Vec2i {
          x: cp_data.cmd.pos.unwrap_or_default().x,
          y: cp_data.cmd.pos.unwrap_or_default().y,
        },
        max_speed_mm_s: speed,
      };

      orca.publish(OrcaRequest {
        world: world.clone(),
        intent,
      });

      let orca_cmd = orca.changed().await.unwrap_or_default();

      println!("Orca output: {:?}", orca_cmd);
      msg = nav_command_to_teensy(msg, orca_cmd);
    }
    2 => {
      // Kick in kick dir

      // First rotate robot
      if (robot_self.orientation - cp_data.cmd.kick_orient.unwrap_or_default() as i32).abs() < 2 {
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
      if (robot_self.orientation - cp_data.cmd.kick_orient.unwrap_or_default() as i32).abs() < 2 {
        // If we are facing the right direction (variance of two degrees)
        msg.orient = cp_data.cmd.kick_orient.unwrap_or_default() as u16;
      } else {
        msg.kick_pwr = cp_data.cmd.kick_speed.unwrap_or_default() as u8;
        msg.set_flag(send_flags::CHIP);
      }
    }
    4 => {
      // Rec Kick
    }
    5 => {
      // Steal Ball
      msg = get_ball(
        cp_data,
        &mut orca.clone(),
        world,
        vision_data,
        msg,
        robot_self,
      )
      .await;
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
      println!("Unknown task: {}", cp_data.cmd.task);
    }
  }

  msg
}
