use crate::communication::{TeensySendMsg, VisionMsg, send_flags};
use crate::config;
use crate::proto::{CpRobot, CpTrackedRobot};
use crate::robot_logic::get_ball::get_ball;
use crate::robot_logic::helpers::{point_at_distance_from_a, raw_move_towards};
use crate::robot_logic::orca::{
  NavIntent, OrcaHandle, OrcaRequest, Vec2i, WorldSnapshot, nav_command_to_teensy,
};
use crate::robot_logic::receive_ball::receive_ball;
use crate::robot_logic::vec::{Vec2f, distance_cpv};
use tracing::info;

mod get_ball;
pub mod goalie;
pub mod helpers;
pub mod orca;
mod receive_ball;
pub mod vec;

// If we are inside this distance in the penalty area, stop using raw motion.
pub(crate) const RAW_STOP_RADIUS_MM: f32 = 40f32;
// Maximum translational speed for raw goalie movement inside the penalty area.
// ToDo: Needs to be higher
pub(crate) const RAW_MAX_SPEED_MM_S: f32 = 4_000f32;

#[inline]
pub fn command(
  cfg: &config::Config, cp_data: &CpRobot, vision_data: &VisionMsg, orca: &OrcaHandle,
  world: &WorldSnapshot, mut msg: TeensySendMsg, stop: bool, robot_self: CpTrackedRobot,
) -> TeensySendMsg {
  // Vars
  let robot_pos = Vec2f::new_from_cp(robot_self.pos);
  let ball_pos = Vec2f::new_from_cp(cp_data.ball.pos);
  let ball_vel = Vec2f::new_from_cp(cp_data.ball.vel.unwrap_or_default());

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
        let nav_intent = NavIntent::GoToPosition {
          target_pos_mm: Vec2i::new_from_cp(cp_data.cmd.pos.unwrap_or_default()),
          max_speed_mm_s,
        };
        orca.publish(OrcaRequest {
          intent: nav_intent,
          world: world.clone(),
        });

        let plan = orca.latest();
        msg = nav_command_to_teensy(msg, plan);
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
      if ball_vel.norm() >= 200f32 {
        msg = receive_ball(cp_data, robot_self, vision_data, msg);
      } else {
        msg.speed = 0;
      }

      // Keep looking at the ball while moving.
      msg.orient = (ball_pos - robot_pos).angle_to_u16();

      // Always enable dribbler
      msg.set_flag(send_flags::DRIBBLER);
      msg.dribbler_pwr = 200;
    }
    5 => {
      // Steal Ball
      msg = get_ball(cp_data, vision_data, orca, world, msg, robot_self);
    }
    6 => {
      // Dribble the Ball
    }
    7 => {
      // Position the Ball
    }
    8 => {
      // Block a robot from receiving the ball
      // Get the robot based on its id and cannot
      let to_block_robot = match cfg.robot_team.as_str() {
        "yellow" => Vec2f::new_from_cp(
          cp_data
            .robots_blue
            .iter()
            .find(|r| r.robot_id == cp_data.cmd.enemy_id.unwrap_or_default())
            .unwrap_or(&CpTrackedRobot::default())
            .pos,
        ),
        "blue" => Vec2f::new_from_cp(
          cp_data
            .robots_yellow
            .iter()
            .find(|r| r.robot_id == cp_data.cmd.enemy_id.unwrap_or_default())
            .unwrap_or(&CpTrackedRobot::default())
            .pos,
        ),
        _ => {
          panic!("Unknown robot_team: {}", cfg.robot_team);
        }
      };

      let target = point_at_distance_from_a(to_block_robot, ball_pos, 500f32)
        .unwrap_or(Vec2f::new(0f32, 0f32));

      // If target is to far away, use orca
      let intent = NavIntent::GoToPosition {
        target_pos_mm: Vec2i::new(target.x as i32, target.y as i32),
        max_speed_mm_s: 3000,
      };
      orca.publish(OrcaRequest {
        intent,
        world: world.clone(),
      });

      msg = nav_command_to_teensy(msg, orca.latest());

      // Keep looking at the ball while moving.
      msg.orient = (ball_pos - robot_pos).angle_to_u16();
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
