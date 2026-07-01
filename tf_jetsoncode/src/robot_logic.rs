use crate::Robot;
use crate::communication::send_flags;
use crate::robot_logic::orca::{
  NavIntent, OrcaRequest, Vec2i, WorldSnapshot, nav_command_to_teensy,
};
use crate::robot_logic::vec::{Vec2f, distance_cpv_squared};
use core_dump::proto::CpTask;

mod defense;
mod get_ball;
pub mod goalie;
pub mod helpers;
pub mod orca;
mod receive_ball;
pub mod vec;

// If we are inside this distance in the penalty area, stop using raw motion.
pub(crate) const RAW_STOP_RADIUS_MM: f32 = 40f32;
// Maximum translational speed for raw goalie movement inside the penalty area.
pub(crate) const RAW_MAX_SPEED_MM_S: f32 = 4_000f32;
const DRIBBLER_PRESPIN_RANGE_MM: f32 = 180f32;
const KICK_HEADING_TOLERANCE_DEG: i32 = 3;
const CHIP_HEADING_TOLERANCE_DEG: i32 = 5;

impl<C> Robot<C> {
  #[inline]
  pub fn command(&mut self, world: &WorldSnapshot, stop: bool) {
    // Vars
    let robot_pos = Vec2f::new_from_cp(self.packets.robot_self.pos);
    let ball_pos = Vec2f::new_from_cp(self.packets.cp_data.ball.pos);
    let ball_vel = Vec2f::new_from_cp(self.packets.cp_data.ball.vel.unwrap_or_default());
    let mut has_kicked: bool = false;

    if !stop && should_prespin_dribbler(robot_pos, ball_pos) {
      self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
      self.packets.robot_msg.dribbler_pwr = 200;
    }

    match CpTask::try_from(self.packets.cp_data.cmd.task).unwrap_or(CpTask::TaskUnspecified) {
      CpTask::TaskUnspecified => {
        // UNKNOWN
        self.packets.robot_msg.set_flag(send_flags::ERROR);
      }
      CpTask::TaskPos => {
        has_kicked = false;
        // Speed check
        let max_speed_mm_s = if self.packets.cp_data.cmd.speed > Some(1500) && stop {
          1500
        } else {
          self.packets.cp_data.cmd.speed.unwrap_or_default()
        };

        // Check if near of pos, and then stop
        if distance_cpv_squared(
          self.packets.robot_self.pos,
          self.packets.cp_data.cmd.pos.unwrap_or_default(),
        ) < 60.0 * 60.0
        {
          let intent = NavIntent::Stop;
          let nav_command = self.orca.step(OrcaRequest { intent, world });
          nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
        } else {
          let nav_intent = NavIntent::GoToPosition {
            target_pos_mm: Vec2i::new_from_cp(self.packets.cp_data.cmd.pos.unwrap_or_default()),
            max_speed_mm_s,
          };
          let nav_command = self.orca.step(OrcaRequest {
            intent: nav_intent,
            world,
          });
          nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
        }

        self.packets.robot_msg.orient =
          self.packets.cp_data.cmd.orientation.unwrap_or_default() as u16;
      }
      CpTask::TaskKick => {
        let kick_orient = self.packets.cp_data.cmd.kick_orient.unwrap_or_default() as u16;
        let kick_power = self.packets.cp_data.cmd.kick_speed.unwrap_or_default();
        self.packets.robot_msg.orient = kick_orient;
        if heading_error_deg(self.packets.robot_self.orientation, kick_orient as i32)
          <= KICK_HEADING_TOLERANCE_DEG
        {
          self.packets.robot_msg.kick_pwr = kick_power as u8;
          self.packets.robot_msg.set_flag(send_flags::KICK);
        }

        if !self.packets.teensy_data.has_ball() {
          has_kicked = true;
          self.packets.robot_msg.clear_all_flags();
        }
      }
      CpTask::TaskChip => {
        // Chip in kick dir
        // First rotate robot
        if heading_error_deg(
          self.packets.robot_self.orientation,
          self.packets.cp_data.cmd.kick_orient.unwrap_or_default() as i32,
        ) > CHIP_HEADING_TOLERANCE_DEG
        {
          // If we are facing the right direction (variance of five degrees)
          self.packets.robot_msg.orient =
            self.packets.cp_data.cmd.kick_orient.unwrap_or_default() as u16;
        } else if !has_kicked {
          self.packets.robot_msg.kick_pwr =
            self.packets.cp_data.cmd.kick_speed.unwrap_or_default() as u8;
          self.packets.robot_msg.set_flag(send_flags::CHIP);
        }

        // Disable kicking if we have kicked (no Ball in the ball capturing zone)
        if !self.packets.teensy_data.has_ball() {
          has_kicked = true;
        }
      }
      CpTask::TaskRecKick => {
        has_kicked = false;
        // Rec Kick
        // Pre-spin while receiving so the ball is already captured when it
        // reaches the mouth instead of bouncing before IR latches.
        self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
        self.packets.robot_msg.dribbler_pwr = 200;

        if self.packets.teensy_data.has_ball() {
          self.packets.robot_msg.speed = 0;
        } else {
          if ball_vel.norm() >= 200f32 && self.receive_ball() {
            // Fast pass: intercept on its path.
          } else {
            // Slow / slightly missed pass: keep the receiving intent and collect
            // the loose ball instead of idling while an opponent takes it.
            self.collect_receive_ball(robot_pos, ball_pos);
          }

          // Keep looking at the ball while moving.
          self.packets.robot_msg.orient = (ball_pos - robot_pos).angle_to_u16();
        }
      }
      CpTask::TaskSteal => {
        has_kicked = false;
        // Steal Ball
        self.get_ball(world);
      }
      CpTask::TaskDribble => {
        has_kicked = false;
        // Dribble the Ball
        // Run the steal algorithm, until we have the ball in the ball capturing zone
        if self.packets.teensy_data.has_ball() {
          let intent = NavIntent::GoToPosition {
            target_pos_mm: Vec2i::new_from_cp(self.packets.cp_data.cmd.pos.unwrap_or_default()),
            max_speed_mm_s: self.packets.cp_data.cmd.speed.unwrap_or_default(),
          };
          let nav_command = self.orca.step(OrcaRequest { intent, world });

          // Enable Dribbler
          self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
          self.packets.robot_msg.dribbler_pwr = 200;

          nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
          // Carry the ball facing the push direction (toward the dribble target)
          // so the carrier is already aimed when it decides to kick.
          self.packets.robot_msg.orient =
            self.packets.cp_data.cmd.orientation.unwrap_or_default() as u16;
        } else {
          self.get_ball(world);
        }
      }
      CpTask::TaskBlock => {
        has_kicked = false;
        // Block a robot from receiving the ball
        // If enemy_id == None, defend own penalty area, else block robot
        match self.packets.cp_data.cmd.enemy_id {
          Some(_) => {
            self.defense_robot(world);
          }
          None => {
            self.defense_goal(world);
          }
        }

        // Keep looking at the ball while moving.
        self.packets.robot_msg.orient = (ball_pos - robot_pos).angle_to_u16();
      }
      CpTask::TaskPosBall => {
        has_kicked = false;
        // Position the Ball
        // Run the steal algorithm, until we have the ball in the ball capturing zone
        // After that slowly turn the dribbler off and drive away from the ball
        if self.packets.teensy_data.has_ball() {
          let intent = NavIntent::GoToPosition {
            target_pos_mm: Vec2i::new_from_cp(self.packets.cp_data.cmd.pos.unwrap_or_default()),
            max_speed_mm_s: self.packets.cp_data.cmd.speed.unwrap_or_default(),
          };
          let nav_command = self.orca.step(OrcaRequest { intent, world });

          // Enable Dribbler
          self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
          self.packets.robot_msg.dribbler_pwr = 200;

          nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
        } else if robot_pos == Vec2f::new_from_cp(self.packets.cp_data.cmd.pos.unwrap_or_default())
        {
          // Logic to drive away from the ball
        } else {
          self.get_ball(world);
        }
      }
      CpTask::StateKickoff => {
        has_kicked = false;
        // Kickoff
      }
      CpTask::StateFreekick => {
        has_kicked = false;
        // Free kick
      }
    }
  }
}

fn heading_error_deg(current: i32, target: i32) -> i32 {
  let error = (target - current + 180).rem_euclid(360) - 180;
  error.abs()
}

#[inline]
fn should_prespin_dribbler(robot_pos: Vec2f, ball_pos: Vec2f) -> bool {
  (ball_pos - robot_pos).norm_squared() <= DRIBBLER_PRESPIN_RANGE_MM * DRIBBLER_PRESPIN_RANGE_MM
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn heading_error_wraps_around_zero() {
    assert_eq!(heading_error_deg(359, 1), 2);
    assert_eq!(heading_error_deg(1, 359), 2);
    assert_eq!(heading_error_deg(10, 350), 20);
  }

  #[test]
  fn kick_heading_tolerance_is_tight_enough_for_goal_shots() {
    assert!(KICK_HEADING_TOLERANCE_DEG <= 3);
  }

  #[test]
  fn prespins_dribbler_inside_near_ball_range() {
    assert!(should_prespin_dribbler(
      Vec2f::new(0f32, 0f32),
      Vec2f::new(180f32, 0f32),
    ));
    assert!(!should_prespin_dribbler(
      Vec2f::new(0f32, 0f32),
      Vec2f::new(181f32, 0f32),
    ));
  }
}
