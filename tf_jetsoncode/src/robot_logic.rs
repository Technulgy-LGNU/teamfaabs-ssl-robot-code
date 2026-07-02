use crate::communication::send_flags;
use crate::robot_logic::helpers::raw_move_towards;
use crate::robot_logic::orca::{
  NavIntent, OrcaRequest, Vec2i, WorldSnapshot, nav_command_to_teensy,
};
use crate::robot_logic::vec::{Vec2f, distance_cpv_squared};
use crate::{DribbleDistanceTrack, Robot};
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
const KICK_HEADING_TOLERANCE_DEG: i32 = 3;
const CHIP_HEADING_TOLERANCE_DEG: i32 = 5;
const DRIBBLE_RELEASE_DISTANCE_MM: f32 = 850f32;
const DRIBBLE_LIMIT_KICK_POWER: u8 = 120;
const POSITION_RECEIVE_MIN_BALL_SPEED_MM_S: f32 = 900f32;
const POSITION_RECEIVE_MAX_TIME_S: f32 = 0.8;
const POSITION_RECEIVE_LATERAL_MM: f32 = 260f32;
const POSITION_RECEIVE_FACING_DOT: f32 = 0.20;
const POSITION_RECEIVE_CENTER_OFFSET_MM: f32 = 80f32;

impl<C> Robot<C> {
  #[inline]
  pub fn command(&mut self, world: &WorldSnapshot, stop: bool) {
    // Vars
    let robot_pos = Vec2f::new_from_cp(self.packets.robot_self.pos);
    let ball_pos = Vec2f::new_from_cp(self.packets.cp_data.ball.pos);
    let ball_vel = Vec2f::new_from_cp(self.packets.cp_data.ball.vel.unwrap_or_default());
    let mut has_kicked: bool = false;

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

        let target_pos = self.packets.cp_data.cmd.pos.unwrap_or_default();
        let position_receive_target =
          self.packets.cp_data.cmd.orientation.and_then(|orient| {
            position_receive_target(robot_pos, ball_pos, ball_vel, orient as f32)
          });

        if let Some(receive_target) = position_receive_target {
          raw_move_towards(&mut self.packets.robot_msg, robot_pos, receive_target);
          self.packets.robot_msg.speed = self
            .packets
            .robot_msg
            .speed
            .min(max_speed_mm_s.max(350) as u16);
        } else if self.packets.cp_data.cmd.raw.unwrap_or(false) {
          raw_move_towards(
            &mut self.packets.robot_msg,
            robot_pos,
            Vec2f::new_from_cp(target_pos),
          );
        } else if distance_cpv_squared(self.packets.robot_self.pos, target_pos) < 60.0 * 60.0 {
          let intent = NavIntent::Stop;
          let nav_command = self.orca.step(OrcaRequest { intent, world });
          nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
        } else {
          let nav_intent = NavIntent::GoToPosition {
            target_pos_mm: Vec2i::new_from_cp(target_pos),
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

        if position_receive_target.is_some() {
          self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
          self.packets.robot_msg.dribbler_pwr = 200;
        }
      }
      CpTask::TaskKick => {
        let kick_orient = self.packets.cp_data.cmd.kick_orient.unwrap_or_default() as u16;
        let kick_power = self.packets.cp_data.cmd.kick_speed.unwrap_or_default();
        self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
        self.packets.robot_msg.dribbler_pwr = 200;
        self.packets.robot_msg.orient = kick_orient;
        if kick_release_ready(
          self.packets.robot_self.orientation,
          kick_orient as i32,
          self.packets.teensy_data.has_ball(),
        ) {
          self.packets.robot_msg.kick_pwr = kick_power as u8;
          self.packets.robot_msg.set_flag(send_flags::KICK);
        } else if !self.packets.teensy_data.has_ball() {
          has_kicked = true;
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
          } else if receive_ball::should_collect_slow_receive_ball(robot_pos, ball_pos) {
            // Slow / slightly missed pass: keep the receiving intent and collect
            // the loose ball instead of idling while an opponent takes it.
            self.collect_receive_ball(robot_pos, ball_pos);
          } else {
            // Pre-kick RecPass should not make the receiver leave its prepared
            // lane and chase a stationary ball still held by the passer.
            self.packets.robot_msg.speed = 0;
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

  pub(crate) fn enforce_dribble_distance_limit(&mut self, ball_pos: Vec2f) {
    let active_dribble = self.packets.robot_msg.flags & send_flags::DRIBBLER != 0
      && self.packets.teensy_data.has_ball();
    let distance_mm =
      update_dribble_distance_track(&mut self.dribble_distance_track, ball_pos, active_dribble);

    if distance_mm < DRIBBLE_RELEASE_DISTANCE_MM {
      return;
    }

    self.packets.robot_msg.speed = 0;
    self.packets.robot_msg.kick_pwr = self
      .packets
      .robot_msg
      .kick_pwr
      .max(DRIBBLE_LIMIT_KICK_POWER);
    self.packets.robot_msg.set_flag(send_flags::KICK);
  }
}

fn heading_error_deg(current: i32, target: i32) -> i32 {
  let error = (target - current + 180).rem_euclid(360) - 180;
  error.abs()
}

fn kick_release_ready(current: i32, target: i32, has_ball: bool) -> bool {
  has_ball && heading_error_deg(current, target) <= KICK_HEADING_TOLERANCE_DEG
}

fn position_receive_target(
  robot_pos: Vec2f, ball_pos: Vec2f, ball_vel: Vec2f, orient_deg: f32,
) -> Option<Vec2f> {
  let speed_sq = ball_vel.norm_squared();
  if speed_sq < POSITION_RECEIVE_MIN_BALL_SPEED_MM_S * POSITION_RECEIVE_MIN_BALL_SPEED_MM_S {
    return None;
  }

  let to_robot = robot_pos - ball_pos;
  let t = to_robot.dot(ball_vel) / speed_sq;
  if !(0f32..=POSITION_RECEIVE_MAX_TIME_S).contains(&t) {
    return None;
  }

  let closest = ball_pos + ball_vel.scale(t);
  if (closest - robot_pos).norm() > POSITION_RECEIVE_LATERAL_MM {
    return None;
  }

  let facing = Vec2f::new(orient_deg.to_radians().cos(), orient_deg.to_radians().sin());
  let incoming = ball_vel.normalized() * -1f32;
  if facing.dot(incoming) < POSITION_RECEIVE_FACING_DOT {
    return None;
  }

  let ball_dir = ball_vel.normalized();
  Some(closest + ball_dir * POSITION_RECEIVE_CENTER_OFFSET_MM)
}

fn update_dribble_distance_track(
  track: &mut Option<DribbleDistanceTrack>, ball_pos: Vec2f, active_dribble: bool,
) -> f32 {
  if !active_dribble {
    *track = None;
    return 0f32;
  }

  match track {
    Some(track) => {
      track.distance_mm += (ball_pos - track.last_ball_pos).norm();
      track.last_ball_pos = ball_pos;
      track.distance_mm
    }
    None => {
      *track = Some(DribbleDistanceTrack {
        last_ball_pos: ball_pos,
        distance_mm: 0f32,
      });
      0f32
    }
  }
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
  fn kick_release_requires_ir_ball_possession() {
    assert!(kick_release_ready(10, 12, true));
    assert!(!kick_release_ready(10, 12, false));
    assert!(!kick_release_ready(10, 20, true));
  }

  #[test]
  fn dribble_distance_track_accumulates_ball_travel() {
    let mut track = None;
    assert_eq!(
      update_dribble_distance_track(&mut track, Vec2f::new(0f32, 0f32), true),
      0f32
    );
    assert_eq!(
      update_dribble_distance_track(&mut track, Vec2f::new(300f32, 400f32), true),
      500f32
    );
    assert_eq!(
      update_dribble_distance_track(&mut track, Vec2f::new(300f32, 700f32), true),
      800f32
    );
  }

  #[test]
  fn dribble_distance_track_resets_without_controlled_dribble() {
    let mut track = None;
    update_dribble_distance_track(&mut track, Vec2f::new(0f32, 0f32), true);
    update_dribble_distance_track(&mut track, Vec2f::new(900f32, 0f32), true);
    assert!(track.is_some());

    assert_eq!(
      update_dribble_distance_track(&mut track, Vec2f::new(900f32, 0f32), false),
      0f32
    );
    assert!(track.is_none());
  }
}
