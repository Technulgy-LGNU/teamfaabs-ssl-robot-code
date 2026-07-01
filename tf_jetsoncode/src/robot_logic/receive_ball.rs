use crate::Robot;
use crate::robot_logic::helpers::{raw_move_towards, raw_movement_accel};
use crate::robot_logic::vec::Vec2f;

const RECEIVE_CENTER_OFFSET_MM: f32 = 80f32;
const RECEIVE_INTERCEPT_HORIZON_S: f32 = 2.5;
const RECEIVE_TIMING_GRACE_S: f32 = 0.05;
const RECEIVE_EXPECTED_SPEED_SCALE: f32 = 0.62;
const SLOW_RECEIVE_COLLECT_RADIUS_MM: f32 = 650f32;

impl<C> Robot<C> {
  #[inline]
  pub(crate) fn receive_ball(&mut self) -> bool {
    let robot_pos = Vec2f::new_from_cp(self.packets.robot_self.pos);
    let ball_pos = Vec2f::new_from_cp(self.packets.cp_data.ball.pos);
    let ball_vel = Vec2f::new_from_cp(self.packets.cp_data.ball.vel.unwrap_or_default());

    // Check if ball is even moving towards robot
    if !is_moving_towards(ball_pos, ball_vel, robot_pos, 2000f32) {
      self.packets.robot_msg.speed = 0;
      return false;
    }

    let interception_point =
      intercept_with_constraints(robot_pos, ball_pos, ball_vel).unwrap_or_else(|| robot_pos);

    raw_move_towards(&mut self.packets.robot_msg, robot_pos, interception_point);

    if (robot_pos - interception_point).norm_squared() <= 50f32 * 50f32 {
      self.packets.robot_msg.speed = 0;
    }
    true
  }

  #[inline]
  pub fn collect_receive_ball(&mut self, robot_pos: Vec2f, ball_pos: Vec2f) {
    let target = collect_center_target(robot_pos, ball_pos);
    let dist = (target - robot_pos).norm();
    if dist < 35f32 {
      self.packets.robot_msg.speed = 0;
      return;
    }

    raw_move_towards(&mut self.packets.robot_msg, robot_pos, target);
    self.packets.robot_msg.speed =
      self
        .packets
        .robot_msg
        .speed
        .max(if dist < 250f32 { 350 } else { 700 });
  }
}

#[inline]
pub(crate) fn should_collect_slow_receive_ball(robot_pos: Vec2f, ball_pos: Vec2f) -> bool {
  (robot_pos - ball_pos).norm_squared()
    <= SLOW_RECEIVE_COLLECT_RADIUS_MM * SLOW_RECEIVE_COLLECT_RADIUS_MM
}

#[inline]
fn intercept_with_constraints(robot_pos: Vec2f, ball_pos: Vec2f, ball_vel: Vec2f) -> Option<Vec2f> {
  if ball_vel.norm_squared() < 1e-6 {
    return None;
  }

  let ball_dir = ball_vel.normalized();
  let center_target_at = |t: f32| ball_pos + ball_vel * t + ball_dir * RECEIVE_CENTER_OFFSET_MM;

  let mut best_reachable: Option<(Vec2f, f32, f32)> = None;
  for step in 0..=30 {
    let t = RECEIVE_INTERCEPT_HORIZON_S * step as f32 / 30f32;
    let target = center_target_at(t);
    let dist = (target - robot_pos).norm();
    let robot_time = estimated_receive_travel_time(dist);

    if robot_time <= t + RECEIVE_TIMING_GRACE_S {
      let replace = best_reachable
        .map(|(_, best_dist, best_t)| {
          dist + 1f32 < best_dist || ((dist - best_dist).abs() <= 1f32 && t < best_t)
        })
        .unwrap_or(true);
      if replace {
        best_reachable = Some((target, dist, t));
      }
    }
  }

  if let Some((target, _, _)) = best_reachable {
    return Some(target);
  }

  let path_start = center_target_at(0f32);
  let closest_t = ((robot_pos - path_start).dot(ball_vel) / ball_vel.norm_squared())
    .clamp(0f32, RECEIVE_INTERCEPT_HORIZON_S);
  Some(center_target_at(closest_t))
}

#[inline]
fn estimated_receive_travel_time(dist_mm: f32) -> f32 {
  let expected_speed = raw_movement_accel(dist_mm) * RECEIVE_EXPECTED_SPEED_SCALE;
  dist_mm / expected_speed.max(1f32)
}

#[inline]
fn collect_center_target(robot_pos: Vec2f, ball_pos: Vec2f) -> Vec2f {
  let to_ball = ball_pos - robot_pos;
  if to_ball.norm() <= 1f32 {
    return robot_pos;
  }

  ball_pos - to_ball.normalized() * RECEIVE_CENTER_OFFSET_MM
}

#[inline]
fn is_moving_towards(
  ball_pos: Vec2f, ball_vel: Vec2f, robot_pos: Vec2f, intercept_radius: f32,
) -> bool {
  let v2 = ball_vel.norm_squared();

  if v2 < 1e-6 {
    return false;
  }

  let t = (robot_pos - ball_pos).dot(ball_vel) / v2;

  if t < -0.5 {
    return false; // closest approach already passed far in the past
  }

  let closest = ball_pos + ball_vel * t;

  let dist_sq = (closest - robot_pos).norm_squared();

  dist_sq <= intercept_radius * intercept_radius
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn intercept_targets_receiver_center_behind_incoming_ball() {
    let robot_pos = Vec2f::new(1000f32, 0f32);
    let ball_pos = Vec2f::new(0f32, 40f32);
    let ball_vel = Vec2f::new(1000f32, 0f32);

    let target = intercept_with_constraints(robot_pos, ball_pos, ball_vel).unwrap();
    let catch_point = target - ball_vel.normalized() * RECEIVE_CENTER_OFFSET_MM;

    assert!((target.x - catch_point.x - RECEIVE_CENTER_OFFSET_MM).abs() < 1f32);
    assert!((target.y - 40f32).abs() < 1f32);
  }

  #[test]
  fn intercept_uses_direct_reachable_cut_instead_of_earliest_chase_point() {
    let robot_pos = Vec2f::new(1000f32, -400f32);
    let ball_pos = Vec2f::new(0f32, 0f32);
    let ball_vel = Vec2f::new(1000f32, 0f32);

    let target = intercept_with_constraints(robot_pos, ball_pos, ball_vel).unwrap();

    assert!(
      target.x > 850f32,
      "expected receiver to cut near its closest ball-path point, got {target:?}"
    );
    assert!(target.y.abs() < 1f32);
  }

  #[test]
  fn receive_timing_estimate_is_below_raw_command_speed() {
    let dist = 600f32;
    let raw_time = dist / raw_movement_accel(dist);
    let receive_time = estimated_receive_travel_time(dist);

    assert!(receive_time > raw_time);
  }

  #[test]
  fn moving_towards_accepts_near_receiver_path() {
    assert!(is_moving_towards(
      Vec2f::new(0f32, 40f32),
      Vec2f::new(1000f32, 0f32),
      Vec2f::new(1000f32, 0f32),
      100f32,
    ));
  }

  #[test]
  fn collect_target_puts_receiver_mouth_on_loose_ball() {
    let robot_pos = Vec2f::new(0f32, 0f32);
    let ball_pos = Vec2f::new(200f32, 0f32);

    assert_eq!(
      collect_center_target(robot_pos, ball_pos),
      Vec2f::new(120f32, 0f32)
    );
  }

  #[test]
  fn slow_receive_collects_only_nearby_loose_ball() {
    let robot_pos = Vec2f::new(0f32, 0f32);

    assert!(should_collect_slow_receive_ball(
      robot_pos,
      Vec2f::new(300f32, 0f32),
    ));
    assert!(!should_collect_slow_receive_ball(
      robot_pos,
      Vec2f::new(900f32, 0f32),
    ));
  }
}
