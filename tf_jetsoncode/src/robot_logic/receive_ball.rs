use crate::communication::{TeensySendMsg, VisionMsg};
use crate::proto::{CpRobot, CpTrackedRobot};
use crate::robot_logic::helpers::{distance_vec2f, raw_move_towards, raw_movement_accel, Vec2f};

#[inline]
pub(crate) fn receive_ball(
  cp_data: &CpRobot, robot_self: CpTrackedRobot, _vision: &VisionMsg, mut msg: TeensySendMsg,
) -> TeensySendMsg {
  let robot_pos = Vec2f::new_from_cp(robot_self.pos);
  let ball_pos = Vec2f::new_from_cp(cp_data.ball.pos);
  let ball_vel = Vec2f::new_from_cp(cp_data.ball.vel.unwrap_or_default());


  // Check if ball is even moving towards robot
  if !is_moving_towards(ball_pos, ball_vel, robot_pos, 2000f32) {
    msg.speed = 0;
    return msg
  }
  let forward = (ball_pos - robot_pos).normalized();
  let interception_point = match intercept_with_constraints(robot_pos, forward, ball_pos, ball_vel) {
    Some(point) => point,
    None => {
      robot_pos
    }
  };
  msg = raw_move_towards(msg, robot_pos, interception_point);
  if distance_vec2f(robot_pos, ball_pos) <= 100f32 {
    msg.speed = 0;
  }

  msg
}

#[inline]
fn intercept_with_constraints(
  robot_pos: Vec2f,
  forward: Vec2f, // normalized direction robot considers "front"
  ball_pos: Vec2f,
  ball_vel: Vec2f,
) -> Option<Vec2f> {
  let max_t = 10f32; // horizon in seconds (tune)

  let mut lo = 0f32;
  let mut hi = max_t;

  let mut best: Option<(f32, Vec2f)> = None;

  for _ in 0..30 {
    let mid = (lo + hi) * 0.5;

    let ball_p = ball_pos + ball_vel * mid;

    let to_ball = ball_p - robot_pos;

    // reject "front" targets
    if to_ball.dot(forward) > 0f32 {
      lo = mid;
      continue;
    }

    let dist = to_ball.norm_squared().sqrt();
    let speed = raw_movement_accel(dist);
    let robot_time = dist / speed;

    let diff = robot_time - mid;

    if diff <= 0f32 {
      best = Some((mid, ball_p));
      hi = mid;
    } else {
      lo = mid;
    }
  }

  best.map(|(_, p)| p)
}

#[inline]
fn is_moving_towards(
  ball_pos: Vec2f,
  ball_vel: Vec2f,
  robot_pos: Vec2f,
  intercept_radius: f32,
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

