use crate::communication::{TeensySendMsg, VisionMsg};
use crate::config::Config;
use crate::proto::{CpRobot, CpTrackedRobot};
use crate::robot_logic::helpers::{distance_cpv, distance_vec2f, raw_move_towards, Vec2f, Vec2i, RAW_MAX_SPEED_MM_S};
use crate::robot_logic::orca::{self, OrcaOptions};
use std::f32::consts::PI;

/// Function drives near the ball with orca and then tries to get the ball using Junior code
#[inline]
pub async fn get_ball(
  cfg: &Config, cp_data: &CpRobot, _vision_data: &VisionMsg, mut msg: TeensySendMsg,
  robot_self: CpTrackedRobot,
) -> TeensySendMsg {
  let dist = distance_cpv(robot_self.pos, cp_data.ball.pos);
  println!("Distance to ball: {:?}", dist);

  // Check distance to ball, either use orca for long distance or use direct control for taking the ball
  if dist > 500f32 {
    let plan = orca::drive_to_target(
      cfg,
      cp_data,
      robot_self,
      cp_data.ball.pos,
      OrcaOptions {
        max_speed_mm_s: 1_500f32,
        stop_radius_mm: 180f32,
        avoid_ball: false,
        ..OrcaOptions::default()
      },
    );
    msg = orca::orca_to_teensy(msg, &plan, robot_self);
  } else {
    // Calculate direction to ball as Vec2i
    let to_ball = Vec2i::calculate_vector_2i(robot_self.pos, cp_data.ball.pos);

    // Transformation vector with respected input angle
    let trans_vector = Vec2f {
      x: -to_ball.x as f32 * f32::sin((cp_data.cmd.orientation() as f32).to_radians())
        + to_ball.y as f32 * f32::cos((cp_data.cmd.orientation() as f32).to_radians()),
      y: -to_ball.x as f32 * f32::cos((cp_data.cmd.orientation() as f32).to_radians())
        - to_ball.y as f32 * f32::sin((cp_data.cmd.orientation() as f32).to_radians()),
    };

    let mut comp_dir: f32;
    let x_c: f32 = 120f32;
    let y_c: f32 = 0f32;
    let d: f32 = 125f32;

    if trans_vector.x < 0f32 {
      comp_dir = compute_vector_angle(x_c, y_c, d, -trans_vector.x, -trans_vector.y).to_degrees();
    } else {
      comp_dir =
        180f32 - compute_vector_angle(x_c, y_c, d, trans_vector.x, -trans_vector.y).to_degrees();
    }

    if (trans_vector.x < 15f32)
      && (trans_vector.x > -15f32)
      && (trans_vector.y < 100f32)
      && (trans_vector.y > 0f32)
    {
      comp_dir = 90f32;
    }

    comp_dir = comp_dir - 90f32 + robot_self.orientation as f32;

    println!("Computed Direction: {:?}", comp_dir);

    while comp_dir.is_sign_negative() {
      comp_dir += 360f32;
    }

    // Set Teensy message
    msg.dir = comp_dir as u16;
    msg.speed = 300;
  }
  msg
}

/// Arduino constraint()
#[inline]
fn constrain(value: f32, min: f32, max: f32) -> f32 {
  value.clamp(min, max)
}

/// Calculate the angle from a vector relative to a circle
fn compute_vector_angle(x_c: f32, y_c: f32, r: f32, x: f32, y: f32) -> f32 {
  // Distance to circle center
  let mut d = (x - x_c).hypot(y - y_c);

  // Avoid dividing through zero
  if d.is_nan() || d == 0f32 {
    d = 1e-6;
  }

  let angle;

  if d > r {
    // Calculate tangential angle
    let theta = (y - y_c).atan2(x - x_c);

    let alpha = constrain(r / d, -1f32, 1f32).asin();

    // Calculate angle of tangent
    angle = PI + theta + alpha;
  } else {
    // Calculate mirror
    let theta = (y - y_c).atan2(x - x_c);

    // Point in circle
    let i_c_x = theta.cos() * d;
    let i_c_y = theta.sin() * d;

    // Mirror point
    let o_c_x = theta.cos() * (2f32 * r - d);
    let o_c_y = theta.sin() * (2f32 * r - d);

    // If value is NaN, return 0
    if i_c_x.is_nan() || i_c_y.is_nan() || o_c_x.is_nan() || o_c_y.is_nan() {
      return 0f32;
    }

    // Calculate mirror matrix
    let theta3 = (i_c_y - y_c).atan2(i_c_x - x_c);

    let s11 = (2f32 * theta3).cos();
    let s12 = (2f32 * theta3).sin();
    let s21 = (2f32 * theta3).sin();
    let s22 = -(2f32 * theta3).cos();

    // Mirrored tangential angle
    let theta_sp = (o_c_y - y_c).atan2(o_c_x - x_c);

    let mut denom = (o_c_x - x_c).hypot(o_c_y - y_c);

    // Avoid NaN because of invalid value
    if denom.is_nan() || denom == 0f32 {
      denom = 1e-6;
    }

    let alpha_sp = constrain(r / denom, -1f32, 1f32).asin();

    let theta1_sp = PI + theta_sp + alpha_sp;

    // Transform the angle
    let new_x = -(s11 * theta1_sp.cos() + s12 * theta1_sp.sin());

    let new_y = -(s21 * theta1_sp.cos() + s22 * theta1_sp.sin());

    // Avoid NaN in atan2()
    if new_x.is_nan() || new_y.is_nan() {
      return 0f32;
    }

    angle = new_y.atan2(new_x);
  }

  angle
}

#[inline]
pub(crate) fn receive_ball(
  cp_data: &CpRobot, robot_self: CpTrackedRobot, _vision: &VisionMsg, mut msg: TeensySendMsg,
) -> TeensySendMsg {
  let self_pos = Vec2f::new_from_cp(robot_self.pos);
  let ball_pos = Vec2f::new_from_cp(cp_data.ball.pos);
  let ball_vel = Vec2f::new_from_cp(cp_data.ball.vel.unwrap_or_default());


  // Check if ball is even moving towards robot
  if !is_moving_towards(ball_pos, ball_vel, self_pos) {
    msg.speed = 0;
    return msg
  }
  // let interception_point = match intercept_ball(ball_pos, ball_vel, self_pos) {
  //   Some(point) => point,
  //   None => {
  //     self_pos
  //   }
  // };
  let forward = (ball_pos - self_pos).normalized();
  let interception_point = match intercept_with_constraints(self_pos, forward, ball_pos, ball_vel) {
    Some(point) => point,
    None => {
      self_pos
    }
  };
  msg = raw_move_towards(msg, self_pos, interception_point);
  if distance_vec2f(self_pos, ball_pos) <= 100f32 {
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
  let max_t = 10.0; // horizon in seconds (tune)

  let mut lo = 0.0;
  let mut hi = max_t;

  let mut best: Option<(f32, Vec2f)> = None;

  for _ in 0..30 {
    let mid = (lo + hi) * 0.5;

    let ball_p = ball_pos + ball_vel * mid;

    let to_ball = ball_p - robot_pos;

    // reject "front" targets
    if to_ball.dot(forward) > 0.0 {
      lo = mid;
      continue;
    }

    let dist = to_ball.norm_squared().sqrt();
    let speed = speed_from_distance(dist);
    let robot_time = dist / speed;

    let diff = robot_time - mid;

    if diff <= 0.0 {
      best = Some((mid, ball_p));
      hi = mid;
    } else {
      lo = mid;
    }
  }

  best.map(|(_, p)| p)
}

#[inline]
fn speed_from_distance(dist: f32) -> f32 {
  (dist * 3.0).clamp(60.0, RAW_MAX_SPEED_MM_S)
}

#[inline]
fn robot_can_reach(
  robot_pos: Vec2f,
  target: Vec2f,
) -> bool {
  let d = target - robot_pos;
  let dist = d.norm_squared().sqrt();

  let speed = speed_from_distance(dist);

  let time_needed = dist / speed;

  // "reachable at time t" is checked outside
  time_needed <= 1.0 // placeholder scaling; overridden in search
}

#[inline]
pub(crate) fn is_moving_towards(
  ball_pos: Vec2f,
  ball_vel: Vec2f,
  robot_pos: Vec2f,
) -> bool {
  let to_robot = robot_pos - ball_pos;
  to_robot.dot(ball_vel) > 0.0
}
