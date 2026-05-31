use crate::communication::{send_flags, TeensySendMsg, VisionMsg};
use crate::config::Config;
use crate::proto::{CpRobot, CpTrackedRobot};
use crate::robot_logic::helpers::{Vec2f, Vec2i, distance_cpv};
use crate::robot_logic::orca;
use crate::robot_logic::orca::OrcaOptions;
use std::f32::consts::PI;

const MAX_ADD_D: f32 = 0f32; // 220 //300
const MAX_ADD_A: f32 = 0f32; // 160 //220
const MIN_BALL_VEL: f32 = 700f32;

/// Function drives near the ball with orca and then tries to get the ball using Junior code
#[inline]
pub fn get_ball(
  cfg: &Config, cp_data: &CpRobot, _vision_data: &VisionMsg, mut msg: TeensySendMsg,
  robot_self: CpTrackedRobot,
) -> TeensySendMsg {
  let dist = distance_cpv(robot_self.pos, cp_data.ball.pos);

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
    let ball_pos = Vec2f::new_from_cp(cp_data.ball.pos);
    let ball_vel = Vec2f::new_from_cp(cp_data.ball.vel.unwrap_or_default());
    let to_ball = Vec2i::calculate_vector_2i(robot_self.pos, (ball_pos+ball_vel.scale(0.005f32)).vec2f_to_cp());

    // Transformation vector with respected input angle
    let trans_vector = Vec2f {
      x: -to_ball.x as f32 * f32::sin((robot_self.orientation as f32).to_radians())
        + to_ball.y as f32 * f32::cos((robot_self.orientation as f32).to_radians()),
      y: -to_ball.x as f32 * f32::cos((robot_self.orientation as f32).to_radians())
        - to_ball.y as f32 * f32::sin((robot_self.orientation as f32).to_radians()),
    };

    let mut comp_dir: f32;
    let x_c: f32 = 100f32;
    let y_c: f32 = 0f32;
    let d: f32 = 160f32;

    if trans_vector.x < 0f32 {
      comp_dir = compute_vector_angle(x_c, y_c, d, -trans_vector.x, -trans_vector.y).to_degrees();
    } else {
      comp_dir =
        180f32 - compute_vector_angle(x_c, y_c, d, trans_vector.x, -trans_vector.y).to_degrees();
    }

    if (trans_vector.x < 25f32)
      && (trans_vector.x > -25f32)
      && (trans_vector.y < 100f32)
      && (trans_vector.y > 0f32)
    {
      comp_dir = 90f32;
    }

    comp_dir = comp_dir - 90f32 + robot_self.orientation as f32;

    while comp_dir.is_sign_negative() {
      comp_dir += 360f32;
    }

    // Speed
    let abs_ball_angle = 90f32 - trans_vector.y.atan2(trans_vector.x).to_degrees().abs();
    let help_d_vel = ((trans_vector.norm() * 0.1).powf(4.2) / 70000f32).min(MAX_ADD_D);
    let help_a_vel = (abs_ball_angle * 0.15).powf(1.3).min(MAX_ADD_A); // 0.15 1.6
    let target_v = MIN_BALL_VEL + help_d_vel + help_a_vel;

    // Set Teensy message
    msg.dir = comp_dir as u16;
    msg.speed = target_v as u16;
  }
  // Enable Dribbler
  msg.set_flag(send_flags::DRIBBLER);
  msg.dribbler_pwr = 200;
  // Set orientation
  msg.orient = cp_data.cmd.orientation.unwrap_or_default() as u16;
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
