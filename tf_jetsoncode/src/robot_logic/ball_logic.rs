use crate::communication::{TeensySendMsg, VisionMsg};
use crate::proto::{CpRobot, CpTrackedRobot, Vector2f};
use crate::robot_logic::helpers::{calculate_vector, distance_cpv};
use crate::robot_logic::orca;
use crate::robot_logic::orca::{
  OrcaHandle, OrcaRequest, Vec2i, WorldSnapshot, nav_command_to_teensy,
};
use std::f32::consts::PI;

pub async fn get_ball(
  cp_data: &CpRobot, orca: &mut OrcaHandle, world: &WorldSnapshot, vision_data: &VisionMsg,
  mut msg: TeensySendMsg, robot_self: CpTrackedRobot,
) -> TeensySendMsg {
  let dist = distance_cpv(robot_self.pos, cp_data.ball.pos);
  println!("Distance to ball: {:?}", dist);

  // Check distance to ball, either use orca for long distance or use direct control for taking the ball
  if dist > 1500.0 {
    let intent = orca::NavIntent::GoToPosition {
      target_pos_mm: Vec2i {
        x: cp_data.ball.pos.x,
        y: cp_data.ball.pos.y,
      },
      max_speed_mm_s: cp_data.cmd.speed(),
    };

    orca.publish(OrcaRequest {
      world: world.clone(),
      intent,
    });

    let orca_cmd = orca.changed().await.unwrap_or_default();

    println!("Orca output: {:?}", orca_cmd);
    msg = nav_command_to_teensy(msg, orca_cmd);
  } else {
    // Calculate direction to ball as Vec2i
    let to_ball = calculate_vector(robot_self.pos, cp_data.ball.pos);

    // Transformation vector with respected input angle
    let trans_vector = Vector2f {
      x: -to_ball.x as f32 * f32::sin((cp_data.cmd.orientation() as f32).to_radians())
        + to_ball.y as f32 * f32::cos((cp_data.cmd.orientation() as f32).to_radians()),
      y: -to_ball.x as f32 * f32::cos((cp_data.cmd.orientation() as f32).to_radians())
        - to_ball.y as f32 * f32::sin((cp_data.cmd.orientation() as f32).to_radians()),
    };

    let mut comp_dir: f32;
    let x_c: f32 = 120.0;
    let y_c: f32 = 0.0;
    let d: f32 = 125.0;

    if trans_vector.x < 0.0 {
      comp_dir = compute_vector_angle(x_c, y_c, d, -trans_vector.x, -trans_vector.y).to_degrees();
    } else {
      comp_dir =
        180.0 - compute_vector_angle(x_c, y_c, d, trans_vector.x, -trans_vector.y).to_degrees();
    }

    if (trans_vector.x < 15.0)
      && (trans_vector.x > -15.0)
      && (trans_vector.y < 100.0)
      && (trans_vector.y > 0.0)
    {
      comp_dir = 90.0;
    }

    comp_dir = comp_dir - 90.0 + robot_self.orientation as f32;

    println!("Computed Direction: {:?}", comp_dir);

    while comp_dir.is_sign_negative() {
      comp_dir += 360.0;
    }

    // Set Teensy message
    msg.dir = comp_dir as u16;
    msg.speed = 300;
  }
  msg
}

/// Arduino constraint()
fn constrain(value: f32, min: f32, max: f32) -> f32 {
  value.clamp(min, max)
}

/// Calculate the angle from a vector relative to a circle
fn compute_vector_angle(x_c: f32, y_c: f32, r: f32, x: f32, y: f32) -> f32 {
  // Distance to circle center
  let mut d = (x - x_c).hypot(y - y_c);

  // Avoid dividing through zero
  if d.is_nan() || d == 0.0 {
    d = 1e-6;
  }

  let angle;

  if d > r {
    // Calculate tangential angle
    let theta = (y - y_c).atan2(x - x_c);

    let alpha = constrain(r / d, -1.0, 1.0).asin();

    // Calculate angle of tangent
    angle = PI + theta + alpha;
  } else {
    // Calculate mirror
    let theta = (y - y_c).atan2(x - x_c);

    // Point in circle
    let i_c_x = theta.cos() * d;
    let i_c_y = theta.sin() * d;

    // Mirror point
    let o_c_x = theta.cos() * (2.0 * r - d);
    let o_c_y = theta.sin() * (2.0 * r - d);

    // If value is NaN, return 0
    if i_c_x.is_nan() || i_c_y.is_nan() || o_c_x.is_nan() || o_c_y.is_nan() {
      return 0.0;
    }

    // Calculate mirror matrix
    let theta3 = (i_c_y - y_c).atan2(i_c_x - x_c);

    let s11 = (2.0 * theta3).cos();
    let s12 = (2.0 * theta3).sin();
    let s21 = (2.0 * theta3).sin();
    let s22 = -(2.0 * theta3).cos();

    // Mirrored tangential angle
    let theta_sp = (o_c_y - y_c).atan2(o_c_x - x_c);

    let mut denom = (o_c_x - x_c).hypot(o_c_y - y_c);

    // Avoid NaN because of invalid value
    if denom.is_nan() || denom == 0.0 {
      denom = 1e-6;
    }

    let alpha_sp = constrain(r / denom, -1.0, 1.0).asin();

    let theta1_sp = PI + theta_sp + alpha_sp;

    // Transform the angle
    let new_x = -(s11 * theta1_sp.cos() + s12 * theta1_sp.sin());

    let new_y = -(s21 * theta1_sp.cos() + s22 * theta1_sp.sin());

    // Avoid NaN in atan2()
    if new_x.is_nan() || new_y.is_nan() {
      return 0.0;
    }

    angle = new_y.atan2(new_x);
  }

  angle
}
