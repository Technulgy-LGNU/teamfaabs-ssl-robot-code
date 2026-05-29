use crate::communication::{TeensySendMsg, VisionMsg};
use crate::proto::{CpRobot, CpTrackedRobot, CpVector2};
use crate::robot_logic::helpers::{Vec2f, Vec2i, cpv_to_vec2i, is_inside_field};
use crate::config;

/// Center Point of circle
const M_K_U: i16 = -4500;

#[inline]
pub fn goalie(
  cfg: &config::Config, cp_data: &CpRobot, robot_self: &CpTrackedRobot, _vision: &VisionMsg,
  mut msg: TeensySendMsg,
) -> TeensySendMsg {
  let goal_side = goal_side(cfg);
  let goal_point = Vec2i {
    x: cfg.field.width_mm() as i32 / 2 * goal_side as i32,
    y: 0,
  };
  let center_imaginary_circle = Vec2i {
    x: M_K_U as i32 * goal_side as i32,
    y: 0,
  };
  let radius_imaginary_circle: u16 = 1200;
  // Orientation for the robot
  // let angle_to_ball = f32::atan2(
  //   (cp_data.ball.pos.y as f32) - (robot_self.pos.y as f32),
  //   (cp_data.ball.pos.x as f32) - (robot_self.pos.x as f32),
  // ).to_degrees();
  let target = get_intersection_in_field(
    cp_data.ball.pos,
    goal_point,
    cfg,
    center_imaginary_circle,
    radius_imaginary_circle,
  )
  .unwrap_or(Vec2i { x: 0, y: 0 });

  msg.dir = get_dir(target, cpv_to_vec2i(robot_self.pos));
  //msg.speed = decel_limit(500.0, distance(target, cpv_to_vec2i(robot_self.pos)), DEFAULT_DECEL_MM_S2) as u16;
  // msg.speed = accel_limit(
  //   ((robot_self.vel.unwrap_or_default().x * robot_self.vel.unwrap_or_default().x
  //     + robot_self.vel.unwrap_or_default().y * robot_self.vel.unwrap_or_default().y) as f32)
  //     .sqrt(),
  //   distance(target, cpv_to_vec2i(robot_self.pos)),
  //   DEFAULT_ACCEL_MM_S2,
  //   2.0,
  // ) as u16;
  msg.speed = 400;

  // msg.orient = angle_to_ball as u16;
  msg.orient = 0;
  msg
}

#[inline]
fn get_intersection_in_field(
  ball: CpVector2, goal_point: Vec2i, cfg: &config::Config, circle_center: Vec2i,
  circle_radius: u16,
) -> Option<Vec2i> {
  // Direction vector of the segment
  let dx = goal_point.x - ball.x;
  let dy = goal_point.y - ball.y;

  // Vector from circle center to segment start
  let fx = ball.x - circle_center.x;
  let fy = ball.y - circle_center.y;

  // Quadratic coefficients
  let a = dx * dx + dy * dy;
  let b = 2.0 * ((fx * dx + fy * dy) as f32);
  let c = fx * fx + fy * fy - circle_radius as i32 * circle_radius as i32;

  // Discriminant
  let discriminant = b * b - 4.0 * a as f32 * c as f32;

  if discriminant < 0.0 {
    return None; // No intersection
  }

  let sqrt_discriminant = discriminant.sqrt();
  let inv_2a = 0.5 / a as f32;

  // Compute both t values
  let t1 = (-b - sqrt_discriminant) * inv_2a;
  let t2 = (-b + sqrt_discriminant) * inv_2a;

  // Helper closure avoids duplicate code
  let check_point = |t: f32| -> Option<Vec2i> {
    if !(0.0..=1.0).contains(&t) {
      return None;
    }

    let p = Vec2f {
      x: ball.x as f32 + t * dx as f32,
      y: ball.y as f32 + t * dy as f32,
    };

    is_inside_field(p, &cfg).then(|| Vec2i {
      x: p.x as i32,
      y: p.y as i32,
    })
  };

  // Return first valid intersection
  check_point(t1).or_else(|| check_point(t2))
}

#[inline]
fn goal_side(cfg: &config::Config) -> f32 {
  if cfg.robot_goal { 1.0 } else { -1.0 }
}

#[inline]
fn goal_line_x(cfg: &config::Config) -> f32 {
  goal_side(cfg) * (cfg.field.width_mm() * 0.5)
}

#[inline]
fn get_dir(point: Vec2i, pos: Vec2i) -> u16 {
  // Vector from pos to point
  let vec: Vec2i = Vec2i {
    x: point.x - pos.x,
    y: point.y - pos.y,
  };
  // Get angle
  f32::atan2(vec.y as f32, vec.x as f32).to_degrees() as u16
}

#[inline]
fn distance(a: Vec2i, b: Vec2i) -> f32 {
  let dx = (a.x - b.x) as f32;
  let dy = (a.y - b.y) as f32;
  (dx * dx + dy * dy).sqrt()
}
