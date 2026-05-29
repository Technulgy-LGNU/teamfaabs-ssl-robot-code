use crate::config;
use crate::proto::{CpVector2, Vector2, Vector2f};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Vec2i {
  pub(crate) x: i32,
  pub(crate) y: i32,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vec2f {
  pub(crate) x: f32,
  pub(crate) y: f32,
}

#[inline]
pub fn distance_cpv(a: CpVector2, b: CpVector2) -> f32 {
  let dx = (a.x - b.x) as f32;
  let dy = (a.y - b.y) as f32;
  (dx * dx + dy * dy).sqrt()
}

#[inline]
pub fn calculate_vector_2i(a: CpVector2, b: CpVector2) -> Vec2i {
  Vec2i { x: a.x - b.x, y: a.y - b.y }
}

#[inline]
pub fn calculate_vector_2f(a: CpVector2, b: CpVector2) -> Vector2f {
  Vector2f { x: (a.x - b.x) as f32, y: (a.y - b.y) as f32 }
}

#[inline]
pub fn vec2i_length(v: Vec2i) -> f32 {
  let x = v.x as f32;
  let y = v.y as f32;
  (x * x + y * y).sqrt()
}

#[inline]
pub fn cp_vec2i_length(v: CpVector2) -> f32 {
  let x = v.x as f32;
  let y = v.y as f32;
  (x * x + y * y).sqrt()
}

#[inline]
pub fn vec2i_to_f32(v: Vec2i) -> (f32, f32) {
  (v.x as f32, v.y as f32)
}

#[inline]
pub fn cpv_to_vec2i(v: CpVector2) -> Vec2i {
  Vec2i { x: v.x, y: v.y }
}

#[inline]
pub(crate) fn vec2(x: f32, y: f32) -> Vector2 {
  Vector2 { x, y }
}

#[inline]
pub(crate) fn sub(a: Vector2, b: Vector2) -> Vector2 {
  vec2(a.x - b.x, a.y - b.y)
}

#[inline]
pub(crate) fn norm(v: Vector2) -> f32 {
  v.x.hypot(v.y)
}

#[inline]
pub(crate) fn lerp(a: f32, b: f32, t: f32) -> f32 {
  a + (b - a) * t.clamp(0.0, 1.0)
}

#[inline]
pub(crate) fn cp_to_cp(v: Vector2) -> CpVector2 {
  CpVector2 { x: v.x as i32, y: v.y as i32 }
}

#[inline]
pub(crate) fn vec2_from_cp(v: CpVector2) -> Vector2 {
  vec2(v.x as f32, v.y as f32)
}

#[inline]
pub(crate) fn own_goal_x(cfg: &config::Config) -> f32 {
  let half_length = cfg.field.width_mm() * 0.5;
  if cfg.robot_goal { half_length } else { -half_length }
}

#[inline]
pub(crate) fn own_goal_side(cfg: &config::Config) -> f32 {
  if cfg.robot_goal { 1.0 } else { -1.0 }
}

#[inline]
pub(crate) fn angle_to_u16(v: Vector2) -> u16 {
  let mut deg = v.y.atan2(v.x).to_degrees();
  while deg < 0.0 {
    deg += 360.0;
  }
  while deg >= 360.0 {
    deg -= 360.0;
  }
  deg.round().clamp(0.0, 359.0) as u16
}

#[inline]
pub(crate) fn inside_own_penalty_area(cfg: &config::Config, pos: Vector2) -> bool {
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  let penalty_depth = cfg.field.penalty_area_height_mm().max(1.0);
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let x_min = goal_x.min(penalty_outer_x);
  let x_max = goal_x.max(penalty_outer_x);
  let y_half = cfg.field.penalty_area_width_mm().max(1.0) * 0.5;

  pos.x >= x_min && pos.x <= x_max && pos.y >= -y_half && pos.y <= y_half
}

#[inline]
pub(crate) fn clamp_outside_own_penalty(cfg: &config::Config, point: Vector2) -> Vector2 {
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  let penalty_depth = cfg.field.penalty_area_height_mm().max(1.0);
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let y_half = cfg.field.penalty_area_width_mm().max(1.0) * 0.5;
  let safety_margin = 40.0;

  let x = if goal_side > 0.0 {
    point.x.min(penalty_outer_x - safety_margin)
  } else {
    point.x.max(penalty_outer_x + safety_margin)
  };

  vec2(
    x,
    point.y.clamp(-y_half + safety_margin, y_half - safety_margin),
  )
}
