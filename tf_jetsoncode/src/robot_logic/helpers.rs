use crate::config;
use crate::proto::{CpVector2, Vector2f};

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
  Vec2i {
    x: a.x - b.x,
    y: a.y - b.y,
  }
}

#[inline]
pub fn calculate_vector_2f(a: CpVector2, b: CpVector2) -> Vector2f {
  Vector2f {
    x: (a.x - b.x) as f32,
    y: (a.y - b.y) as f32,
  }
}

#[inline]
pub fn vec2i_length(v: Vec2i) -> f32 {
  let x = v.x as f32;
  let y = v.y as f32;
  (x * x + y * y).sqrt()
}

pub fn cp_vec2i_length(v: CpVector2) -> f32 {
  let x = v.x as f32;
  let y = v.y as f32;
  (x * x + y * y).sqrt()
}

#[inline]
pub fn vec2i_to_f32(v: Vec2i) -> (f32, f32) {
  (v.x as f32, v.y as f32)
}

pub fn cpv_to_vec2i(v: CpVector2) -> Vec2i {Vec2i{x: v.x, y: v.y}}

#[inline]
fn normalize_degrees(mut deg: f32) -> u16 {
  while deg.is_sign_negative() {
    deg += 360.0;
  }
  deg %= 360.0;
  deg as u16
}

#[inline]
pub fn is_inside_field(point: Vec2f, cfg: &config::Config) -> bool {
  point.x >= 0.0
    && point.x <= cfg.field.width_mm()
    && point.y >= 0.0
    && point.y <= cfg.field.height_mm()
}
