use crate::proto::{CpVector2, Vector2f};

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

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Vec2i {
  pub(crate) x: i32,
  pub(crate) y: i32,
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

