use crate::proto::CpVector2;

pub fn distance_cpv(a: CpVector2, b: CpVector2) -> f32 {
  let dx = (a.x - b.x) as f32;
  let dy = (a.y - b.y) as f32;
  (dx * dx + dy * dy).sqrt()
}

pub fn calculate_vector(a: CpVector2, b: CpVector2) -> Vec2i {
  Vec2i {
    x: a.x - b.x,
    y: a.y - b.y,
  }
}

#[derive(Debug, Clone, Copy), inline]
pub struct Vec2i {
  pub(crate) x: i32,
  pub(crate) y: i32,
}
