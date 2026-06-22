use core_dump::proto::CpVector2;
use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Vec2i {
  pub(crate) x: i32,
  pub(crate) y: i32,
}

impl Vec2i {
  #[inline]
  pub(crate) fn new(x: i32, y: i32) -> Self {
    Vec2i { x, y }
  }

  #[inline]
  pub(crate) fn new_from_cp(v: CpVector2) -> Vec2i {
    Vec2i::new(v.x, v.y)
  }

  #[inline]
  pub(crate) fn from_cp_vec2(v: &CpVector2) -> Self {
    Self { x: v.x, y: v.y }
  }

  #[inline]
  pub(crate) fn norm_squared(self) -> i32 {
    self.x * self.x + self.y * self.y
  }

  // #[inline]
  // pub(crate) fn calculate_vector_2i(a: CpVector2, b: CpVector2) -> Vec2i {
  //   Self::new(a.x - b.x, a.y - b.y)
  // }

  #[inline]
  pub(crate) fn with_speed_clamped(self, max_speed_mm_s: u32) -> Self {
    let max_speed = max_speed_mm_s as f64;
    let vx = self.x as f64;
    let vy = self.y as f64;
    let s = (vx * vx + vy * vy).sqrt();
    if s < 1e-6 {
      return Self::default();
    }
    if s <= max_speed {
      return self;
    }
    let k = max_speed / s;
    Self {
      x: (vx * k).round() as i32,
      y: (vy * k).round() as i32,
    }
  }
}

impl Add for Vec2i {
  type Output = Vec2i;

  fn add(self, rhs: Vec2i) -> Self::Output {
    Vec2i::new(self.x.saturating_add(rhs.x), self.y.saturating_add(rhs.y))
  }
}

impl Sub for Vec2i {
  type Output = Vec2i;

  fn sub(self, rhs: Vec2i) -> Self::Output {
    Vec2i::new(self.x.saturating_sub(rhs.x), self.y.saturating_sub(rhs.y))
  }
}

impl Mul<i32> for Vec2i {
  type Output = Vec2i;

  fn mul(self, rhs: i32) -> Self::Output {
    Vec2i::new(self.x.saturating_mul(rhs), self.y.saturating_mul(rhs))
  }
}

impl From<Vec2f> for Vec2i {
  fn from(value: Vec2f) -> Self {
    Vec2i::new(value.x.round() as i32, value.y.round() as i32)
  }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct Vec2f {
  pub(crate) x: f32,
  pub(crate) y: f32,
}

impl Vec2f {
  #[inline]
  pub(crate) fn new(x: f32, y: f32) -> Vec2f {
    Vec2f { x, y }
  }

  #[inline]
  pub(crate) fn new_from_vec2i(v: Vec2i) -> Self {
    Self::new(v.x as f32, v.y as f32)
  }

  #[inline]
  pub(crate) fn new_from_cp(v: CpVector2) -> Vec2f {
    Self::new(v.x as f32, v.y as f32)
  }

  #[inline]
  pub(crate) fn norm_squared(&self) -> f32 {
    self.x * self.x + self.y * self.y
  }

  #[inline]
  pub(crate) fn norm(self) -> f32 {
    self.x.hypot(self.y)
  }

  #[inline]
  pub(crate) fn normalized(self) -> Vec2f {
    let n = self.norm();
    if n <= 1e-6 {
      Self::new(0f32, 0f32)
    } else {
      self.scale(1f32 / n)
    }
  }

  #[inline]
  pub(crate) fn scale(self, s: f32) -> Vec2f {
    Self::new(self.x * s, self.y * s)
  }

  /// Scalar Product
  #[inline]
  pub(crate) fn dot(self, other: Vec2f) -> f32 {
    self.x * other.x + self.y * other.y
  }

  #[inline]
  pub(crate) fn det(self, other: Vec2f) -> f32 {
    self.x * other.y - self.y * other.x
  }

  #[inline]
  pub(crate) fn calculate_vector_2f(a: Vec2f, b: Vec2f) -> Self {
    Self::new(a.x - b.x, a.y - b.y)
  }

  #[inline]
  pub(crate) fn angle_from_y_axis(self) -> f32 {
    let mut angle = self.y.atan2(self.x).to_degrees() - 90f32;

    if angle <= -180f32 {
      angle += 360f32;
    }
    if angle > 180f32 {
      angle -= 360f32;
    }

    angle
  }

  // #[inline]
  // pub(crate) fn vec2f_to_cp(self) -> CpVector2 {
  //   CpVector2 {
  //     x: self.x as i32,
  //     y: self.y as i32,
  //   }
  // }

  #[inline]
  pub(crate) fn angle_to_u16(self) -> u16 {
    let mut deg = self.y.atan2(self.x).to_degrees();
    while deg < 0f32 {
      deg += 360f32;
    }
    while deg >= 360f32 {
      deg -= 360f32;
    }
    deg.round().clamp(0f32, 359f32) as u16
  }
}

impl Add for Vec2f {
  type Output = Vec2f;

  fn add(self, rhs: Self) -> Self::Output {
    Vec2f::new(self.x + rhs.x, self.y + rhs.y)
  }
}

impl Sub for Vec2f {
  type Output = Vec2f;

  fn sub(self, rhs: Self) -> Self::Output {
    Vec2f {
      x: self.x - rhs.x,
      y: self.y - rhs.y,
    }
  }
}

impl Mul for Vec2f {
  type Output = Vec2f;

  #[inline]
  fn mul(self, rhs: Self) -> Self::Output {
    Vec2f::new(self.x * rhs.x, self.y * rhs.y)
  }
}

impl Mul<f32> for Vec2f {
  type Output = Vec2f;

  #[inline]
  fn mul(self, rhs: f32) -> Self::Output {
    Vec2f::new(self.x * rhs, self.y * rhs)
  }
}

impl Div for Vec2f {
  type Output = Vec2f;

  fn div(self, rhs: Self) -> Self::Output {
    Vec2f::new(self.x / rhs.x, self.y / rhs.y)
  }
}

impl Div<f32> for Vec2f {
  type Output = Vec2f;

  #[inline]
  fn div(self, rhs: f32) -> Self::Output {
    Vec2f::new(self.x / rhs, self.y / rhs)
  }
}

impl Neg for Vec2f {
  type Output = Vec2f;
  fn neg(self) -> Self::Output {
    Vec2f::new(-self.x, -self.y)
  }
}

#[inline]
pub(crate) fn distance_cpv(a: CpVector2, b: CpVector2) -> f32 {
  let dx = (a.x - b.x) as f32;
  let dy = (a.y - b.y) as f32;
  (dx * dx + dy * dy).sqrt()
}

#[inline]
pub(crate) fn distance_vec2f(a: Vec2f, b: Vec2f) -> f32 {
  let c: Vec2f = Vec2f::new(a.x - b.x, a.y - b.y);
  c.norm()
}

#[inline]
pub(crate) fn lerp(a: f32, b: f32, t: f32) -> f32 {
  a + (b - a) * t.clamp(0f32, 1f32)
}

mod test {

  #[test]
  fn test_vec2f_add() {
    let a = crate::robot_logic::helpers::Vec2f::new(10f32, 20f32);
    let b = crate::robot_logic::helpers::Vec2f::new(40f32, 30f32);

    let c = a + b;

    assert_eq!(c, crate::robot_logic::helpers::Vec2f::new(50f32, 50f32));
  }

  #[test]
  fn test_vec2f_sub() {
    let a = crate::robot_logic::helpers::Vec2f::new(10f32, 20f32);
    let b = crate::robot_logic::helpers::Vec2f::new(40f32, 30f32);

    let c = a - b;

    assert_eq!(c, crate::robot_logic::helpers::Vec2f::new(-30f32, -10f32));
  }

  #[test]
  fn test_vec2f_mul() {
    let a = crate::robot_logic::helpers::Vec2f::new(10f32, 20f32);
    let b = crate::robot_logic::helpers::Vec2f::new(40f32, 30f32);

    let c = a * b;

    assert_eq!(c, crate::robot_logic::helpers::Vec2f::new(400f32, 600f32));
  }

  #[test]
  fn test_vec2f_div() {
    let a = crate::robot_logic::helpers::Vec2f::new(10f32, 40f32);
    let b = crate::robot_logic::helpers::Vec2f::new(50f32, 10f32);

    let c = a / b;

    assert_eq!(c, crate::robot_logic::helpers::Vec2f::new(0.2f32, 4f32));
  }
}
