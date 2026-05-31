//! ORCA (Optimal Reciprocal Collision Avoidance) – *starter* scaffolding.
//!
//! This module is intentionally **high level**:
//! - It gives you a clean async boundary (`OrcaHandle`) so the rest of your robot code can remain
//!   simple and deterministic.
//! - The actual ORCA math is represented by a placeholder `orca_step()` implementation.
//!
//! ## Intended data flow (120 Hz main loop)
//! 1. Main loop receives latest packets (`CpRobot`, Teensy IMU, onboard vision, ...).
//! 2. Main loop builds a `WorldSnapshot` + `NavIntent`.
//! 3. Main loop calls `orca.publish(...)` (latest-only).
//! 4. ORCA worker produces a `NavCommand` (desired velocity) which the main loop reads and
//!    translates to your motor/teensy protocol.
//!
//! ## Why a worker task?
//! ORCA can be CPU heavy. Putting it behind a worker:
//! - prevents your 120 Hz loop from stalling
//! - gives you one place to cache / pre-allocate ORCA state later (agent list, KD-tree, etc.)
//! - lets you `spawn_blocking` if you want to isolate CPU usage

use std::time::{Duration, Instant};

use tokio::sync::watch;

use crate::communication::TeensySendMsg;
use crate::proto::{CpRobot, CpTrackedRobot, CpVector2};

/// 2D integer vector in millimeters (or millimeters/second, depending on context).
///
/// You said “everything is i32”; internally we sometimes upcast to i64 for safety.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Vec2i {
  pub x: i32,
  pub y: i32,
}

impl Vec2i {
  #[inline]
  pub const fn new(x: i32, y: i32) -> Self {
    Self { x, y }
  }

  pub(crate) fn new_from_cp(v: CpVector2) -> Vec2i {
    Vec2i::new(v.x, v.y)
  }

  #[inline]
  pub fn len_sq_i64(self) -> i64 {
    let x = self.x as i64;
    let y = self.y as i64;
    x * x + y * y
  }

  #[inline]
  fn from_cp_vec2(v: &CpVector2) -> Self {
    Self { x: v.x, y: v.y }
  }

  /// Scales the vector to a target speed (mm/s) while preserving direction.
  /// If the vector is ~zero, returns zero.
  #[inline]
  pub fn with_speed_clamped(self, max_speed_mm_s: u32) -> Self {
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

impl std::ops::Add for Vec2i {
  type Output = Vec2i;
  fn add(self, rhs: Vec2i) -> Self::Output {
    Vec2i::new(self.x.saturating_add(rhs.x), self.y.saturating_add(rhs.y))
  }
}

impl std::ops::Sub for Vec2i {
  type Output = Vec2i;
  fn sub(self, rhs: Vec2i) -> Self::Output {
    Vec2i::new(self.x.saturating_sub(rhs.x), self.y.saturating_sub(rhs.y))
  }
}

impl std::ops::Mul<i32> for Vec2i {
  type Output = Vec2i;
  fn mul(self, rhs: i32) -> Self::Output {
    Vec2i::new(self.x.saturating_mul(rhs), self.y.saturating_mul(rhs))
  }
}

/// A tracked robot (other agent) as seen in the world model.
#[derive(Debug, Clone, Copy, Default)]
pub struct OtherRobot {
  pub id: u32,
  pub pos_mm: Vec2i,
  pub vel_mm_s: Vec2i,
  /// Collision radius for this robot (including safety margin).
  pub radius_mm: u32,
}

#[derive(Debug, Clone)]
pub struct WorldSnapshot {
  pub now: Instant,

  pub self_id: u32,
  pub self_pos_mm: Vec2i,
  pub self_vel_mm_s: Vec2i,
  /// Optional: your current heading (e.g. milliradians / degrees / whatever your stack uses).
  pub self_orientation: Option<i32>,

  /// All other robots (both teams), excluding self.
  pub others: Vec<OtherRobot>,
}

impl Default for WorldSnapshot {
  fn default() -> Self {
    Self {
      now: Instant::now(),
      self_id: 0,
      self_pos_mm: Vec2i::default(),
      self_vel_mm_s: Vec2i::default(),
      self_orientation: None,
      others: Vec::new(),
    }
  }
}

impl WorldSnapshot {
  /// Build a [`WorldSnapshot`] from a CrashPilot world packet.
  ///
  /// Notes:
  /// - CrashPilot supplies both teams; we simply merge them.
  /// - We find `self` by `self_id` inside either `robots_yellow` or `robots_blue`.
  /// - Units are assumed to already be *millimeters* and *millimeters/second* as described.
  pub fn from_cp(cp: &CpRobot, self_id: u32, default_robot_radius_mm: u32) -> Self {
    let (self_pos_mm, self_vel_mm_s, self_orientation) = find_self(cp, self_id)
      .map(|r| {
        (
          Vec2i::from_cp_vec2(&r.pos),
          r.vel
            .as_ref()
            .map(Vec2i::from_cp_vec2)
            .unwrap_or_default(),
          Some(r.orientation),
        )
      })
      .unwrap_or((Vec2i::default(), Vec2i::default(), None));

    let mut others = Vec::with_capacity(cp.robots_yellow.len() + cp.robots_blue.len());
    append_others(&mut others, &cp.robots_yellow, self_id, default_robot_radius_mm);
    append_others(&mut others, &cp.robots_blue, self_id, default_robot_radius_mm);

    Self {
      now: Instant::now(),
      self_id,
      self_pos_mm,
      self_vel_mm_s,
      self_orientation,
      others,
    }
  }
}

fn find_self(cp: &CpRobot, self_id: u32) -> Option<&CpTrackedRobot> {
  cp.robots_yellow
    .iter()
    .find(|r| r.robot_id == self_id)
    .or_else(|| cp.robots_blue.iter().find(|r| r.robot_id == self_id))
}

fn append_others(out: &mut Vec<OtherRobot>, src: &[CpTrackedRobot], self_id: u32, default_radius_mm: u32) {
  for r in src {
    if r.robot_id == self_id {
      continue;
    }
    out.push(OtherRobot {
      id: r.robot_id,
      pos_mm: Vec2i::from_cp_vec2(&r.pos),
      vel_mm_s: r.vel.as_ref().map(Vec2i::from_cp_vec2).unwrap_or_default(),
      radius_mm: default_radius_mm,
    });
  }
}

/// Higher-level navigation intent coming from “skills/behavior”.
///
/// ORCA typically returns a *velocity* that tries to follow a preferred velocity. Your behavior
/// layer decides that preferred velocity.
#[derive(Debug, Clone, Copy)]
pub enum NavIntent {
  /// Stop (preferred velocity 0).
  Stop,
  /// Drive towards a target position.
  GoToPosition {
    target_pos_mm: Vec2i,
    /// Maximum desired speed.
    max_speed_mm_s: u32,
  },
  /// Directly request a preferred velocity (world frame).
  PreferredVelocity {
    vel_mm_s: Vec2i,
    max_speed_mm_s: u32,
  },
}

/// Output of ORCA worker.
#[derive(Debug, Clone, Copy, Default)]
pub struct NavCommand {
  /// Collision-avoiding velocity in world frame.
  pub vel_mm_s: Vec2i,
  /// If you want: use this to expose why ORCA chose something (debug / tuning).
  pub debug: Option<OrcaDebug>,
}

/// Convert a [`NavCommand`] (world-frame velocity in mm/s) into fields understood by the Teensy.
///
/// Assumptions (based on your notes):
/// - Robot expects the same coordinate system as vision.
/// - `TeensySendMsg.dir` is **degrees**, integer, in range `0..360`.
/// - `TeensySendMsg.speed` is a magnitude in **mm/s**.
///
/// Direction definition used here:
/// - `dir = 0°` points along +X
/// - `dir = 90°` points along +Y
/// - `dir` increases CCW (standard `atan2(y, x)`)
///
/// If velocity is (near) zero: returns `dir = 0`, `speed = 0`.
///
/// ### Direction resolution
/// 1° resolution is usually okay for SSL drive, but you *will* see quantization at low speeds.
/// If you later want finer resolution without changing the type, a common approach is to send
/// `dir_scaled = degrees * 100` (0..36000 fits in `u16`). That does require changing your Teensy
/// interpretation.
pub fn nav_command_to_teensy(mut base: TeensySendMsg, nav: NavCommand) -> TeensySendMsg {
  let (dir_deg, speed_mm_s) = vel_to_dir_speed_deg_1(nav.vel_mm_s);
  base.dir = dir_deg;
  base.speed = speed_mm_s;
  base
}

/// Convert velocity vector (mm/s) to `(dir_degrees, speed_mm_s)`.
///
/// - dir is integer degrees with 1° resolution.
/// - speed is magnitude, clamped to `u16::MAX`.
pub fn vel_to_dir_speed_deg_1(vel_mm_s: Vec2i) -> (u16, u16) {
  let vx = vel_mm_s.x as f64;
  let vy = vel_mm_s.y as f64;
  let speed = (vx * vx + vy * vy).sqrt();
  if speed < 1e-6 {
    return (0, 0);
  }

  let mut dir = vy.atan2(vx).to_degrees();
  if dir < 0.0 {
    dir += 360.0;
  }
  // Wrap just in case numeric conversion yields 360.
  let dir_u16 = (dir.round() as i32).rem_euclid(360) as u16;
  let speed_u16 = speed.round().clamp(0.0, u16::MAX as f64) as u16;
  (dir_u16, speed_u16)
}

#[derive(Debug, Clone, Copy, Default)]
pub struct OrcaDebug {
  pub preferred_vel_mm_s: Vec2i,
  pub num_neighbors: usize,
  pub compute_time: Duration,
}

/// Parameters for ORCA.
///
/// These are not “the correct ORCA parameters” yet—just a place to put them.
#[derive(Debug, Clone, Copy)]
pub struct OrcaParams {
  /// How far ahead we consider collisions.
  pub time_horizon_ms: u32,
  /// Extra safety margin around robots.
  pub safety_margin_mm: u32,
  /// Default radius if you don't have per-robot info.
  pub default_robot_radius_mm: u32,
  /// The controller tick interval. Used for ORCA "already colliding" handling.
  pub time_step_ms: u32,
  /// ORCA assumes both agents share responsibility. Standard value is 0.5.
  ///
  /// If you want to be more conservative (treat others as non-cooperative obstacles),
  /// increase this towards 1.0.
  pub responsibility: f64,
  /// Maximum change in translational velocity per second.
  pub max_accel_mm_s2: u32,
  /// Maximum change in translational velocity when slowing down per second.
  pub max_decel_mm_s2: u32,
  /// If true, compute runs on the blocking pool (`spawn_blocking`).
  pub run_blocking: bool,
}

impl Default for OrcaParams {
  fn default() -> Self {
    Self {
      time_horizon_ms: 2000,
      safety_margin_mm: 60,
      default_robot_radius_mm: 90,
      time_step_ms: 8,
      responsibility: 0.5,
      max_accel_mm_s2: 2_800,
      max_decel_mm_s2: 3_800,
      run_blocking: true,
    }
  }
}

#[derive(Debug, Clone)]
pub struct OrcaRequest {
  pub world: WorldSnapshot,
  pub intent: NavIntent,
}

/// Handle to the ORCA worker.
///
/// Design goal: provide *latest-only* semantics.
/// - If you publish 120Hz updates, but ORCA computes at 60Hz, you don't want a backlog.
/// - `watch` channels always keep only the newest value.
#[derive(Clone)]
pub struct OrcaHandle {
  tx_req: watch::Sender<OrcaRequest>,
  rx_cmd: watch::Receiver<NavCommand>,
}

impl OrcaHandle {
  /// Spawn the ORCA worker task.
  pub fn spawn(params: OrcaParams) -> Self {
    let (tx_req, mut rx_req) = watch::channel(OrcaRequest {
      world: WorldSnapshot {
        now: Instant::now(),
        self_id: 0,
        ..Default::default()
      },
      intent: NavIntent::Stop,
    });

    let (tx_cmd, rx_cmd) = watch::channel(NavCommand::default());

    tokio::spawn(async move {
      // Local worker loop. In a real ORCA implementation you would keep allocations/cache here.
      let mut last_command_vel = Vec2i::default();
      let mut last_world_time: Option<Instant> = None;
      loop {
        // Wait for a new request (or channel closed).
        if rx_req.changed().await.is_err() {
          break;
        }
        let req = rx_req.borrow().clone();
        let start = Instant::now();
        let req_world_time = req.world.now;
        let dt_s = last_world_time
          .map(|t| req_world_time.saturating_duration_since(t).as_secs_f64())
          .unwrap_or_else(|| (params.time_step_ms as f64 / 1000.0).max(0.001));

        // `spawn_blocking` requires a `'static` closure, so capture by value.
        let params = params;
        let compute = move || {
          let (preferred, max_speed) = preferred_velocity(&req.world, req.intent);
          let raw_vel = orca_step(&params, &req.world, preferred, max_speed);
          let vel = limit_velocity_change(
            last_command_vel,
            raw_vel,
            dt_s,
            params.max_accel_mm_s2 as f64,
            params.max_decel_mm_s2 as f64,
          );
          let debug = OrcaDebug {
            preferred_vel_mm_s: preferred,
            num_neighbors: req.world.others.len(),
            compute_time: start.elapsed(),
          };
          NavCommand {
            vel_mm_s: vel,
            debug: Some(debug),
          }
        };

        let cmd = if params.run_blocking {
          tokio::task::spawn_blocking(compute).await.unwrap_or_else(|_| NavCommand::default())
        } else {
          compute()
        };

        last_world_time = Some(req_world_time);
        last_command_vel = cmd.vel_mm_s;

        // It's okay if receivers are gone.
        let _ = tx_cmd.send(cmd);
      }
    });

    Self { tx_req, rx_cmd }
  }

  /// Publish the newest world+intent.
  pub fn publish(&self, req: OrcaRequest) {
    // `watch::Sender::send` only fails if there are no receivers, but that's fine.
    let _ = self.tx_req.send(req);
  }

  /// Get the latest command (non-async).
  pub fn latest(&self) -> NavCommand {
    *self.rx_cmd.borrow()
  }

  /// Async wait for the next produced command.
  pub async fn changed(&mut self) -> Option<NavCommand> {
    if self.rx_cmd.changed().await.is_err() {
      return None;
    }
    Some(*self.rx_cmd.borrow())
  }
}

fn preferred_velocity(world: &WorldSnapshot, intent: NavIntent) -> (Vec2i, u32) {
  match intent {
    NavIntent::Stop => (Vec2i::default(), 0),
    NavIntent::GoToPosition {
      target_pos_mm,
      max_speed_mm_s,
    } => {
      let to_target = (target_pos_mm - world.self_pos_mm)*2;
      // Simple P-controller: "direction towards target" with capped magnitude.
      // Later you can add slowing down near target, orientation constraints, etc.
      (to_target.with_speed_clamped(max_speed_mm_s), max_speed_mm_s)
    }
    NavIntent::PreferredVelocity {
      vel_mm_s,
      max_speed_mm_s,
    } => (vel_mm_s.with_speed_clamped(max_speed_mm_s), max_speed_mm_s),
  }
}

/// Compute one ORCA step: preferred velocity -> collision-free velocity.
///
/// This follows the classic RVO2 ORCA formulation:
/// - Build half-plane constraints ("ORCA lines") in velocity space
/// - Find the feasible velocity closest to the preferred velocity
fn orca_step(params: &OrcaParams, world: &WorldSnapshot, preferred_vel: Vec2i, max_speed_mm_s: u32) -> Vec2i {
  if max_speed_mm_s == 0 {
    return Vec2i::default();
  }

  let max_speed = max_speed_mm_s as f64;
  let time_horizon = (params.time_horizon_ms as f64 / 1000.0).max(0.01);
  let time_step = (params.time_step_ms as f64 / 1000.0).max(0.001);

  let self_pos = Vec2::from_i32(world.self_pos_mm);
  let self_vel = Vec2::from_i32(world.self_vel_mm_s);
  let pref_vel = Vec2::from_i32(preferred_vel);

  let lines = create_orca_lines(params, self_pos, self_vel, world, time_horizon, time_step);
  let mut new_vel = Vec2::default();
  let fail = linear_program_2(&lines, max_speed, pref_vel, false, &mut new_vel);
  if fail < lines.len() {
    linear_program_3(&lines, 0, fail, max_speed, &mut new_vel);
  }

  Vec2i {
    x: new_vel.x.round() as i32,
    y: new_vel.y.round() as i32,
  }
    .with_speed_clamped(max_speed_mm_s)
}

fn limit_velocity_change(previous: Vec2i, desired: Vec2i, dt_s: f64, max_accel_mm_s2: f64, max_decel_mm_s2: f64) -> Vec2i {
  if dt_s <= 0.0 {
    return previous;
  }

  let previous = Vec2::from_i32(previous);
  let desired = Vec2::from_i32(desired);
  let delta = desired - previous;
  let delta_len = delta.abs();
  if delta_len < 1e-12 {
    return desired.into();
  }

  let prev_speed = previous.abs();
  let desired_speed = desired.abs();
  let max_delta = if desired_speed >= prev_speed {
    max_accel_mm_s2
  } else {
    max_decel_mm_s2
  } * dt_s;

  if max_delta <= 0.0 || delta_len <= max_delta {
    desired.into()
  } else {
    (previous + delta / delta_len * max_delta).into()
  }
}

// -------------------------------------------------------------------------------------------------
// ORCA math (f64 internally)

#[derive(Debug, Clone, Copy, Default)]
struct Vec2 {
  x: f64,
  y: f64,
}

impl Vec2 {
  #[inline]
  fn new(x: f64, y: f64) -> Self {
    Self { x, y }
  }

  #[inline]
  fn from_i32(v: Vec2i) -> Self {
    Self {
      x: v.x as f64,
      y: v.y as f64,
    }
  }

  #[inline]
  fn abs_sq(self) -> f64 {
    self.x * self.x + self.y * self.y
  }

  #[inline]
  fn abs(self) -> f64 {
    self.abs_sq().sqrt()
  }

  #[inline]
  fn dot(self, other: Vec2) -> f64 {
    self.x * other.x + self.y * other.y
  }

  #[inline]
  fn det(self, other: Vec2) -> f64 {
    self.x * other.y - self.y * other.x
  }

  #[inline]
  fn normalize(self) -> Vec2 {
    let a = self.abs();
    if a < 1e-12 {
      Vec2::default()
    } else {
      self / a
    }
  }
}

impl std::ops::Add for Vec2 {
  type Output = Vec2;
  fn add(self, rhs: Vec2) -> Self::Output {
    Vec2::new(self.x + rhs.x, self.y + rhs.y)
  }
}

impl std::ops::Sub for Vec2 {
  type Output = Vec2;
  fn sub(self, rhs: Vec2) -> Self::Output {
    Vec2::new(self.x - rhs.x, self.y - rhs.y)
  }
}

impl std::ops::Mul<f64> for Vec2 {
  type Output = Vec2;
  fn mul(self, rhs: f64) -> Self::Output {
    Vec2::new(self.x * rhs, self.y * rhs)
  }
}

impl std::ops::Div<f64> for Vec2 {
  type Output = Vec2;
  fn div(self, rhs: f64) -> Self::Output {
    Vec2::new(self.x / rhs, self.y / rhs)
  }
}

impl std::ops::Neg for Vec2 {
  type Output = Vec2;
  fn neg(self) -> Self::Output {
    Vec2::new(-self.x, -self.y)
  }
}

impl From<Vec2> for Vec2i {
  fn from(v: Vec2) -> Self {
    Self {
      x: v.x.round() as i32,
      y: v.y.round() as i32,
    }
  }
}

#[derive(Debug, Clone, Copy)]
struct Line {
  /// A point on the line in velocity space.
  point: Vec2,
  /// Direction along the line (half-plane is to the left of this direction).
  direction: Vec2,
}

fn create_orca_lines(
  params: &OrcaParams,
  self_pos: Vec2,
  self_vel: Vec2,
  world: &WorldSnapshot,
  time_horizon_s: f64,
  time_step_s: f64,
) -> Vec<Line> {
  let inv_time_horizon = 1.0 / time_horizon_s;
  let inv_time_step = 1.0 / time_step_s;
  let self_radius = params.default_robot_radius_mm as f64;

  let mut lines = Vec::with_capacity(world.others.len());
  for other in &world.others {
    let other_pos = Vec2::from_i32(other.pos_mm);
    let other_vel = Vec2::from_i32(other.vel_mm_s);
    let other_radius = other.radius_mm as f64;
    let combined_radius = self_radius + other_radius + params.safety_margin_mm as f64;

    let relative_position = other_pos - self_pos;
    let relative_velocity = self_vel - other_vel;
    let dist_sq = relative_position.abs_sq();
    let combined_radius_sq = combined_radius * combined_radius;

    let mut line = Line {
      point: Vec2::default(),
      direction: Vec2::default(),
    };
    let u;

    if dist_sq > combined_radius_sq {
      // No collision.
      let w = relative_velocity - relative_position * inv_time_horizon;
      let w_length_sq = w.abs_sq();
      let dot_1 = w.dot(relative_position);

      if dot_1 < 0.0 && dot_1 * dot_1 > combined_radius_sq * w_length_sq {
        // Project on cut-off circle.
        let w_len = w_length_sq.sqrt();
        let unit_w = if w_len < 1e-12 { Vec2::default() } else { w / w_len };
        line.direction = Vec2::new(unit_w.y, -unit_w.x);
        u = unit_w * (combined_radius * inv_time_horizon - w_len);
      } else {
        // Project on legs.
        let leg = (dist_sq - combined_radius_sq).max(0.0).sqrt();
        if relative_position.det(w) > 0.0 {
          line.direction = Vec2::new(
            relative_position.x * leg - relative_position.y * combined_radius,
            relative_position.x * combined_radius + relative_position.y * leg,
          ) / dist_sq;
        } else {
          line.direction = -Vec2::new(
            relative_position.x * leg + relative_position.y * combined_radius,
            -relative_position.x * combined_radius + relative_position.y * leg,
          ) / dist_sq;
        }
        let dot_2 = relative_velocity.dot(line.direction);
        u = line.direction * dot_2 - relative_velocity;
      }

      line.point = self_vel + u * params.responsibility;
    } else {
      // Collision. Use time step to guarantee separation.
      let w = relative_velocity - relative_position * inv_time_step;
      let w_len = w.abs();
      let unit_w = if w_len < 1e-12 { Vec2::default() } else { w / w_len };
      line.direction = Vec2::new(unit_w.y, -unit_w.x);
      u = unit_w * (combined_radius * inv_time_step - w_len);
      line.point = self_vel + u * params.responsibility;
    }

    lines.push(line);
  }

  lines
}

/// Solve:
///   minimize |v - opt_velocity|
/// subject to:
///   v inside circle(radius)
///   v in all half-planes given by `lines`
///
/// Returns `None` if numerical issues occur.
/// Returns the index of the first failing line, or `lines.len()` if feasible.
fn linear_program_2(lines: &[Line], radius: f64, opt_velocity: Vec2, direction_opt: bool, result: &mut Vec2) -> usize {
  *result = if direction_opt {
    // Optimize direction: pick point on circle.
    opt_velocity.normalize() * radius
  } else {
    // Optimize closest point.
    if opt_velocity.abs_sq() > radius * radius {
      opt_velocity.normalize() * radius
    } else {
      opt_velocity
    }
  };

  for i in 0..lines.len() {
    let line = &lines[i];
    if line.direction.det(line.point - *result) > 0.0 {
      // Result violates constraint i.
      let prev = *result;
      if let Some(r) = linear_program_1(lines, i, radius, opt_velocity, direction_opt) {
        *result = r;
      } else {
        *result = prev;
        return i;
      }
    }
  }

  lines.len()
}

fn linear_program_1(lines: &[Line], line_no: usize, radius: f64, opt_velocity: Vec2, direction_opt: bool) -> Option<Vec2> {
  let line = lines.get(line_no)?;
  let dot = line.point.dot(line.direction);
  let discriminant = dot * dot + radius * radius - line.point.abs_sq();
  if discriminant < 0.0 {
    // Max speed circle fully invalidates this line.
    return None;
  }

  let sqrt_discriminant = discriminant.sqrt();
  let mut t_left = -dot - sqrt_discriminant;
  let mut t_right = -dot + sqrt_discriminant;

  for i in 0..line_no {
    let other = lines.get(i)?;
    let denom = line.direction.det(other.direction);
    let numer = other.direction.det(line.point - other.point);

    if denom.abs() <= 1e-12 {
      // Parallel.
      if numer < 0.0 {
        return None;
      }
      continue;
    }

    let t = numer / denom;
    if denom >= 0.0 {
      t_right = t_right.min(t);
    } else {
      t_left = t_left.max(t);
    }
    if t_left > t_right {
      return None;
    }
  }

  let t = if direction_opt {
    if opt_velocity.dot(line.direction) > 0.0 {
      t_right
    } else {
      t_left
    }
  } else {
    let t_opt = line.direction.dot(opt_velocity - line.point);
    if t_opt < t_left {
      t_left
    } else if t_opt > t_right {
      t_right
    } else {
      t_opt
    }
  };

  Some(line.point + line.direction * t)
}

fn linear_program_3(lines: &[Line], num_obst_lines: usize, begin_line: usize, radius: f64, result: &mut Vec2) {
  let mut distance = 0.0;
  for i in begin_line..lines.len() {
    let line_i = &lines[i];
    let violation = line_i.direction.det(line_i.point - *result);
    if violation > distance {
      // Recompute result by projecting onto intersection of previous constraints and line i.
      let mut proj_lines: Vec<Line> = Vec::with_capacity(i + 1);
      proj_lines.extend_from_slice(&lines[..num_obst_lines.min(lines.len())]);

      for j in num_obst_lines..i {
        let line_j = &lines[j];
        let determinant = line_i.direction.det(line_j.direction);
        let mut point: Vec2 = Vec2::default();
        if determinant.abs() <= 1e-12 {
          // Parallel lines: if they point the same way, skip; else take midpoint.
          if line_i.direction.dot(line_j.direction) > 0.0 {
            continue;
          }
          let _ = (line_i.point + line_j.point) * 0.5;
        } else {
          let t = line_j.direction.det(line_i.point - line_j.point) / determinant;
          point = line_i.point + line_i.direction * t;
        }

        let direction = (line_j.direction - line_i.direction).normalize();
        proj_lines.push(Line { point, direction });
      }

      let temp_result = *result;
      let perp = Vec2::new(-line_i.direction.y, line_i.direction.x);
      let fail = linear_program_2(&proj_lines, radius, perp, true, result);
      if fail < proj_lines.len() {
        *result = temp_result;
      }
      distance = line_i.direction.det(line_i.point - *result);
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn clamp_speed() {
    let v = Vec2i::new(3000, 0).with_speed_clamped(1000);
    assert!(v.x.abs() <= 1000);
    assert_eq!(v.y, 0);
  }

  #[test]
  fn repulsion_pushes_away() {
    let params = OrcaParams {
      run_blocking: false,
      ..Default::default()
    };
    let world = WorldSnapshot {
      now: Instant::now(),
      self_id: 1,
      self_pos_mm: Vec2i::new(0, 0),
      self_vel_mm_s: Vec2i::default(),
      self_orientation: None,
      others: vec![OtherRobot {
        id: 2,
        pos_mm: Vec2i::new(100, 0),
        vel_mm_s: Vec2i::default(),
        radius_mm: 90,
      }],
    };
    let preferred = Vec2i::new(1000, 0);
    let out = orca_step(&params, &world, preferred, 1000);
    assert!(out.len_sq_i64() <= (1000i64 * 1000i64) + 10);
    // With an obstacle directly ahead, ORCA shouldn't accelerate into it.
    assert!(out.x <= preferred.x);
  }

  #[test]
  fn orca_result_satisfies_halfplanes() {
    let params = OrcaParams {
      run_blocking: false,
      ..Default::default()
    };
    let world = WorldSnapshot {
      now: Instant::now(),
      self_id: 1,
      self_pos_mm: Vec2i::new(0, 0),
      self_vel_mm_s: Vec2i::new(1000, 0),
      self_orientation: None,
      others: vec![
        OtherRobot {
          id: 2,
          pos_mm: Vec2i::new(600, 0),
          vel_mm_s: Vec2i::default(),
          radius_mm: 90,
        },
        OtherRobot {
          id: 3,
          pos_mm: Vec2i::new(600, 250),
          vel_mm_s: Vec2i::default(),
          radius_mm: 90,
        },
      ],
    };
    let preferred = Vec2i::new(1200, 0);
    let out = orca_step(&params, &world, preferred, 1200);

    let self_pos = Vec2::from_i32(world.self_pos_mm);
    let self_vel = Vec2::from_i32(world.self_vel_mm_s);
    let lines = create_orca_lines(
      &params,
      self_pos,
      self_vel,
      &world,
      (params.time_horizon_ms as f64 / 1000.0).max(0.01),
      (params.time_step_ms as f64 / 1000.0).max(0.001),
    );
    let out_v = Vec2::from_i32(out);
    for line in lines {
      // For each constraint, output velocity should be on the valid side (or extremely close).
      assert!(line.direction.det(line.point - out_v) <= 1e-6);
    }
  }

  #[test]
  fn vel_to_dir_speed_deg_1_convention() {
    assert_eq!(vel_to_dir_speed_deg_1(Vec2i::new(0, 0)), (0, 0));
    assert_eq!(vel_to_dir_speed_deg_1(Vec2i::new(1000, 0)).0, 0);
    assert_eq!(vel_to_dir_speed_deg_1(Vec2i::new(0, 1000)).0, 90);
    assert_eq!(vel_to_dir_speed_deg_1(Vec2i::new(-1000, 0)).0, 180);
    assert_eq!(vel_to_dir_speed_deg_1(Vec2i::new(0, -1000)).0, 270);
  }

  #[test]
  fn velocity_change_is_rate_limited() {
    let prev = Vec2i::new(0, 0);
    let desired = Vec2i::new(1000, 0);
    let out = limit_velocity_change(prev, desired, 0.1, 200.0, 400.0);
    assert_eq!(out, Vec2i::new(20, 0));
  }
}
