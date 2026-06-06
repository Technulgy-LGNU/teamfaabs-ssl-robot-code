//! ORCA (Optimal Reciprocal Collision Avoidance)
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

use crate::communication::TeensySendMsg;
use crate::config;
use crate::proto::{CpRobot, CpTrackedRobot};
use crate::robot_logic::vec::Vec2f;
pub use crate::robot_logic::vec::Vec2i;

#[derive(Debug, Clone, Copy, Default)]
pub struct MovingObstacle {
  pub pos_mm: Vec2i,
  pub vel_mm_s: Vec2i,
  pub radius_mm: u32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Rect {
  pub min_x_mm: f32,
  pub max_x_mm: f32,
  pub min_y_mm: f32,
  pub max_y_mm: f32,
}

impl Rect {
  fn new(min_x_mm: f32, max_x_mm: f32, min_y_mm: f32, max_y_mm: f32) -> Self {
    Self {
      min_x_mm,
      max_x_mm,
      min_y_mm,
      max_y_mm,
    }
  }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct FieldGeometry {
  pub width_mm: f32,
  pub height_mm: f32,
  pub runoff_width_mm: f32,
  pub penalty_area_width_mm: f32,
  pub penalty_area_height_mm: f32,
  pub robot_goal: bool,
}

impl FieldGeometry {
  fn from_config(cfg: &config::Config) -> Self {
    Self {
      width_mm: cfg.field.width_mm(),
      height_mm: cfg.field.height_mm(),
      runoff_width_mm: cfg.field.runoff_width_mm(),
      penalty_area_width_mm: cfg.field.penalty_area_width_mm(),
      penalty_area_height_mm: cfg.field.penalty_area_height_mm(),
      robot_goal: cfg.robot_goal,
    }
  }

  fn safe_play_rect(&self, clearance_mm: f32) -> Rect {
    let half_w = (self.width_mm * 0.5 - self.runoff_width_mm - clearance_mm).max(0.0);
    let half_h = (self.height_mm * 0.5 - self.runoff_width_mm - clearance_mm).max(0.0);
    Rect::new(-half_w, half_w, -half_h, half_h)
  }

  fn own_penalty_rect(&self, clearance_mm: f32) -> Rect {
    self.penalty_rect(self.robot_goal, clearance_mm)
  }

  fn opponent_penalty_rect(&self, clearance_mm: f32) -> Rect {
    self.penalty_rect(!self.robot_goal, clearance_mm)
  }

  fn penalty_rect(&self, robot_goal: bool, clearance_mm: f32) -> Rect {
    let goal_x = if robot_goal {
      -self.width_mm * 0.5
    } else {
      self.width_mm * 0.5
    };
    let goal_side = if robot_goal { -1.0 } else { 1.0 };
    let outer_x = goal_x - goal_side * self.penalty_area_height_mm;
    let x_min = goal_x.min(outer_x) - clearance_mm;
    let x_max = goal_x.max(outer_x) + clearance_mm;
    let y_half = self.penalty_area_width_mm * 0.5 + clearance_mm;
    Rect::new(x_min, x_max, -y_half, y_half)
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
  /// The ball, treated like a moving obstacle when the current state wants avoidance.
  pub ball: Option<MovingObstacle>,
  /// Raw field geometry used to build static keep-out zones.
  pub field: FieldGeometry,
  /// Allows goalie to enter its own penalty area while all other behaviors avoid it.
  pub allow_own_penalty_area: bool,
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
      ball: None,
      field: FieldGeometry::default(),
      allow_own_penalty_area: false,
    }
  }
}

impl WorldSnapshot {
  /// Build a [`WorldSnapshot`] from a CrashPilot world packet and the already selected self robot.
  ///
  /// Notes:
  /// - CrashPilot supplies both teams; we simply merge them.
  /// - The caller is responsible for providing the correct self robot.
  /// - Units are assumed to already be *millimeters* and *millimeters/second* as described.
  pub fn from_cp(
    cfg: &config::Config, cp: &CpRobot, self_robot: &CpTrackedRobot, default_robot_radius_mm: u32,
    ball_avoidance_radius_mm: u32, allow_own_penalty_area: bool,
  ) -> Self {
    let self_id = self_robot.robot_id;
    let self_pos_mm = Vec2i::from_cp_vec2(&self_robot.pos);
    let self_vel_mm_s = self_robot
      .vel
      .as_ref()
      .map(Vec2i::from_cp_vec2)
      .unwrap_or_default();
    let self_orientation = Some(self_robot.orientation);

    let mut others = Vec::with_capacity(cp.robots_yellow.len() + cp.robots_blue.len());
    append_others(
      &mut others,
      &cp.robots_yellow,
      self_id,
      default_robot_radius_mm,
    );
    append_others(
      &mut others,
      &cp.robots_blue,
      self_id,
      default_robot_radius_mm,
    );

    let ball = if ball_avoidance_radius_mm == 0 {
      None
    } else {
      Some(MovingObstacle {
        pos_mm: Vec2i::from_cp_vec2(&cp.ball.pos),
        vel_mm_s: cp
          .ball
          .vel
          .as_ref()
          .map(Vec2i::from_cp_vec2)
          .unwrap_or_default(),
        radius_mm: ball_avoidance_radius_mm,
      })
    };

    Self {
      now: Instant::now(),
      self_id,
      self_pos_mm,
      self_vel_mm_s,
      self_orientation,
      others,
      ball,
      field: FieldGeometry::from_config(cfg),
      allow_own_penalty_area,
    }
  }
}

fn append_others(
  out: &mut Vec<OtherRobot>, src: &[CpTrackedRobot], self_id: u32, default_radius_mm: u32,
) {
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
pub fn nav_command_to_teensy(base: &mut TeensySendMsg, nav: NavCommand) {
  let (dir_deg, speed_mm_s) = vel_to_dir_speed_deg_1(nav.vel_mm_s);
  base.dir = dir_deg;
  base.speed = speed_mm_s;
}

/// Convert velocity vector (mm/s) to `(dir_degrees, speed_mm_s)`.
///
/// - dir is integer degrees with 1° resolution.
/// - speed is magnitude, clamped to `u16::MAX`.
pub fn vel_to_dir_speed_deg_1(vel_mm_s: Vec2i) -> (u16, u16) {
  let vx = vel_mm_s.x as f32;
  let vy = vel_mm_s.y as f32;
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
  let speed_u16 = speed.round().clamp(0.0, u16::MAX as f32) as u16;
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
  pub responsibility: f32,
  /// Maximum change in translational velocity per second.
  pub max_accel_mm_s2: u32,
  /// Maximum change in translational velocity when slowing down per second.
  pub max_decel_mm_s2: u32,
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
pub struct Orca {
  last_command_vel: Vec2i,
  last_world_time: Option<Instant>,
  params: OrcaParams,
}

impl Orca {
  pub fn new(params: OrcaParams) -> Self {
    Self {
      last_command_vel: Vec2i::default(),
      last_world_time: None,
      params,
    }
  }

  pub fn step(&mut self, req: OrcaRequest) -> NavCommand {
    let start = Instant::now();
    let req_world_time = req.world.now;
    let dt_s = self
      .last_world_time
      .map(|t| req_world_time.saturating_duration_since(t).as_secs_f32())
      .unwrap_or_else(|| (self.params.time_step_ms as f32 / 1000f32).max(0.001f32));

    let (preferred, max_speed) = preferred_velocity(&req.world, req.intent);
    let raw_vel = orca_step(&self.params, &req.world, preferred, max_speed);
    let limited = limit_velocity_change(
      self.last_command_vel,
      raw_vel,
      dt_s,
      self.params.max_accel_mm_s2 as f32,
      self.params.max_decel_mm_s2 as f32,
    );
    // Safety filter on the final command: the rate limiter can lag the planned braking, so
    // re-clamp the inward component to the braking limit (no go-around injection here). This
    // is the hard guarantee that the commanded velocity never drives into a keep-out zone.
    let self_pos = Vec2f::new_from_vec2i(req.world.self_pos_mm);
    let vel: Vec2i = apply_static_avoidance(
      &self.params,
      &req.world,
      self_pos,
      Vec2f::new_from_vec2i(limited),
      false,
    )
    .into();
    let debug = OrcaDebug {
      preferred_vel_mm_s: preferred,
      num_neighbors: req.world.others.len() + usize::from(req.world.ball.is_some()),
      compute_time: start.elapsed(),
    };

    let cmd = NavCommand {
      vel_mm_s: vel,
      debug: Some(debug),
    };

    self.last_world_time = Some(req_world_time);
    self.last_command_vel = cmd.vel_mm_s;

    // It's okay if receivers are gone.
    cmd
  }
}

fn preferred_velocity(world: &WorldSnapshot, intent: NavIntent) -> (Vec2i, u32) {
  match intent {
    NavIntent::Stop => (Vec2i::default(), 0),
    NavIntent::GoToPosition {
      target_pos_mm,
      max_speed_mm_s,
    } => {
      let to_target = (target_pos_mm - world.self_pos_mm) * 2;
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
fn orca_step(
  params: &OrcaParams, world: &WorldSnapshot, preferred_vel: Vec2i, max_speed_mm_s: u32,
) -> Vec2i {
  if max_speed_mm_s == 0 {
    return Vec2i::default();
  }

  let max_speed = max_speed_mm_s as f32;
  let time_horizon = (params.time_horizon_ms as f32 / 1000.0).max(0.01);
  let time_step = (params.time_step_ms as f32 / 1000.0).max(0.001);

  let self_pos = Vec2f::new_from_vec2i(world.self_pos_mm);
  let self_vel = Vec2f::new_from_vec2i(world.self_vel_mm_s);
  let pref_vel = apply_static_avoidance(
    params,
    world,
    self_pos,
    Vec2f::new_from_vec2i(preferred_vel),
    true,
  );

  let lines = create_orca_lines(params, self_pos, self_vel, world, time_horizon, time_step);
  let mut new_vel = Vec2f::default();
  let fail = linear_program_2(&lines, max_speed, pref_vel, false, &mut new_vel);
  if fail < lines.len() {
    linear_program_3(&lines, 0, fail, max_speed, &mut new_vel);
  }

  let adjusted = apply_static_avoidance(params, world, self_pos, new_vel, true);

  Vec2i {
    x: adjusted.x.round() as i32,
    y: adjusted.y.round() as i32,
  }
  .with_speed_clamped(max_speed_mm_s)
}

/// Extra gap kept between the robot body and the penalty-area line, on top of the robot radius.
///
/// The keep-out rectangle is inflated by `robot_radius + this`, so the robot body ends up only a
/// few centimeters from the actual line instead of the much larger robot-vs-robot safety margin.
const PENALTY_KEEPOUT_MARGIN_MM: f32 = 20.0;

/// Fraction of the robot's max deceleration used to plan static-zone braking. Staying below the
/// rate limiter's actual capability leaves it headroom to correct discrete-step overshoot, so the
/// robot reliably stops *before* the boundary instead of nicking it.
const STATIC_BRAKE_FACTOR: f32 = 0.5;

/// Apply the static keep-in/keep-out zones to a velocity.
///
/// `allow_go_around` controls the dead-on behavior:
/// - `true` (planning stage, before the rate limiter): when the robot is aimed straight at a box
///   with no sideways motion, the stripped inward speed is redirected sideways so it actively
///   drives around.
/// - `false` (safety stage, after the rate limiter): the inward component is only *clamped* to the
///   braking limit, never grown. This is the hard guarantee that the *commanded* velocity can never
///   plow into a box — the rate limiter can lag the planned braking, but this filter cannot.
fn apply_static_avoidance(
  params: &OrcaParams, world: &WorldSnapshot, self_pos: Vec2f, mut vel: Vec2f,
  allow_go_around: bool,
) -> Vec2f {
  // Static zones are braked against the robot's own deceleration capability rather than a fixed
  // time horizon: full speed until braking distance, then a smooth stop right at the boundary.
  let decel = params.max_decel_mm_s2 as f32 * STATIC_BRAKE_FACTOR;

  let field_clearance = params.default_robot_radius_mm as f32 + params.safety_margin_mm as f32;
  vel = world
    .field
    .safe_play_rect(field_clearance)
    .clamp_velocity_keep_inside(self_pos, vel, decel);

  let keepout_clearance = params.default_robot_radius_mm as f32 + PENALTY_KEEPOUT_MARGIN_MM;
  if !world.allow_own_penalty_area {
    vel = world
      .field
      .own_penalty_rect(keepout_clearance)
      .clamp_velocity_keep_outside(self_pos, vel, decel, allow_go_around);
  }
  vel = world
    .field
    .opponent_penalty_rect(keepout_clearance)
    .clamp_velocity_keep_outside(self_pos, vel, decel, allow_go_around);

  vel
}

/// Maximum speed from which the robot can still brake to a stop within `distance_mm`.
fn braking_speed(distance_mm: f32, decel_mm_s2: f32) -> f32 {
  if distance_mm <= 0.0 {
    0.0
  } else {
    (2.0 * decel_mm_s2 * distance_mm).sqrt()
  }
}

fn limit_velocity_change(
  previous: Vec2i, desired: Vec2i, dt_s: f32, max_accel_mm_s2: f32, max_decel_mm_s2: f32,
) -> Vec2i {
  if dt_s <= 0.0 {
    return previous;
  }

  let previous = Vec2f::new_from_vec2i(previous);
  let desired = Vec2f::new_from_vec2i(desired);
  let delta = desired - previous;
  let delta_len = delta.norm();
  if delta_len < 1e-12 {
    return desired.into();
  }

  let prev_speed = previous.norm();
  let desired_speed = desired.norm();
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

impl Rect {
  /// Keep the robot inside this rectangle. Each axis runs at full speed until it is within braking
  /// distance of the wall, then decelerates to a stop exactly at the wall — so the robot stays fast
  /// near field edges instead of crawling.
  fn clamp_velocity_keep_inside(self, pos: Vec2f, vel: Vec2f, decel: f32) -> Vec2f {
    let mut out = vel;
    if out.x > 0.0 {
      out.x = out.x.min(braking_speed(self.max_x_mm - pos.x, decel));
    } else if out.x < 0.0 {
      out.x = out.x.max(-braking_speed(pos.x - self.min_x_mm, decel));
    }
    if out.y > 0.0 {
      out.y = out.y.min(braking_speed(self.max_y_mm - pos.y, decel));
    } else if out.y < 0.0 {
      out.y = out.y.max(-braking_speed(pos.y - self.min_y_mm, decel));
    }
    out
  }

  /// Keep the robot out of this rectangle while letting it slide *around* the boundary.
  ///
  /// Instead of only checking the projected endpoint (which lets a path cut through a corner)
  /// or truncating the velocity to a dead stop in front of the box (which can never drive
  /// around it), this treats the rectangle as a convex keep-out obstacle and only removes the
  /// velocity component pointing *into* it:
  ///
  /// - The component tangent to the nearest face/corner is preserved, so the robot slides along
  ///   the boundary and rounds the corners.
  /// - The inward (normal) component is limited so the robot can approach but never cross the
  ///   boundary within the time horizon. Because the nearest feature's supporting line fully
  ///   separates the robot from the (convex) box, this guarantees the straight-line path never
  ///   enters the rectangle.
  /// - When `allow_go_around` is set, the forward motion that gets blocked by the box is redirected
  ///   *along* the boundary (preserving speed) so the robot rounds the box at full pace instead of
  ///   crawling. The slide direction follows the robot's existing tangential intent; only when it is
  ///   aimed dead-on with no sideways intent at all does it fall back to biasing toward the field
  ///   center (so it rounds the open side, not the goal line).
  fn clamp_velocity_keep_outside(
    self, pos: Vec2f, vel: Vec2f, decel: f32, allow_go_around: bool,
  ) -> Vec2f {
    // Small buffer (inside the already-inflated rect) to absorb one tick of discrete overshoot.
    const MARGIN_MM: f32 = 20.0;

    if self.contains(pos) {
      // Abnormal: we're already inside the keep-out zone. Drive straight out the nearest face,
      // preserving (at least) the requested speed.
      let outward = (self.closest_boundary_point(pos) - pos).normalized();
      let speed = vel.norm().max(1.0);
      return outward * speed;
    }

    let closest = self.closest_boundary_point(pos);
    let to_robot = pos - closest;
    let dist = to_robot.norm();
    if dist <= 1e-6 {
      return vel;
    }
    let normal = to_robot / dist; // outward unit normal at the nearest feature

    // Inward speed (toward the box) the robot is currently carrying.
    let inward = -vel.dot(normal);
    // Inward speed it may keep and still brake to a stop before the boundary.
    let max_inward = braking_speed(dist - MARGIN_MM, decel);
    if inward <= max_inward {
      // Moving away, parallel, or approaching slowly enough to stop in time: nothing to do.
      return vel;
    }

    // Strip the excess inward component; keep the tangential (slide-around) part.
    let tangent = Vec2f::new(-normal.y, normal.x);
    let tangential = vel - normal * vel.dot(normal);
    let mut out = tangential - normal * max_inward;

    // Redirect the blocked forward motion along the boundary so the robot rounds the box at speed
    // instead of stalling. The available tangential budget keeps total speed constant.
    if allow_go_around {
      let tang_budget = (vel.norm_squared() - max_inward * max_inward)
        .max(0.0)
        .sqrt();
      if tang_budget > tangential.norm() {
        let tang_signed = vel.dot(tangent);
        // Only trust the robot's own sideways intent when it is a meaningful fraction of its speed.
        // A near-perpendicular (dead-on) approach has a tiny, unreliable tangential sign that must
        // not pick the side — it could send the robot the long way, into the goal line. There, bias
        // toward the field center so it rounds the open side instead.
        let side = if tang_signed.abs() > 0.2 * vel.norm() {
          tang_signed.signum()
        } else {
          let to_center = Vec2f::default() - pos;
          if tangent.dot(to_center) >= 0.0 {
            1.0
          } else {
            -1.0
          }
        };
        out = tangent * (tang_budget * side) - normal * max_inward;
      }
    }

    out
  }

  fn contains(self, p: Vec2f) -> bool {
    p.x >= self.min_x_mm && p.x <= self.max_x_mm && p.y >= self.min_y_mm && p.y <= self.max_y_mm
  }

  /// Closest point on the rectangle boundary. For an outside point this is the nearest face or
  /// corner; for an inside point it is the nearest face (used to escape).
  fn closest_boundary_point(self, p: Vec2f) -> Vec2f {
    if self.contains(p) {
      let dist_left = p.x - self.min_x_mm;
      let dist_right = self.max_x_mm - p.x;
      let dist_bottom = p.y - self.min_y_mm;
      let dist_top = self.max_y_mm - p.y;
      let min = dist_left.min(dist_right).min(dist_bottom).min(dist_top);
      if min == dist_left {
        Vec2f::new(self.min_x_mm, p.y)
      } else if min == dist_right {
        Vec2f::new(self.max_x_mm, p.y)
      } else if min == dist_bottom {
        Vec2f::new(p.x, self.min_y_mm)
      } else {
        Vec2f::new(p.x, self.max_y_mm)
      }
    } else {
      Vec2f::new(
        p.x.clamp(self.min_x_mm, self.max_x_mm),
        p.y.clamp(self.min_y_mm, self.max_y_mm),
      )
    }
  }
}

#[derive(Debug, Clone, Copy)]
struct Line {
  /// A point on the line in velocity space.
  point: Vec2f,
  /// Direction along the line (half-plane is to the left of this direction).
  direction: Vec2f,
}

fn create_orca_lines(
  params: &OrcaParams, self_pos: Vec2f, self_vel: Vec2f, world: &WorldSnapshot,
  time_horizon_s: f32, time_step_s: f32,
) -> Vec<Line> {
  let mut lines = Vec::with_capacity(world.others.len() + usize::from(world.ball.is_some()));
  for other in &world.others {
    lines.push(moving_obstacle_line(
      params,
      self_pos,
      self_vel,
      Vec2f::new_from_vec2i(other.pos_mm),
      Vec2f::new_from_vec2i(other.vel_mm_s),
      other.radius_mm as f32,
      time_horizon_s,
      time_step_s,
    ));
  }

  if let Some(ball) = world.ball {
    lines.push(moving_obstacle_line(
      params,
      self_pos,
      self_vel,
      Vec2f::new_from_vec2i(ball.pos_mm),
      Vec2f::new_from_vec2i(ball.vel_mm_s),
      ball.radius_mm as f32,
      time_horizon_s,
      time_step_s,
    ));
  }

  lines
}

fn moving_obstacle_line(
  params: &OrcaParams, self_pos: Vec2f, self_vel: Vec2f, obstacle_pos: Vec2f, obstacle_vel: Vec2f,
  obstacle_radius: f32, time_horizon_s: f32, time_step_s: f32,
) -> Line {
  let inv_time_horizon = 1.0 / time_horizon_s;
  let inv_time_step = 1.0 / time_step_s;
  let self_radius = params.default_robot_radius_mm as f32;
  let combined_radius = self_radius + obstacle_radius + params.safety_margin_mm as f32;

  let relative_position = obstacle_pos - self_pos;
  let relative_velocity = self_vel - obstacle_vel;
  let dist_sq = relative_position.norm_squared();
  let combined_radius_sq = combined_radius * combined_radius;

  let mut line = Line {
    point: Vec2f::default(),
    direction: Vec2f::default(),
  };
  let u;

  if dist_sq > combined_radius_sq {
    let w = relative_velocity - relative_position * inv_time_horizon;
    let w_length_sq = w.norm_squared();
    let dot_1 = w.dot(relative_position);

    if dot_1 < 0.0 && dot_1 * dot_1 > combined_radius_sq * w_length_sq {
      let w_len = w_length_sq.sqrt();
      let unit_w = if w_len < 1e-12 {
        Vec2f::default()
      } else {
        w / w_len
      };
      line.direction = Vec2f::new(unit_w.y, -unit_w.x);
      u = unit_w * (combined_radius * inv_time_horizon - w_len);
    } else {
      let leg = (dist_sq - combined_radius_sq).max(0.0).sqrt();
      if relative_position.det(w) > 0.0 {
        line.direction = Vec2f::new(
          relative_position.x * leg - relative_position.y * combined_radius,
          relative_position.x * combined_radius + relative_position.y * leg,
        ) / dist_sq;
      } else {
        line.direction = -Vec2f::new(
          relative_position.x * leg + relative_position.y * combined_radius,
          -relative_position.x * combined_radius + relative_position.y * leg,
        ) / dist_sq;
      }
      let dot_2 = relative_velocity.dot(line.direction);
      u = line.direction * dot_2 - relative_velocity;
    }

    line.point = self_vel + u * params.responsibility;
  } else {
    let w = relative_velocity - relative_position * inv_time_step;
    let w_len = w.norm();
    let unit_w = if w_len < 1e-12 {
      Vec2f::default()
    } else {
      w / w_len
    };
    line.direction = Vec2f::new(unit_w.y, -unit_w.x);
    u = unit_w * (combined_radius * inv_time_step - w_len);
    line.point = self_vel + u * params.responsibility;
  }

  line
}

/// Solve:
///   minimize |v - opt_velocity|
/// subject to:
///   v inside circle(radius)
///   v in all half-planes given by `lines`
///
/// Returns `None` if numerical issues occur.
/// Returns the index of the first failing line, or `lines.len()` if feasible.
fn linear_program_2(
  lines: &[Line], radius: f32, opt_velocity: Vec2f, direction_opt: bool, result: &mut Vec2f,
) -> usize {
  *result = if direction_opt {
    // Optimize direction: pick point on circle.
    opt_velocity.normalized() * radius
  } else {
    // Optimize closest point.
    if opt_velocity.norm_squared() > radius * radius {
      opt_velocity.normalized() * radius
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

fn linear_program_1(
  lines: &[Line], line_no: usize, radius: f32, opt_velocity: Vec2f, direction_opt: bool,
) -> Option<Vec2f> {
  let line = lines.get(line_no)?;
  let dot = line.point.dot(line.direction);
  let discriminant = dot * dot + radius * radius - line.point.norm_squared();
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

fn linear_program_3(
  lines: &[Line], num_obst_lines: usize, begin_line: usize, radius: f32, result: &mut Vec2f,
) {
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
        let mut point: Vec2f = Vec2f::default();
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

        let direction = (line_j.direction - line_i.direction).normalized();
        proj_lines.push(Line { point, direction });
      }

      let temp_result = *result;
      let perp = Vec2f::new(-line_i.direction.y, line_i.direction.x);
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

  fn test_field() -> FieldGeometry {
    FieldGeometry {
      width_mm: 9000.0,
      height_mm: 6000.0,
      runoff_width_mm: 300.0,
      penalty_area_width_mm: 2000.0,
      penalty_area_height_mm: 1000.0,
      robot_goal: false,
    }
  }

  #[test]
  fn clamp_speed() {
    let v = Vec2i::new(3000, 0).with_speed_clamped(1000);
    assert!(v.x.abs() <= 1000);
    assert_eq!(v.y, 0);
  }

  #[test]
  fn repulsion_pushes_away() {
    let params = OrcaParams {
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
      ball: None,
      field: test_field(),
      allow_own_penalty_area: false,
    };
    let preferred = Vec2i::new(1000, 0);
    let out = orca_step(&params, &world, preferred, 1000);
    assert!(out.norm_squared() <= (1000i32 * 1000i32) + 10);
    // With an obstacle directly ahead, ORCA shouldn't accelerate into it.
    assert!(out.x <= preferred.x);
  }

  #[test]
  fn orca_result_satisfies_halfplanes() {
    let params = OrcaParams {
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
      ball: None,
      field: test_field(),
      allow_own_penalty_area: false,
    };
    let preferred = Vec2i::new(1200, 0);
    let out = orca_step(&params, &world, preferred, 1200);

    let self_pos = Vec2f::new_from_vec2i(world.self_pos_mm);
    let self_vel = Vec2f::new_from_vec2i(world.self_vel_mm_s);
    let lines = create_orca_lines(
      &params,
      self_pos,
      self_vel,
      &world,
      (params.time_horizon_ms as f32 / 1000.0).max(0.01),
      (params.time_step_ms as f32 / 1000.0).max(0.001),
    );
    let out_v = Vec2f::new_from_vec2i(out);
    for line in lines {
      // For each constraint, output velocity should be on the valid side (or extremely close).
      assert!(line.direction.det(line.point - out_v) <= 1e-6);
    }
  }

  #[test]
  fn field_boundaries_clamp_outward_motion() {
    let params = OrcaParams {
      ..Default::default()
    };
    let world = WorldSnapshot {
      now: Instant::now(),
      self_id: 1,
      self_pos_mm: Vec2i::new(0, 2_900),
      self_vel_mm_s: Vec2i::default(),
      self_orientation: None,
      others: Vec::new(),
      ball: None,
      field: test_field(),
      allow_own_penalty_area: false,
    };
    let out = orca_step(&params, &world, Vec2i::new(0, 2_000), 2_000);
    assert!(out.y <= 0, "out={:?}", out);
  }

  /// Run the full reactive control loop (preferred velocity -> orca_step -> accel/decel limiter ->
  /// integrate) so tests exercise the braking model the way the robot actually does, instead of a
  /// single-step linear projection.
  fn run_closed_loop(
    mut world: WorldSnapshot, target: Vec2i, max_speed: u32, steps: usize,
  ) -> Vec<Vec2f> {
    let params = OrcaParams {
      ..Default::default()
    };
    let dt = params.time_step_ms as f32 / 1000.0;
    let mut pos = Vec2f::new_from_vec2i(world.self_pos_mm);
    let mut vel = Vec2f::default();
    let mut path = Vec::with_capacity(steps + 1);
    path.push(pos);
    for _ in 0..steps {
      world.self_pos_mm = pos.into();
      world.self_vel_mm_s = vel.into();
      let pref = ((target - world.self_pos_mm) * 2).with_speed_clamped(max_speed);
      let raw = orca_step(&params, &world, pref, max_speed);
      let limited = limit_velocity_change(
        vel.into(),
        raw,
        dt,
        params.max_accel_mm_s2 as f32,
        params.max_decel_mm_s2 as f32,
      );
      // Mirror the worker's post-limiter safety filter.
      let safe =
        apply_static_avoidance(&params, &world, pos, Vec2f::new_from_vec2i(limited), false);
      vel = safe;
      pos = pos + vel * dt;
      path.push(pos);
    }
    path
  }

  fn base_world(pos: Vec2i) -> WorldSnapshot {
    WorldSnapshot {
      now: Instant::now(),
      self_id: 1,
      self_pos_mm: pos,
      self_vel_mm_s: Vec2i::default(),
      self_orientation: None,
      others: Vec::new(),
      ball: None,
      field: test_field(),
      allow_own_penalty_area: false,
    }
  }

  #[test]
  fn approaches_penalty_box_at_full_speed() {
    let params = OrcaParams {
      ..Default::default()
    };
    // Well away from the box, aimed at it: must not be throttled by the keep-out zone.
    let world = base_world(Vec2i::new(-1_000, 0));
    let out = orca_step(&params, &world, Vec2i::new(-3_000, 0), 3_000);
    assert!(
      out.x < -2_800,
      "should drive fast toward the box, out={out:?}"
    );
  }

  #[test]
  fn operates_close_to_penalty_box() {
    let params = OrcaParams {
      ..Default::default()
    };
    // The robot can sit only a few cm off the line: just outside the inflated keep-out face, with
    // its body (radius 90) about PENALTY_KEEPOUT_MARGIN_MM from the actual penalty line.
    let line = test_field().opponent_penalty_rect(0.0);
    // Field-side face of the actual penalty area on the opponent (x-) side is at x = -3500; field
    // side is +x of it. The keep-out only pushes the center out by radius + margin.
    let near_x =
      -3500 + params.default_robot_radius_mm as i32 + PENALTY_KEEPOUT_MARGIN_MM as i32 + 10;
    let world = base_world(Vec2i::new(near_x, 0));

    // Driving tangentially (along the box) right next to it must not be throttled.
    let along = orca_step(&params, &world, Vec2i::new(0, 3_000), 3_000);
    assert!(
      along.y > 2_800,
      "should move full speed alongside the box, out={along:?}"
    );
    assert!(
      !line.contains(Vec2f::new(near_x as f32, 0.0)),
      "test position should be outside the actual penalty area"
    );

    // It is this close because the keep-out only inflates by radius + a small margin.
    let body_gap = (near_x as f32 + 3_500.0).abs() - params.default_robot_radius_mm as f32;
    assert!(
      body_gap < 60.0,
      "robot body should sit within a few cm of the line: {body_gap} mm"
    );
  }

  #[test]
  fn drives_around_box_to_the_far_side() {
    // Target sits on the opposite side of the box; the straight line passes through it, so the
    // robot must detour around and still arrive.
    let path = run_closed_loop(
      base_world(Vec2i::new(-4_000, -1_300)),
      Vec2i::new(-4_000, 1_300),
      3_000,
      800,
    );
    let line = test_field().opponent_penalty_rect(0.0);
    for p in &path {
      assert!(
        !line.contains(*p),
        "robot center crossed the penalty line at {p:?}"
      );
    }
    let last = *path.last().unwrap();
    let reached = (last - Vec2f::new(-4_000.0, 1_300.0)).norm();
    assert!(
      reached < 250.0,
      "robot did not get around to the far side, last={last:?}"
    );
  }

  #[test]
  fn goalie_can_enter_own_penalty_area() {
    let params = OrcaParams {
      ..Default::default()
    };
    let world = WorldSnapshot {
      now: Instant::now(),
      self_id: 1,
      self_pos_mm: Vec2i::new(3_600, 0),
      self_vel_mm_s: Vec2i::default(),
      self_orientation: None,
      others: Vec::new(),
      ball: None,
      field: test_field(),
      allow_own_penalty_area: true,
    };
    let out = orca_step(&params, &world, Vec2i::new(2_000, 0), 2_000);
    assert!(out.x > 0, "out={:?}", out);
  }

  #[test]
  fn ball_is_treated_like_a_moving_obstacle() {
    let params = OrcaParams {
      ..Default::default()
    };
    let world = WorldSnapshot {
      now: Instant::now(),
      self_id: 1,
      self_pos_mm: Vec2i::new(0, 0),
      self_vel_mm_s: Vec2i::default(),
      self_orientation: None,
      others: Vec::new(),
      ball: Some(MovingObstacle {
        pos_mm: Vec2i::new(300, 0),
        vel_mm_s: Vec2i::new(0, 0),
        radius_mm: 180,
      }),
      field: test_field(),
      allow_own_penalty_area: false,
    };
    let out = orca_step(&params, &world, Vec2i::new(1_000, 0), 1_000);
    assert!(out.x <= 1_000);
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
