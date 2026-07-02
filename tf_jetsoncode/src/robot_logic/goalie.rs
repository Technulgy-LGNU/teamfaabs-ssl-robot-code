use crate::communication::send_flags;
use crate::robot_logic::RAW_MAX_SPEED_MM_S;
use crate::robot_logic::helpers::{
  clamp_to_own_penalty, inside_own_penalty_area, own_goal_side, own_goal_x,
};
use crate::robot_logic::orca::{NavIntent, OrcaRequest, WorldSnapshot, nav_command_to_teensy};
use crate::robot_logic::vec::{Vec2f, Vec2i};
use crate::{GoalieCarrierTrack, Robot};
use core_dump::proto::{CpInfos, CpRobot, CpTrackedRobot};

// How far the goalie should stay in front of the goal line when guarding.
const GOAL_LINE_MARGIN_MM: f32 = 120f32;
// Prediction horizon used to detect a kick/shot that is likely to reach goal.
const SHOT_LOOKAHEAD_S: f32 = 4f32;
// Allowed vertical miss tolerance when deciding that a ball is heading at goal.
const SHOT_Y_MARGIN_MM: f32 = 10_000f32;
// Only run the dribbler when the ball is close enough to be collected.
const DRIBBLER_RANGE_MM: f32 = 150f32;
// If the ball is in or just leaving our defense area, actively collect it.
const CATCH_BALL_RANGE_MM: f32 = 520f32;
const CATCH_BALL_SPEED_MM_S: f32 = 60f32;
const CATCH_LEAD_S: f32 = 0.25;
const CATCH_FIELD_EXIT_MARGIN_MM: f32 = 260f32;
const GOALIE_PASS_CHAIN_ADVANCE_MM: f32 = 500f32;
// Keeps the goalie inside the goal opening instead of hugging the exact edge.
const GUARD_Y_MARGIN_MM: f32 = 20f32;
// Opponent center distance to the ball that is treated as active possession.
const OPPONENT_HAS_BALL_DIST_MM: f32 = 180f32;
// Opponent must be at least this aligned with the vector to our goal.
const CARRIER_HEADING_TO_GOAL_DOT: f32 = 0.45;
// How far ahead to extrapolate a still-turning carrier's heading.
const CARRIER_ANGULAR_LEAD_S: f32 = 0.18;
// Above this turn rate, keep the prediction to only 10% outside the goal width.
const CARRIER_SETTLED_ANGULAR_DEG_S: f32 = 45f32;
const TURNING_EXTRA_GOAL_WIDTH: f32 = 0.10;
const CARRIER_STABLE_POSSESSION_S: f64 = 0.30;
const SHOT_BLOCKER_CORRIDOR_MM: f32 = 180f32;
const SHOT_BLOCKER_MIN_FORWARD_MM: f32 = 120f32;
const SHOT_BLOCKER_MAX_FRACTION_TO_GOAL: f32 = 0.86;

#[derive(Debug, Clone, Copy, PartialEq)]
struct CarrierShotPrediction {
  goal_y: f32,
  guard_y: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct GoaliePassTarget {
  pos: Vec2f,
}

impl<C> Robot<C> {
  #[inline]
  pub fn goalie(&mut self, world: &WorldSnapshot) {
    let self_pos = Vec2f::new_from_cp(self.packets.robot_self.pos);
    let ball_pos = Vec2f::new_from_cp(self.packets.cp_data.ball.pos);
    let ball_vel = self
      .packets
      .cp_data
      .ball
      .vel
      .map_or(Vec2f::new(0f32, 0f32), Vec2f::new_from_cp);
    let self_vel = self
      .packets
      .robot_self
      .vel
      .map_or(Vec2f::new(0f32, 0f32), Vec2f::new_from_cp);

    if self.packets.teensy_data.has_ball() {
      self.packets.robot_msg.speed = 0;
      self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
      self.packets.robot_msg.dribbler_pwr = 200;
      if let Some(target) = goalie_pass_target(&self.packets.cp_data, self_pos) {
        let to_target = target.pos - self_pos;
        let pass_angle = to_target.angle_to_u16();
        self.packets.robot_msg.orient = pass_angle;
        if heading_error_deg(self.packets.robot_self.orientation, pass_angle as i32) <= 10 {
          let pass_dist = to_target.norm();
          self.packets.robot_msg.kick_pwr = (pass_dist * 0.06).clamp(90f32, 200f32) as u8;
          self.packets.robot_msg.set_flag(send_flags::KICK);
        }
      } else {
        self.packets.robot_msg.orient = (Vec2f::new(0f32, 0f32) - self_pos).angle_to_u16();
      }
      return;
    }

    // Always face the ball globally, independent of the movement direction.
    self.packets.robot_msg.orient = (ball_pos - self_pos).angle_to_u16();
    if should_preempt_goalie_dribbler(&self.packets.cp_data.infos, self_pos, ball_pos, ball_vel) {
      self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
      self.packets.robot_msg.dribbler_pwr = 200;
    }

    let carrier_prediction = predict_carrier_shot(
      &self.packets.cp_data,
      ball_pos,
      &mut self.goalie_carrier_track,
    );

    // Choose a defensive target: either a predicted interception point or a guard point.
    let target = goalie_target(
      &self.packets.cp_data.infos,
      self_pos,
      ball_pos,
      ball_vel,
      carrier_prediction,
    );

    if inside_own_penalty_area(&self.packets.cp_data.infos, self_pos) {
      // Once inside the penalty area, use raw field-global motion instead of ORCA.
      goalie_move_towards(&mut self.packets.robot_msg, self_pos, self_vel, target);

      // Keep looking at the ball while moving.
      self.packets.robot_msg.orient = (ball_pos - self_pos).angle_to_u16();
      // self.packets.robot_msg.orient = ball_pos.scale(-1f32).angle_to_u16();
    } else {
      // ORCA is only used for the approach into the penalty area.
      let intent = NavIntent::GoToPosition {
        target_pos_mm: Vec2i::new(target.x as i32, target.y as i32),
        max_speed_mm_s: RAW_MAX_SPEED_MM_S as u32,
      };

      let nav_command = self.orca.step(OrcaRequest { intent, world });
      nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
      self.packets.robot_msg.orient = (ball_pos - self_pos).angle_to_u16();
    }
  }
}

#[inline]
fn should_run_goalie_dribbler(self_pos: Vec2f, ball_pos: Vec2f, ball_vel: Vec2f) -> bool {
  (ball_pos - self_pos).norm_squared() <= DRIBBLER_RANGE_MM * DRIBBLER_RANGE_MM
    && ball_vel.norm_squared() > 100f32 * 100f32
}

#[inline]
fn should_preempt_goalie_dribbler(
  infos: &CpInfos, self_pos: Vec2f, ball_pos: Vec2f, ball_vel: Vec2f,
) -> bool {
  if should_run_goalie_dribbler(self_pos, ball_pos, ball_vel) {
    return true;
  }

  let ball_dist_sq = (ball_pos - self_pos).norm_squared();
  if ball_dist_sq <= CATCH_BALL_RANGE_MM * CATCH_BALL_RANGE_MM
    && (inside_own_penalty_area(infos, ball_pos)
      || ball_is_exiting_own_penalty(infos, ball_pos, ball_vel))
  {
    return true;
  }

  predict_intercept(infos, self_pos, ball_pos, ball_vel).is_some()
}

#[inline]
fn should_collect_goalie_ball(
  infos: &CpInfos, self_pos: Vec2f, ball_pos: Vec2f, ball_vel: Vec2f,
) -> bool {
  if (ball_pos - self_pos).norm_squared() > CATCH_BALL_RANGE_MM * CATCH_BALL_RANGE_MM {
    return false;
  }
  if ball_vel.norm_squared() < CATCH_BALL_SPEED_MM_S * CATCH_BALL_SPEED_MM_S {
    return false;
  }

  inside_own_penalty_area(infos, ball_pos) || ball_is_exiting_own_penalty(infos, ball_pos, ball_vel)
}

#[inline]
fn ball_is_exiting_own_penalty(infos: &CpInfos, ball_pos: Vec2f, ball_vel: Vec2f) -> bool {
  let goal_x = own_goal_x(infos);
  let goal_side = own_goal_side(infos);
  let penalty_depth = infos.penalty_area_height as f32;
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let y_half = infos.penalty_area_width as f32 * 0.5 + 120f32;
  let field_side_speed = ball_vel.x * -goal_side;
  let past_front = (ball_pos.x - penalty_outer_x) * -goal_side;

  ball_pos.y.abs() <= y_half
    && field_side_speed > CATCH_BALL_SPEED_MM_S
    && (-80f32..=CATCH_FIELD_EXIT_MARGIN_MM).contains(&past_front)
}

#[inline]
fn goalie_collect_target(infos: &CpInfos, ball_pos: Vec2f, ball_vel: Vec2f) -> Vec2f {
  let speed = ball_vel.norm();
  let lead = if speed > CATCH_BALL_SPEED_MM_S {
    ball_vel * CATCH_LEAD_S
  } else {
    Vec2f::new(0f32, 0f32)
  };
  clamp_to_goalie_collect_area(infos, ball_pos + lead)
}

#[inline]
fn clamp_to_goalie_collect_area(infos: &CpInfos, point: Vec2f) -> Vec2f {
  let goal_x = own_goal_x(infos);
  let goal_side = own_goal_side(infos);
  let penalty_depth = infos.penalty_area_height as f32;
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let field_exit_x = penalty_outer_x - goal_side * CATCH_FIELD_EXIT_MARGIN_MM;
  let x_min = goal_x.min(field_exit_x) + 40f32;
  let x_max = goal_x.max(field_exit_x) - 40f32;
  let y_half = infos.penalty_area_width as f32 * 0.5 - 40f32;

  Vec2f::new(point.x.clamp(x_min, x_max), point.y.clamp(-y_half, y_half))
}

#[inline]
fn goalie_move_towards(
  msg: &mut crate::communication::TeensySendMsg, self_pos: Vec2f, self_vel: Vec2f, target: Vec2f,
) {
  let delta = target - self_pos;
  let distance = delta.norm();
  if distance <= crate::robot_logic::RAW_STOP_RADIUS_MM && self_vel.norm() < 120f32 {
    msg.speed = 0;
    return;
  }

  let desired = delta * 12f32 - self_vel * 1.25f32;
  let speed = desired.norm();
  if speed <= 1f32 {
    msg.speed = 0;
    return;
  }

  msg.dir = desired.angle_to_u16();
  msg.speed = speed.clamp(0f32, RAW_MAX_SPEED_MM_S) as u16;
}

#[inline]
fn goalie_target(
  infos: &CpInfos, self_pos: Vec2f, ball_pos: Vec2f, ball_vel: Vec2f,
  carrier_prediction: Option<CarrierShotPrediction>,
) -> Vec2f {
  // Own goal is on x- or x+ depending on the robot_goal setting.
  let goal_x = own_goal_x(infos);
  let goal_side = own_goal_side(infos);
  // Clamp to the goal width, allowing roughly one robot radius beyond each post.
  let goal_half_width = infos.goal_width as f32 * 0.5 + 90f32;
  let goal_guard_x = goal_x - goal_side * GOAL_LINE_MARGIN_MM;

  // If the ball is moving toward goal fast enough, try to intercept it.
  if let Some(intercept) = predict_intercept(infos, self_pos, ball_pos, ball_vel) {
    return clamp_to_own_penalty(infos, intercept);
  }

  if should_collect_goalie_ball(infos, self_pos, ball_pos, ball_vel) {
    return goalie_collect_target(infos, ball_pos, ball_vel);
  }

  if let Some(prediction) = carrier_prediction {
    return clamp_to_own_penalty(infos, Vec2f::new(goal_guard_x, prediction.guard_y));
  }

  // No imminent shot: hug the goal line and slide laterally to stay between the
  // ball and the centre of the goal.
  Vec2f::new(
    goal_guard_x,
    ball_pos.y.clamp(
      -goal_half_width + GUARD_Y_MARGIN_MM,
      goal_half_width - GUARD_Y_MARGIN_MM,
    ),
  )
}

#[inline]
fn predict_carrier_shot(
  cp_data: &CpRobot, ball_pos: Vec2f, track: &mut Option<GoalieCarrierTrack>,
) -> Option<CarrierShotPrediction> {
  let carrier = opponent_carrier(cp_data, ball_pos)?;
  let (angular_vel_deg_s, possession_time_s) =
    estimate_carrier_angular_velocity(cp_data, carrier, track);
  if possession_time_s < CARRIER_STABLE_POSSESSION_S {
    return None;
  }
  if opponent_shot_lane_blocked(cp_data, carrier) {
    return None;
  }
  predict_carrier_shot_from_state(&cp_data.infos, carrier, angular_vel_deg_s)
}

#[inline]
fn opponent_carrier(cp_data: &CpRobot, ball_pos: Vec2f) -> Option<CpTrackedRobot> {
  let opponents = if cp_data.infos.team_color {
    &cp_data.robots_yellow
  } else {
    &cp_data.robots_blue
  };

  opponents
    .iter()
    .copied()
    .filter(|robot| robot.visibility > 20)
    .filter(|robot| {
      (Vec2f::new_from_cp(robot.pos) - ball_pos).norm_squared()
        <= OPPONENT_HAS_BALL_DIST_MM * OPPONENT_HAS_BALL_DIST_MM
    })
    .min_by(|a, b| {
      let a_dist = (Vec2f::new_from_cp(a.pos) - ball_pos).norm_squared();
      let b_dist = (Vec2f::new_from_cp(b.pos) - ball_pos).norm_squared();
      a_dist
        .partial_cmp(&b_dist)
        .unwrap_or(std::cmp::Ordering::Equal)
    })
}

#[inline]
fn goalie_pass_target(cp_data: &CpRobot, self_pos: Vec2f) -> Option<GoaliePassTarget> {
  let own = if cp_data.infos.team_color {
    &cp_data.robots_blue
  } else {
    &cp_data.robots_yellow
  };
  let opponents = if cp_data.infos.team_color {
    &cp_data.robots_yellow
  } else {
    &cp_data.robots_blue
  };
  let goal_side = own_goal_side(&cp_data.infos);
  let penalty_front_x =
    own_goal_x(&cp_data.infos) - goal_side * cp_data.infos.penalty_area_height as f32;
  let own_positions = own
    .iter()
    .filter(|robot| robot.robot_id != cp_data.robot_id)
    .filter(|robot| robot.visibility > 20)
    .map(|robot| Vec2f::new_from_cp(robot.pos))
    .filter(|pos| !inside_own_penalty_area(&cp_data.infos, *pos))
    .filter(|pos| (*pos - self_pos).norm() > 700f32)
    .collect::<Vec<_>>();

  own_positions
    .iter()
    .copied()
    .map(|pos| {
      let field_side = ((pos.x - penalty_front_x) * -goal_side).max(0f32);
      let central = (cp_data.infos.height as f32 * 0.5 - pos.y.abs()).max(0f32) * 0.05;
      let distance = (pos - self_pos).norm();
      let nearest_opp = opponents
        .iter()
        .filter(|robot| robot.visibility > 20)
        .map(|robot| (Vec2f::new_from_cp(robot.pos) - pos).norm())
        .fold(f32::INFINITY, f32::min);
      let has_chain_outlet = own_positions.iter().any(|other| {
        *other != pos && ((other.x - pos.x) * -goal_side) >= GOALIE_PASS_CHAIN_ADVANCE_MM
      });
      let chain_bonus = if has_chain_outlet { 2_000f32 } else { 0f32 };
      let score = chain_bonus + field_side * 0.55 + nearest_opp.min(1800f32) * 0.35 + central
        - distance * 0.08;
      (GoaliePassTarget { pos }, score)
    })
    .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
    .map(|(target, _)| target)
}

#[inline]
fn estimate_carrier_angular_velocity(
  cp_data: &CpRobot, carrier: CpTrackedRobot, track: &mut Option<GoalieCarrierTrack>,
) -> (f32, f64) {
  let heading_deg = carrier.orientation as f32;
  let timestamp_s = cp_data.timestamp;
  let first_seen_timestamp_s = track
    .filter(|previous| previous.robot_id == carrier.robot_id)
    .map(|previous| previous.first_seen_timestamp_s)
    .unwrap_or(timestamp_s);
  let angular_vel = track
    .and_then(|previous| {
      if previous.robot_id != carrier.robot_id {
        return None;
      }
      let dt = timestamp_s - previous.timestamp_s;
      if !(0.005..=0.5).contains(&dt) {
        return None;
      }
      Some(angle_delta_deg(heading_deg, previous.heading_deg) / dt as f32)
    })
    .unwrap_or(0f32);

  *track = Some(GoalieCarrierTrack {
    robot_id: carrier.robot_id,
    heading_deg,
    timestamp_s,
    first_seen_timestamp_s,
  });
  (angular_vel, timestamp_s - first_seen_timestamp_s)
}

#[inline]
fn opponent_shot_lane_blocked(cp_data: &CpRobot, carrier: CpTrackedRobot) -> bool {
  let carrier_pos = Vec2f::new_from_cp(carrier.pos);
  let goal = Vec2f::new(own_goal_x(&cp_data.infos), 0f32);
  let lane = goal - carrier_pos;
  let lane_len = lane.norm();
  if lane_len <= 1f32 {
    return false;
  }

  let own = if cp_data.infos.team_color {
    &cp_data.robots_blue
  } else {
    &cp_data.robots_yellow
  };
  let opponents = if cp_data.infos.team_color {
    &cp_data.robots_yellow
  } else {
    &cp_data.robots_blue
  };

  own
    .iter()
    .chain(opponents.iter())
    .copied()
    .filter(|robot| robot.visibility > 20)
    .filter(|robot| robot.robot_id != carrier.robot_id || robot.pos != carrier.pos)
    .any(|robot| {
      let blocker_pos = Vec2f::new_from_cp(robot.pos);
      let along = (blocker_pos - carrier_pos).dot(lane) / lane_len;
      let fraction = along / lane_len;
      along >= SHOT_BLOCKER_MIN_FORWARD_MM
        && fraction <= SHOT_BLOCKER_MAX_FRACTION_TO_GOAL
        && point_segment_dist(blocker_pos, carrier_pos, goal) <= SHOT_BLOCKER_CORRIDOR_MM
    })
}

#[inline]
fn predict_carrier_shot_from_state(
  infos: &CpInfos, carrier: CpTrackedRobot, angular_vel_deg_s: f32,
) -> Option<CarrierShotPrediction> {
  let carrier_pos = Vec2f::new_from_cp(carrier.pos);
  let goal_x = own_goal_x(infos);
  let goal_side = own_goal_side(infos);
  let goal = Vec2f::new(goal_x, 0f32);
  let current_dir = heading_dir(carrier.orientation as f32);
  let to_goal = (goal - carrier_pos).normalized();

  if current_dir.dot(to_goal) < CARRIER_HEADING_TO_GOAL_DOT {
    return None;
  }

  let current_goal_y = project_y_at_x(carrier_pos, current_dir, goal_x)?;
  let lead_heading = carrier.orientation as f32 + angular_vel_deg_s * CARRIER_ANGULAR_LEAD_S;
  let led_goal_y =
    project_y_at_x(carrier_pos, heading_dir(lead_heading), goal_x).unwrap_or(current_goal_y);
  let goal_y = clamp_carrier_goal_y(infos, led_goal_y, angular_vel_deg_s);
  let guard_x = goal_x - goal_side * GOAL_LINE_MARGIN_MM;
  let guard_y = project_guard_y_from_goal_y(carrier_pos, goal_x, guard_x, goal_y);

  Some(CarrierShotPrediction { goal_y, guard_y })
}

#[inline]
fn clamp_carrier_goal_y(infos: &CpInfos, goal_y: f32, angular_vel_deg_s: f32) -> f32 {
  let goal_width = infos.goal_width as f32;
  let goal_half_width = goal_width * 0.5;
  let turning_limit = goal_half_width + goal_width * TURNING_EXTRA_GOAL_WIDTH;
  let settled_limit = (infos.penalty_area_width as f32 * 0.5 - 60f32).max(turning_limit);
  let settled = 1f32 - (angular_vel_deg_s.abs() / CARRIER_SETTLED_ANGULAR_DEG_S).clamp(0f32, 1f32);
  let limit = turning_limit + (settled_limit - turning_limit) * settled;
  goal_y.clamp(-limit, limit)
}

#[inline]
fn project_y_at_x(origin: Vec2f, dir: Vec2f, x: f32) -> Option<f32> {
  if dir.x.abs() <= 1e-5 {
    return None;
  }

  let t = (x - origin.x) / dir.x;
  (t.is_finite() && t > 0f32).then_some(origin.y + dir.y * t)
}

#[inline]
fn project_guard_y_from_goal_y(carrier_pos: Vec2f, goal_x: f32, guard_x: f32, goal_y: f32) -> f32 {
  let total_x = goal_x - carrier_pos.x;
  if total_x.abs() <= 1e-5 {
    return goal_y;
  }

  let t = ((guard_x - carrier_pos.x) / total_x).clamp(0f32, 1f32);
  carrier_pos.y + (goal_y - carrier_pos.y) * t
}

#[inline]
fn point_segment_dist(point: Vec2f, a: Vec2f, b: Vec2f) -> f32 {
  let ab = b - a;
  let len2 = ab.dot(ab);
  if len2 <= 1e-6 {
    return (point - a).norm();
  }
  let t = (point - a).dot(ab) / len2;
  (point - (a + ab * t.clamp(0f32, 1f32))).norm()
}

#[inline]
fn heading_dir(heading_deg: f32) -> Vec2f {
  let radians = heading_deg.to_radians();
  Vec2f::new(radians.cos(), radians.sin())
}

#[inline]
fn angle_delta_deg(current: f32, previous: f32) -> f32 {
  (current - previous + 180f32).rem_euclid(360f32) - 180f32
}

#[inline]
fn heading_error_deg(current: i32, target: i32) -> i32 {
  let error = (target - current + 180).rem_euclid(360) - 180;
  error.abs()
}

#[inline]
pub(crate) fn predict_intercept(
  infos: &CpInfos, self_pos: Vec2f, ball_pos: Vec2f, ball_vel: Vec2f,
) -> Option<Vec2f> {
  let goal_x = own_goal_x(infos);
  let goal_side = own_goal_side(infos);
  // Positive values mean the ball is moving toward our goal line.
  let vel_toward_goal = ball_vel.x * goal_side;

  if vel_toward_goal <= 120f32 || ball_vel.x.abs() <= 1f32 || ball_vel.norm() <= 100f32 {
    return None;
  }

  // Estimate when the ball reaches the goal line in the current trajectory.
  let t_goal = (goal_x - ball_pos.x) / ball_vel.x;
  if !(0f32..=SHOT_LOOKAHEAD_S).contains(&t_goal) {
    return None;
  }

  // Estimate the y-position at impact to see whether this is actually a shot.
  let predicted_y = ball_pos.y + ball_vel.y * t_goal;
  let goal_half_width = infos.goal_width as f32 * 0.5;
  if predicted_y.abs() > goal_half_width + SHOT_Y_MARGIN_MM {
    return None;
  }

  let penalty_depth = infos.penalty_area_height as f32;
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let x_min = goal_x.min(penalty_outer_x) + 60f32;
  let x_max = goal_x.max(penalty_outer_x) - 60f32;
  let y_half = infos.penalty_area_width as f32 * 0.5 - 60f32;

  if predicted_y.abs() > y_half && ball_vel.y.abs() > 1f32 {
    let side_y = predicted_y.signum() * y_half;
    let t_side = (side_y - ball_pos.y) / ball_vel.y;
    if (0f32..=SHOT_LOOKAHEAD_S).contains(&t_side) {
      let side_x = (ball_pos.x + ball_vel.x * t_side).clamp(x_min, x_max);
      return Some(Vec2f::new(side_x, side_y));
    }
  }

  let v2 = ball_vel.norm_squared();
  let t_closest = ((self_pos - ball_pos).dot(ball_vel) / v2).clamp(0f32, SHOT_LOOKAHEAD_S);
  let mut target = ball_pos + ball_vel * t_closest;

  if target.x < x_min || target.x > x_max {
    let target_x = target.x.clamp(x_min, x_max);
    let t_at_x = ((target_x - ball_pos.x) / ball_vel.x).clamp(0f32, SHOT_LOOKAHEAD_S);
    target = ball_pos + ball_vel * t_at_x;
  }

  target.x = target.x.clamp(x_min, x_max);
  target.y = target.y.clamp(-y_half, y_half);

  Some(target)
}

#[cfg(test)]
mod tests {
  use super::*;
  use core_dump::proto::CpVector2;

  fn infos() -> CpInfos {
    CpInfos {
      team_color: true,
      width: 9000,
      height: 6000,
      goal_width: 1000,
      penalty_area_width: 2000,
      penalty_area_height: 1000,
      team_site: true,
      ..Default::default()
    }
  }

  fn robot(id: u32, x: i32, y: i32, heading_deg: i32) -> CpTrackedRobot {
    CpTrackedRobot {
      robot_id: id,
      pos: CpVector2 { x, y },
      orientation: heading_deg,
      vel: None,
      visibility: 255,
    }
  }

  fn cp_robot() -> CpRobot {
    CpRobot {
      robot_id: 0,
      infos: infos(),
      ..Default::default()
    }
  }

  #[test]
  fn turning_carrier_prediction_stays_within_ten_percent_goal_width() {
    let infos = infos();
    let carrier = robot(1, -1500, 0, 180);

    let prediction = predict_carrier_shot_from_state(&infos, carrier, 300f32).unwrap();

    let turning_limit =
      infos.goal_width as f32 * 0.5 + infos.goal_width as f32 * TURNING_EXTRA_GOAL_WIDTH;
    assert!(prediction.goal_y.abs() <= turning_limit + 1e-3);
  }

  #[test]
  fn settled_carrier_prediction_can_move_past_ten_percent_goal_width() {
    let infos = infos();
    let carrier = robot(1, -1500, 0, 166);

    let prediction = predict_carrier_shot_from_state(&infos, carrier, 0f32).unwrap();

    let turning_limit =
      infos.goal_width as f32 * 0.5 + infos.goal_width as f32 * TURNING_EXTRA_GOAL_WIDTH;
    let settled_limit = infos.penalty_area_width as f32 * 0.5 - 60f32;
    assert!(prediction.goal_y.abs() > turning_limit);
    assert!(prediction.goal_y.abs() <= settled_limit);
  }

  #[test]
  fn carrier_heading_delta_wraps_around_zero() {
    assert_eq!(angle_delta_deg(2f32, 358f32), 4f32);
    assert_eq!(angle_delta_deg(358f32, 2f32), -4f32);
  }

  #[test]
  fn new_carrier_is_not_used_for_goalie_shot_prediction() {
    let mut cp_data = cp_robot();
    cp_data.timestamp = 10f64;
    cp_data.ball.pos = CpVector2 { x: -1500, y: 0 };
    cp_data.robots_yellow = vec![robot(1, -1500, 0, 180)];
    let mut track = None;

    let prediction = predict_carrier_shot(&cp_data, Vec2f::new(-1500f32, 0f32), &mut track);

    assert_eq!(prediction, None);
  }

  #[test]
  fn blocked_carrier_is_not_used_for_goalie_shot_prediction() {
    let mut cp_data = cp_robot();
    cp_data.timestamp = 10f64;
    cp_data.ball.pos = CpVector2 { x: -1500, y: 0 };
    cp_data.robots_blue = vec![robot(0, -2500, 0, 0)];
    cp_data.robots_yellow = vec![robot(1, -1500, 0, 180)];
    let mut track = Some(GoalieCarrierTrack {
      robot_id: 1,
      heading_deg: 180f32,
      timestamp_s: 9.5,
      first_seen_timestamp_s: 9.5,
    });

    let prediction = predict_carrier_shot(&cp_data, Vec2f::new(-1500f32, 0f32), &mut track);

    assert_eq!(prediction, None);
  }

  #[test]
  fn goalie_collects_ball_rolling_out_of_defense_area() {
    let infos = infos();
    let self_pos = Vec2f::new(-3850f32, 0f32);
    let ball_pos = Vec2f::new(-3420f32, 0f32);
    let ball_vel = Vec2f::new(300f32, 0f32);

    assert!(ball_is_exiting_own_penalty(&infos, ball_pos, ball_vel));
    assert!(should_collect_goalie_ball(
      &infos, self_pos, ball_pos, ball_vel
    ));

    let target = goalie_collect_target(&infos, ball_pos, ball_vel);
    assert!(target.x > -3500f32);
    assert!(target.x <= -3240f32 + 1e-3);
  }

  #[test]
  fn goalie_preemptively_runs_dribbler_for_slow_defense_ball() {
    let infos = infos();
    let self_pos = Vec2f::new(-3850f32, 0f32);
    let ball_pos = Vec2f::new(-3750f32, 0f32);
    let slow_ball = Vec2f::new(10f32, 0f32);

    assert!(should_preempt_goalie_dribbler(
      &infos, self_pos, ball_pos, slow_ball
    ));
    assert!(!should_collect_goalie_ball(
      &infos, self_pos, ball_pos, slow_ball
    ));
  }

  #[test]
  fn goalie_preemptively_runs_dribbler_for_incoming_shot() {
    let infos = infos();
    let self_pos = Vec2f::new(-4200f32, 0f32);
    let ball_pos = Vec2f::new(-1500f32, 120f32);
    let ball_vel = Vec2f::new(-1800f32, -40f32);

    assert!(predict_intercept(&infos, self_pos, ball_pos, ball_vel).is_some());
    assert!(should_preempt_goalie_dribbler(
      &infos, self_pos, ball_pos, ball_vel
    ));
  }

  #[test]
  fn goalie_pass_target_uses_teammate_outside_defense_area() {
    let mut cp_data = cp_robot();
    cp_data.robots_blue = vec![
      robot(0, -4300, 0, 0),
      robot(1, -3700, 0, 0),
      robot(2, -2100, 650, 0),
      robot(3, -900, -350, 0),
    ];
    cp_data.robots_yellow = vec![robot(4, -2200, -700, 180)];

    let target = goalie_pass_target(&cp_data, Vec2f::new(-4300f32, 0f32)).unwrap();

    assert_eq!(target.pos, Vec2f::new(-2100f32, 650f32));
    assert!(!inside_own_penalty_area(&cp_data.infos, target.pos));
  }
}
