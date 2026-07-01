use crate::Robot;
use crate::communication::send_flags;
use crate::robot_logic::helpers::point_at_distance_from_a;
use crate::robot_logic::orca::{NavIntent, OrcaRequest, WorldSnapshot, nav_command_to_teensy};
use crate::robot_logic::vec::{Vec2f, Vec2i};

/// Distance to ball where we switch to direct control
const BALL_APPROACH_DISTANCE: f32 = 150f32;
/// Close acquisition should not be softened by ORCA. At this range the robot
/// has been tactically selected to win the ball, so drive directly into pickup.
const RAW_ACQUIRE_DISTANCE: f32 = 900f32;
/// Functions drives behind the ball and then drives forward and stops with ball in capturing zone

impl<C> Robot<C> {
  #[inline]
  pub fn get_ball(&mut self, world: &WorldSnapshot) {
    let direction_vec = Vec2f::new(
      f32::cos((self.packets.cp_data.cmd.orientation.unwrap_or_default() as f32).to_radians()),
      f32::sin((self.packets.cp_data.cmd.orientation.unwrap_or_default() as f32).to_radians()),
    );
    let robot_pos = Vec2f::new_from_cp(self.packets.robot_self.pos);
    let ball_pos = Vec2f::new_from_cp(self.packets.cp_data.ball.pos);
    let ball_vel = Vec2f::new_from_cp(self.packets.cp_data.ball.vel.unwrap_or_default());
    let ball_dist = (ball_pos - robot_pos).norm();

    // We are intentionally acquiring the ball, so spin the dribbler before IR
    // contact instead of waiting for the ball to already be in the mouth.
    self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
    self.packets.robot_msg.dribbler_pwr = 200;

    if self.packets.teensy_data.has_ball() {
      self.packets.robot_msg.speed = 0;
      self.packets.robot_msg.orient = self
        .packets
        .cp_data
        .cmd
        .orientation
        .unwrap_or(self.packets.robot_self.orientation as u32)
        as u16;
      return;
    }

    if ball_vel.norm() > 250f32 && ball_is_incoming(ball_pos, ball_vel, robot_pos, 1500f32) {
      let target = lead_intercept(robot_pos, ball_pos, ball_vel);

      if ball_dist <= RAW_ACQUIRE_DISTANCE {
        self.raw_acquire_towards(robot_pos, target, ball_pos);
        return;
      }

      let intent = NavIntent::GoToPosition {
        target_pos_mm: Vec2i::new(target.x as i32, target.y as i32),
        max_speed_mm_s: self.packets.cp_data.cmd.speed.unwrap_or_default().max(2000),
      };
      let nav_command = self.orca.step(OrcaRequest { intent, world });
      nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
      self.packets.robot_msg.speed = self.packets.robot_msg.speed.max(500);
      self.packets.robot_msg.orient = (ball_pos - robot_pos).angle_to_u16();
      return;
    }

    if ball_dist <= RAW_ACQUIRE_DISTANCE {
      self.raw_acquire_towards(robot_pos, ball_pos, ball_pos);
      return;
    }

    let to_ball = Vec2f::calculate_vector_2f(robot_pos, ball_pos);
    let capture_zone_to_ball =
      Vec2f::calculate_vector_2f(ball_pos, robot_pos + direction_vec.scale(80f32)).angle_to_u16();

    // First check, if we already are in front of the ball, if yes, move forwards
    // Transformation vector with respected input angle
    let trans_vector = Vec2f {
      x: -to_ball.x * f32::sin((self.packets.robot_self.orientation as f32).to_radians())
        + to_ball.y * f32::cos((self.packets.robot_self.orientation as f32).to_radians()),
      y: -to_ball.x * f32::cos((self.packets.robot_self.orientation as f32).to_radians())
        - to_ball.y * f32::sin((self.packets.robot_self.orientation as f32).to_radians()),
    };
    if trans_vector.y.is_sign_positive() && trans_vector.x.abs() < 15f32 {
      self.packets.robot_msg.dir = self.packets.cp_data.cmd.orientation.unwrap_or_default() as u16;
      self.packets.robot_msg.speed = acquisition_speed(self.packets.cp_data.cmd.speed);
      self.packets.robot_msg.orient = (ball_pos - robot_pos).angle_to_u16();

      return;
    } else if trans_vector.y.is_sign_positive() && trans_vector.x.abs() < 35f32 {
      self.packets.robot_msg.dir = capture_zone_to_ball;
      self.packets.robot_msg.speed = acquisition_speed(self.packets.cp_data.cmd.speed);
      self.packets.robot_msg.orient = (ball_pos - robot_pos).angle_to_u16();

      return;
    }

    // else move behind the ball
    let target =
      point_at_distance_from_a(ball_pos, ball_pos - direction_vec, BALL_APPROACH_DISTANCE)
        .unwrap_or(ball_pos);

    let intent = NavIntent::GoToPosition {
      target_pos_mm: Vec2i::new(target.x as i32, target.y as i32),
      max_speed_mm_s: self.packets.cp_data.cmd.speed.unwrap_or_default(),
    };
    let nav_command = self.orca.step(OrcaRequest { intent, world });
    nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
    self.packets.robot_msg.speed = self.packets.robot_msg.speed.max(500);
    self.packets.robot_msg.orient = (ball_pos - robot_pos).angle_to_u16();
  }

  #[inline]
  fn raw_acquire_towards(&mut self, robot_pos: Vec2f, target: Vec2f, face: Vec2f) {
    let delta = target - robot_pos;
    let dist = delta.norm();
    if dist <= 1f32 {
      self.packets.robot_msg.speed = 0;
    } else {
      self.packets.robot_msg.dir = delta.angle_to_u16();
      self.packets.robot_msg.speed =
        raw_acquisition_speed(dist, self.packets.cp_data.cmd.speed.unwrap_or_default());
    }
    self.packets.robot_msg.orient = (face - robot_pos).angle_to_u16();
  }
}

#[inline]
fn ball_is_incoming(ball_pos: Vec2f, ball_vel: Vec2f, robot_pos: Vec2f, radius: f32) -> bool {
  let v2 = ball_vel.norm_squared();
  if v2 < 1e-6 {
    return false;
  }

  let t = (robot_pos - ball_pos).dot(ball_vel) / v2;
  if t < -0.2 {
    return false;
  }

  let closest = ball_pos + ball_vel.scale(t.max(0f32));
  (closest - robot_pos).norm_squared() <= radius * radius
}

#[inline]
fn lead_intercept(robot_pos: Vec2f, ball_pos: Vec2f, ball_vel: Vec2f) -> Vec2f {
  const ROBOT_SPEED_MM_S: f32 = 2200f32;
  let mut t = 0f32;
  while t < 1.5f32 {
    let ball = ball_pos + ball_vel.scale(t);
    if (ball - robot_pos).norm() / ROBOT_SPEED_MM_S <= t {
      return ball;
    }
    t += 0.05f32;
  }

  let v2 = ball_vel.norm_squared().max(1e-6);
  let t = ((robot_pos - ball_pos).dot(ball_vel) / v2).clamp(0f32, 1.5f32);
  ball_pos + ball_vel.scale(t)
}

#[inline]
fn acquisition_speed(command_speed: Option<u32>) -> u16 {
  command_speed.unwrap_or(500).clamp(500, 900) as u16
}

#[inline]
fn raw_acquisition_speed(dist_mm: f32, command_speed: u32) -> u16 {
  let requested = command_speed.max(1600) as f32;
  (dist_mm * 4.0).clamp(450f32, requested.min(2200f32)) as u16
}
