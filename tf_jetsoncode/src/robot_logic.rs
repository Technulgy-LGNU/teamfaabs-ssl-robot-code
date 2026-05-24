use crate::communication::{TeensySendMsg, VisionMsg, send_flags};
use crate::config;
use crate::proto::{CpRobot, CpTrackedRobot, CpVector2, Vector2f};
use crate::robot_logic::orca::{
  OrcaHandle, OrcaRequest, Vec2i, WorldSnapshot, nav_command_to_teensy,
};

pub mod goalie;
pub mod orca;

pub async fn command(
  cfg: &config::Config, cp_data: &CpRobot, orca: &OrcaHandle, world: &WorldSnapshot,
  vision_data: &VisionMsg, mut msg: TeensySendMsg, stop: bool,
) -> TeensySendMsg {
  let mut robot_self: CpTrackedRobot = Default::default();
  if cfg.robot_team == "yellow" {
    robot_self = *cp_data
      .robots_yellow
      .iter()
      .find(|r| r.robot_id == cfg.robot_id as u32)
      .unwrap_or_else(|| {
        return &robot_self;
      });
  } else if cfg.robot_team == "blue" {
    robot_self = *cp_data
      .robots_blue
      .iter()
      .find(|r| r.robot_id == cfg.robot_id as u32)
      .unwrap_or_else(|| {
        return &robot_self;
      });
  } else {
    panic!("Unknown team: {}", cfg.robot_team);
  }

  match cp_data.cmd.task {
    0 => {
      // UNKNOWN
      println!("UNKNOWN");
    }
    1 => {
      // Speed check

      // Drive to pos
      let intent = orca::NavIntent::GoToPosition {
        target_pos_mm: Vec2i {
          x: cp_data.cmd.pos.unwrap_or_default().x,
          y: cp_data.cmd.pos.unwrap_or_default().y,
        },
        max_speed_mm_s: cp_data.cmd.speed.unwrap_or(1500),
      };

      orca.publish(OrcaRequest {
        world: world.clone(),
        intent,
      })
    }
    2 => {
      // Kick in kick dir

      // First rotate robot
      if (robot_self.orientation - cp_data.cmd.kick_orient.unwrap_or_default() as i32).abs() < 2 {
        // If we are facing the right direction (variance of two degrees)
        msg.orient = cp_data.cmd.kick_orient.unwrap_or_default() as u16;
      } else {
        msg.kick_pwr = cp_data.cmd.kick_speed.unwrap_or_default() as u8;
        msg.set_flag(send_flags::KICK);
      }
    }
    3 => {
      // Chip in kick dir

      // First rotate robot
      if (robot_self.orientation - cp_data.cmd.kick_orient.unwrap_or_default() as i32).abs() < 2 {
        // If we are facing the right direction (variance of two degrees)
        msg.orient = cp_data.cmd.kick_orient.unwrap_or_default() as u16;
      } else {
        msg.kick_pwr = cp_data.cmd.kick_speed.unwrap_or_default() as u8;
        msg.set_flag(send_flags::CHIP);
      }
    }
    4 => {
      // Rec Kick
    }
    5 => {
      // Steal Ball
      msg = get_ball(cp_data, orca, world, vision_data, msg, robot_self).await;
    }
    6 => {
      // Dribble the Ball
    }
    7 => {
      // Position the Ball
    }
    9 => {
      // Kickoff
    }
    11 => {
      // Free kick
    }
    _ => {
      println!("Unknown task: {}", cp_data.cmd.task);
    }
  }

  msg
}

async fn get_ball(
  cp_data: &CpRobot, orca: &OrcaHandle, world: &WorldSnapshot, vision_data: &VisionMsg,
  mut msg: TeensySendMsg, robot_self: CpTrackedRobot,
) -> TeensySendMsg {
  let dist = distance_cpv(robot_self.pos, cp_data.ball.pos);
  println!("Distance to ball: {:?}", dist);

  // Check distance to ball, either use orca for long distance or use direct control for taking the ball
  if dist > 500.0 {
    let intent = orca::NavIntent::GoToPosition {
      target_pos_mm: Vec2i {
        x: cp_data.ball.pos.x as i32,
        y: cp_data.ball.pos.y as i32,
      },
      max_speed_mm_s: cp_data.cmd.speed(),
    };

    orca.publish(OrcaRequest {
      world: world.clone(),
      intent,
    });

    let latest = match orca.changed().await {
      Ok(nav_command) => nav_command,
      Err(e) => {
        println!("Error waiting for orca update: {:?}", e);
        return msg;
      }
    };

    println!("Orca output: {:?}", latest);
    msg = nav_command_to_teensy(msg, latest);
  } else {
    // Calculate direction to ball as Vec2i
    let to_ball = calculate_vector(robot_self.pos, cp_data.ball.pos);

    // Transformation vector with respected input angle
    let trans_vector = Vector2f {
      x: -to_ball.x as f32 * f32::sin((cp_data.cmd.orientation() as f32).to_radians())
        + to_ball.y as f32 * f32::cos((cp_data.cmd.orientation() as f32).to_radians()),
      y: -to_ball.x as f32 * f32::cos((cp_data.cmd.orientation() as f32).to_radians())
        - to_ball.y as f32 * f32::sin((cp_data.cmd.orientation() as f32).to_radians()),
    };

    let mut comp_dir: f32;
    let x_c: f32 = 120.0;
    let y_c: f32 = 0.0;
    let d: f32 = 125.0;

    if trans_vector.x < 0.0 {
      comp_dir = compute_vector_angle(x_c, y_c, d, -trans_vector.x, -trans_vector.y).to_degrees();
    } else {
      comp_dir =
        180.0 - compute_vector_angle(x_c, y_c, d, trans_vector.x, -trans_vector.y).to_degrees();
    }

    if (trans_vector.x < 15.0)
      && (trans_vector.x > -15.0)
      && (trans_vector.y < 100.0)
      && (trans_vector.y > 0.0)
    {
      comp_dir = 90.0;
    }

    comp_dir = comp_dir - 90.0 + robot_self.orientation as f32;

    println!("Computed Direction: {:?}", comp_dir);

    while comp_dir.is_sign_negative() {
      comp_dir += 360.0;
    }

    // Set Teensy mesage
    msg.dir = comp_dir as u16;
    msg.speed = 300;
  }
  msg
}

fn distance_cpv(a: CpVector2, b: CpVector2) -> f32 {
  let dx = (a.x - b.x) as f32;
  let dy = (a.y - b.y) as f32;
  (dx * dx + dy * dy).sqrt()
}

fn calculate_vector(a: CpVector2, b: CpVector2) -> Vec2i {
  Vec2i {
    x: a.x - b.x,
    y: a.y - b.y,
  }
}

use std::f32::consts::PI;

/// Entspricht Arduino constrain()
fn constrain(value: f32, min: f32, max: f32) -> f32 {
  value.clamp(min, max)
}

/// Berechnet den Winkel eines Vektors relativ zu einem Kreis
pub fn compute_vector_angle(x_c: f32, y_c: f32, r: f32, x: f32, y: f32) -> f32 {
  // Abstand zum Kreismittelpunkt
  let mut d = (x - x_c).hypot(y - y_c);

  // Verhindere Division durch 0
  if d.is_nan() || d == 0.0 {
    d = 1e-6;
  }

  let angle;

  if d > r {
    // Tangentenwinkel berechnen
    let theta = (y - y_c).atan2(x - x_c);

    let alpha = constrain(r / d, -1.0, 1.0).asin();

    // Winkel der Tangente
    angle = PI + theta + alpha;
  } else {
    // Spiegelung berechnen
    let theta = (y - y_c).atan2(x - x_c);

    // Punkt im Kreis
    let i_c_x = theta.cos() * d;
    let i_c_y = theta.sin() * d;

    // Spiegelpunkt
    let o_c_x = theta.cos() * (2.0 * r - d);
    let o_c_y = theta.sin() * (2.0 * r - d);

    // Falls Werte NaN sind -> Default zurückgeben
    if i_c_x.is_nan() || i_c_y.is_nan() || o_c_x.is_nan() || o_c_y.is_nan() {
      return 0.0;
    }

    // Spiegelmatrix-Berechnung
    let theta3 = (i_c_y - y_c).atan2(i_c_x - x_c);

    let s11 = (2.0 * theta3).cos();
    let s12 = (2.0 * theta3).sin();
    let s21 = (2.0 * theta3).sin();
    let s22 = -(2.0 * theta3).cos();

    // Gespiegelter Tangentenwinkel
    let theta_sp = (o_c_y - y_c).atan2(o_c_x - x_c);

    let mut denom = (o_c_x - x_c).hypot(o_c_y - y_c);

    // Verhindere NaN durch ungültige Werte
    if denom.is_nan() || denom == 0.0 {
      denom = 1e-6;
    }

    let alpha_sp = constrain(r / denom, -1.0, 1.0).asin();

    let theta1_sp = PI + theta_sp + alpha_sp;

    // Transformation des Winkels
    let new_x = -(s11 * theta1_sp.cos() + s12 * theta1_sp.sin());

    let new_y = -(s21 * theta1_sp.cos() + s22 * theta1_sp.sin());

    // Verhindere NaN in atan2()
    if new_x.is_nan() || new_y.is_nan() {
      return 0.0;
    }

    angle = new_y.atan2(new_x);
  }

  angle
}
