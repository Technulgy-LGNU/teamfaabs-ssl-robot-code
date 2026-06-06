#[tokio::main]
async fn main() {
  // Start tracing
  tracing_subscriber::fmt().with_ansi(true).init();

  let mut robot = tf_jetsoncode::Robot::default().await;

  robot.run().await
}
