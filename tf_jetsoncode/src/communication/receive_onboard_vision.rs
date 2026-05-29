use crate::communication::{EventShare, VisionMsg};
use crate::config;

pub async fn receive_onboard_vision(cfg: &config::Config, tx: EventShare) {
  let ov_stream = match tokio::net::UnixStream::connect(&cfg.onboard_vision_socket_path).await {
    Ok(s) => s,
    Err(e) => {
      eprintln!("Failed to connect to onboard vision socket: {}", e);
      return;
    }
  };

  tokio::task::spawn(async move {
    let mut buf = [0u8; 1024];

    loop {
      match ov_stream.try_read(&mut buf) {
        Ok(size) => {
          if size > 12 {
            let msg = VisionMsg {
              x: f32::from_le_bytes(buf[0..4].try_into().unwrap_or([0; 4])),
              y: f32::from_le_bytes(buf[4..8].try_into().unwrap_or([0; 4])),
              size: f32::from_be_bytes(buf[8..12].try_into().unwrap_or([0; 4])),
            };

            let mut lock = tx.lock().await;

            lock.1 = Some(msg);
          }
        }
        Err(e) => {
          eprintln!("Failed to read from onboard vision socket: {}", e);
        }
      };
    }
  });
}
