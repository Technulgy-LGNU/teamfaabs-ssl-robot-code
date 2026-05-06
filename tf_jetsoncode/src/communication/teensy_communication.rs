use crate::communication::{EventShare, TeensyOut, TeensyRecMSG};
use crate::config;

pub async fn teensy_communication(cfg: &config::Config, tx: EventShare, rx: TeensyOut) {
  let teensy = match hidapi::HidApi::new() {
    Ok(api) => match api.open(cfg.teensy.vid, cfg.teensy.pid) {
      Ok(s) => s,
      Err(e) => {
        eprintln!("Failed to open HID device for Teensy: {}", e);
        return;
      }
    },
    Err(e) => {
      eprintln!("Failed to initialize HID API: {}", e);
      return;
    }
  };

  if let Err(e) = teensy.set_blocking_mode(false) {
    eprintln!("Failed to set Teensy HID device to nonblocking mode: {}", e);
    return;
  }

  tokio::spawn(async move {
    let mut buf = [0u8; 512];
    let mut last_seq: u64 = 0;

    loop {
      loop {
        match teensy.read(&mut buf) {
          Ok(size) if size >= 8 => {
            let msg = TeensyRecMSG {
              has_ball: buf[0] != 0,
              kick_ready: buf[1] != 0,
              batt_volt: f32::from_le_bytes(buf[2..6].try_into().unwrap_or([0; 4])),
              orientation: u16::from_le_bytes([buf[4], buf[5]]),
            };

            let mut lock = tx.lock().await;
            lock.2 = Some(msg);
          }
          Ok(_) => break,
          Err(e) => {
            let msg = e.to_string();
            if msg.contains("Would block") || msg.contains("would block") || msg.contains("Resource temporarily unavailable") {
              break;
            }

            eprintln!("Failed to read from Teensy HID device: {}", e);
            break;
          }
        }
      }

      if let Some((seq, payload)) = rx.try_latest_after(last_seq).await {
        last_seq = seq;

        if let Err(e) = teensy.write(&payload) {
          eprintln!("Failed to write to Teensy HID device: {}", e);
        }
      } else {
        tokio::task::yield_now().await;
      }
    }
  });
}
