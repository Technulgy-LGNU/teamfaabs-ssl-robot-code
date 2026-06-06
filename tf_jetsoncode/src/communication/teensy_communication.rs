use crate::communication::{EventShare, TeensyOut, TeensyRecMSG};
use crate::{TEENSY_RECEIVE_MSG_SIZE, TEENSY_SEND_MSG_SIZE, config};
use std::time::Duration;
use tokio::time::sleep;
use tracing::error;

/// Robust Teensy communication task that will try to reconnect when the device
/// disappears (for example during firmware upload). Uses exponential backoff
/// (starting at 100ms, doubling up to 5000ms).
pub async fn teensy_communication(cfg: &config::Config, tx: EventShare, rx: TeensyOut) {
  // Extract VID/PID before spawning (they're all we need from config).
  let vid = cfg.teensy.vid;
  let pid = cfg.teensy.pid;

  // Spawn a long-running tokio task so the caller doesn't block.
  tokio::spawn(async move {
    // Backoff parameters (tunable)
    let mut backoff_ms: u64 = 100;
    const MAX_BACKOFF_MS: u64 = 5_000;

    loop {
      // Try to initialize HID API and open the device.
      let device_res = match hidapi::HidApi::new() {
        Ok(api) => match api.open(vid, pid) {
          Ok(dev) => Ok(dev),
          Err(e) => Err(format!("Failed to open HID device for Teensy: {}", e)),
        },
        Err(e) => Err(format!("Failed to initialize HID API: {}", e)),
      };

      let teensy = match device_res {
        Ok(dev) => {
          // Reset backoff on success
          backoff_ms = 100;
          dev
        }
        Err(err_msg) => {
          error!("{}", err_msg);
          // Wait with backoff before retrying
          sleep(Duration::from_millis(backoff_ms)).await;
          backoff_ms = (backoff_ms * 2).min(MAX_BACKOFF_MS);
          continue;
        }
      };

      // Try to set non-blocking mode; if it fails, close and retry.
      if let Err(e) = teensy.set_blocking_mode(false) {
        error!("Failed to set Teensy HID device to nonblocking mode: {}", e);
        // Drop device and retry after backoff
        sleep(Duration::from_millis(backoff_ms)).await;
        backoff_ms = (backoff_ms * 2).min(MAX_BACKOFF_MS);
        continue;
      }

      eprintln!(
        "Teensy HID device connected (vid=0x{:04x}, pid=0x{:04x})",
        vid, pid
      );

      // Run the read/write loop until we detect a fatal error (device removed).
      let mut buf = [0u8; 64];
      let mut last_seq: u64 = 0;

      'device_loop: loop {
        // Read as many incoming packets as available.
        loop {
          match teensy.read(&mut buf) {
            Ok(size) if size >= TEENSY_RECEIVE_MSG_SIZE => {
              let msg = TeensyRecMSG {
                flags: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
                batt_level: buf[4],
                current: u16::from_le_bytes([buf[5], buf[6]]),
              };

              let mut lock = tx.lock().await;
              lock.teensy = Some(msg);
            }
            Ok(_) => {
              // Not enough bytes - exit inner read loop and continue to write handling
              break;
            }
            Err(e) => {
              let msg = e.to_string();
              // These are expected when non-blocking and there's nothing to read.
              if msg.contains("Would block")
                || msg.contains("would block")
                || msg.contains("Resource temporarily unavailable")
              {
                break;
              }

              // Any other read error likely indicates disconnection - log and break to reconnect.
              error!("Failed to read from Teensy HID device: {}", e);
              break 'device_loop;
            }
          }
        }

        // Write latest outgoing packet if available.
        if let Some((seq, payload)) = rx.try_latest_after(last_seq).await {
          last_seq = seq;

          // HID report 0 first byte reserved for report-id in many platforms.
          let mut packet = [0u8; 65];
          packet[0] = 0;
          packet[1..(TEENSY_SEND_MSG_SIZE + 1)].copy_from_slice(&payload);

          if let Err(e) = teensy.write(&packet) {
            eprintln!("Failed to write to Teensy HID device: {}", e);
            // Treat write errors as disconnection and attempt reconnect.
            break 'device_loop;
          }
        } else {
          // No outbound data; yield to avoid busy-looping.
          tokio::task::yield_now().await;
        }
      } // end 'device_loop

      // If we've reached here, the device was dropped or had a fatal error.
      error!(
        "Teensy HID device disconnected, will attempt to reconnect in {} ms",
        backoff_ms
      );

      // Drop the device handle explicitly (it will happen when goes out of scope).
      // Wait backoff and then retry.
      sleep(Duration::from_millis(backoff_ms)).await;
      backoff_ms = (backoff_ms * 2).min(MAX_BACKOFF_MS);
    } // end outer loop (reconnect attempts)
  });
}
