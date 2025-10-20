
# Pi Fall Detector (Offline-first)

CareLens is an offline-first fall detection demo for Raspberry Pi using OpenCV. It detects possible falls from a camera feed with lightweight heuristics, starts a 10-second cancel timer (to reduce false alarms), and if not canceled it instantly triggers an alert to caregivers in real-time. It eliminates the need for wearables.

- **Local alert (always works offline):** plays a siren and speaks a message via the Pi speaker.
- **Optional SMS/phone call (needs internet):** uses Twilio if credentials are provided.

## Features
- Runs fully on a Raspberry Pi with camera (USB or CSI).
- No cloud needed for detection.
- 5–10s cancel grace period (configurable).
- Optional GPIO button to cancel (pin configurable).
- Event logs in `events.log`.

## Quick Start
```bash
# 1) Install dependencies
sudo apt-get update
sudo apt-get install -y python3-opencv python3-pip espeak ffmpeg
pip3 install -r requirements.txt

# 2) Edit config
cp config.example.yaml config.yaml
nano config.yaml   # set phone numbers, durations, GPIO, etc.

# 3) Run
python3 fall_detect.py

# (Optional) Run as a service (after testing)
sudo cp pi-falld.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable pi-falld
sudo systemctl start pi-falld
```

## Hardware
- Raspberry Pi 4 (recommended) or 3B+.
- Camera (USB webcam is fine). A wide-angle 720p works well.
- (Optional) Speaker for alerts (Pi audio jack/HDMI or USB audio).
- (Optional) Momentary push button for cancel (see `gpio.cancel_button_pin`).

## How detection works (simple, hackathon-friendly)
- Detect a person using OpenCV's HOG person detector on downscaled frames (e.g., 320x240).
- Track the primary subject with a centroid tracker.
- **Fall heuristic:** A rapid change in vertical position or box size followed by a "lying" aspect ratio (width/height > threshold) near the floor area and **low motion** for N seconds.
- Start a cancel timer. If not canceled, trigger notifiers.

> This is a demo heuristic; it's not a medical device. For production, consider pose estimation (e.g., lightweight tflite models) and multi-sensor fusion (IMU/ultrasonic).

## Configuration
See `config.example.yaml`. Important knobs:
- `camera.index`: which camera to open (0 for default).
- `detect.frame_width` / `detect.frame_height`: lower = faster.
- `heuristics.*`: thresholds for aspect ratio, motion window, and “near floor”.
- `alerts.cancel_seconds`: countdown length.
- `notifiers.*`: enable/disable local/Twilio.
- `gpio.use_gpio`: enable if you wired a cancel button.

## Twilio (optional)
Set environment variables (recommended):
```
export TWILIO_ACCOUNT_SID=ACxxxxxxxx
export TWILIO_AUTH_TOKEN=xxxxxxxx
export TWILIO_FROM=+1xxxxxxxxxx
export ALERT_SMS_TO=+1yyyyyyyyyy
export ALERT_CALL_TO=+1zzzzzzzzzz
```
Or fill them in `config.yaml` (less secure).

## Demo tips
- Induce "fall-like" behavior by squatting then quickly lying sideways within camera view.
- Show before/after on the UI window; press `C` to cancel during the countdown.
- Kill with `Q` in the window, or `Ctrl+C` in the terminal.

## Disclaimer
This repository is for hackathon demo purposes only. It is **not** a certified medical or safety system.
