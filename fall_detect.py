
import cv2
import time
import yaml
import os
import numpy as np
from imutils.object_detection import non_max_suppression
from notifier import Notifier

# Optional GPIO cancel button
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False

def load_config(path="config.yaml"):
    if not os.path.exists(path):
        path = "config.example.yaml"
    with open(path, "r") as f:
        return yaml.safe_load(f)

class GraceCancel:
    """Handle cancel countdown via keyboard 'c' or GPIO button."""
    def __init__(self, seconds, use_gpio=False, pin=17):
        self.seconds = int(seconds)
        self.canceled = False
        self.use_gpio = use_gpio and HAS_GPIO
        self.pin = pin
        if self.use_gpio:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def check_gpio(self):
        if not self.use_gpio:
            return False
        return GPIO.input(self.pin) == 0

    def run(self, frame=None, window_name="FallDetector"):
        start = time.time()
        while time.time() - start < self.seconds:
            remaining = self.seconds - int(time.time() - start)
            if cv2.waitKey(1) & 0xFF in (ord('c'), ord('C')):
                self.canceled = True
                break
            if self.check_gpio():
                self.canceled = True
                break
            if frame is not None and window_name:
                tmp = frame.copy()
                cv2.putText(tmp, f"ALERT in {remaining}s - press 'C' or button to cancel",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                cv2.imshow(window_name, tmp)
                cv2.waitKey(1)
            time.sleep(0.05)
        return not self.canceled

class FallHeuristic:
    """Heuristic based on aspect ratio, floor region, vertical drop, and low motion."""
    def __init__(self, cfg, fps=10):
        hcfg = cfg["heuristics"]
        self.fall_aspect_ratio = float(hcfg.get("fall_aspect_ratio", 1.2))
        self.floor_region_ratio = float(hcfg.get("floor_region_ratio", 0.2))
        self.motion_window_secs = float(hcfg.get("motion_window_secs", 2.0))
        self.vertical_drop_thresh = float(hcfg.get("vertical_drop_thresh", 0.2))
        self.min_person_area = int(hcfg.get("min_person_area", 1200))
        self.fps = fps
        self.traj = []  # (t, cx, cy, w, h)
        self.window = int(max(3, self.motion_window_secs * self.fps))

    def update_and_check(self, bbox, frame_shape):
        H, W = frame_shape[:2]
        x, y, w, h = bbox
        area = w * h
        if area < self.min_person_area:
            return False

        cx = x + w/2
        cy = y + h/2
        ar = w / float(h + 1e-6)

        now = time.time()
        self.traj.append((now, cx, cy, w, h))
        if len(self.traj) > self.window * 3:
            self.traj = self.traj[-self.window*3:]

        if len(self.traj) < self.window:
            return False
        recent = self.traj[-self.window:]

        # Find past point ~1s ago
        one_sec_ago = now - 1.0
        past = None
        for t, pcx, pcy, pw, ph in reversed(self.traj):
            if t <= one_sec_ago:
                past = (pcx, pcy, pw, ph)
                break
        if past is None:
            return False
        pcy = past[1]

        drop = (cy - pcy) / float(H)  # downwards positive
        near_floor = (y + h) >= (H * (1.0 - self.floor_region_ratio))

        # Low motion in recent window
        recent_cx = np.array([r[1] for r in recent])
        recent_cy = np.array([r[2] for r in recent])
        motion_mag = (np.std(recent_cx) + np.std(recent_cy)) / max(W, H)

        cond_drop = drop > self.vertical_drop_thresh
        cond_flat = ar > self.fall_aspect_ratio
        cond_floor = near_floor
        cond_low_motion = motion_mag < 0.01

        return cond_drop and cond_flat and cond_floor and cond_low_motion

def get_person_boxes_hog(gray):
    """HOG-based person detection; returns list of (x,y,w,h)."""
    if not hasattr(get_person_boxes_hog, "hog"):
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        get_person_boxes_hog.hog = hog

    hog = get_person_boxes_hog.hog
    rects, weights = hog.detectMultiScale(gray, winStride=(8,8), padding=(8,8), scale=1.05)
    rects = np.array([[x, y, x+w, y+h] for (x,y,w,h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
    boxes = []
    for (xA, yA, xB, yB) in pick:
        boxes.append((int(xA), int(yA), int(xB-xA), int(yB-yA)))
    return boxes

def main():
    cfg = load_config()
    cam_idx = int(cfg["camera"]["index"])
    W = int(cfg["camera"]["frame_width"])
    H = int(cfg["camera"]["frame_height"])
    show = bool(cfg["camera"].get("display_window", True))

    cap = cv2.VideoCapture(cam_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, 10)
    fps = cap.get(cv2.CAP_PROP_FPS) or 10

    notifier = Notifier(cfg)
    heuristic = FallHeuristic(cfg, fps=fps)

    use_gpio = cfg.get("gpio", {}).get("use_gpio", False)
    cancel_pin = cfg.get("gpio", {}).get("cancel_button_pin", 17)

    name = cfg.get("person", {}).get("name", "Resident")
    location = cfg.get("person", {}).get("location", "Home")
    cancel_secs = int(cfg.get("alerts", {}).get("cancel_seconds", 10))
    speak_template = cfg.get("alerts", {}).get("speak_template",
        "Emergency. Possible fall detected for {name}. Location: {location}.")

    print("[pi-falld] Starting capture... Press 'q' to quit, 'c' to cancel during countdown.")

    last_alert_time = 0
    cooldown = 20  # seconds between alerts

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue

        small = cv2.resize(frame, (W, H))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        boxes = get_person_boxes_hog(gray)

        fall_flag = False
        best_box = None
        if boxes:
            areas = [w*h for (x,y,w,h) in boxes]
            best_box = boxes[int(np.argmax(areas))]
            fall_flag = heuristic.update_and_check(best_box, small.shape)

        if show:
            vis = small.copy()
            for (x,y,w,h) in boxes:
                color = (0,255,0)
                if (x,y,w,h) == best_box and fall_flag:
                    color = (0,0,255)
                cv2.rectangle(vis, (x,y), (x+w, y+h), color, 2)
            cv2.putText(vis, "Q: quit | C: cancel during countdown", (10, H-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv2.imshow("FallDetector", vis)
            if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
                break

        now = time.time()
        if fall_flag and (now - last_alert_time > cooldown):
            print("[pi-falld] Possible fall detected. Starting cancel timer...")
            gc = GraceCancel(cancel_secs, use_gpio=use_gpio, pin=cancel_pin)
            proceed = gc.run(frame=small, window_name="FallDetector" if show else "")
            if not proceed:
                print("[pi-falld] Alert canceled.")
                continue

            last_alert_time = time.time()
            speak_text = speak_template.format(name=name, location=location)
            sms_text = f"[Pi Fall Detector] Possible fall for {name} at {location}."
            notifier.alert_all(speak_text, sms_text)

    cap.release()
    if show:
        cv2.destroyAllWindows()
    if HAS_GPIO:
        try:
            GPIO.cleanup()
        except Exception:
            pass

if __name__ == "__main__":
    main()
