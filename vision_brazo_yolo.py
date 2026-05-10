"""
PROJECT: Autonomous Robotic Arm - Advanced Vision Control (ToF + BBox Fusion)
SYSTEM: Linux | OpenCV + PySerial + Ultralytics + Teleplot
HARDWARE: 5 Servomotors + VL53L1X ToF Sensor + ESP32 Camera + Arduino Uno
AUTHOR: Cristian Montoya
"""

import cv2
import serial
import numpy as np
import time
import threading
import csv
import os
import socket
from datetime import datetime
from ultralytics import YOLO

# UDP Socket for Teleplot Telemetry
teleplot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ==========================================
# 1. COMMUNICATIONS SETUP
# ==========================================
CAMERA_PORT = '/dev/ttyUSB0' 
LASER_PORT  = '/dev/ttyUSB1'
ARDUINO_PORT = '/dev/ttyACM0'

try:
    # ESP32 Camera Stream
    camera = serial.Serial(CAMERA_PORT, 115200, timeout=1)
    camera.reset_input_buffer()
    print(f"[+] ESP32 Camera connected on {CAMERA_PORT}")
except Exception as e:
    print(f"[-] Camera Error ({CAMERA_PORT}): {e}"); exit()

try:
    # Arduino Uno (Servo Controller)
    arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=1)
    time.sleep(2)
    print(f"[+] Arduino Uno connected on {ARDUINO_PORT}")
except Exception as e:
    print(f"[-] Arduino Error ({ARDUINO_PORT}): {e}"); exit()

try:
    # ToF Laser Sensor
    tof_sensor = serial.Serial(LASER_PORT, 115200, timeout=0.1)
    print(f"[+] ToF Laser Sensor connected on {LASER_PORT}")
except Exception as e:
    print(f"[-] Sensor Error ({LASER_PORT}): {e}"); exit()

# ==========================================
# 2. YOLO MODEL INITIALIZATION
# ==========================================
print("[*] Loading YOLOv8n model...")
model = YOLO("yolov8n.pt")
print("[+] Model ready.\n")

# ==========================================
# 3. GLOBAL CONFIGURATION
# ==========================================
TARGET_CLASSES = [
    "bottle", "cup", "bowl", "apple", "orange", "banana",
    "cell phone", "remote", "mouse", "book", "scissors",
    "clock", "vase", "keyboard", "laptop", "spoon",
    "fork", "knife", "toothbrush", "backpack", "handbag"
]
MIN_CONFIDENCE      = 0.35
GRAB_BBOX_THRESHOLD = 250    # px — minimum bbox width to trigger grab
GRAB_TOF_THRESHOLD  = 14.0   # cm — maximum ToF distance to trigger grab
MAX_LOST_FRAMES     = 5      # Frames before resetting RADAR (~0.5 seconds)

# ==========================================
# 4. MECHANICAL LIMITS
# ==========================================
LIMITS = {
    "base_min":   0,    "base_max":   180,
    "shoulder_min": 85, "shoulder_max": 175,
    "elbow_min":   50,  "elbow_max":   140,
    "wrist_min": 0,     "wrist_max":   180,
    "gripper_min":  45, "gripper_max":  120,
}

# ==========================================
# 5. DATA LOGGING (CSV)
# ==========================================
CSV_FILE = "robotic_arm_telemetry.csv"

def init_csv():
    exists = os.path.exists(CSV_FILE)
    with open(CSV_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        if not exists:
            writer.writerow([
                "timestamp", "event",
                "base", "shoulder", "elbow", "wrist", "gripper",
                "tof_cm", "error_x", "error_y", "bbox_width",
                "detected_object", "target_object"
            ])
    print(f"[+] Logging telemetry to: {os.path.abspath(CSV_FILE)}")

def log_to_csv(event, tof="", ex="", ey="", bbox="", det_obj="", tar_obj=""):
    with open(CSV_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            event,
            base_angle, shoulder_angle, elbow_angle, wrist_angle, gripper_angle,
            tof, ex, ey, bbox,
            det_obj, tar_obj or ""
        ])

init_csv()

# ==========================================
# 6. STATE VARIABLES
# ==========================================
cam_buffer         = b""
serial_ack_buffer  = ""

base_angle      = 90
shoulder_angle  = 100
elbow_angle     = 85
wrist_angle     = 90
gripper_angle   = 120

current_state     = "WAITING"
search_direction  = 1
user_target       = None
scene_objects     = {}
identified_tags   = set()
lost_frames_count = 0
tof_distance_cm   = -1.0

# PID Control Memory (Error Tracking)
prev_error_x = 0
prev_error_y = 0
integral_x   = 0
integral_y   = 0

# ==========================================
# 7. ASYNC INPUT & SENSOR READERS
# ==========================================
input_queue = []
input_lock = threading.Lock()

def terminal_input_thread():
    """Non-blocking thread for user commands."""
    while True:
        text = input()
        with input_lock:
            input_queue.append(text.strip().lower())

threading.Thread(target=terminal_input_thread, daemon=True).start()

def tof_reader_thread():
    """High-frequency thread for Laser sensor distance sampling."""
    global tof_distance_cm
    while True:
        try:
            if tof_sensor.in_waiting > 0:
                line = tof_sensor.readline().decode('utf-8', errors='ignore').strip()
                if "Distancia:" in line:
                    value = line.split(":")[1].replace("cm", "").strip()
                    tof_distance_cm = float(value)
        except Exception:
            pass
        time.sleep(0.01)

threading.Thread(target=tof_reader_thread, daemon=True).start()

# ==========================================
# 8. CORE CONTROL FUNCTIONS
# ==========================================
def process_arduino_ack():
    """Handle feedback from the Arduino servo controller."""
    global serial_ack_buffer
    try:
        while arduino.in_waiting > 0:
            byte = arduino.read(1).decode('utf-8', errors='ignore')
            if byte == '\n':
                line = serial_ack_buffer.strip()
                serial_ack_buffer = ""
                if line.startswith("OK:") or line.startswith("ERR:"):
                    print(f"  [Hardware] {line}")
            else:
                serial_ack_buffer += byte
    except Exception:
        pass

def send_coordinates(log=True, tof="", ex="", ey="", bbox=""):
    """Constraint check and Serial transmission to servos."""
    global base_angle, shoulder_angle, elbow_angle, wrist_angle, gripper_angle

    base_angle     = max(LIMITS["base_min"],     min(LIMITS["base_max"],     base_angle))
    shoulder_angle = max(LIMITS["shoulder_min"], min(LIMITS["shoulder_max"], shoulder_angle))
    elbow_angle    = max(LIMITS["elbow_min"],    min(LIMITS["elbow_max"],    elbow_angle))
    wrist_angle    = max(LIMITS["wrist_min"],    min(LIMITS["wrist_max"],    wrist_angle))
    gripper_angle  = max(LIMITS["gripper_min"],   min(LIMITS["gripper_max"],  gripper_angle))

    command = f"{base_angle},{shoulder_angle},{elbow_angle},{wrist_angle},{gripper_angle}\n"
    arduino.write(command.encode())
    arduino.flush()

    if log:
        log_to_csv("MOVEMENT", tof=tof, ex=ex, ey=ey, bbox=bbox, tar_obj=user_target or "")

def transmit_telemetry(ex, ey, tof, bbox):
    """Real-time data broadcast via UDP for Teleplot visualizations."""
    try:
        tof_str = f"{tof:.1f}" if tof >= 0 else "-1"
        msg = f">Error_X:{ex}\n>Error_Y:{ey}\n>ToF:{tof_str}\n>BBox:{bbox}"
        teleplot_sock.sendto(msg.encode(), ("127.0.0.1", 47269))
    except Exception:
        pass

def print_target_menu():
    global user_target, current_state, identified_tags, lost_frames_count
    user_target       = None
    identified_tags   = set()
    current_state     = "WAITING"
    lost_frames_count = 0

    print("\n" + "="*55)
    print("  AUTONOMOUS ROBOTIC ARM — TARGET SELECTION")
    print("="*55)
    print("  Available Objects:")
    line = "  "
    for i, c in enumerate(TARGET_CLASSES):
        line += f"{c}  "
        if (i+1) % 5 == 0:
            print(line); line = "  "
    if line.strip(): print(line)
    print("="*55)
    print("  Which object should I retrieve?")
    print("  Type the name and press Enter:\n")

def reset_arm():
    global base_angle, shoulder_angle, elbow_angle, wrist_angle, gripper_angle
    global current_state, search_direction, scene_objects
    global lost_frames_count, tof_distance_cm, identified_tags

    search_direction  = 1
    scene_objects     = {}
    identified_tags   = set()
    lost_frames_count = 0
    tof_distance_cm   = -1.0

    base_angle      = 90
    shoulder_angle  = 100
    elbow_angle     = 85
    wrist_angle     = 90
    gripper_angle   = 120

    send_coordinates(log=False)
    log_to_csv("SYSTEM_RESET")
    print_target_menu()

def perform_yolo_detection(frame):
    results = model(frame, verbose=False, conf=MIN_CONFIDENCE)[0]
    detections = {}
    for det in results.boxes:
        class_id = int(det.cls[0])
        label    = model.names[class_id]
        conf     = float(det.conf[0])
        if label not in TARGET_CLASSES:
            continue
        if label in detections and detections[label][2] >= conf:
            continue
        x1, y1, x2, y2 = map(int, det.xyxy[0])
        detections[label] = ((x1+x2)//2, (y1+y2)//2, conf, x1, y1, x2, y2)
    return detections

def draw_tof_indicator(frame, dist, height, width):
    if dist < 0: return
    MAX_DIST = 50.0
    bx, bh, by = width - 35, height - 80, 40
    cv2.rectangle(frame, (bx, by), (bx+20, by+bh), (50,50,50), -1)
    pct    = min(dist/MAX_DIST, 1.0)
    fill_h = int(bh * pct)
    color  = (0,200,0) if dist > 20 else (0,200,255) if dist > GRAB_TOF_THRESHOLD else (0,0,255)
    cv2.rectangle(frame, (bx, by+bh-fill_h), (bx+20, by+bh), color, -1)
    cv2.putText(frame, f"{dist:.1f}", (bx-12, by+bh+15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)

# ==========================================
# 9. INITIALIZATION
# ==========================================
reset_arm()
last_mov_time  = time.time()
last_yolo_time = time.time()

# ==========================================
# 10. MAIN CONTROL LOOP
# ==========================================
while True:
    process_arduino_ack()

    # User Input Processing
    with input_lock:
        if input_queue:
            text = input_queue.pop(0)
            if current_state == "WAITING":
                match = next((c for c in TARGET_CLASSES if text in c), None)
                if match:
                    user_target   = match
                    current_state = "RADAR_SEARCH"
                    log_to_csv("TARGET_REQUESTED", tar_obj=user_target)
                    print(f"\n[✓] Searching for: '{user_target}'")
                else:
                    print(f"[!] '{text}' not recognized.")
            elif text == 'r':
                reset_arm()

    # Camera Buffer Processing
    while camera.in_waiting > 0:
        cam_buffer += camera.read(camera.in_waiting)
        time.sleep(0.005)

    start_idx = cam_buffer.find(b"START")
    end_idx   = cam_buffer.find(b"END")

    if start_idx != -1 and end_idx != -1 and start_idx < end_idx:
        jpg_data   = cam_buffer[start_idx+5 : end_idx]
        cam_buffer = cam_buffer[end_idx+3:]

        if len(jpg_data) > 100:
            nparr = np.frombuffer(jpg_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                h, w = frame.shape[:2]
                cx_f, cy_f = w // 2, h // 2

                # YOLO Inference (300ms throttle)
                if time.time() - last_yolo_time > 0.3:
                    scene_objects  = perform_yolo_detection(frame)
                    last_yolo_time = time.time()

                    for label in scene_objects:
                        if label not in identified_tags:
                            identified_tags.add(label)
                            log_to_csv("OBJECT_IDENTIFIED", det_obj=label, tar_obj=user_target)

                    if current_state == "RADAR_SEARCH" and user_target in scene_objects:
                        current_state     = "TRACKING"
                        lost_frames_count = 0
                        print(f"\n[★] Target Found: '{user_target}'!")

                # STATE MACHINE (10Hz Control Loop)
                if time.time() - last_mov_time > 0.1:
                    
                    if current_state == "RADAR_SEARCH":
                        base_angle += search_direction * 2
                        if base_angle >= 160 or base_angle <= 20: search_direction *= -1
                        send_coordinates(log=False)

                    elif current_state == "TRACKING":
                        # Object Persistence Logic (Handling YOLO class ambiguity)
                        similar_classes = ["bottle", "cup", "vase", "bowl"]
                        valid_detection = next((lbl for lbl in similar_classes if lbl in scene_objects), None)

                        if valid_detection:
                            lost_frames_count = 0
                            cx, cy, conf, x1, y1, x2, y2 = scene_objects[valid_detection]
                            error_x, error_y, bbox_w = cx - cx_f, cy - cy_f, x2 - x1

                            # PID Control with Dynamic Damping
                            damping = 0.5 if bbox_w > 300 else 1.0
                            Kp, Kd, Ki = 0.02 * damping, 0.015 * damping, 0.001 * damping

                            # Horizontal Axis (Base)
                            integral_x += error_x
                            base_angle += int((error_x * Kp) + (integral_x * Ki) + ((error_x - prev_error_x) * Kd))
                            prev_error_x = error_x

                            # Vertical Axis (Shoulder)
                            integral_y += error_y
                            shoulder_angle += int((error_y * Kp) + (integral_y * Ki) + ((error_y - prev_error_y) * Kd))
                            prev_error_y = error_y

                            # Approach Logic
                            tolerance = 100 if bbox_w > 250 else 60
                            if abs(error_x) < tolerance and abs(error_y) < tolerance:
                                if not (0 < tof_distance_cm <= GRAB_TOF_THRESHOLD) or not (bbox_w >= GRAB_BBOX_THRESHOLD):
                                    elbow_angle += 3 if tof_distance_cm > 15 else 1
                                else:
                                    current_state = "GRABBING"

                            send_coordinates(tof=str(tof_distance_cm), ex=str(error_x), ey=str(error_y), bbox=str(bbox_w))
                            transmit_telemetry(error_x, error_y, tof_distance_cm, bbox_w)
                        else:
                            lost_frames_count += 1
                            if lost_frames_count > MAX_LOST_FRAMES:
                                current_state = "RADAR_SEARCH"

                    elif current_state == "GRABBING":
                        print("  [Action] Closing Gripper...")
                        gripper_angle = LIMITS["gripper_min"]
                        send_coordinates()
                        time.sleep(1.5)
                        
                        print("  [Action] Lifting Target...")
                        shoulder_angle -= 30
                        send_coordinates()
                        time.sleep(1.0)
                        
                        current_state = "FINISHED"
                        print(f"\n[✓] '{user_target}' successfully retrieved!")

                    last_mov_time = time.time()

                # UI OVERLAY (HUD)
                cv2.putText(frame, f"STATE: {current_state}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
                if user_target:
                    cv2.putText(frame, f"Target: {user_target}", (20, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                draw_tof_indicator(frame, tof_distance_cm, h, w)
                cv2.imshow("AR4 Vision System", frame)

    # Keyboard Controls
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): break
    elif key == ord('r'): reset_arm()

print("Shutting down system...")
camera.close()
arduino.close()
tof_sensor.close()
cv2.destroyAllWindows()
