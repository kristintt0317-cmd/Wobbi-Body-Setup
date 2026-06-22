import time
import cv2
import numpy as np
import serial
from ultralytics import YOLO
from pyorbbecsdk import *


# ==========================
# Arduino serial settings
# ==========================
ARDUINO_PORT = "COM5"
BAUDRATE = 9600


# ==========================
# RGB camera index
# If the wrong camera opens, try 1 or 2
# ==========================
RGB_CAMERA_INDEX = 0


# ==========================
# Distance logic
# ==========================
# >= 0.8m + person centred -> send r
# all other states -> send x
WALK_DISTANCE = 0.8


# ==========================
# Person detection settings
# ==========================
PERSON_CONFIDENCE = 0.5

# Larger value = easier to start walking
VERY_OFF_CENTER = 0.45


# ==========================
# ID9 pan servo settings
# 400 = robot right
# 500 = centre
# 600 = robot left
# ==========================
PAN_RIGHT = 400
PAN_CENTER = 500
PAN_LEFT = 600

PAN_MIN = 400
PAN_MAX = 600

pan_pos = PAN_CENTER
pending_pan_pos = PAN_CENTER
scan_direction = 1


# ==========================
# Command memory
# ==========================

# Movement command memory
# Only r / x will be sent during normal detection
last_move_command = None
last_move_send_time = 0

# Pan command memory
last_sent_pan_pos = None
last_pan_send_time = 0

# Terminal print memory
last_print_time = 0


# ==========================
# Frequency control
# ==========================

# ID9 Pxxx will be sent at most once every 10 seconds
PAN_SEND_INTERVAL =5.0

# If pan target changes less than 40, do not send Pxxx
PAN_MIN_DELTA = 40

# After sending r / x, wait before sending Pxxx
PAN_AFTER_MOVE_COOLDOWN = 5.0

# Terminal print interval
PRINT_INTERVAL = 1.0


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def send_raw_command(ser, command):
    """
    Send raw serial command to Arduino.
    """
    ser.write(command.encode("utf-8"))
    ser.flush()
    print(f"Sent to Arduino: {command.strip()}")


# =========================================================
# Movement command sender
# Only sends r / x during normal detection
# =========================================================
def send_move_command_if_changed(ser, cmd, force=False):
    """
    Send movement command only when the actual command changes.

    r = repeat walking loop
    x = stop legs, ID9 keeps current direction

    Example:
    last command = x, new command = x -> do not send
    last command = x, new command = r -> send r
    last command = r, new command = r -> do not send
    last command = r, new command = x -> send x
    """
    global last_move_command, last_move_send_time

    if cmd is None:
        return

    if cmd not in ["r", "x"]:
        print(f"Blocked invalid movement command: {cmd}")
        return

    if force or cmd != last_move_command:
        send_raw_command(ser, cmd + "\n")
        last_move_command = cmd
        last_move_send_time = time.time()


# =========================================================
# Pan target updater
# Only updates target. It does not send Pxxx directly.
# =========================================================
def set_pan_target(pos):
    """
    Update pan target only.
    This function does NOT send anything to Arduino.
    """
    global pan_pos, pending_pan_pos

    pan_pos = int(clamp(pos, PAN_MIN, PAN_MAX))
    pending_pan_pos = pan_pos


# =========================================================
# Pan command dispatcher
# The only function that sends Pxxx
# =========================================================
def dispatch_pan_command(ser, force=False):
    """
    Send Pxxx separately from movement commands.

    This function sends Pxxx at low frequency to avoid serial congestion.
    """
    global last_sent_pan_pos, last_pan_send_time, pending_pan_pos

    now = time.time()
    target = int(clamp(pending_pan_pos, PAN_MIN, PAN_MAX))

    if not force:
        # Do not send pan immediately after movement command
        if now - last_move_send_time < PAN_AFTER_MOVE_COOLDOWN:
            return

        # Do not send pan too frequently
        if now - last_pan_send_time < PAN_SEND_INTERVAL:
            return

        # Do not send if pan target change is too small
        if last_sent_pan_pos is not None:
            if abs(target - last_sent_pan_pos) < PAN_MIN_DELTA:
                return

    send_raw_command(ser, f"P{target}\n")
    last_sent_pan_pos = target
    last_pan_send_time = now


def scan_pan_target():
    """
    No person detected:
    update ID9 target for slow left-right scanning.
    Does not send Pxxx directly.
    """
    global pan_pos, scan_direction

    pan_pos += scan_direction * 4

    if pan_pos >= PAN_MAX:
        pan_pos = PAN_MAX
        scan_direction = -1

    elif pan_pos <= PAN_MIN:
        pan_pos = PAN_MIN
        scan_direction = 1

    set_pan_target(pan_pos)


def update_pan_target_towards_person(offset):
    """
    Person is off-centre:
    update pan target towards the person.
    Does not send Pxxx directly.

    offset negative = person on left
    offset positive = person on right

    Servo direction:
    600 = robot left
    400 = robot right
    """
    global pan_pos

    PAN_GAIN = 10

    pan_pos -= int(offset * PAN_GAIN)
    pan_pos = clamp(pan_pos, PAN_MIN, PAN_MAX)

    set_pan_target(pan_pos)


def distance_to_state(distance_m):
    """
    Convert depth distance into distance state.
    """
    if distance_m <= 0:
        return "NO_VALID_DEPTH"

    if distance_m < WALK_DISTANCE:
        return "CLOSE_DISTANCE"

    return "WALK_DISTANCE"


def get_behavior_state(distance_state, offset):
    """
    Convert distance state + image offset into behaviour state.
    """
    person_is_very_off_center = abs(offset) > VERY_OFF_CENTER

    if distance_state == "CLOSE_DISTANCE":
        return "CLOSE_DISTANCE"

    if distance_state == "WALK_DISTANCE":
        if person_is_very_off_center:
            return "WALK_OFF_CENTER"
        else:
            return "WALK_CENTERED"

    return "NO_VALID_DEPTH"


def behavior_state_to_command(behavior_state):
    """
    Convert behaviour state into movement command.

    Only two movement commands are used:

    WALK_CENTERED -> r
    all other states -> x
    """

    if behavior_state == "WALK_CENTERED":
        return "r"

    return "x"


def choose_depth_profile(pipeline):
    profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)

    candidates = [
        (640, 576, OBFormat.Y16, 15),
        (640, 480, OBFormat.Y16, 15),
        (640, 400, OBFormat.Y16, 15),
        (320, 240, OBFormat.Y16, 15),
    ]

    for width, height, fmt, fps in candidates:
        try:
            profile = profile_list.get_video_stream_profile(width, height, fmt, fps)
            print(f"Using depth profile: {width}x{height} @ {fps} FPS")
            return profile
        except Exception:
            pass

    print("Using default depth profile.")
    return profile_list.get_default_video_stream_profile()


def get_depth_image(depth_frame):
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    scale = depth_frame.get_depth_scale()

    raw_data = depth_frame.get_data()
    depth_data = np.frombuffer(raw_data, dtype=np.uint16)

    expected_size = width * height

    if depth_data.size != expected_size:
        return None, None, None, None

    depth_data = depth_data.reshape((height, width))

    return depth_data, width, height, scale


def get_valid_depth_near_point(depth_data, x, y, scale, window_size=9):
    """
    Read depth around the target point and return median valid depth.
    This is more stable than reading only one pixel.
    """
    h, w = depth_data.shape
    half = window_size // 2

    x1 = int(clamp(x - half, 0, w - 1))
    x2 = int(clamp(x + half, 0, w - 1))
    y1 = int(clamp(y - half, 0, h - 1))
    y2 = int(clamp(y + half, 0, h - 1))

    patch = depth_data[y1:y2 + 1, x1:x2 + 1]
    valid = patch[patch > 0]

    if valid.size == 0:
        return 0.0

    depth_mm = np.median(valid) * scale
    depth_m = depth_mm / 1000.0

    return float(depth_m)


def print_status_slowly(person_depth_m, distance_state, behavior_state, offset, pan_pos, move_cmd):
    """
    Print status once every PRINT_INTERVAL seconds.
    """
    global last_print_time

    now = time.time()

    if now - last_print_time > PRINT_INTERVAL:
        print(
            f"Person: {person_depth_m:.2f} m | "
            f"Distance: {distance_state} | "
            f"Behaviour: {behavior_state} | "
            f"Offset: {offset:.2f} | "
            f"Pan target: {pan_pos} | "
            f"Move cmd: {move_cmd}"
        )
        last_print_time = now


def main():
    global pan_pos

    # --------------------------
    # Connect Arduino
    # --------------------------
    print("Connecting to Arduino...")
    ser = serial.Serial(ARDUINO_PORT, BAUDRATE, timeout=1)

    # Arduino UNO resets when serial opens
    time.sleep(2.5)

    print("Arduino connected.")

    # Initial movement command: stop legs
    send_move_command_if_changed(ser, "x", force=True)

    # Give movement command a moment before sending pan
    time.sleep(0.5)

    # Initial pan centre
    set_pan_target(PAN_CENTER)
    dispatch_pan_command(ser, force=True)

    time.sleep(0.5)

    # --------------------------
    # Load YOLO
    # --------------------------
    print("Loading YOLO model...")
    model = YOLO("yolov8n.pt")
    print("YOLO loaded.")

    # --------------------------
    # Start Orbbec depth pipeline
    # --------------------------
    pipeline = Pipeline()
    config = Config()

    depth_profile = choose_depth_profile(pipeline)
    config.enable_stream(depth_profile)

    pipeline.start(config)
    print("Orbbec depth camera started.")

    # Warm up depth camera
    for _ in range(10):
        pipeline.wait_for_frames(100)

    # --------------------------
    # Open RGB camera
    # --------------------------
    cap = cv2.VideoCapture(RGB_CAMERA_INDEX)

    if not cap.isOpened():
        print("Could not open RGB camera. Try changing RGB_CAMERA_INDEX to 1 or 2.")
        pipeline.stop()
        ser.close()
        return

    print("Wobbi person depth pan follow started.")
    print("Press 'q' to quit.")

    try:
        while True:
            ret, color_frame = cap.read()

            if not ret:
                print("No RGB frame.")
                continue

            color_h, color_w = color_frame.shape[:2]

            frames = pipeline.wait_for_frames(100)

            if frames is None:
                continue

            depth_frame = frames.get_depth_frame()

            if depth_frame is None:
                continue

            depth_data, depth_w, depth_h, depth_scale = get_depth_image(depth_frame)

            if depth_data is None:
                print("Skipping invalid depth frame.")
                continue

            # --------------------------
            # YOLO person detection
            # --------------------------
            results = model(color_frame, verbose=False)

            best_person = None
            best_score = 0.0

            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])

                    # YOLO class 0 = person
                    if cls_id == 0 and conf > PERSON_CONFIDENCE:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                        area = max(1, (x2 - x1) * (y2 - y1))
                        score = conf * area

                        if score > best_score:
                            best_score = score
                            best_person = box

            # --------------------------
            # No person detected
            # --------------------------
            if best_person is None:
                behavior_state = "NO_PERSON"
                move_cmd = behavior_state_to_command(behavior_state)

                # Movement command: only r / x
                send_move_command_if_changed(ser, move_cmd)

                # Update pan target only
                scan_pan_target()

                # Pan dispatch is separate and low-frequency
                dispatch_pan_command(ser)

                cv2.putText(
                    color_frame,
                    "No person | move x | pan scan separate",
                    (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )

                cv2.imshow("Wobbi Person Depth Pan Follow", color_frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                continue

            # --------------------------
            # Person detected
            # --------------------------
            x1, y1, x2, y2 = best_person.xyxy[0].cpu().numpy().astype(int)

            cx_color = (x1 + x2) // 2
            cy_color = (y1 + y2) // 2

            offset = (cx_color - color_w / 2) / (color_w / 2)

            # Map RGB coordinate to depth coordinate by simple scaling
            cx_depth = int(cx_color * depth_w / color_w)
            cy_depth = int(cy_color * depth_h / color_h)

            cx_depth = int(clamp(cx_depth, 0, depth_w - 1))
            cy_depth = int(clamp(cy_depth, 0, depth_h - 1))

            person_depth_m = get_valid_depth_near_point(
                depth_data,
                cx_depth,
                cy_depth,
                depth_scale,
                window_size=9
            )

            distance_state = distance_to_state(person_depth_m)
            behavior_state = get_behavior_state(distance_state, offset)
            move_cmd = behavior_state_to_command(behavior_state)

            # --------------------------
            # Movement command
            # Only sends r or x, and only if changed
            # --------------------------
            send_move_command_if_changed(ser, move_cmd)

            # --------------------------
            # Pan target update
            # Does NOT send Pxxx directly
            # --------------------------
            if behavior_state == "WALK_OFF_CENTER":
                update_pan_target_towards_person(offset)

            elif behavior_state == "CLOSE_DISTANCE" and abs(offset) > VERY_OFF_CENTER:
                update_pan_target_towards_person(offset)

            elif behavior_state == "NO_VALID_DEPTH" and abs(offset) > VERY_OFF_CENTER:
                update_pan_target_towards_person(offset)

            # --------------------------
            # Pan command dispatcher
            # Separate from movement command
            # --------------------------
            dispatch_pan_command(ser)

            print_status_slowly(
                person_depth_m,
                distance_state,
                behavior_state,
                offset,
                pan_pos,
                move_cmd
            )

            # --------------------------
            # Draw UI
            # --------------------------
            cv2.rectangle(color_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(color_frame, (cx_color, cy_color), 6, (0, 0, 255), -1)

            if behavior_state == "CLOSE_DISTANCE":
                status_text = "CLOSE < 0.8m | MOVE X"

            elif behavior_state == "WALK_OFF_CENTER":
                status_text = ">= 0.8m OFF-CENTRE | MOVE X | PAN TARGET UPDATE"

            elif behavior_state == "WALK_CENTERED":
                status_text = ">= 0.8m CENTRED | MOVE R"

            elif behavior_state == "NO_VALID_DEPTH":
                status_text = "NO VALID DEPTH | MOVE X"

            else:
                status_text = "UNKNOWN | MOVE X"

            label = (
                f"{person_depth_m:.2f}m | "
                f"{behavior_state} | "
                f"offset {offset:.2f} | "
                f"pan target {pan_pos} | "
                f"move {move_cmd}"
            )

            cv2.putText(
                color_frame,
                label,
                (x1, max(30, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2
            )

            cv2.putText(
                color_frame,
                status_text,
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )

            cv2.imshow("Wobbi Person Depth Pan Follow", color_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Keyboard interrupted.")

    finally:
        print("Stopping Wobbi...")
        try:
            # Stop legs first
            send_raw_command(ser, "x\n")
            time.sleep(0.5)

            # Then centre ID9 separately
            send_raw_command(ser, f"P{PAN_CENTER}\n")
            time.sleep(0.5)
        except Exception:
            pass

        cap.release()
        pipeline.stop()
        cv2.destroyAllWindows()
        ser.close()

        print("Camera stopped. Arduino serial closed.")


if __name__ == "__main__":
    main()