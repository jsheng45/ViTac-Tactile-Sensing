import numpy as np
import serial
import threading
import cv2
import time

# ----------------------------
# Sensor frame shape (from Arduino)
# ----------------------------
# Arduino streams ONE full tactile frame per cycle:
#   - 12 rows
#   - 32 taxels per row
# Each line on the serial port contains 32 integers.
# A complete frame = 12 consecutive valid lines.
ROWS = 12
COLUMNS = 32

# ----------------------------
# Signal processing parameters
# ----------------------------
PORT = "/dev/ttyUSB1"
BAUD = 2_000_000
SER_TIMEOUT = 0.5
THRESHOLD = 2       
NOISE_SCALE = 60
INIT_FRAMES = 30
SCALE = 30
ALPHA = 0.2        

# shared
contact_data_norm = np.zeros((ROWS, COLUMNS), dtype=np.float32)
flag = False
data_lock = threading.Lock()

# window
cv2.namedWindow("Contact Data_right", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Contact Data_right", COLUMNS * SCALE, ROWS * SCALE)


def temporal_filter(new_frame, prev_frame, alpha=ALPHA):
    return alpha * new_frame + (1 - alpha) * prev_frame

def parse_row(line: str):
    # Parse a single serial line.
    # Only accepts lines that contain exactly 32 integers (one complete tactile row)

    if not line:
        return None
    parts = line.split()
    if len(parts) != COLUMNS:   # 32
        return None
    try:
        return [int(v) for v in parts]
    except ValueError:
        return None

def read_one_frame(serDev):
    # Read one complete tactile frame from the serial stream.
    # A frame consists of 12 consecutive rows, each with 32 values.

    rows = []
    while len(rows) < ROWS:     # 12
        line = serDev.readline().decode("utf-8", errors="ignore").strip()
        row = parse_row(line)
        if row is None:
            continue
        rows.append(row)
    return np.array(rows, dtype=np.float32)  # (12,32)

def readThread(serDev):
    global contact_data_norm, flag

    # -------- init median baseline --------
    frames = []
    t_last = time.time()
    print("Initializing baseline... do NOT touch the sensor.")

    while len(frames) < INIT_FRAMES:
        frame = read_one_frame(serDev)
        frames.append(frame)

        dt = time.time() - t_last
        if dt > 0:
            print("init fps:", 1.0 / dt)
        t_last = time.time()

        if len(frames) % 5 == 0:
            print("init frames:", len(frames))

    median = np.median(np.stack(frames, axis=0), axis=0).astype(np.float32)
    flag = True
    print("Finish Initialization!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    # -------- main loop --------
    while True:
        frame = read_one_frame(serDev)

        # subtract baseline
        contact = frame - median - THRESHOLD
        contact = np.clip(contact, 0, 200)

        mx = float(np.max(contact))
        if mx < 1e-6:
            norm = np.zeros_like(contact, dtype=np.float32)
        elif mx < 10:
            norm = contact / NOISE_SCALE
        else:
            norm = contact / mx

        with data_lock:
            contact_data_norm[:] = norm


if __name__ == "__main__":
    print("receive data test")

    serDev = serial.Serial(PORT, BAUD, timeout=SER_TIMEOUT)
    serDev.reset_input_buffer()

    th = threading.Thread(target=readThread, args=(serDev,), daemon=True)
    th.start()

    prev_frame = np.zeros((ROWS, COLUMNS), dtype=np.float32)

    while True:
        if flag:
            with data_lock:
                frame = contact_data_norm.copy()
            frame = temporal_filter(frame, prev_frame, alpha=ALPHA)
            prev_frame = frame

            img8 = np.clip(frame * 255, 0, 255).astype(np.uint8)
            colormap = cv2.applyColorMap(img8, cv2.COLORMAP_VIRIDIS)
            cv2.imshow("Contact Data_right", colormap)
        else:
            # keep GUI responsive during init
            blank = np.zeros((ROWS, COLUMNS), dtype=np.uint8)
            cv2.imshow("Contact Data_right", cv2.applyColorMap(blank, cv2.COLORMAP_VIRIDIS))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    try:
        serDev.close()
    except Exception:
        pass
