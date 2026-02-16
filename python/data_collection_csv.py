import os
import time
import threading
import numpy as np
import serial
import cv2

# ============================================================
# Saved CSV format per session folder:
#   raw.csv      : (N, 384) float values, each row is one 12x32 frame flattened
#   gray.csv     : (N, 384) uint8 values (0..255), each row is one frame flattened
#   t_ns.csv     : (N, 1) int64 timestamps (ns)
#   frame_id.csv : (N, 1) int32 frame indices
# ============================================================

ROWS = 12
COLUMNS = 32
FLAT_DIM = ROWS * COLUMNS  # 384

PORT = "/dev/ttyUSB0"
BAUD = 2_000_000
SER_TIMEOUT = 0.5

THRESHOLD = 2
NOISE_SCALE = 60
INIT_FRAMES = 30
SCALE = 30
ALPHA = 0.2

# ----------------------------
# Fixed logging rate (100 Hz)
# ----------------------------
TARGET_HZ = 100.0
DT = 1.0 / TARGET_HZ

# In-memory buffers (short episodes)
STORE_DTYPE = np.float16

latest_frame_norm = np.zeros((ROWS, COLUMNS), dtype=np.float32)
flag_init_done = False
data_lock = threading.Lock()

cv2.namedWindow("Contact Data_left", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Contact Data_left", COLUMNS * SCALE, ROWS * SCALE)


def temporal_filter(new_frame, prev_frame, alpha=ALPHA):
    return alpha * new_frame + (1 - alpha) * prev_frame


def parse_row(line: str):
    """Parse one serial line into a row of length COLUMNS."""
    if not line:
        return None
    parts = line.split()
    if len(parts) != COLUMNS:
        return None
    try:
        return [int(v) for v in parts]
    except ValueError:
        return None


def read_one_frame(serDev):
    """Read one (ROWS, COLUMNS) frame from serial."""
    rows = []
    while len(rows) < ROWS:
        line = serDev.readline().decode("utf-8", errors="ignore").strip()
        row = parse_row(line)
        if row is None:
            continue
        rows.append(row)
    return np.array(rows, dtype=np.float32)


def readThread(serDev):
    global latest_frame_norm, flag_init_done

    # -------- baseline init --------
    frames = []
    print("Initializing baseline... do NOT touch the sensor.")
    while len(frames) < INIT_FRAMES:
        frames.append(read_one_frame(serDev))

    median = np.median(np.stack(frames, axis=0), axis=0).astype(np.float32)
    flag_init_done = True
    print("Finish Initialization!")

    # -------- main read loop --------
    while True:
        frame = read_one_frame(serDev)

        contact = frame - median - THRESHOLD
        contact = np.clip(contact, 0, 200)

        mx = float(np.max(contact))
        if mx < 1e-6:
            norm = np.zeros_like(contact)
        elif mx < 10:
            norm = contact / NOISE_SCALE
        else:
            norm = contact / mx

        with data_lock:
            latest_frame_norm[:] = norm


def save_episode_csv(session_dir, buf_t, buf_id, buf_raw, buf_gray):
    """Save buffers into 4 CSV files."""
    os.makedirs(session_dir, exist_ok=True)

    # 1) t_ns.csv and frame_id.csv as (N,1)
    t_arr = np.array(buf_t, dtype=np.int64).reshape(-1, 1)
    id_arr = np.array(buf_id, dtype=np.int32).reshape(-1, 1)

    # 2) raw.csv and gray.csv as (N,384)
    if buf_raw:
        raw_arr = np.stack(buf_raw, axis=0)  # (N,12,32)
        raw_2d = raw_arr.reshape(raw_arr.shape[0], -1)  # (N,384)
    else:
        raw_2d = np.zeros((0, FLAT_DIM), dtype=np.float32)

    if buf_gray:
        gray_arr = np.stack(buf_gray, axis=0)  # (N,12,32)
        gray_2d = gray_arr.reshape(gray_arr.shape[0], -1)  # (N,384)
    else:
        gray_2d = np.zeros((0, FLAT_DIM), dtype=np.uint8)

    # Save CSV (no headers to keep it simple / consistent)
    np.savetxt(os.path.join(session_dir, "t_ns.csv"), t_arr, delimiter=",", fmt="%d")
    np.savetxt(os.path.join(session_dir, "frame_id.csv"), id_arr, delimiter=",", fmt="%d")

    # raw: keep 4 decimals (you can change fmt if you want more)
    np.savetxt(os.path.join(session_dir, "raw.csv"), raw_2d.astype(np.float32), delimiter=",", fmt="%.4f")
    np.savetxt(os.path.join(session_dir, "gray.csv"), gray_2d.astype(np.uint16), delimiter=",", fmt="%d")


if __name__ == "__main__":
    print("receive data test")
    print("Controls: 'r' start/stop recording, 'q' quit.")
    print(f"Logging at {TARGET_HZ:.0f} Hz -> saving as .csv files.")

    serDev = serial.Serial(PORT, BAUD, timeout=SER_TIMEOUT)
    serDev.reset_input_buffer()

    th = threading.Thread(target=readThread, args=(serDev,), daemon=True)
    th.start()

    recording = False
    session_dir = None

    # Buffers for one episode
    buf_t = []
    buf_id = []
    buf_raw = []
    buf_gray = []

    frame_id = 0
    prev_disp = np.zeros((ROWS, COLUMNS), dtype=np.float32)
    next_tick = time.perf_counter()

    while True:
        # ---- display ----
        if flag_init_done:
            with data_lock:
                cur = latest_frame_norm.copy()

            disp = temporal_filter(cur, prev_disp, alpha=ALPHA)
            prev_disp = disp

            img8_disp = np.clip(disp * 255, 0, 255).astype(np.uint8)
            cmap = cv2.applyColorMap(img8_disp, cv2.COLORMAP_VIRIDIS)
            cv2.imshow("Contact Data_left", cmap)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        # ---- toggle recording ----
        if key == ord("r") and flag_init_done:
            recording = not recording
            if recording:
                ts = time.strftime("%Y-%m-%d_%H-%M-%S")
                session_dir = f"session_{ts}"
                os.makedirs(session_dir, exist_ok=True)

                buf_t.clear()
                buf_id.clear()
                buf_raw.clear()
                buf_gray.clear()

                frame_id = 0
                next_tick = time.perf_counter()
                print(f"[REC ON] Recording to {session_dir}/ as .csv files")

            else:
                save_episode_csv(session_dir, buf_t, buf_id, buf_raw, buf_gray)
                print(f"[REC OFF] Saved {len(buf_id)} frames:")
                print("   raw.csv / gray.csv / t_ns.csv / frame_id.csv")

        # ---- fixed-rate logging (100 Hz) ----
        if recording and flag_init_done:
            now = time.perf_counter()
            if now >= next_tick:
                next_tick += DT

                with data_lock:
                    frame_to_log = latest_frame_norm.copy()

                raw = frame_to_log.astype(STORE_DTYPE)  # (12,32)
                gray = np.clip(frame_to_log * 255.0, 0, 255).astype(np.uint8)

                buf_t.append(time.time_ns())
                buf_id.append(frame_id)
                buf_raw.append(raw)
                buf_gray.append(gray)

                frame_id += 1

    cv2.destroyAllWindows()
    try:
        serDev.close()
    except Exception:
        pass
