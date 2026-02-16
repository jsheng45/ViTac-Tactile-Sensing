import os
import time
import threading
import numpy as np
import serial
import cv2

# ============================================================
# Saved NPZ format (data.npz):
#   t_ns     : (N,) int64      timestamps (ns)
#   frame_id : (N,) int32      frame index
#   raw      : (N,12,32) float16  normalized tactile frame (rounded to 4 decimals)
#   gray     : (N,12,32) uint8   grayscale image (0..255) from raw
# ============================================================

ROWS = 12
COLUMNS = 32

PORT = "/dev/ttyUSB0"
BAUD = 2_000_000
SER_TIMEOUT = 0.5

THRESHOLD = 2
NOISE_SCALE = 60
INIT_FRAMES = 30
SCALE = 30
ALPHA = 0.2

TARGET_HZ = 100.0
DT = 1.0 / TARGET_HZ

# In-memory storage (Version 1 simple mode)
# If you record very long sessions, memory may grow large.
STORE_DTYPE = np.float16  # float16 saves space; use float32 if you prefer

latest_frame_norm = np.zeros((ROWS, COLUMNS), dtype=np.float32)
flag_init_done = False
data_lock = threading.Lock()

cv2.namedWindow("Contact Data_left", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Contact Data_left", COLUMNS * SCALE, ROWS * SCALE)

def temporal_filter(new_frame, prev_frame, alpha=ALPHA):
    return alpha * new_frame + (1 - alpha) * prev_frame

def parse_row(line: str):
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

    frames = []
    print("Initializing baseline... do NOT touch the sensor.")
    while len(frames) < INIT_FRAMES:
        frames.append(read_one_frame(serDev))

    median = np.median(np.stack(frames, axis=0), axis=0).astype(np.float32)
    flag_init_done = True
    print("Finish Initialization!")

    while True:
        frame = read_one_frame(serDev)

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
            latest_frame_norm[:] = norm



if __name__ == "__main__":
    print("receive data test")
    print("Controls: 'r' start/stop recording, 'q' quit.")
    print(f"Logging at {TARGET_HZ:.0f} Hz into NPZ (raw array + grayscale array).")

    serDev = serial.Serial(PORT, BAUD, timeout=SER_TIMEOUT)
    serDev.reset_input_buffer()

    th = threading.Thread(target=readThread, args=(serDev,), daemon=True)
    th.start()

    recording = False
    session_dir = None

    # in-memory buffers for one recording session
    buf_t = []
    buf_id = []
    buf_raw = []
    buf_gray = []

    frame_id = 0
    prev_disp = np.zeros((ROWS, COLUMNS), dtype=np.float32)
    next_tick = time.perf_counter()

    while True:
        if flag_init_done:
            with data_lock:
                cur = latest_frame_norm.copy()

            # display smoothing only
            disp = temporal_filter(cur, prev_disp, alpha=ALPHA)
            prev_disp = disp

            img8_disp = np.clip(disp * 255, 0, 255).astype(np.uint8)
            cmap = cv2.applyColorMap(img8_disp, cv2.COLORMAP_VIRIDIS)
            cv2.imshow("Contact Data_left", cmap)
        else:
            blank = np.zeros((ROWS, COLUMNS), dtype=np.uint8)
            cv2.imshow("Contact Data_left", cv2.applyColorMap(blank, cv2.COLORMAP_VIRIDIS))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

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
                print(f"[REC ON] Recording to {session_dir}/data.npz")
            else:
                # save session
                t_arr = np.array(buf_t, dtype=np.int64)
                id_arr = np.array(buf_id, dtype=np.int32)

                raw_arr = np.stack(buf_raw, axis=0) if len(buf_raw) else np.zeros((0, ROWS, COLUMNS), dtype=STORE_DTYPE)
                gray_arr = np.stack(buf_gray, axis=0) if len(buf_gray) else np.zeros((0, ROWS, COLUMNS), dtype=np.uint8)

                out_path = os.path.join(session_dir, "data.npz")
                np.savez_compressed(out_path, t_ns=t_arr, frame_id=id_arr, raw=raw_arr, gray=gray_arr)

                print(f"[REC OFF] Saved {len(id_arr)} frames to {out_path}")
                print(f"          raw dtype={raw_arr.dtype}, gray dtype={gray_arr.dtype}")

        # fixed-rate logging
        if recording and flag_init_done:
            now = time.perf_counter()
            if now >= next_tick:
                next_tick += DT

                with data_lock:
                    frame_to_log = latest_frame_norm.copy()

                # raw: keep 4 decimals + store dtype
                raw = frame_to_log.astype(STORE_DTYPE)

                # grayscale image (0..255)
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
