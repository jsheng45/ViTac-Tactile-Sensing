# Tactile Raw Data – ViTac Hardware Project

This repository contains raw tactile data collected using a 12×32 3D Vitac tactile sensor array mounted on a parallel gripper platform.

The data was collected as part of the ViTac hardware validation and manipulation benchmarking pipeline.


## Hardware Configuration

- Tactile Sensor: 3d Vitac
- Communication: USB serial (2,000,000 baud)
- Logging Rate: 100 Hz
- Baseline: Median of first 30 frames
- Threshold: 2 (noise suppression)
- Normalization: Frame-wise max scaling

## Folder Structure

Each recording session is stored as:

session_YYYY-MM-DD_HH-MM-SS/
    raw.csv
    gray.csv
    t_ns.csv
    frame_id.csv

## File Descriptions

### raw.csv
- Shape: (N, 384)
- Type: float16
- Description:
  Flattened tactile frame (12×32 → 384 values)
  Normalized values in range [0,1]

To reshape:
    frame.reshape(12, 32)

---

### gray.csv
- Shape: (N, 384)
- Type: uint8
- Description:
  8-bit grayscale version (0–255)

---

### t_ns.csv
- Shape: (N,)
- Type: int64
- Description:
  Timestamp in nanoseconds

---

### frame_id.csv
- Shape: (N,)
- Type: int32
- Description:
  Sequential frame index



---

## Data Collection Demonstration

To provide transparency and reproducibility, we include visual documentation of the data collection process.

### Setup Overview
- Gripper mounted with dual tactile sensors
- Parallel plate configuration (Stage 1 prototype)
- Manual interaction trials

## Hardware Setup
Image: (media/Images)

Video: (media/Videos)

---
### Pressing Down Trials
- Static normal force application
- Approx. ~2 seconds per trial
- ~10 total recordings

### Press and Slide Trials
- Normal contact followed by vertical and lateral movement. (up-down-left-right)
- ~10 total recordings

All trials were manually executed.


## Notes

- Some taxels may exhibit slight baseline drift.
- No force ground truth was recorded in this dataset.
- Gripper state was not logged in this session.



