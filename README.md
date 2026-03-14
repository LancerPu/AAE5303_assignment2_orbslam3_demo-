# AAE5303 Assignment: Visual Odometry with ORB-SLAM3

<div align="center">

![ORB-SLAM3](https://img.shields.io/badge/SLAM-ORB--SLAM3-blue?style=for-the-badge)
![VO](https://img.shields.io/badge/Mode-Visual_Odometry-green?style=for-the-badge)
![Dataset](https://img.shields.io/badge/Dataset-HKisland__GNSS03-orange?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Baseline-success?style=for-the-badge)

**Monocular Visual Odometry Evaluation on UAV Aerial Imagery**

*Hong Kong Island GNSS Dataset - MARS-LVIG*

</div>

---

## 📋 Table of Contents

1. [Executive Summary](#-executive-summary)  
2. [Introduction](#-introduction)  
3. [Methodology](#-methodology)  
4. [Dataset Description](#-dataset-description)  
5. [Implementation Details](#-implementation-details)  
6. [Results and Analysis](#-results-and-analysis)  
7. [Visualizations](#-visualizations)  
8. [Discussion](#-discussion)  
9. [Conclusions](#-conclusions)  
10. [References](#-references)  
11. [Appendix](#-appendix)  

---

## 📊 Executive Summary

This report presents the implementation and evaluation of **Monocular Visual Odometry (VO)** using the **ORB-SLAM3** framework on the **HKisland_GNSS03** UAV aerial imagery dataset.  
The VO trajectory is evaluated against RTK ground truth using **four parallel, monocular-appropriate metrics** computed with the `evo` toolkit, following the official assignment protocol with a **5 Hz downsampled RTK trajectory (1955 poses)**.

### Key Results (Full 5 Hz Sequence)

| Metric                | Value                | Description                                                        |
|----------------------|----------------------|--------------------------------------------------------------------|
| **ATE RMSE**         | **59.78 m**          | Global accuracy after Sim(3) alignment (scale corrected)          |
| **RPE Trans Drift**  | **1.99 m/m**         | Translation drift rate (mean error per meter, delta = 10 m)       |
| **RPE Rot Drift**    | **143.32 deg/100m**  | Rotation drift rate (mean angle per 100 m, delta = 10 m)          |
| **Completeness**     | **49.46%**           | Matched poses / total ground-truth poses (967 / 1955)             |
| **Estimated poses**  | 982                  | Trajectory poses in `CameraTrajectory_sec.txt`                    |

Additionally, on a **shorter, more stable subsequence** (700 GT poses, 404 estimates), the completeness increases to **57.71% (404 / 700)** with a translation drift of **1.43 m/m**, but this only reflects **local** performance and is not representative of the full flight.

---

## 📖 Introduction

### Background

ORB-SLAM3 is a state-of-the-art visual SLAM system capable of performing:

- **Monocular Visual Odometry** (pure camera-based)  
- **Stereo Visual Odometry**  
- **Visual-Inertial Odometry** (with IMU fusion)  
- **Multi-map SLAM** with relocalization and loop closing  

This assignment focuses on **Monocular VO mode**, which:

- Uses only camera images for pose estimation  
- Cannot observe absolute metric scale (scale ambiguity)  
- Relies on ORB feature extraction and feature matching for tracking  
- Is susceptible to drift and tracking loss without loop closure  

### Objectives

1. Implement monocular Visual Odometry using ORB-SLAM3.  
2. Run ORB-SLAM3 VO on the HKisland_GNSS03 UAV dataset.  
3. Extract RTK (Real-Time Kinematic) GPS data as ground truth and convert it to a TUM-format ENU trajectory.  
4. Evaluate trajectory accuracy using four monocular-friendly metrics (ATE, translational/rotational drift, completeness) computed by `evo`.  
5. Document a complete and reproducible workflow, including ROS integration, code modifications, and evaluation pipeline.

### Scope

This assignment evaluates:

- **ATE (Absolute Trajectory Error)**: Global trajectory accuracy after Sim(3) alignment (monocular-friendly).  
- **RPE drift rates (translation + rotation)**: Local consistency (drift per traveled distance).  
- **Completeness**: Robustness / coverage (how much of the RTK sequence can be evaluated).

---

## 🔬 Methodology


### ORB-SLAM3 Visual Odometry Overview

ORB-SLAM3 performs visual odometry through the following pipeline:

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Input Image    │────▶│   ORB Feature   │────▶│   Feature       │
│  Sequence       │     │   Extraction    │     │   Matching      │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
                                                         │
┌─────────────────┐     ┌─────────────────┐     ┌────────▼────────┐
│   Trajectory    │◀────│   Pose          │◀────│   Motion        │
│   Output        │     │   Estimation    │     │   Model         │
└─────────────────┘     └────────┬────────┘     └─────────────────┘
                                 │
                        ┌────────▼────────┐
                        │   Local Map     │
                        │   Optimization  │
                        └─────────────────┘
```



### Evaluation Metrics

#### 1. ATE (Absolute Trajectory Error)

ATE measures the RMSE of the aligned trajectory after Sim(3) alignment:

\[
ATE_{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}\|\mathbf{p}_{est}^i - \mathbf{p}_{gt}^i\|^2}
\]

**Reference**: Sturm et al., *“A Benchmark for the Evaluation of RGB-D SLAM Systems”*, IROS 2012.

#### 2. RPE (Relative Pose Error) – Drift Rates

RPE measures local consistency by comparing relative motions:

\[
RPE_{trans} = \|\Delta\mathbf{p}_{est} - \Delta\mathbf{p}_{gt}\|
\]

where \(\Delta\mathbf{p}(t, \Delta) = \mathbf{p}(t + \Delta) - \mathbf{p}(t)\).

**Reference**: Geiger et al., *“Vision meets Robotics: The KITTI Dataset”*, IJRR 2013.

We report drift **rates** that are easier to interpret:

- **Translation drift rate** (m/m):  
  \[
  \text{Trans drift} = \frac{\text{RPE}_{trans, mean}}{\Delta d}
  \]
- **Rotation drift rate** (deg/100m):  
  \[
  \text{Rot drift} = \frac{\text{RPE}_{rot, mean}}{\Delta d} \times 100
  \]

with \(\Delta d = 10\) m.

#### 3. Completeness

Completeness measures how many ground-truth poses can be associated:

\[
Completeness = \frac{N_{matched}}{N_{gt}} \times 100\%
\]

where `N_matched` is the number of successfully associated pose pairs under `t_max_diff`.

#### Why these metrics (and why Sim(3) alignment)?

Monocular VO suffers from **scale ambiguity**: metric scale cannot be recovered from images alone. Therefore:

- All error metrics are computed after **Sim(3) alignment** (rotation + translation + scale), so they reflect **trajectory shape and drift** rather than arbitrary scale.  
- RPE is evaluated in the **distance domain** (10 m) to make drift comparable along long trajectories.  
- **Completeness** is explicitly reported to discourage trivial solutions that only output short, “easy” segments.

### Trajectory Alignment

We use 7-DOF Sim(3) alignment (Umeyama method) to optimally align the estimated trajectory to RTK ground truth in ENU:

- 3-DOF translation  
- 3-DOF rotation  
- 1-DOF scale  

---

## 📁 Dataset Description

### HKisland_GNSS03 Dataset

The dataset is from the **MARS-LVIG** UAV dataset, captured over Hong Kong Island.

| Property              | Value                          |
|-----------------------|--------------------------------|
| **Dataset Name**      | HKisland_GNSS03               |
| **Source**            | MARS-LVIG / UAVScenes         |
| **Duration**          | 390.78 s (~6.5 min)           |
| **Total Images**      | 3,833 frames                  |
| **Image Resolution**  | 2448 × 2048 pixels            |
| **Frame Rate**        | ~10 Hz                        |
| **Trajectory Length** | ~1,900 m                      |
| **Height Variation**  | 0–90 m                        |

### Data Sources

| Resource          | Link                                  |
|-------------------|---------------------------------------|
| MARS-LVIG Dataset | https://mars.hku.hk/dataset.html     |
| UAVScenes GitHub  | https://github.com/sijieaaa/UAVScenes |

### Ground Truth

RTK GNSS provides centimeter-level positioning accuracy:

| Property              | Value                                |
|-----------------------|--------------------------------------|
| **RTK Positions**     | 1,955 poses (after 5 Hz downsampling) |
| **Rate**              | ~5 Hz (from original 10 Hz NavSatFix) |
| **Accuracy**          | ±2 cm (horizontal), ±5 cm (vertical) |
| **Coordinate System** | WGS84 → Local ENU                   |

---

## ⚙️ Implementation Details

### System Configuration

| Component            | Specification             |
|----------------------|---------------------------|
| **Framework**        | ORB-SLAM3 (C++)           |
| **Mode**             | Monocular Visual Odometry |
| **Vocabulary**       | ORBvoc.txt (pre-trained)  |
| **Operating System** | Linux (Ubuntu under WSL2) |

### Camera Calibration

```yaml
Camera.type: "PinHole"
Camera1.fx: 1444.43
Camera1.fy: 1444.34
Camera1.cx: 1179.50
Camera1.cy: 1044.90

Camera1.k1: -0.0560
Camera1.k2: 0.1180
Camera1.p1: 0.00122
Camera1.p2: 0.00064
Camera1.k3: -0.0627

Camera.width: 2448
Camera.height: 2048
Camera.fps: 10.0
Camera.RGB: 1
```

## ORB Feature Extraction Parameters (final)
Parameter	Value	Description
nFeatures	2500	Features per frame
scaleFactor	1.2	Pyramid scale factor
nLevels	8	Pyramid levels
iniThFAST	15	Initial FAST threshold
minThFAST	5	Minimum FAST threshold
These parameters were tuned (from the default 1500 / 20 / 7) to increase feature coverage and reduce tracking loss for HKisland aerial imagery.

Code modification and execution
To output the full-frame trajectory CameraTrajectory.txt from the ROS node, ros_mono_compressed.cc was modified:

// Stop all threads
SLAM.Shutdown();

// Save trajectories (full frame trajectory + keyframe trajectory)
SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

ROS execution (three terminals):
### Terminal 1: ROS master
source /opt/ros/noetic/setup.bash
roscore
### Terminal 2: ORB-SLAM3 Mono_Compressed node
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
./Examples_old/ROS/ORB_SLAM3/Mono_Compressed \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/HKisland_Mono.yaml
### Terminal 3: ROS bag playback
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
rosbag play data/HKisland_GNSS03.bag \
    /left_camera/image/compressed:=/camera/image_raw/compressed
At the end of the run, CameraTrajectory.txt and KeyFrameTrajectory.txt are saved in /root/ORB_SLAM3.

## 📈 Results and Analysis
1) Full 5 Hz Evaluation (1955 GT poses)
Using:

Ground truth: ground_truth_5hz.txt (1955 poses)
Estimated: CameraTrajectory_sec.txt (982 poses, timestamps in seconds)
The official assignment evaluation script reports:
ATE RMSE (m):                 59.782344
RPE trans drift (m/m):        1.994013
RPE rot drift (deg/100m):     143.322932
Completeness (%):             49.46  (967 / 1955)
Summary table:

Metric	Value
ATE RMSE (m)	59.78
ATE mean (m)	53.70
ATE std (m)	26.27
RPE trans mean (10 m, m)	19.94
RPE trans RMSE (10 m, m)	35.58
RPE trans drift (m/m)	1.99
RPE rot mean (10 m, deg)	14.33
RPE rot RMSE (10 m, deg)	24.18
RPE rot drift (deg/100m)	143.32
Matched poses	967
GT poses	1955
Completeness (%)	49.46%
Interpretation:

Only about half of the 5 Hz RTK poses are successfully associated and evaluated.
Global trajectory error (ATE) is on the order of tens of meters, indicating noticeable global distortion.
Local drift rates (both translation and rotation) are high, reflecting unstable tracking and accumulated drift.
2) High-Completeness Short Subsequence (Optional)
To visualize “best-case local performance”, a high-completeness subsequence was automatically selected:

Ground truth: ground_truth_sub_high.txt (700 poses)
Estimated: CameraTrajectory_sub_high.txt (404 poses)
Matched: 404 / 700 → Completeness = 57.71%
Metrics:

ATE RMSE ≈ 62.59 m
RPE trans drift ≈ 1.43 m/m
RPE rot drift ≈ 147.51 deg/100m
This shows that in a relatively stable segment, the coverage can approach 60%, but the trajectory still suffers from large global and local errors.

## 📊 Visualizations
Full 5 Hz Evaluation Figure
The following figure is generated using:

The four subplots show:

Top-Left: 2D trajectory before alignment (matched poses only), highlighting scale and rotation mismatch typical of monocular VO.
Top-Right: 2D trajectory after Sim(3) alignment (scale corrected). Remaining discrepancies reflect accumulated drift and local tracking errors.
Bottom-Left: Distribution of ATE translation errors (meters) across all matched poses.
Bottom-Right: ATE translation error as a function of pose index, clearly showing error growth over time.

## 💭 Discussion
Strengths
End-to-end pipeline completed
The entire process—from ROS node modification, bag playback, and trajectory export, to RTK processing and evo-based evaluation—has been implemented, forming a reproducible VO evaluation workflow.

Reasonable local performance in stable segments
In shorter, relatively stable sub-sequences, completeness can reach ~58%, and translation drift is around 1.4–2.0 m/m, demonstrating that ORB-SLAM3 can track for a while before severe drift accumulates.

Visualization and diagnostics
The four-panel evaluation figure provides intuitive insight into how and where the VO system fails, guiding further debugging and parameter tuning.

Limitations
Low completeness on the full sequence (~49.46%)
Out of 1955 5 Hz RTK poses, only 967 are successfully associated. This is mainly due to:

Frequent tracking failure (“Fail to track local map!”),
ORB-SLAM3 stopping effective estimation in later parts of the sequence,
Computational limitations under WSL2.
High drift rates
ATE RMSE ≈ 60 m and RPE drift ≈ 2 m/m and 143 deg/100m are far from acceptable for practical UAV navigation. This indicates that tracking is unstable even in segments that are successfully processed.

No loop closure and no IMU fusion
The current setup uses pure monocular VO without loop closure or IMU. For long aerial trajectories with low parallax and repeated textures, this is fundamentally fragile.

Error Sources
Algorithm level: intrinsic scale ambiguity and drift in monocular VO, absence of loop closure.
Parameter level: although ORB parameters are tuned, the combination of high-resolution imagery and WSL2 may still cause frame drops and under-utilization of potential tracks.
Data level: aggressive UAV motion, motion blur, weak textures (rooftops, water, sky) and large scene depth all degrade feature tracking.
System level: WSL2 and ROS introduce additional overhead and timing jitter, making real-time tracking harder.

## 🎯 Conclusions
This assignment successfully implements monocular Visual Odometry with ORB-SLAM3 on the HKisland_GNSS03 UAV dataset and evaluates its performance using a standardized, monocular-friendly metric suite.

Key takeaways:

End-to-end system works, but robustness is limited
ORB-SLAM3 can process the dataset and produce a usable TUM trajectory, but only about half of the RTK poses can be evaluated on the full 5 Hz sequence.

Accuracy and drift are insufficient for reliable navigation
ATE RMSE ~60 m and high drift rates indicate that pure monocular VO (without IMU or loop closure) struggles with long-range aerial imagery.

Shorter stable segments show better coverage
In a ~700-pose subsequence, completeness approaches 58%, highlighting that there are periods where VO tracking is reasonably stable.

Improvements require both algorithmic and system-level changes
Simply tuning ORB parameters and bag playback rate under WSL2 can only go so far; further gains likely require native Linux, better hardware, IMU fusion, and possibly loop closure.

## 📚 References

Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M., & Tardós, J. D. (2021). ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM. IEEE Transactions on Robotics, 37(6), 1874–1890.

Sturm, J., Engelhard, N., Endres, F., Burgard, W., & Cremers, D. (2012). A Benchmark for the Evaluation of RGB-D SLAM Systems. IEEE/RSJ IROS.

Geiger, A., Lenz, P., & Urtasun, R. (2012). Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite. IEEE CVPR.

MARS-LVIG Dataset: https://mars.hku.hk/dataset.html

ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAM3


## 📎 Appendix
A. Repository Structure (Evaluation Repo)
```

AAE5303_assignment2_orbslam3_demo-/
├── README.md                    # This report
├── requirements.txt             # Python dependencies
├── figures/
│   └── trajectory_evaluation_5hz.png
├── output/
│   └── hkisland_5hz/
│       ├── ate.zip
│       ├── rpe_trans.zip
│       └── rpe_rot.zip
├── scripts/
│   ├── evaluate_vo_accuracy.py
│   └── generate_report_figures.py
├── docs/
│   └── camera_config.yaml
└── leaderboard/
    ├── README.md
    ├── LEADERBOARD_SUBMISSION_GUIDE.md
    └── submission_template.json
```
    B. Running Commands (Reproducible Pipeline)
    # 1. Run ORB-SLAM3 ROS node (Mono_Compressed)
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
./Examples_old/ROS/ORB_SLAM3/Mono_Compressed \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/HKisland_Mono.yaml

### 2. Play ROS bag (image topic remapping)
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
rosbag play data/HKisland_GNSS03.bag \
    /left_camera/image/compressed:=/camera/image_raw/compressed

### 3. Convert estimated trajectory timestamps from ns to seconds
cd /root/ORB_SLAM3
awk '{printf "%.9f %s %s %s %s %s %s %s\n", $1/1e9, $2, $3, $4, $5, $6, $7, $8}' \
    CameraTrajectory.txt > CameraTrajectory_sec.txt

### 4. Evaluate with official assignment script (full 5 Hz GT)
```
cd /root/AAE5303_assignment2_orbslam3_demo-
python3 scripts/evaluate_vo_accuracy.py \
    --groundtruth /root/ORB_SLAM3/ground_truth_5hz.txt \
    --estimated /root/ORB_SLAM3/CameraTrajectory_sec.txt \
    --t-max-diff 0.1 \
    --delta-m 10 \
    --workdir output/hkisland_5hz \
    --json-out output/hkisland_5hz/metrics.json
```
    C. Output Trajectory Format (TUM)
    # timestamp x y z qx qy qz qw
```
1698132964.499888000 -0.0198143  0.0105970 -0.1055819  0.0048541 -0.0099622  0.0003808  0.9999385
1698132964.599976000 -0.0511473  0.0295398 -0.3137089  0.0094823 -0.0158869  0.0014637  0.9998278
...
```
AAE5303 – Robust Control Technology in Low-Altitude Aerial Vehicle

Department of Aeronautical and Aviation Engineering

The Hong Kong Polytechnic University

Mar 2026
    
