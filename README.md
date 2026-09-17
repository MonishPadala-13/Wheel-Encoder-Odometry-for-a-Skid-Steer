# Skid-Steer Rover Odometry & UMBmark Calibration in ROS 2 & Ignition Gazebo

A complete ROS 2 (Humble) simulation pipeline and odometry estimator designed for a 4-wheel skid-steer rover in Ignition Gazebo (Fortress). This project analyzes the non-holonomic mechanics of skid-steer wheel scrubbing, calibrates effective kinematic parameters, evaluates drift via the University of Michigan Benchmark (UMBmark), and quantifies systematic errors resulting from asymmetric wheel dimensions.

---

## System Architecture
  +-----------------------------+
                   |  Ignition Gazebo (Fortress) |
                   |    - Physics & Friction     |
                   |    - Diff-Drive System      |
                   +--------------+--------------+
                                  |
     /world/empty/model/.../joint_state & /model/.../ground_truth
                                  v
                   +-----------------------------+
                   |        ros_gz_bridge        |
                   |   (parameter_bridge YAML)   |
                   +--------------+--------------+
                                  |
                       /joint_states & /ground_truth
                                  v
                   +-----------------------------+
                   |     encoder_emulator.py     |
                   | (radians -> discrete ticks) |
                   | (optional scale injection)  |
                   +--------------+--------------+
                                  |
                             /wheel_ticks
                                  v
                   +-----------------------------+
                   |      wheel_odometry.py      |
                   |  (Midpoint Runge-Kutta 2)   |
                   |  (Track scale & chi slip)   |
                   +--------------+--------------+
                                  |
                             /wheel_odom & /tf
                                  v
         +------------------------+------------------------+
         |                                                 |
         v                                                 v
+-------------------------+                       +-------------------------+
|     calibrate.py        |                       |     umbmark_test.py     |
|   - Linear scale (k_s)  |                       |   - 5 CW / 5 CCW runs   |
|   - Slip factor (chi)   |                       |   - CSV error logging   |
+-------------------------+                       +-------------------------+

---

## Kinematics & Skid-Steer Scrubbing Theory

Unlike standard differential-drive robots with independent caster wheels, a 4-wheel skid-steer vehicle has fixed parallel axles. Executing a turn forces all four contact patches to scrub and slide laterally across the terrain.

### 1. Effective Track Width ($B_{\text{eff}}$)
Due to lateral scrubbing resistance, the rover turns significantly less than the geometric track width ($B$) would predict. We model this using a slip factor $\chi$:

$$B_{\text{eff}} = \chi \cdot B, \quad (\chi > 1.0)$$

### 2. Kinematic Equations
Given left and right wheel displacements $d_L$ and $d_R$:

$$\Delta s = \frac{d_R + d_L}{2}$$

$$\Delta \theta = \frac{d_R - d_L}{B_{\text{eff}}} = \frac{d_R - d_L}{\chi \cdot B}$$

### 3. Midpoint (2nd-Order Runge-Kutta) Numerical Integration
To mitigate discretization truncation error during continuous turns:

$$\theta_{\text{mid}} = \theta_k + \frac{\Delta \theta}{2}$$

$$x_{k+1} = x_k + \Delta s \cdot \cos(\theta_{\text{mid}})$$

$$y_{k+1} = y_k + \Delta s \cdot \sin(\theta_{\text{mid}})$$

$$\theta_{k+1} = \theta_k + \Delta \theta$$

---

## Calibration Methodology

Calibration is split into two decoupled procedures to isolate linear scaling from rotational scrubbing effects:

1. **Distance Scale Calibration ($k_s$):**
   * Robot drives straight for $5.0\text{ m}$ ($\omega = 0$). No scrubbing occurs.
   * Compares estimated distance against simulator ground truth:
     $$k_s = \frac{\Delta s_{\text{true}}}{\Delta s_{\text{odom}}}$$
   * Calibrated wheel radius: $r_{\text{calib}} = r_{\text{nominal}} \cdot k_s$.

2. **Slip Factor Calibration ($\chi$):**
   * Robot commands pure in-place rotation ($\omega_z = 0.6\text{ rad/s}$, $v_x = 0$) until odometry registers 5 full rotations ($10\pi\text{ rad}$).
   * Continuous yaw unwrapping prevents $[-\pi, \pi]$ boundary jump artifacts.
   * Compares the integrated encoder heading change to actual ground truth:
     $$\chi = \frac{|\theta_{\text{odom, uncalibrated}}|}{|\theta_{\text{true}}|}$$

### Calibration Benchmark Results

| Parameter | Nominal | Calibrated | Physical Interpretation |
| :--- | :--- | :--- | :--- |
| Wheel Radius ($r$) | $0.08000\text{ m}$ | **$0.080165\text{ m}$** | Corrects minor $0.2\%$ travel scaling discrepancy ($k_s \approx 1.00206$). |
| Track Width ($B$) | $0.45000\text{ m}$ | **$0.45000\text{ m}$** | Fixed chassis geometry. |
| Slip Factor ($\chi$) | $1.0000$ | **$1.3642$** | Accounts for $26.7\%$ rotational loss due to wheel scrubbing ($B_{\text{eff}} \approx 0.6139\text{ m}$). |

---

## Package Structure

skid_steer_odom/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── bridge_config.yaml       # ros_gz_bridge topic & type mappings
│   └── params.yaml              # Calibrated kinematics parameters
├── launch/
│   ├── sim.launch.py            # Main launch (Gazebo + Bridge + Odom Pipeline)
│   └── part6_systematic.launch.py # Systematic error experiment launch
├── scripts/
│   ├── encoder_emulator.py      # Translates joint states into integer tick arrays
│   ├── wheel_odometry.py        # Custom odometry estimator node & TF publisher
│   ├── calibrate.py             # Automated 5m straight & 5-spin calibration routine
│   ├── umbmark_test.py          # 10-run UMBmark square path execution & CSV logger
│   ├── plot_results.py          # Matplotlib live recorder & trajectory plotter
│   └── part6_evaluator.py       # Systematic error benchmark runner & statistics logger
└── urdf/
└── rover.urdf.xacro         # 4-wheel rover URDF with friction & Gazebo plugins


---

## Installation & Setup

### Prerequisites
* Ubuntu 22.04 LTS
* ROS 2 Humble Desktop
* Ignition Gazebo (Fortress)
* ROS–Ignition Bridge:
  ```bash
  sudo apt-get install ros-humble-ros-gz
  warg
     
  
## Build 

 
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone <repository_url> skid_steer_odom
    cd ~/ros2_ws
    colcon build --packages-select skid_steer_odom
    source install/setup.bash

##
1. Launch the Base SimulationStarts Ignition Gazebo, spawns the rover, starts the bridge, and runs the odometry pipeline:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch skid_steer_odom sim.launch.py
   

2. Run Automated Kinematic CalibrationIn a separate terminal:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run skid_steer_odom calibrate.py
   This executes the $5\text{ m}$ straight run followed by the 5-revolution spin, logging the computed $k_s$ and $\chi$ directly to stdout.3. Run the UMBmark Square BenchmarkExecutes 5 Clockwise (CW) and 5 Counter-Clockwise (CCW) $2\text{ m} \times 2\text{ m}$ closed-loop squares:Bash# Terminal 2: Trajectory Plotter
   source ~/ros2_ws/install/setup.bash
   ros2 run skid_steer_odom plot_results.py
   
Bash


# Terminal 3: Benchmark Driver
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run skid_steer_odom umbmark_test.py



Results are exported to ~/ros2_ws/umbmark_results.csv.Press Ctrl+C on the plotter to generate ~/ros2_ws/trajectory_comparison.png.Systematic Error Analysis (Part 6)To model real-world manufacturing or inflation asymmetry, a $+2.0\%$ diameter mismatch is injected into the right-side wheels (right_wheel_scale: 1.02):Bash# Terminal 1: Launch with right-side scale distortion
ros2 launch skid_steer_odom part6_systematic.launch.py

# Terminal 2: Run systematic evaluation suite
ros2 run skid_steer_odom part6_evaluator.py
