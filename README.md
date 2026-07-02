# Autonomous Ground Robot — PID + EKF Simulation

## Overview

This project simulates an autonomous ground robot using MATLAB, combining PID control and Extended Kalman Filter (EKF) based state estimation. The system demonstrates real-time waypoint navigation, sensor fusion, and differential drive robot modeling in a realistic simulation environment.

The robot is controlled using multiple PID loops for position, heading, and velocity control, while an EKF fuses noisy GPS and encoder measurements to estimate the robot’s true state.

---

## Key Features

* Autonomous waypoint navigation system
* Multi-loop PID control (Position, Heading, Velocity)
* Extended Kalman Filter (EKF) for sensor fusion
* Differential drive robot dynamics simulation
* Noisy sensor modeling (GPS, encoders, gyroscope)
* Real-time trajectory visualization
* Performance metrics (RMSE, heading error, waypoint tracking)
* Motor PWM command simulation
* Full MATLAB-based visualization suite

---

## System Architecture

The system consists of:

1. **Motion Controller**

   * Position PID
   * Heading PID
   * Velocity PID

2. **State Estimator**

   * Extended Kalman Filter (EKF)
   * Fusion of GPS + encoder + gyro data

3. **Robot Plant Model**

   * Differential drive kinematics
   * Motor lag and velocity dynamics

4. **Navigation System**

   * Waypoint-based autonomous navigation
   * Goal switching logic with capture radius

---

## Technologies Used

* MATLAB
* Control Systems (PID Design)
* Estimation Theory (Extended Kalman Filter)
* Robotics Kinematics
* Numerical Simulation

---

## How to Run

1. Open MATLAB (R2020b or newer)
2. Navigate to the project folder
3. Run:

```matlab
pid_robot_main
```

4. Optional export mode:

```matlab
pid_robot_main('export')
```

---

## Outputs

* Robot trajectory plot (true vs EKF estimate)
* Time-series state analysis
* PID control response comparison
* Motor PWM command signals
* Position estimation error metrics

---

## Performance Metrics

The system outputs:

* Position RMSE (EKF accuracy)
* Heading RMSE
* Waypoint completion count
* Maximum tracking error

---

## Applications

* Autonomous robotics research
* Control systems education
* Sensor fusion studies
* MATLAB simulation projects
* EKF + PID control demonstrations

---

## Author

AMRUTH CHINTA

---

## License

For academic and educational use.
