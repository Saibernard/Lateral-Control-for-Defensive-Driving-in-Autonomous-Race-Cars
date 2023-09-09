# autonomous-racing-lateral-control
Implemented Python and ROS2-based lateral controller for defensive racing in a F1 1/10 scale autonomous vehicle. Utilized vision for opponent detection, pure pursuit algorithms for path planning, and developed a custom planner using lidar and Intel Real Sense depth camera for safe, efficient race decisions.

# Lateral Control for Defensive Driving in Autonomous Race Cars

## 1. Introduction

In the domain of autonomous vehicles, motorsport has undergone a transformative merger of technology, focusing on safer racing experiences. This report emphasizes the development and subsequent testing of a lateral control algorithm to enhance defensive driving in autonomous race cars.

### Car setup with stereo camera at the back to detect opponent cars

![image](https://github.com/Saibernard/Lateral-Control-for-Defensive-Driving-in-Autonomous-Race-Cars/assets/112599512/42e4432b-f6b4-4373-84c2-74bbaa56c9fa)


## 2. Algorithm and Implementation

### 2.1 Source Code

The entirety of the project's source code can be explored on our [Github repository](https://github.com/Saibernard/Lateral-Control-for-Defensive-Driving-in-Autonomous-Race-Cars).

### 2.2 Initial Testing and Observations

We determined the vehicle's lateral position (LP) through the formula:

LP = 2π × arctan(δ/L)


Where:
- \( \delta \) represents the vehicle's lateral deviation.
- \( L \) represents the vehicle's wheelbase.

Initial simulations showed substantial yaw rate fluctuations during high-speed changes, primarily due to omitting lateral acceleration considerations.

![image](https://github.com/Saibernard/Lateral-Control-for-Defensive-Driving-in-Autonomous-Race-Cars/assets/112599512/31da18d3-71f4-4d72-b8c4-40af8b43b30f)


### 2.3 Introduction of Lateral Control

The foundation of our lateral control algorithm is the formula:

U_lat = -Kp × e_lat - Kd × d(e_lat)/dt


Where:
- \( U_lat \) is the control output,
- \( K_p \) and \( K_d \) denote the proportional and derivative gains,
- \( e_lat \) is the lateral error,
- \( d(e_lat)/dt \) is the rate of change of the lateral error.

This formula's inclusion saw a significant uptick in the car's performance, primarily regarding stability during racing line adjustments. The visual comparison of these adjustments is documented in this [video](#https://www.youtube.com/watch?v=iy26FNFZgEw).

![image](https://github.com/Saibernard/Lateral-Control-for-Defensive-Driving-in-Autonomous-Race-Cars/assets/112599512/002dcfc2-574b-4281-9be7-808308845f40)


## 3. Physical Model Testing and Results

### 3.1 Implementation on F1TENTH Car

We transitioned to real-world testing by integrating the algorithm into an F1TENTH car. Distinct differences in performance were evident, especially at lower speeds. For a detailed comparative view, refer to this [video](#https://www.youtube.com/watch?v=xJaZCDzaTfY)

### April tag detection from the Real sense camera

![image](https://github.com/Saibernard/Lateral-Control-for-Defensive-Driving-in-Autonomous-Race-Cars/assets/112599512/610828c5-241c-4433-b136-b86439358cd0)


### 3.2 Defensive Maneuvering

After establishing the algorithm's efficacy, higher speeds were introduced. Our control system, paired with an onboard camera, detected competitors and replicated defensive racing line changes. Depending on the adversary's speed, the changes varied from short, rapid ones to lengthier, deliberate shifts. Observations from these tests are available [here](#test-link).

### 3.3 Racing Environment Simulation

Race simulations were conducted in RViz, with lap times presented in the table below:

| Metric                                        | Time       |
|-----------------------------------------------|------------|
| Ego car lap time (original)                   | 6.308 s    |
| Ego car lap time (lateral controller)         | 6.338 s    |
| Speed difference to opponent                  | -1 m/s     |
| Opponent lap time (with increased speed)      | 6.1 s      |
| Opponent lap time (against ego car defending) | 6.5 s      |

### Results showing the stability of the car during defensive manuever

![image](https://github.com/Saibernard/Lateral-Control-for-Defensive-Driving-in-Autonomous-Race-Cars/assets/112599512/62299d08-25af-49c3-b6d4-545cbd271838)


## 4. Conclusions and Future Work

This project marries technology to devise an efficacious lateral control algorithm tailored for defensive driving in autonomous race cars. An amalgamation of a forward-facing LiDAR and rear camera grants the vehicle an almost 360° environment view.

We're eager to explore the following future enhancements:

- **Integration of YOLO Models:** To further enhance vehicle detection capabilities.
- **Compatibility with High-Powered Nvidia GPUs:** To delve deeper into performance enhancements.
- **High-Speed Testing:** To discern the boundaries of lateral acceleration.
- **Resource Management:** Focusing on energy consumption and tire wear in genuine race conditions.

As the realm of autonomous racing broadens, our unwavering focus remains on delivering a thrilling yet safe racing experience.

---

