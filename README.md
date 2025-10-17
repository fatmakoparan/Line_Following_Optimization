# 🤖 Autonomous Robot Path Optimization and Obstacle Detection in Gazebo Simulation  

## 🚀 Overview  
This project focuses on developing an **autonomous mobile robot simulation** in the **Gazebo environment**, integrated with **ROS (Robot Operating System)**.  
The goal is to simulate real-world navigation tasks such as **lane tracking**, **obstacle detection**, and **path optimization** using algorithms like **A\*** and **Dijkstra**.  

The project emphasizes realistic implementation by combining **computer vision (OpenCV)** for lane detection, **LIDAR** for obstacle sensing, and optimization algorithms for intelligent decision-making.  

---

## 🧩 Features  
- 🧠 **Autonomous Lane Tracking** – Implemented using camera input and OpenCV.  
- 🧱 **Obstacle Detection** – LIDAR-based detection with reactive behavior and buzzer alerts.  
- 🗺️ **Path Optimization** – Compares A\*, Dijkstra, and an ant-inspired heuristic approach.  
- ⚙️ **ROS Integration** – Utilizes ROS topics, publishers, and subscribers for communication.  
- 🧪 **Realistic Simulation** – Developed in Gazebo with Teknofest-style track design.  
- 🦾 **Robust Testing** – Multiple test phases ensuring stability and real-world adaptability.  

---

## ⚙️ Tech Stack  
| Category | Tools / Frameworks |
|-----------|-------------------|
| **Simulation** | Gazebo |
| **Framework** | ROS (Robot Operating System) |
| **Programming Language** | Python |
| **Libraries** | OpenCV, NumPy, rospy |
| **Robot Model** | TurtleBot3 Waffle (Camera + LIDAR) |

---

## 🧠 Methodology  

### 1. Simulation Setup  
- Selected **Gazebo** after analyzing multiple robotic simulation platforms.  
- Chose **TurtleBot3 Waffle** for its integrated camera and LIDAR modules.  
- Created a **Teknofest-inspired testing field** for consistent and realistic simulations.  

### 2. Lane Detection  
- Used **OpenCV** to process grayscale and binary images for lane tracking.  
- Lane color chosen as **black** to minimize detection errors in the Gazebo environment.  
- Verified camera data visualization using **rqt_image_view** via ROS topics.  

### 3. Obstacle Detection & Avoidance  
- LIDAR detects obstacles in front of the robot.  
- If an obstacle persists for **10 seconds**, the robot calculates an optimized detour path.  
- Integrated a **buzzer alert system** to emulate real robotic warning feedback.  

### 4. Path Optimization  
- Defined **start and goal points** within `.launch` files for consistent testing.  
- Compared **A\*** and **Dijkstra** algorithms — both yielded equal path lengths.  
- Applied an **ant-inspired heuristic**: when distances are equal, the path with fewer turns is preferred.  
- Selected **A\*** for its optimal trade-off between efficiency and simplicity.  

### 5. Lane-Centered Correction Mechanism  
- Implemented automatic alignment based on camera feedback:  
  - If the robot drifts **0.5m left**, it adjusts right.  
  - If it drifts right, it adjusts left.  
- Ensures continuous lane-centering and stability during navigation.  

---

## 🧪 Testing & Results  
- Conducted multiple tests in varied simulation setups.  
- The final system successfully:  
  - Followed lanes autonomously.  
  - Detected and avoided obstacles intelligently.  
  - Selected the most efficient and shortest route.  
  - Maintained high stability and precision during all movements.  

The results demonstrate a strong balance between **optimization efficiency** and **real-world simulation accuracy**.  


---

## 📄 License  
This project is open for academic and research purposes.  
Please provide proper credit if referenced or modified.  

---
