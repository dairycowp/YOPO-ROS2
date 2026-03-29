# YOPO-ROS2
A ROS 2 version for [TJU-YOPO-Simple](https://github.com/TJU-Aerial-Robotics/YOPO/tree/YOPO-Simple)
![image](https://github.com/dairycowp/YOPO-ROS2/blob/main/pics/mmexport1762278973380.gif)
---

## 🛠️ Build Instructions

### 1. Build the `utils` library inside `Controller`

The `Controller/src/utils` directory contains C++ and message utilities required by other components. Build it first **within the `Controller` workspace**:

```bash
cd Controller
colcon build --packages-select cmake-utils --symlink-install
```

### 2. Build the full Controller package
```bash
cd Controller
colcon build
```

### 3. Build the Simulator package
```bash
cd Simulator
colcon build 
```
## ▶️ Launch Instructions
Open three separate terminals to run the full system:

### Terminal 1: Launch drone simulator (SO(3) controller)
```bash
cd Controller
source install/setup.bash
ros2 launch so3_quadrotor_simulator simulator_attitude_control.launch.py
```

### Terminal 2: Launch CUDA sensor simulator
```bash
cd Simulator
source install/setup.bash
ros2 run sensor_simulator sensor_simulator_cuda
```
### Terminal 3: Run YOPO policy node
```bash
cd YOPO
# Source Controller to access quadrotor_msgs
source ~/Controller/install/setup.bash
python3 test_yopo_ros.py
```
