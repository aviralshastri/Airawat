# Airawat

## Setup

```bash
source ./Airawat/airawat_ws/install/setup.bash
```

## YDLidar Driver

```bash
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node   --ros-args --params-file ~/Airawat/airawat_ws/src/ydlidar_ros2_driver/params/X4-Pro.yaml
```

## Arm Controller

```bash
ros2 run arm_controller arm_controller_node --ros-args --params-file src/arm_controller/config/config.yaml
```

## Hardware Interfaces

### Servo Interface

```bash
ros2 run hardware_interfaces servo_interface --ros-args --params-file src/hardware_interfaces/config/servo_config.yaml
```

### Drive Interface

```bash
ros2 run hardware_interfaces drive_interface --ros-args --params-file src/hardware_interfaces/config/drive_config.yaml
```

### IMU Interface

```bash
ros2 run hardware_interfaces imu_interface --ros-args --params-file src/hardware_interfaces/config/imu_config.yaml
```

### RC Interface

```bash
ros2 run hardware_interfaces rc_interface --ros-args --params-file src/hardware_interfaces/config/rc_config.yaml
```
