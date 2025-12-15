# Airawat

source ./Airawat/airawat_ws/install/setup.bash

ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node   --ros-args --params-file ~/Airawat/airawat_ws/src/ydlidar_ros2_driver/params/X4-Pro.yaml

ros2 run bno055 bno055_imu_pub

