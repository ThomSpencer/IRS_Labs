# Needed installs
``` bash
sudo apt install ros-humble-rqt-tf-tree
```

# Start/Setup ros2
```bash
source /opt/ros/humble/setup.bash 
```

# Run Pineapple Grand Challenge
For those marking this repo is all of the ros2 code the pineapple grand challenge code is pineapple_box_sorter.
```bash
colcon build --packages-select pineapple_box_sorter
ros2 launch pineapple_box_sorter nav_launch.py
```
#### Then in a separate terminal
```bash
ros2 run pineapple_box_sorter hs_waypoint_follower
```

<br><br><br><br><br>

# To make a package
```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name\
{NAME_OF_NODE} {NAME_OF_PACKAGE}
## Config Files for RVIS
mkdir rviz config launch
```

# Colcon Build
```bash
colcon build --packages-select {NAME_OF_PACKAGE}
source install/local_setup.bash 
ros2 run {NAME_OF_PACKAGE} {NAME_OF_NODE}
## OR
ros2 launch {NAME_OF_PACKAGE} {PATH_TO_LAUNCH_FILE}
```

# Run Mapping node
```bash
ros2 launch hand_solo_virtual_nav mapping_launch.py
```

# Run Autonomous mode
```bash
ros2 launch hand_solo_virtual_nav nav_launch.py
```

# Waypoint Follower
Make sure you run the autonomous mode first.
```bash
ros2 run hand_solo_virtual_nav hs_waypoint_follower
```

# PLC Status Updater
Make sure the PLC is on and enabled.
```bash
ros2 run pa_warehouse_status plc_hmi_listener 
```

# Transformation tree
```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

# Image Viewer
```bash
ros2 run rqt_image_view rqt_image_view
```