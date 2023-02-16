To launch the kinect:
```
cd catkin_ws
source devel/setup.bash
roslaunch kinect2_bridge kinect2_bridge.launch
```

To launch Spot driver and rviz:
```
roslaunch spot_mediapipe spot_config.launch
```

To match the kinect frame and the Spot body frame:
```
rosrun tf static_transform_publisher 0.22 -0.009 0.075 1.57 0 1.57 body kinect_rgb_optical_frame 100
```

To start the analysis part:
```
rosrun spot_mediapipe motion.py
rosrun spot_mediapipe trauma.py
rosrun spot_mediapipe file.py
rosrun spot_mediapipe state_machine.py
```
