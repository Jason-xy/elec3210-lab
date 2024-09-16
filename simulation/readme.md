# Simulation (Optional)

You can try this simulation project to collect your own data for the LiDAR odometry estimation project.

## Introduction

This simulation project is based on the [ROS Gazebo](http://gazebosim.org/) simulation environment. The robot model is a differential drive robot with a LiDAR sensor. You can control the robot to move around the environment by keyboard control, and collect the LiDAR data by rosbag.

## How to use

1. Start the simulation environment

```bash
./simulation_dev.sh
```

2. launch the simulatior

```bash
catkin_make
source devel/setup.bash
roslaunch 3d_slam_simulation gazebo_turtlebot3.launch
```

3. Open browser and visit [noVNC](http://localhost:8080) to access the Gazebo and Rviz GUI.

You can see the robot model in the Gazebo.

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240916141800.png)

And the LiDAR scan can be visualized in the Rviz.

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240916141853.png)

4. Control the robot to move around the environment by keyboard control using following command.

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240916141920.png)

5. Record the rosbag data

```bash
# List the topics
rostopic list
# Record the LiDAR data
rosbag record <LiDAR_topic> -O <bag_name>.bag
```