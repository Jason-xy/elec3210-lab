# Project 1 - ICP Odometry

## Introduction

In this project, you will implement an ICP registration algorithm to estimate the relative transformation between two point clouds. The function of the ICP algorithm is in the file `src/icp.cpp`. Also, we provide a ICP registration example implemented by PCL in the same file, which can be used to verify the correctness of your implementation.

### toy_example

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Galleryicp1.png)

Run the toy example to test the correctness of your implementation.

```bash
cd /ws
catkin_make
source devel/setup.bash
rosrun project_icp_odom toy_example2
```

You can see the registration result in the pop-up window.

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240828171521.png)

### odom_node

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Galleryicp2.png)

This node is used to estimate the odometry of the robot based on the ICP algorithm. The input is the point cloud data from the LiDAR sensor, and the output is the odometry of the robot. After implementing the ICP algorithm, you can run the following command to test your implementation.

```bash
cd /ws
catkin_make
source devel/setup.bash
roslaunch project_icp_odom icp.launch
# Press `blankspace` to start the odometry estimation
```

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240828172400.png)


### Evaluation

You can evaluate the performance of your implementation by comparing the estimated odometry with the ground truth. Recommended to use [evo](https://github.com/MichaelGrupp/evo) to evaluate the performance.

1. Record the rosbag data with estimated odometry and ground truth odometry.

```bash
rosbag record /odom_node/icp_odom /odom -O icp_odom.bag
```

2. Run evo to evaluate the performance.

```bash
evo_traj bag icp_odom.bag /odom_node/icp_odom --ref /odom -a -p
evo_ape bag icp_odom.bag xxxxxxxxx
evo_rpe bag icp_odom.bag xxxxxxxxx
```

example of `evo_traj`:

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240828174640.png)

## Scoring Rules
1. Please title your submission "PROJECT1-YOUR-NAMES.zip". Your submission should include a maximum 2-page report and all necessary files to run your code, a video in 1 minute, rosbag with the estimated odometry and ground truth odometry.
2. The report should contain at least a visualization of your final point cloud map and trajectory.
3. If your code is similar to others’ or any open-source code on the internet, the score will be 0 unless the sources are properly referenced in your report. Refer to $^1$ for details.
4. Excessive white space, overly large images, or redundant descriptions in report will not contribute to a higher score. Simple but efficient code is preferred.
5. If any problem, please google first. If unsolved, create a discussion on Canvas.
6. If you have no experience of C++ coding, start this project as early as possible. Programming proficiency is not determined by the courses you’ve taken, but rather by ample practice and persistent debugging.
7.  Submit your code and documents on Canvas before the deadline. Scores will be deducted for late submission.

### Resubmission Policy
Late submissions are accepted up to 7 days after the due date, with 3% (of the total grade of the item) penalty per day.


## References

[1] https://registry.hkust.edu.hk/resource-library/academic-integrity

## Third-party Libraries references

[Eigen](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html): A C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

[PCL](https://pcl.readthedocs.io/projects/tutorials/en/master/): The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D image and point cloud processing.