# Project 3 - A* Path Planning

## Introduction

This project is to implement the A* path planning algorithm to find the path from the start point to the end point on a 3D grid map. The project is implemented in C++ with ROS.

### Instruction

The grid path searcher pipeline is implemented in the `grid_path_searcher/src/demo_node.cpp` file. `rcvWaypointsCallback` is the callback function to receive the goal point from the rviz interface. `rcvPointCloudCallBack` is the callback function to receive the random map with obstacles from the `random_complex_generator` node. `pathFinding` is the entry function to find the path from the start point to the end point.

The A* algorithm is implemented in the `grid_path_searcher/src/Astar_searcher.cpp` file. `AstarPathFinder::AstarGraphSearch` is the main loop of the A* algorithm. `AstarPathFinder::getHeu` is the heuristic function to estimate the cost from the current point to the goal point. `AstarPathFinder::AstarGetSucc` is the function to get the successors of the current point. Mainly, you need to implement theses three functions to complete the A* algorithm. There are `TODO` marks in the code to guide you to implement the A*.

### grid_path_searcher

```bash
cd /ws
catkin_make
source devel/setup.bash
roslaunch grid_path_searcher demo.launch
```

Set Navigation Goal in Rviz:

Click the `3D Nav Goal` button in the toolbar, then click and drag on the map to set the goal point.

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240828200134.png)

Then you can see the path from the start node to the end node in the rviz with green blocks, and the visited nodes with blue blocks.

In the terminal, you can see the path length, visited node number, and the time cost of the A* algorithm.

### Comparison with different heuristic functions

You can compare the performance of the A* algorithm with different heuristic functions. The heuristic function is defined in the `AstarPathFinder::getHeu` function. You can implement your own heuristic function to compare with the following heuristic functions:

1. Manhattan distance
2. Euclidean distance
3. Diagonal distance
4. Dijkstra's algorithm (0)

## Scoring Rules
1. Please title your submission "PROJECT3-YOUR-NAMES.zip". Your submission should include a maximum 2-page report and all necessary files to run your code, a video in 1 minute.
2. The report should contain visualization of your path planning result with different heuristic functions.
3. If your code is similar to others’ or any open-source code on the internet, the score will be 0 unless the sources are properly referenced in your report. Refer to $^1$ for details.
4. Excessive white space, overly large images, or redundant descriptions in report will not contribute to a higher score. Simple but efficient code is preferred.
5. If any problem, please google first. If unsolved, create a discussion on Canvas.
6. If you have no experience of C++ coding, start this project as early as possible. Programming proficiency is not determined by the courses you’ve taken, but rather by ample practice and persistent debugging.
7.  Submit your code and documents on Canvas before the deadline. Scores will be deducted for late submission.

## References

[1] https://registry.hkust.edu.hk/resource-library/academic-integrity