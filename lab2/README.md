Using Laser Range Finder to perform wall following

This project has two goals:
1. Use Laser Range Finder's output, to detect lines. This is performed using RANSAC algorithm.
2. Perform Wall Following using the Laser Range Finder. And also, move towards goal when the wall doesn't occlude. This is Bug 2 algorithm.

How to run:
1. Add the project to the catkin workspace
2. catkin_make
3. roslaunch perception.launch (launches robot and RViz visualization of the line detected)
4. roslaunch bug2.launch (launches robot that performs goal seeking and following wall - Bug 2)
