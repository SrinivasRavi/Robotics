Evader and pursuer robots

This project has two robots in Stage Simulator each with a lazer range finder.

Evader robot travels straight with a constant speed unless it's close to an obstacle - it stops, rotates and moves in a random new direction. 

Pursuer robot follows the evader robot using it's transformations. tf package is used to publish co-ordinate frame message from the evader and the pursuer subscribes to the messages and moves towards the pursuer.

How to run:
1. Add the project to the catkin workspace
2. catkin_make
3. roslaunch evader.launch (launches just the evader robot)
4. roslaunch pursuer-evader.launch (launches both the evader and pursuer robots)
