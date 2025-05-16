Our robotics project to get the thymio robot to the endzone whilst avoiding obstacles. For this project we obtained a grade of 6/6 and this is the final result.

The goal of the project was to make the thymio reach the endgoal and to avoid the black obstacles. In order to do so we created a graph from the corners of our obstacles
to then use a simple path finding algorithm to find the shortest path. We used a tweaked PID controller to get the thymio to reach the next point and we used a kallman filter to estimate the pose of the robot when the camera was obstructed.
The Kallman was also useful to know whether the thymio had been moved about. We also implemented a local avoidance system to handle invisible obstacles to the camera.

![Video of the thymio in the project](thymio_final.gif)