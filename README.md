# Balance Bot

For this project, I implemented a cascaded PID controller to stabilize an inverted pendulum style robot. A functional block diagram of the controller is shown below:

![Controls Block Diagram](controlblock.png)

The user gives a desired velocity and/or rate of rotation with a remote controller. This reference input is integrated over time to give a reference position to the outer PID loop, which itself generates a reference body angle for the robot to acheive. This allows the robot to oscillate about an equilibrium point. The backlash present in the gearboxes used meant that the robot would not attain a perfectly stationary position.

![Outer Loop Controller](outer_loop_control.gif)

All of the code for controlling the robot was written in C, and compiled and run on a BeagleBone Green board. Our team of three each developed on independent branches, and merged to master after verifying the code's performance on the robot.

![Trajectory following plot](sqpath.png)

