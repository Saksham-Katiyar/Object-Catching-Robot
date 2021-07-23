# Object-Catching-Robot
This is simulation of a camera+robot system that is capable of detecting and catching small objects.<br>
The stationary camera detects the objects and predicts the trajectory of the object using motion detection using OpenCV, and then a wheeled robot is commanded to go at the desired location to catch the object.<br>
The simulation is achieved through **Gazebo**, the code is managed using **Robot operating system (ROS)** and the nodes of ROS are written in **Python**.

Simulation package includes the world files, model(vehicle) configuration files and launch file of the simulation.

Commander package includes the publisher and subscriber nodes for the robot.

## Installation
* Clone this repo inside the `src` folder of your ROS workspace
* Go back to the worksace directory and use `catkin_make` to build the package
* Use command `roslaunch simulation my_world.launch` to start the simulation of the robot and world. The camera module start publishing the raw data.
* Then in a new terminal, start the subsriber node using `rosrun commander detect_image.py` which then streams the camera input to a screen

## License
The MIT License (MIT) Copyright (c) 2020 Nils Rottmann

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE
