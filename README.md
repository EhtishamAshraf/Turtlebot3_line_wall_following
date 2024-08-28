# Line and Wall Following using Turtlebot3
This repository features a custom ROS package, follower_robot_pkg, designed to enable Turtlebot3 to autonomously navigate through a corridor by following walls and avoiding collisions. Leveraging Lidar sensor data, the robot detects and tracks walls (treated as obstacles) to maintain a safe path. Additionally, the package equips Turtlebot3 with the ability to follow a line using camera sensor data, utilizing the cv_bridge package to process images and detect lines.

### Demo Video
You can watch the demo video by clicking on the below image
[![Watch the video](https://github.com/EhtishamAshraf/Turtlebot3_line_wall_following/blob/main/src/follower_robot_pkg/Images/2.png)](https://www.youtube.com/watch?v=d6_b9ii4WU4&t=2s)

## Gazebo World
Below image shows the Gazebo world used in this project. The world contains walls (blue), with a yellow color obstacle (door), and green line.
![Gazebo World](https://github.com/EhtishamAshraf/Turtlebot3_line_wall_following/blob/main/src/follower_robot_pkg/Images/1.png)

### Note 
1.  Details about cloning the repository are given at the end of this **readme file**

## Wall Following Logic
- If an obstacle is detected directly in front of the robot, robot should stop and wait for the wall to move up or clear. Otherwise, the robot 
  continues moving forward. 
- If the left wall is closer than the right wall and the distance between the two walls exceeds a certain threshold, the robot will rotate away 
  from the left wall to avoid a collision. 
- Similarly, if the right wall is closer than the left and the distance between them is too large, the robot will rotate away from the right 
  wall to prevent hitting it. 
- If the left wall ends (indicated by an infinite distance value), the robot will move forward slightly to re-align itself with the line for 
  following. 
- In all other cases, the robot continues to move forward.
![Wall Following](https://github.com/EhtishamAshraf/Turtlebot3_line_wall_following/blob/main/src/follower_robot_pkg/Images/3.png)

If the door stops while the robot is following the walls, the robot stops and waits for the door to open.
![Door Closed](https://github.com/EhtishamAshraf/Turtlebot3_line_wall_following/blob/main/src/follower_robot_pkg/Images/4.png)

## Line Following Logic
- Convert the RGB image to HSV format and define the HSV range to detect the green color line in the image. 
- A masked image is created to highlight the detected green line, and contours are then identified within this masked image.
- If contours are detected, the largest contour is selected based on area, and a bitwise_and operation is applied to extract the green line from 
  the image.
- For visualization, the contour is drawn, and the moments of the largest contour are calculated to determine its centroid.This centroid is     
  highlighted by drawing a circle on it.
- The error is computed as the difference between the camera's center and the x-position of the centroid, and this error is used to calculate 
  the PID controller output, which guides the robot to follow the line efficiently.

## Running the Simulation
To run the simulation, launch the wall_line_following launch file, In order to launch the launch file (you should first navigate inside the launch folder of the package and then, use the following command): 
```bash
roslaunch wall_line_following.launch
```

## Create Ros Workspace
Open shell and execute the following commands:
```bash
mkdir wall_line_following_ws
```
```bash
cd wall_line_following_ws
```
# Clone the repository
```bash
sudo apt-get update
```
```bash
sudo apt-get install git
```
```bash
git clone https://github.com/EhtishamAshraf/Turtlebot3_line_wall_following.git
```
```bash
cd Turtlebot3_line_wall_following
```
Run the below commands in root folder of the workspace
```bash
catkin_make 
```
```bash
source devel/setup.bash 
```
Navigate to the Scripts folder inside the package and make the Python files executable by running the following commands:
```bash
chmod +x wall_line_following.py
chmod +x world_control.py
```

Press Enter and navigate to the launch folder inside the package
```bash
roslaunch wall_line_following.launch
```
