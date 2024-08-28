#!/usr/bin/env python3

"""
                        --- Wall and Line follower Robot ---
This ROS node allows the Turtlebot3 Burger Robot to follow the walls and line.

The node performs the following functions:
1. **Subscribes to the `/scan` topic, where it receives data about obstacles in the environment. 
   Robot follows the corridor without colliding with the walls based on the lidar sensor data.
   
2. **Subscribes to the `/camera/cimage` topic, where it receives image data. 
   Robot follows the wall using the image data.
   use rqt_image_view tool to figure out the exact topic where camera feed is being published.

3. **Publishes on the `/cmd_vel` topic, where velocity commands are published. 
   Robot moves based on the vvelocity commands.

4. **OpenCV integration - Line Following**:
   - Once the line is detected, openCV is used for line following.
   - After detecting the line, a contour is drawn on the line and it's centroids are found out.
   - The robot then adjusts it's movement using PID controller to follow the line efficiently.

"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# PID components for Line following:
previous_error = 0
integral = 0
derivative = 0

# Callback function to process data published on /scan topic
def scanCallback(scan_range):
    
    """
    Convert the ranges data from the LaserScan message to a NumPy array.
    Find the total number of distance measurements in the scan data.
    Find the smallest value of distance measurement in the scan data
    Find the smallest value of distance measurement in the range [355:360] - the front of the robot
    Find the smallest value of distance measurement in the range [0:70]    - the front left of the robot
    Find the smallest value of distance measurement in the range [270:340] - the front right of the robot
    Find the smallest value of distance measurement in the range [0:50]    - the front left of the robot
    """
    ranges = np.array(scan_range.ranges)
    length = len(ranges)
    smallest_dist = np.min(ranges)
    front_min_dist = np.min(ranges[355:360])
    front_left_min_dist = np.min(ranges[0:70])
    front_right_min_dist = np.min(ranges[270:340])
    front_side_dist = np.min(ranges[0:50])
    
    # Displaying information on the terminal:
    rospy.loginfo("Total number of measurements are: %d", length) 
    rospy.loginfo("Smallest distance is: %f", smallest_dist)
    rospy.loginfo("Smallest distance in front (355° to 360°): %f", front_min_dist)
    rospy.loginfo("Smallest distance to the front left (0° to 70°): %f", front_left_min_dist)
    rospy.loginfo("Smallest distance to the front right (270° to 340°): %f", front_right_min_dist)
    rospy.loginfo("Smallest distance to the front left (0° to 50°): %f", front_side_dist)
        
    twist = Twist() # creating an instance of twist message
    
    """
    Wall Following logic:
    
    If:    there's an obstacle in front of the robot, it should stop and wait for the wall {to go up}.
    Else:  Robot keeps moving forward.
        If:   left wall is more closer than the right wall & distance b/w both walls is greater than a certain value.
            -  rotate in the opposite direction to avoide hitting the wall 
        Elif: right wall is more closer than the left wall & distance b/w both walls is greater than a certain value.
            -  rotate in the opposite direction to avoide hitting the wall
        Elif: when the left wall ends (value is inf), robot moves forward a little to catch the line to follow it.
        Else: move forward            
    """
    
    if front_min_dist <= 0.15 or smallest_dist >= 0.5:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        rospy.loginfo("Obstacle ahead - Stop")
        
    else:
        twist.linear.x = 0.125
        twist.angular.z = 0.0
        rospy.loginfo("Path cleared - Moving Forward")
        
        if front_right_min_dist <= front_left_min_dist and (front_left_min_dist - front_right_min_dist) >= 0.03:
            twist.linear.x = 0.125
            twist.angular.z = 0.65
            rospy.loginfo("Wall on Right - Moving Left")
        elif front_left_min_dist < front_right_min_dist and (front_right_min_dist - front_left_min_dist) >= 0.03:
            twist.linear.x = 0.125
            twist.angular.z = -0.65
            rospy.loginfo("Wall on Left - Moving Right")
        elif abs(front_side_dist - front_right_min_dist) >= 1000:
            twist.linear.x = 0.5
            twist.angular.z = 0
            rospy.loginfo("Path cleared - Moving Forward")
        else:
            twist.linear.x = 0.125
            twist.angular.z = 0.0
            rospy.loginfo("Path cleared - Moving Forward")
    
    pub.publish(twist) # Publish the twist message to the /cmd_vel topic, to control the robot's movements

# Callback function to process data published on /camera/image topic
def ImageCallback(image_msg):
    bridge = CvBridge()
    try:
        rgb_image = bridge.imgmsg_to_cv2(image_msg, "bgr8") # ros image message converted to rgb image
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return
    
    """
    Line following logic:
    - Convert RGB image to HSV
    - Define the HSV range for detecting green color line in the image
    - Create and display the masked image (highlighting the green detected line)
    - Find the contours in the masked image
    - If contours exist:
        - Get the biggest contour based on the area
        - apply bitwise_and to extract the green line in the hsv_image
        - draw the contour for visualization
        - find the moments of the largest contour
        - find the centroids of the largest contour from the moments
        - draw a circle to highlight the centroid 
        - calculate the error by finding the difference b/w camera_center and the x position of the centroid
        - Calculate the PID controller output based on the proportional, integral, and derivative terms
        - Follow the line using PID controller
    """
    
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV) 
    
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])

    mask = cv2.inRange(hsv_image, lower_green, upper_green)
    cv2.imshow("Masked Image", mask)
        
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    numContours = len(contours)
    rospy.loginfo("Total number of detected Contours are: %d", numContours)
    
    line = {} # creating a dictionary to store x and y position of the centroid
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
                
        green_line_mask = cv2.bitwise_and(rgb_image, rgb_image, mask = mask)
        cv2.imshow("Green line detected", green_line_mask)  
        
        cv2.drawContours(green_line_mask, [largest_contour], -1, (0, 0, 255), thickness=1)
        
        M = cv2.moments(largest_contour)
        line['x'] = int(M["m10"]/M["m00"])
        line['y'] = int(M["m01"]/M["m00"])
        
        cv2.circle(green_line_mask, (line['x'], line['y']), 3, (255, 0, 0), 5)
        cv2.imshow("Contours with Centroid on green line", green_line_mask)
        cv2.waitKey(1)  

        height, width, channels = green_line_mask.shape
        
        camera_center = width/2         # finding the center of the camera
        desired_center = line['x']
        error = float(desired_center - camera_center)
        
        global integral, derivative, previous_error
        
        # Accumulate the error over time to compute the integral term of the PID controller
        # Max value for the integral term
        # Clip the integral term to be within the range to avoid windup
        integral = integral + error 
        max_integral = 0.2          
        integral = max(min(integral, max_integral), -max_integral)

        # Taking difference of the current and previous error to compute the derivative term of the PID controller
        # Assigning error value to previous error value for next iteration
        derivative = error - previous_error 
        previous_error = error

        # Define the PID controller gains:
        # Kp: Proportional gain that determines how much the control output should respond to the current error.
        # Ki: Integral gain that accounts for accumulated past errors to eliminate steady-state error.
        # Kd: Derivative gain that predicts future error based on the rate of change, helping to dampen oscillations.        
        Kp = -0.05
        Ki = -0.0005
        Kd = -0.04
        
        proportional_component = Kp * error     # proportional component of PID
        Integral_component = Ki * integral      # integral component of PID
        Derivative_component = Kd * derivative  # derivative component of PID
        PID_controller = proportional_component + Integral_component + Derivative_component # PID controller
                
        rospy.loginfo("Error: {}, P_component: {}, I_component: {}, D_component: {}, PID: {}".format(error, proportional_component, Integral_component, Derivative_component, PID_controller))
        
        twist = Twist() # creating an instance of twist message
        
        # moving the robot accordingly
        twist.linear.x = 0.4
        twist.angular.z = PID_controller
    
        pub.publish(twist)  # Publish the twist message to the /cmd_vel topic, to control the robot's movements
    
def main():
    global pub
    
    rospy.init_node('wall_following', anonymous=True)                   # initializing rosnode
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)            # create a publisher for the '/cmd_vel'
    sub_lidar = rospy.Subscriber("/scan", LaserScan, scanCallback)      # create a subscriber for the '/scan'
    sub_camera = rospy.Subscriber("camera/image", Image, ImageCallback) # create a subscriber for the '/camera/image'
    
    rospy.spin()                                                        # Keep the node running until shut down

if __name__ == '__main__':
    try:
        main()                          # Call the main function to start the node
    except rospy.ROSInterruptException: # Handle the case where ROS is interrupted (e.g., by shutting down)
        pass