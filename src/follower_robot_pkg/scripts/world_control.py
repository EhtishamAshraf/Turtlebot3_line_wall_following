#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
                        --- Gazebo Model Animation Node ---

This ROS node controls the animation of a model within a Gazebo simulation environment, specifically moving the model up and down.

The node performs the following functions:

1. **Initializes the ROS Node**: 
   - The node is named 'gazebo_models_animation'.
   
2. **Publisher Setup**:
   - Publishes to the `/gazebo/set_model_state` topic to control the state of the model in Gazebo. 
   - A `ModelState` message is used to set the position, orientation, and velocity of the model.
   - The `latch=True` option ensures that the last message is saved and sent to any future subscribers, which is useful for static data.

3. **Initial Model Setup**:
   - A `ModelState` message is initialized to represent the state of the model named "Portail". 
   - The current state of all models is retrieved from the `/gazebo/model_states` topic to locate the "Portail" model and store its index for further reference.

4. **Main Loop**:
   - The loop continuously updates the model's position in Gazebo until the node is shut down.
   - **Stay Command**: 
     - The model's position is maintained by publishing its current position with zero linear velocity in the z-axis (vertical direction).
   - **Moving Away**: 
     - The model is moved upward by setting a positive linear velocity in the z-axis.
   - **Moving Closer**: 
     - The model is moved downward by setting a negative linear velocity in the z-axis.

5. **Position Smoothing**:
   - To prevent sudden jumps in the model’s position, the current position is always retrieved from `/gazebo/model_states` and used in subsequent position updates.

6. **Exception Handling**:
   - The node gracefully handles shutdowns and interruptions using a `try-except` block around the main function.

7. **Usage Context**:
   - This node can be used to animate any model within a Gazebo simulation environment, by simply changing the model name in the script.

"""

import rospy
from gazebo_msgs.msg import ModelState, ModelStates

if __name__ == '__main__':
    try:
        # initializing the node
        rospy.init_node('gazebo_models_animation', anonymous=False)

        # Lets define a publisher on the topic_name /gazebo/set_model_state (Twist message)
        # When a connection is latched, a reference to the last message published is saved and sent to any future subscribers that connect. 
        # This is useful for slow-changing or static data like a map. 
        pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1, latch=True) 
        msg = ModelState()
        msg.model_name = "Portail"
        rospy.sleep(1)

        current_state = rospy.wait_for_message('/gazebo/model_states', ModelStates)

        # Find the index of the model in the list of models published on the topic model_states
        index = current_state.name.index(msg.model_name)

        while not rospy.is_shutdown():

            rospy.loginfo("Stay")
            current_state = rospy.wait_for_message('/gazebo/model_states', ModelStates)   # store the current position to set it in the next published message in order to avoid jumps in position.
            msg.pose = current_state.pose[index]
            msg.twist.linear.z = 0.0
            pub.publish(msg)
            rospy.sleep(20)

            vitesse=0.5
            rospy.loginfo("Moving Away")
            current_state = rospy.wait_for_message('/gazebo/model_states', ModelStates)   # store the current position to set it in the next published message in order to avoid jumps in position.
            msg.pose = current_state.pose[index]
            msg.twist.linear.z = vitesse
            pub.publish(msg)
            rospy.sleep(1)

            rospy.loginfo("Stay")
            current_state = rospy.wait_for_message('/gazebo/model_states', ModelStates)   # store the current position to set it in the next published message afin d'eviter les sauts de position
            msg.pose = current_state.pose[index]
            msg.twist.linear.z = 0.0
            pub.publish(msg)
            rospy.sleep(10)

            rospy.loginfo("Moving Closer")  
            current_state = rospy.wait_for_message('/gazebo/model_states', ModelStates) # store the current position to set it in the next published message afin d'eviter les sauts de position
            msg.pose = current_state.pose[index] 
            msg.twist.linear.z = -vitesse
            pub.publish(msg)                       
            rospy.sleep(1)
            
    except rospy.ROSInterruptException:
        pass
