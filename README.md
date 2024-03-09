# **Astorino - ROS2 Object Detection for Pick-and-Place Tasks**

This project uses Cascade and SVM to detect objects. The objects are then localized and pick-and-place tasks are performed. The target poses are sent to the real robot for controlling it to reach desired targets.

All the packages are written in ROS2 Humble

## **Table of Contents**
<hr>

* [Requirements](#requirements)
* [ROS2 Packages](#ros2-packages)
    * [send_to_astorino](#send_to_astorino)
    * [astorino_detect](#astorino_detect)
    * [astorino_tf_transform](#astorino_tf_transform)
    * [astorino_planning](#astorino_planning)
    * [astorino-urdf_gripper_description](#astorino-urdf_gripper_description)
    * [astorino_gripper_moveit_config](#astorino_gripper_moveit_config)
* [Teensy Code](#teensy-code)
* [Bag Files](#bag-file)
* [Usage](#usage)
<hr>

## **Requirements**
In order to use the packages in this repo, there are some packages that need to be installed first

* [micro-ROS](https://micro.ros.org/) for creating a connection between ROS2 and a microcontroller (in this project, Teensy 4.1 is used)
* [MoveIt](https://moveit.picknik.ai/) for performing motion planning tasks
* [IntelRealSense](https://github.com/IntelRealSense/realsense-ros) for using Intel ReaSense Camera with ROS

<hr>


## **ROS2 Packages**
<hr>

## send_to_astorino
This package is used to make ROS2 communicate with the Teensy 4.1 by sending and receiving ROS messages

<details>
    <summary>
        Topics 
    </summary>

### Topics
* **/enable_motors** *(std_msgs/msg/Bool)*: enable/disable motors
* **/open_gripper** *(std_msgs/msg/Bool)*: open/close gripper
* **/rotate_motors** *(std_msgs/msg/Float64MultiArray)*: send the desired targets to the robot
</details>

<details>
    <summary>
        Nodes 
    </summary>

### Nodes
* **enable_motors**: to enable or disable motors
    * `ros2 run send_to_astorino enable_motors true` to enable
    * `ros2 run send_to_astorino enable_motors false` to disable 
* **open_gripper**: to open or close gripper
    * `ros2 run send_to_astorino open_gripper true` to open
    * `ros2 run send_to_astorino open_gripper false` to close
* **check_movement**: to move to specific joints' value through command line. It should be run before other running operations to check whether the joints' order is correct. For example:
    * `ros2 run send_to_astorino check_movement 5 0 0 0 0 0` move all the joints form 1 to 6 to [5 0 0 0 0 0] 
* **move_to_turn_off_position**: to move the robot to the turn-off position. Robot should move to this position before turning off
    * `ros2 run send_to_astorino move_to_turn_off_position`
* **move_to_zero_position**: to move the robot to the zero position, which makes the robot fully stretched upward
    * `ros2 run send_to_astorino move_to_zero_position`
* **move_as_planning**: to move the robot to the position as planning in MoveIt. This position is from the `/joint_states` topic from MoveIt
    * `ros2 run send_to_astorino move_as_planning`

</details>

<hr>

## astorino_detect
This package is used to perform object detection and extract the position of detect object in camera frame

<details>
    <summary>
        Topics 
    </summary>
    
### Topics
* **/camera/color/image_raw** *(sensor_msgs/msg/Image)*: color image from the camera
* **/camera/aligned_depth_to_color/image_raw** *(sensor_msgs/msg/Image)*: depth information from the camera, which is aligned to the color camera frame
* **/detected** *(sensor_msgs/msg/Image)*: send the result image which includes the bounding boxes covering detected objects
* **/pose_stamped** *(geometry_msgs/msg/PoseStamped)* or *(geometry_msgs/msg/PoseArray)*: send the result pose of detected objects in camera frame

</details>

<details>
    <summary>
        Nodes 
    </summary>
### Nodes
* **cascade**: to detect and localize only one object
    * `ros2 run astorino_detect cascade` 

* **cascade_multiple**: to detect and localize only multiple objects
    * `ros2 run astorino_detect cascade_multiple` 
</details>

<hr>

## astorino_tf_transform
This package is used to perform the transformation the pose of detected objects in camera frame to robot base frame

<details>
    <summary>
        Topics 
    </summary>
    
### Topics

   

* **/pose_stamped** *(geometry_msgs/msg/PoseStamped)* or *(geometry_msgs/msg/PoseArray)*: pose of detected objects in camera frame
* **/pose_to_move** *(geometry_msgs/msg/PoseStamped)* or *(geometry_msgs/msg/PoseArray)*: send the pose detected objects in robot base frame (or world frame) to MoveIt for motion planning

</details>


<details>
    <summary>
        Nodes
    </summary>
### Nodes

* **pose_stamped_to_world**: to transform the pose of only one object
    * `ros2 run astorino_tf_transform pose_stamped_to_world` 

* **multiple_pose_stamped_to_world**: to transform the pose of multiple object
    * `ros2 run astorino_tf_transform multiple_pose_stamped_to_world`
</details>
<hr>

## astorino_planning
This package is used to perform motion planning tasks using MoveIt

<details>
    <summary>
        Topics
    </summary>

### Topics
* **/pose_to_move** *(geometry_msgs/msg/PoseStamped)* or *(geometry_msgs/msg/PoseArray)*: the target poses for planning
* **/response** *(std_msgs/msg/Bool)*: the execute_multiple node subscribes to this topic to know whenever the an action (move motor, open gripper, etc.) is finished on the real robot. Only after that, the node will continue to plan the next movement. It can be disable in the execute_multiple node. 

</details>

<details>
    <summary>
        Nodes
    </summary>

### Nodes
* **move_to_a_specific_position**: to plan the robot to a target pose defined in the code
    * `ros2 run astorino_planning move_to_a_specific_position`

* **move_command_line**: to plan the robot to a target pose defined through command line (the orientation is based on quaternion). For example
    * `ros2 run astorino_planning move_command_line x y z q.x q.y q.z q.w`: x y z are the position, q.x q.y q.z q.w is the orientation in quaternion form 

* **execute**: to execute motion planning to perform pick-and-place task for only one object
    * `ros2 run astorino_planning execute`
* **execute_multiple**: to execute motion planning to perform pick-and-place task for multiple objects as well as visualizing in Rviz. There are some lines related to the */response* topic that can be disabled
    * `ros2 run astorino_planning execute_multiple`

</details>

<hr>

## astorino-urdf_gripper_description
This package contain urdf model for the Astorino Robot

<hr>

## astorino_gripper_moveit_config
This package is generated by MoveIt Setup Assistant. Only 2 lanch files are added: one contains the information about the pose of camera in the robot base frame. The other one is just the file including both demo launch file and the camera_pose launch file. To use this package, launching the main launch file is enough

`ros2 launch astorino_gripper_moveit_config main.launch.py`

<hr>

## **Teensy Code**
The code using for Teensy is included in the astorino_Arduino_code folder

<hr>

## **Bag File**
There is also a bag file in the bag_files folder. It recorded all the information from the RealSense camera for about 10s. The bag file can be use for testing the algorithm.
<hr>

## **Usage**

Run and launch the below lines in the corresponding order if having a RealSense Camera:
<details>
    <summary>
        Have a carema
    </summary>

* `ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true depth_module.profile:=1280x720x30 rgb_camera.profile:=1280x720x30` for running the Intel RealSense Camera and sending information to ROS2
* `ros2 run send_to_astorino move_as_planning` for continously sending the target signals from MoveIt to the Teensy 4.1
* `ros2 run send_to_astorino enable_motors true` for turning on the motors
* `ros2 launch astorino_gripper_moveit_config main.launch.py` for running the MoveIt motion planning, Rviz, and publishing the camera_pose position in robot base frame
* `ros2 run astorino_detect cascade_multiple` for detecting and localizing objects in the camera frame (do not forget to change the address to the model xml files)
* `ros2 run astorino_tf_transform multiple_pose_stamped_to_world` for transforming the pose of detected objects in camera frame to robot base frame
* `ros2 run astorino_planning execute_multiple` for performing motion planning for multiple-object pick-and-place tasks
</details>
<hr>

Run and launch the below lines in the corresponding order if using the recorded bag file, which include the camera's signal (for testing the algorithms or simulation):
<details>
    <summary>
        Use recorded bag file
    </summary>

* `ros2 launch astorino_gripper_moveit_config main.launch.py` for running the MoveIt motion planning, Rviz, and publishing the camera_pose position in robot base frame
* `ros2 run astorino_detect cascade_multiple` for detecting and localizing objects in the camera frame (do not forget to change the address to the model xml files)
* `ros2 run astorino_tf_transform multiple_pose_stamped_to_world` for transforming the pose of detected objects in camera frame to robot base frame
* `ros2 bag play src/bag_files/camera1711/` for playing the recorded bag file
* `ros2 run astorino_planning execute_multiple` for performing motion planning for multiple-object pick-and-place tasks
</details>
