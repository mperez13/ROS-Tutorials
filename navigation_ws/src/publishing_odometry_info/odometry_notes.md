#Publishing Odometry Information over ROS

Link to Tutorial - (http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)

**Description**: This tutorial covers both publishing the nav_msgs/Odometry messages over ROS, and a transform from a "odom" coordinate frame to a "base_link" coordinate frame over tf.

- tf does not provide any info about the velocity of the robot
- so navigation stack requires that any odometry source publish both a transform and a `nav_msgs/Odometry` message over ROS that contains velocity info.

##The nav_msgs/Odometry Message

- `nav_msgs/Odometry` message stores an estimate of the position and velocity of a robot in free space:

  ```
  # This represents an estimate of a position and velocity in free space.  
  # The pose in this message should be specified in the coordinate frame given by header.frame_id.  
  # The twist in this message should be specified in the coordinate frame given by the child_frame_id
  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
  ```
