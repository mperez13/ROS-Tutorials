#Navigation Stack - Base Controller

Link - (http://wiki.ros.org/navigation/Tutorials/RobotSetup) 

- Navigation stack assumes it can send velocity commands using a `geometry_msgs/Twist` message assumed to be in the base coordinate frame of the robot in the "cmd_vel" topic
	- means there must be a node subscribing to the "cmd_vel" topic capable of 
		- taking (vx, vy, vtheta) <==> (cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z) velocities, and
		- converting them into motor commands to send to a mobile base

######Supported platforms:

- Videre Erratic: [erratic_player] (http://wiki.ros.org/erratic_player)

- PR2: [pr2_mechanism_controllers] (http://wiki.ros.org/pr2_mechanism_controllers)