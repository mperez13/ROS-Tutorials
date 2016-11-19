#include <ros/ros.h>
/*Need since we are going to be publishing both a transfrom from the "odom" coordinate frame to the "base_link" coordinate frame and a nav_msgs/Odometry message*/
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  /*need to create both a ros::Publisher and a tf::TransformBroadcaster to be able to 
  	send messages out using ROS and tf*/
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  /*Assume robot starts at the origin of the "odom" coordinate frame initially*/
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  /*Set velocities that will cause the "base_link" frame to move in the "odom" frame 
  	at a rate of 0.1 m/s in the x-direction, -0.1 m/s in the y-direction & 0.1 rad/s 
  	in the th direction
  	This will cause fake robot to drive in a circle. */
  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  /*odometry info is published at 1Hz in ta his example (most systems will want to 
  	publish at a much higher rate)*/
  ros::Rate r(1.0);
  
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    /* Update odometry info based on constant velocities we set. (real system would
    	integrate computed velocities instead. */
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    /* Try to use 3D versions of all msgs in our system to allow 2D & 3D components
    	to work together appropriate & to keep # of msgs we have to create to a min.
       Necessary to convert our yaw value for odometry into a Quaternion to send over 
       	wire.
       tf provides functions allowing easy creation and access of Quarternions from 
       yaw values */


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    /*	Create TransformStamped message that will send out over tf.
    	want to publish transform from "odom" frame to "base_link" frame at 
    		current_time
    	therefore, set the header of the message and the child_frame_id accordingly, 
    		making sure to use "odom" as the parent coordinate frame and "base_link" 
    		as the child coordinate frame. */
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    /* fill in the transform message from our odometry data, and then send the 
    	transform using our TransformBroadcaster*/
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS to get velocity info from it
    /* Set header of msg to current_time & "odom" coordinate frame*/
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    /* set the position, which will populate msg w/ odometry data & send it out over 
    	the wire */
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    /* set the velocity; set child_frame_id to "base_link" frame since that's the 
    	coordinate frame we're sending velocity info in */
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}