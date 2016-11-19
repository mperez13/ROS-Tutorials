// Setting up your robot using tf
//BROADCASTING A TRANSFORM
#include <ros/ros.h>
/*implementation of TransformBroadcaster helpd publish transforms easier*/
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  /*create TransformBroadcaster object to use later to send base_link -> base_laser 
  transform over the wire*/
  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
  	/*Sending a transform requires five arguments:
  			1. pass in the rotation specified by btQuaterion
  				-in this case: want to apply no rotation so send in btQuaterion constructucted 
  					from pitch, roll, yaw values equal to zero
  			2. btVector3 for any translation that we'll like to apply
  				- create btVector3 corresponding to the laser's x offset of 10cm and z offset 
  					of 20cm from robot base
  			3. need to give transform being published a timestamp: ros::Time:now()
  			4. need to pass name of parent node of the link created (base_link)
  			5. need to pass name of child node of the link created (base_laser_*/
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "base_laser"));
    r.sleep();
  }
}