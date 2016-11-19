//Using a Transform - Setting up your robot using tf
/*Write a node that will transform to take a point in the "base_laser" frame & transform it to 
  a point in the "base_link" frame*/
#include <ros/ros.h> 
#include <geometry_msgs/PointStamped.h>
/*A transformListener object automatically subscribes to the transform message topic over ROS and manages all 
  tranform data coming in over the wire*/
#include <tf/transform_listener.h> 

/*Function transform point takes a point in the "base_laser" frame and transforms it to the "base_link" frame
  Function serves as a callback for the ros::Timer created in the main() of our program & will fire every second*/
void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;

  /*Want to transform it into the "base_link" frame*/
  try{
    geometry_msgs::PointStamped base_point;
    //transformPoint() arguments: name of the frame we want to transform, point we're transforming, storage for the transformed point
    listener.transformPoint("base_link", laser_point, base_point);

    //now base_point holds same info as laser_point did before only now in the "base_link frame"

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  } 
  catch(tf::TransformException& ex){
    //if base_laser --> base_link transform is unavailable it might be because tf_broadcaster is not running
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}
int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}