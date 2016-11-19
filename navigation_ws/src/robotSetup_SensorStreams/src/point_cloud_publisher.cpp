#include <ros/ros.h>
/*include sensor_msgs/PointCloud*/
#include <sensor_msgs/PointCloud.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "point_cloud_publisher");

  ros::NodeHandle n;
  /*ros::Publisher will be used to send PointCloud messages out over the wire*/
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);

  unsigned int num_points = 100;

  int count = 0;
  ros::Rate r(1.0);
  while(n.ok()){
    /*Fill in the header of the PointCloud message to be sent out w/ relevant frame & timestamp info*/
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "sensor_frame";

    /*Set # of points in the point cloud so we can populate w/ dummy data*/
    cloud.points.resize(num_points);

    //we'll also add an intensity channel to the cloud
    /*size it to match the # of points that we'll have in the cloud*/
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);

    //generate some fake data for our point cloud
    /*Also populate intensity channel w/ dummy data*/
    for(unsigned int i = 0; i < num_points; ++i){
      cloud.points[i].x = 1 + count;
      cloud.points[i].y = 2 + count;
      cloud.points[i].z = 3 + count;
      cloud.channels[0].values[i] = 100 + count;
    }

    /*Publishes PointCloud using ROS*/
    cloud_pub.publish(cloud);
    ++count;
    r.sleep();
  }
}