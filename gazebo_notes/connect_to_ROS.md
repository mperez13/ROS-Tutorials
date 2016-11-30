#Connect to ROS

Link to Tutorial - http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i6

##Add ROS transport

Will need to modify our current plugin to include the ROS transport mechanism, like in the previous tutorial.

1. Add a header files to `velodyne_plugin.cc` file.

  ```
  #include "ros/ros.h"
  #include "ros/callback_queue.h"
  #include "ros/subscribe_options.h"
  #include "std_msgs/Float32.h"
  ```
2. A a few member variables to the plugin

  ```
  // A node use for ROS transport
  private: std::unique_ptr<ros::NodeHandle> rosNode;
  
  // A ROS subscriber
  private: ros::Subscriber rosSub;
  
  // A ROS callbackqueue that helps process messages
  private: ros::Callbackqueue rosQueue;
  
  // A thread the keeps running the rosQueue
  private: std::thread rosQueueThread;
  ```
3. At end of `Load` function add:

  ```
  // Initialize ros, if it has not already been initialized.
  if (!ros::isInitialized()){
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so = 
    ros::SubscribeOptions::create<std_msgs::Float32>("/" + this->model->GetName() + "/vel_cmd",
                                                      1, boost::bind(&VelodynePlugin::OnRosMsg, this, _1), 
                                                      ros::VoidPtr(), &this->rosQueue);
  
  this->rosSub = this->rosNode->subscribe(so);

  // Spin up the queue helper thread.
  this->rosQueueThread = std::thread(std::bind(&VelodynePlugin::QueueThread, this));
  ```
