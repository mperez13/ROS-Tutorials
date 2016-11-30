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
                                                      1, boost::bind(&VelodynePlugin::OnRosMsg, 
                                                                      this, _1), 
                                                      ros::VoidPtr(), &this->rosQueue);
  
  this->rosSub = this->rosNode->subscribe(so);

  // Spin up the queue helper thread.
  this->rosQueueThread = std::thread(std::bind(&VelodynePlugin::QueueThread, this));
  ```
4. Notice from previous code we need 2 new functions: `onRosMsg` & `QueueThread`

   ```
   // Handle an incoming message from ROS
   // _msg: A float value that is used to set the velocity of the Velodyne
   public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg){
      this->SetVelocity(_msg->data);
   }

   // \brief ROS helper function that processes messages
   private: void QueueThread(){
      static const double timeout = 0.01;
      while (this->rosNode->ok()){
          this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
   }
   ```
5. Modify `CMakeLists.txt` to look like:
  
  ```
  cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

  find_package(roscpp REQUIRED)
  find_package(std_msgs REQUIRED)
  include_directories(${roscpp_INCLUDE_DIRS})
  include_directories(${std_msgs_INCLUDE_DIRS})

  # Find Gazebo
  find_package(gazebo REQUIRED)
  include_directories(${GAZEBO_INCLUDE_DIRS})
  link_directories(${GAZEBO_LIBRARY_DIRS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

  # Build our plugin
  add_library(velodyne_plugin SHARED velodyne_plugin.cc)
  target_link_libraries(velodyne_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

  # Build the stand-alone test program
  add_executable(vel vel.cc)

  if (${gazebo_VERSION_MAJOR} LESS 6)
    include(FindBoost)
    find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
    target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
  else()
    target_link_libraries(vel ${GAZEBO_LIBRARIES})
  endif()
  ```
6. Recompile the plugin
  
  ```
  cd ~/velodyne_plugin/build
  cmake ../
  make
  ```

##Control Velodyne from ROS




