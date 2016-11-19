#Publish Sensor streams Over ROS Tutorials
Link to tutorial (http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors

- Current navigation stack only accepts sensor data published using:
  - sensor_msgs/LaserScan Message type or 
  - sensor_msgs/PointCloud Message types
  
###ROS Message Headers

- `sensor_msgs/LaserScan` Message type and `sensor_msgs/PointCloud` Message types contain tf frame and time dependent informaion
- To standarize how this info is sent, `Header` message type is used as a field in all such messages.
    - `Header` has three fields:
        1. `seq` field: identifier that automatically increases as Messages are sent from a given publisher
        2. `stamp` field: stores info that should be associated w/ data in a Message
          - ex: for laser scan, stamp might correspond to time when scan was taken
        3. `frame_id` field: stores tf frame info associated w/ data in a message
          - ex: for laser scan, this would be set to the frame where the scan was taken

###LaserScans

- ROS provides Message type in sensor_msgs package called `LaserScan` to hold info about a given scan
- `LaserScan` Messages make it easy for code to work w/ virtually any laser as long as the data coming back from scanner can be formatted to fit into the message
- Message Specification:

  ```
  # Laser scans angles are measured counter clockwise, with 0 facing forward
  # (along the x-axis) of the device frame
  #
  Header header
  float32 angle_min        # start angle of the scan [rad]
  float32 angle_max        # end angle of the scan [rad]
  float32 angle_increment  # angular distance between measurements [rad]
  float32 time_increment   # time between measurements [seconds]
  float32 scan_time        # time between scans [seconds]
  float32 range_min        # minimum range value [m]
  float32 range_max        # maximum range value [m]
  float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
  float32[] intensities    # intensity data [device-specific units]
  ```
#####Writing Code to Publish a LaserScan Message

- Code for [LaserScan Message Publisher](https://github.com/mperez13/ROS-Tutorials/blob/master/navigation_ws/src/robot_setup_sensor_streams/src/laser_scan_publisher.cpp) 

###PointClouds

- `sensor/PointCloud` message: used for storing and sharing data a number of points in the world
- message (shown below) meant to support arrays of points in 3D along w/ any associated data stored as a channel
  -ex: `PointCloud` can be sent over the wire w/ "intesity" hannel that hold info about intensity value of each point in the cloud
  ```
  #This message holds a collection of 3d points, plus optional additional information about each point.
  #Each Point32 should be interpreted as a 3d point in the frame given in the header
  
  Header header
  geometry_msgs/Point32[] points  #Array of 3d points
  ChannelFloat32[] channels       #Each channel should have the same number of elements as points array, and the data in each channel should correspond 1:1 with each point
  ```

#####Writing Code to Publish a PointCloud Message

- Code for [PointCloud Message Publisher](https://github.com/mperez13/ROS-Tutorials/edit/master/navigation_ws/src/robot_setup_sensor_streams/src/point_cloud_publisher.cpp)
