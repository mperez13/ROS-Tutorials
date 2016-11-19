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

