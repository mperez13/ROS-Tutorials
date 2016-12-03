#Topics

Link to documenation - http://wiki.ros.org/Topics

- are [named](http://wiki.ros.org/Names) buses over which [nodes](http://wiki.ros.org/Nodes) exchange [messages](http://wiki.ros.org/Messages)
- have anonymous publish/subscibe semantics, which decouples the production of information from its consumption
- are intended for unidirectional, streaming communication

- node are not aware who they are communicating with
  - nodes that are interested in data subscribe to the relevant topic
  - nodes that generate data publish to the relevant topic
- Node that need to perform remote procedure call, i.e. receive a response to a request, should use [services](http://wiki.ros.org/Services) instead

- There is also [Parameter Server](http://wiki.ros.org/Parameter%20Server) for maintaining small amounts of state

##Topic Types

- Each topic is strongly typed by the ROS messages type used to publish to it & nodes can only receive messages w/ a matching type
- [Master](http://wiki.ros.org/Master) does not enforce type consistency among publishers, but subscribers will not establish message transport unless the types match
- All ROS clients check to make sure that an MD5 computed from the [msg files](http://wiki.ros.org/msg) match
  - this checks to ensure that ROS nodes were compiled from consistent code bases

##Topic Transports

- ROS supports TCP/IP-based & UDP-based message transport
- TCP/IP-based transport aka [TCPROS](http://wiki.ros.org/ROS/TCPROS) 
  - streams message data over persistent TCP/IP connections
  - default transport used in ROS; is the only transport that client libraries are required to support
- UDP-based transport
  - known as [UDPROS](http://wiki.ros.org/ROS/UDPROS) & currently only supported in [roscpp](http://wiki.ros.org/roscpp)
  - separates messages into UDP packets
  - is low-latency, lossy transport, so it's best suited for tasks like teleoperation

- Nodes negotiate the desired runtime 
  - example: if node prefers UDPROS transport but other node does not support it, it can fall back on TCPROS transport.
  - This negotiation model enables new transports to be added over time as compelling use cases arise
  
##Topic Tools

- [rostopic](http://wiki.ros.org/rostopic) : command-line tool for interacting w/ ROS topics
- example:
    
    ```
    $ rostopic list
    ```
    will list:
    
    ```
    $ rostopic echo /topic_name
    ```
    will display Messages plubished to /topic_name. 
    
    For rostopic documentation: [HERE](http://wiki.ros.org/rostopic)

##Topic statistics

-  [roscpp](http://wiki.ros.org/roscpp) & [rospy](http://wiki.ros.org/rospy) offer integrated measurement of the following parameters for every connection:
  - period messages by all publishers (avg., max, standard deviation)
  - age of messages, based on header timestamp (avg, min, standard deviation)
  - # of dropped messages
  - traffic volume (Bytes)
- Measurements are performed on a window, that reiszed automatically depending on the number of messages published.

- Following command enables statistics measuremens (disabled by default; parameters have to be set before node is started):
    
    ```
    $ rosparam set enable_statistics true
    ```
- Then topic statistics are published on the topic/statistics (of type [rosgraph_msgs/TopicStatistics](http://docs.ros.org/api/rosgraph_msgs/html/msg/TopicStatistics.html)).
  - it can be viewed by [rqt_graph](http://wiki.ros.org/rqt_graph)

##Publishers and Subscriber Documentation

- Python - see [rospy overview](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)
- C++ - see [roscpp overview](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers)
