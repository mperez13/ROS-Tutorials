# Topics Subscription

Link to tutorial - http://gazebosim.org/tutorials?tut=topics_subscribed&cat=transport),%20and%20[this%20example](https://bitbucket.org/osrf/gazebo/src/8fee9d9628195bf6841c24e8d67a0f6b08faec4a/examples/stand_alone/listener/?at=default

- Gazebo communicates on TCP/IP sockets, which allows separate programs to interface w/ Gazebo.
- [Boost ASIO](http://www.boost.org/doc/libs/1_53_0/doc/html/boost_asio.html): used by Gazebo to manage the communication layer
- [Google Protobufs](https://code.google.com/p/protobuf/): used as the message passing and serialization library
- To send messages: publish messages using a publisher on a named topic
- To receive messages: subscribe to a named topic using a subscriber

- Easiest way to communicate with Gazebo over TCP/IP sockets is to link against the Gazebo libraries & use the provided functions

[Document of Gazebo transport system](http://gazebosim.org/api/code/dev/group__gazebo__transport.html)

[Document of the messages](http://gazebosim.org/api/msgs/dev)

- To list all the topics in use run:

  ```
  gz topic -l
  ```

### Example: Subscribe to a [WorldStatistics message](http://gazebosim.org/api/msgs/dev/world__stats_8proto.html) & assume that you can link against Gazebo.

1. Download `listener.cc` & `CMakeLists.txt` from above link. Compile the example:

  ```
  mkdir ~/listener
  cd ~/listener
  wget https://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/listener/listener.cc
  wget https://bitbucket.org/osrf/gazebo/raw/default/examples/stand_alone/listener/CMakeLists.txt
  mkdir build
  cd build
  cmake ..
  make
  ```

2. Run the listener program when an instance of Gazebo is already running.
  
  ```
  cd ~/listener/build
  ./listener
  ```

  - output should be similar to:
    
    ```
    sim_time {
      sec: 1104
      nsec: 855000000
    }
    pause_time {
      sec: 0
      nsec: 0
    }
    real_time {
      sec: 1108
      nsec: 263362269
    }
    paused: false
    iterations: 1104855
    sim_time {
      sec: 1105
      nsec: 55000000
    }
    pause_time {
      sec: 0
      nsec: 0
    }
    real_time {
      sec: 1108
      nsec: 464165998
    }
    paused: false
    iterations: 1105055
    ```

### Code Explained

    ```c++
    #include <gazebo/transport/transport.hh>
    #include <gazebo/msgs/msgs.hh>
    #include <gazebo/gazebo_client.hh>

    #include <iostream>

    // Callback function that will print the messages to the console, which we called cb in gazebo::transport::NodePtr
    void cb(ConstWorldStatisticsPtr &_msg){
      // Dump the message contents to stdout.
      std::cout << _msg->DebugString();
    }

    
    int main(int _argc, char **_argv)
    {
      // Load gazebo and run the transport system
      gazebo::client::setup(_argc, _argv);

      // Create a nodem which provides functions to create publishers and subscribers
      gazebo::transport::NodePtr node(new gazebo::transport::Node());
      node->Init();

      // Create a subscriber on the 'world_stats' topic. Gazebo publishes a stream of stats on this topic
      gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", cb);

      // Wait loop while messages come in
      while (true)
        gazebo::common::Time::MSleep(10);

      // Finalize the transport system by shutting it down
      gazebo::client::shutdown();
    }
    ```



















