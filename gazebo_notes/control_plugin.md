#Control Plugin 

Link to tutorial - http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5

- Plugin 
  - is a C++ library that is loaded by Gazebo at runtime
  - has access to Gazebo's API, which allows a plugin to perform tasks inlcuding moving objects, adding/removing objects & accessing sensor data
  - [Tutorials on plugins](http://gazebosim.org/tutorials?cat=write_plugin)

##Write a plugin

**Overview**: will craete the plugin in a new directory.  The contents of this directory will include the plugin source code, and CMake build script.

####Step 1: Create a [velodyne_plugin](https://github.com/mperez13/ROS-Tutorials/tree/master/velodyne_plugin) workspace

####Step 2: Create the plugin source file

1. Create [velodyne_plugin.cc](https://github.com/mperez13/ROS-Tutorials/tree/master/velodyne_plugin/velodyne_plugin.cc)

####Step 3: Create CMake build script

1. Create [CMakeLists.txt](https://github.com/mperez13/ROS-Tutorials/tree/master/velodyne_plugin/CMakeLists.txt) 

####Step 4: Attach the plugin to the Velodyne sensor

- use SDF's `\<inlcude>` capability to test out plugin w/out touching the main Velodyne SDF file

1. Create a new world file inside your workspace: [velodyne.world](https://github.com/mperez13/ROS-Tutorials/tree/master/velodyne_plugin/velodyne.world)

####Step 5: Build and test

1. W/in your workspace, create build directory and compile the plugin:

    ```
    mkdir build
    cd build
    cmake ..
    make
    ```
    - had an error when I ran `make`:
        ![Image of plugin error](https://github.com/mperez13/ROS-Tutorials/blob/master/gazebo_notes/images/plugin_error.png)
        
        - Fixed error by adding `#include <gazebo/physics/Model.hh>` to velodyne_plugin.cc
2. Run the world. Make sure to run gazebo within the `build` directory so Gazebo can find the plugin library.

    ```
    cd ~/velodyne_plugin/build
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/velodyne_plugin/build
    gazebo ../velodyne.world
    ```
3. Check your terminal, it should say:
    
    ```
    The velodyne plugin is attached to model[my_velodyne]
    ```
##Move the Velodyne

- Next add code that controls the Velodyne's joint
- We will use simple PID controller to control the velocity of the Velodyne's joint

1. Modify `Load` function in velodyne_plugin.ccand then
    
    ```
    cd build
    make
    gazebo ../velodyne.world
    ```
    The Velodyne should be spinning.
    
    **[Got an error; just continue, the rest of this tutorial will fix this problem]**

##Plugin Configuration

- In this section, we'll modify plugin to read a custom SDF parameter that is the target velocity of the Velodyne

1. Add a new element as a child of the `<plugin>` in [velodyne.world](https://github.com/mperez13/ROS-Tutorials/blob/master/velodyne_plugin/velodyne.world)
  - new element can be anything as long as it is a valid XML
  - our plugin will have access to this value in the `Load` function
2. Modify `Load` function to read `<velocity>` using `sdf::ElementPtr` parameter
3. Compile and run simulation
  
    ```
    cd build
    cmake ../
    make
    gazebo ../velodyne.world
    ```
Adjust `<velocity>` SDF value & restart simulation to see the effects.

  **[Still getting an error, going to next step: Create an API]**

##Create an API

- Adding an API that other programs can use to change the velocity value will allow us to make dynamic adjustments to target's velocity.

- 2 API types that we can use:
  - message passing
    - relies on Gazebo's transport mechanism & would involve creating a named topic where a publisher can send double values
    - plugin would receive messages containing a `double` & set velocity appropriately
  - functions
    - would create a new public function that adjusts the velocity
    - most often used when interfacing Gazebo to ROS
    - For this to work:
      - a new plugin would inherit from our current plugin
      - child plugin would be instantiated by Gazebo instead of our current plugin & would control the velocity by calling our function
      
Since our plugin is simple, it's easy to implement both simultaneously.

1. Create new public function that can set the target velocity. This will fulfill the functional API.
    
    [velodyne_plugin.cc](https://github.com/mperez13/ROS-Tutorials/blob/master/velodyne_plugin/velodyne_plugin.cc)
    
2. Setup the message passing infrastructure.
  1. Add Node & subscriber to the plugin
  
    ```
    // \brief A node used for transport
    private: transport::NodePtr node;

    // \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;
    ```
  2. Instantiate the Node & subscriber at the end of `Load` function
  
    ```
    // Create the node
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->GetName());

    // Create a topic name
    std::string topicname = "~/" + this->model->GetName() + "/vel_cmd";

    // Subscribe to the topic, and register a callback
    this->sub = this->node->Subscribe(topicName, &VelodynePlugin::OnMsg, this);
    ```
  3. Create the callback function that handles incoming messages
  
    ```
    // \brief Handle incoming message
    // \param[in] _msg Repurpose a vector3 message. This function will only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg){
      this->SetVelocity(_msg->x());
    }
    ```
  4. Add 2 necessary headers to the plugin
    
    ```
    #include <gazebo/transport/transport.hh>
    #include <gazebo/msgs/msgs.hh>
    
    ```

##Test the message passing API

1. Create new source file [vel.cc](https://github.com/mperez13/ROS-Tutorials/blob/master/velodyne_plugin/vel.cc) in your workspace
2. Add following to `CMakeLists.txt` to build the new `vel` program.
    ```
    # Build the stand-alone test program
    add_executable(vel vel.cc)

    if (${gazebo_VERSION_MAJOR} LESS 6)
      # These two
      include(FindBoost)
      find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
      target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
    else()
      target_link_libraries(vel ${GAZEBO_LIBRARIES})
    endif()
    ```
3. Compile and run simulation
    ```
    cd build
    cmake ../
    make
    gazebo ../velodyne.world
    ```
4. In new terminal, go into build directory & run `vel` command. Make sure to set target velocity value.
    
    ```
    cd ~/velodyne_plugin/build
    ./vel 2
    ```
