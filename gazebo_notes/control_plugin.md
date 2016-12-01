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

    ```c++
    #ifndef _VELODYNE_PLUGIN_HH_
    #define _VELODYNE_PLUGIN_HH_
    
    #include <gazebo/gazebo.hh>
    
    namespace gazebo{
      // \brief A plugin to control a Velodyne sensor
      class VelodynePlugin : public ModelPlugin{
        public : VelodynePlugin() { }
        
        // \brief Load function is called by Gazebo when the plugin is inserted into simulation
        // \param[in] _model : A pointer to the model that this plugin is attached to
        // \param[in _sdf : A pointer to the plugin's SDF element.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
          //Jus output a msg for now
          std::cerr << "\nThe velodyne plugin is attach to model[" << _model->GetName() << "]\n";
        }
      };
      
      //Tell Gazebo about this plugin, so that Gazebo can tell Load on this plugin
      GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
    }
    #endif
    ```

####Step 3: Create CMake build script

1. Create [CMakeLists.txt](https://github.com/mperez13/ROS-Tutorials/tree/master/velodyne_plugin/CMakeLists.txt)

    ```
    cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
    
    # Find Gazebo
    find_package(gazebo REQUIRED)
    include_directories(${GAZEBO_INCLUDE_DIRS})
    link_directories(${GAZEBO_LIBRARY_DIRS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

    # Build our plugin
    add_library(velodyne_plugin SHARED velodyne_plugin.cc)
    target_link_libraries(velodyne_plugin ${GAZEBO_libraries})
    ```

####Step 4: Attach the plugin to the Velodyne sensor

- use SDF's `\<inlcude>` capability to test out plugin w/out touching the main Velodyne SDF file

1. Create a new world file inside your workspace: [velodyne.world](https://github.com/mperez13/ROS-Tutorials/tree/master/velodyne_plugin/velodyne.world)

    ```c++
    <?xml version="1.0" ?>
    <sdf version="1.5">
      <world name="default">
        <!-- A global light source -->
        <include>
          <uri>model://sun</uri>
        </include>
        
        <!-- A ground plane -->
        <include>
          <uri>model://ground_plane</uri>
        </include>

        <!-- A testing model that includes the Velodyne sensor model -->
        <model name="my_velodyne">
          <include>
            <uri>model://velodyne_hdl32</uri>
          </include>

          <!-- Attach the plugin to this model -->
          <plugin name="velodyne_control" filename="libvelodyne_plugin.so"/>
        </model>
  </world>
</sdf>
    ```

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
        
        - Fixed error by adding `#include <gazebo/physics/physics.hh>` to velodyne_plugin.cc
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

1. Modify `Load` function in velodyne_plugin.cc 

    ```
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      // Safety check
      if (_model->GetJointCount() == 0){
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint (assumption about the model having one joint that is the rotational joint)
      this->joint = _model->GetJoints()[0];

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

      // Set the joint's target velocity. This target velocity is just
      // for demonstration purposes.
      this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), 10.0);
    }
    ```
2. Recompile and run Gazebo
    
    ```
    cd build
    make
    gazebo ../velodyne.world
    ```
    The Velodyne should be spinning.

##Plugin Configuration

- In this section, we'll modify plugin to read a custom SDF parameter that is the target velocity of the Velodyne

1. Add a new element as a child of the `<plugin>` in [velodyne.world](https://github.com/mperez13/ROS-Tutorials/blob/master/velodyne_plugin/velodyne.world)
  - new element can be anything as long as it is a valid XML
  - our plugin will have access to this value in the `Load` function
  
    ```
    <plugin name="velodyne_control" filename="libvelodyne_plugin.so">
      <velocity>25</velocity>
    </plugin>
    ```
2. Modify `Load` function to read `<velocity>` using `sdf::ElementPtr` parameter

    ```c++
    //default to zero velocity
    double velocity = 0;
   
    //check velocity element exists, then read value
    if(_sdf->HasElement("velocity")){
      velocity = _sdf->Get<double>("velocity");
    }
    
    //set joint's target velocity (just for demonstration purposes)
    this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), velocity);
    ```
3. Compile and run simulation
  
    ```
    cd build
    cmake ../
    make
    gazebo ../velodyne.world
    ```
Adjust `<velocity>` SDF value & restart simulation to see the effects.

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
    
    ```
    // \brief : Set the velocity of the Velodyne
    // \param[in] : _vel New target velocity
    public: void SetVelocity(const double &_vel){
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }
    ```

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
3. The complete plugin should look like this:
    
    ```
    #ifndef _VELODYNE_PLUGIN_HH_
    #define _VELODYNE_PLUGIN_HH_

    #include <gazebo/gazebo.hh>
    #include <gazebo/physics/physics.hh>
    #include <gazebo/transport/transport.hh>
    #include <gazebo/msgs/msgs.hh>
    
    namespace gazebo{
      // \brief A plugin to control a Velodyne sensor
      class VelodynePlugin : public ModelPlugin{
        public : VelodynePlugin() { }
        
        // \brief Load function is called by Gazebo when the plugin is inserted into simulation
        // \param[in] _model : A pointer to the model that this plugin is attached to
        // \param[in _sdf : A pointer to the plugin's SDF element.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
          // Safety check
          if (_model->GetJointCount() == 0){
            std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
            return;
          }

          // Store the model pointer for convenience.
          this->model = _model;

          // Get the first joint (assumption about the model having one joint that is the rotational joint)
          this->joint = _model->GetJoints()[0];

          // Setup a P-controller, with a gain of 0.1.
          this->pid = common::PID(0.1, 0, 0);

          // Apply the P-controller to the joint.
          this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

          //default to zero velocity
          double velocity = 0;
   
          //check velocity element exists, then read value
          if(_sdf->HasElement("velocity")){
            velocity = _sdf->Get<double>("velocity");
          }
          
          this->SetVelocity(velocity);
          
          // Create the node
          this->node = transport::NodePtr(new transport::Node());
          this->node->Init(this->model->GetWorld()->GetName());

          // Create a topic name
          std::string topicname = "~/" + this->model->GetName() + "/vel_cmd";

          // Subscribe to the topic, and register a callback
          this->sub = this->node->Subscribe(topicName, &VelodynePlugin::OnMsg, this);
        }
        
        // \brief Handle incoming message
        // \param[in] _msg Repurpose a vector3 message. This function will only use the x component.
        private: void OnMsg(ConstVector3dPtr &_msg){
          this->SetVelocity(_msg->x());
        }
        
        // \brief A node used for transport
        private: transport::NodePtr node;

        // \brief A subscriber to a named topic.
        private: transport::SubscriberPtr sub;
        
        // \brief Pointer to the model
        private: physics::ModelPtr model; 
        
        // \brief Pointer to the joint
        private: physics::JointPtr joint;
        
        // \brief A PID controller for the joint
        private: common::PID pid;
        
      };
      
      //Tell Gazebo about this plugin, so that Gazebo can tell Load on this plugin
      GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
    }
    #endif
    ```

##Test the message passing API

1. Create new source file [vel.cc](https://github.com/mperez13/ROS-Tutorials/blob/master/velodyne_plugin/vel.cc) in your workspace

    ```
    #include <gazebo/gazebo_config.h>
    #include <gazebo/transport/transport.hh>
    #include <gazebo/msgs/msgs.hh>

    // Gazebo's API has changed between major releases (changes are accounted for with #if..#endif blocks in this file)
    #if GAZEBO_MAJOR_VERSION < 6
    #include <gazebo/gazebo.hh>
    #else
    #include <gazebo/gazebo_client.hh>
    #endif

    /////////////////////////////////////////////////
    int main(int _argc, char **_argv){
      // Load gazebo as a client
    #if GAZEBO_MAJOR_VERSION < 6
      gazebo::setupClient(_argc, _argv);
    #else
      gazebo::client::setup(_argc, _argv);
    #endif

      // Create our node for communication
      gazebo::transport::NodePtr node(new gazebo::transport::Node());
      node->Init();

      // Publish to the  velodyne topic
      gazebo::transport::PublisherPtr pub =
        node->Advertise<gazebo::msgs::Vector3d>("~/my_velodyne/vel_cmd");

      // Wait for a subscriber to connect to this publisher
      pub->WaitForConnection();

      // Create a a vector3 message
      gazebo::msgs::Vector3d msg;

      // Set the velocity in the x-component
    #if GAZEBO_MAJOR_VERSION < 6
      gazebo::msgs::Set(&msg, gazebo::math::Vector3(std::atof(_argv[1]), 0, 0));
    #else
      gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), 0, 0));
    #endif

      // Send the message
      pub->Publish(msg);

      // Make sure to shut everything down.
    #if GAZEBO_MAJOR_VERSION < 6
      gazebo::shutdown();
    #else
      gazebo::client::shutdown();
    #endif
    }
    ```
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
