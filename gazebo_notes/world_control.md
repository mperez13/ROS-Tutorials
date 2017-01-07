# Programmatic World Control

[**Link to Tutorial**][1]

## Pre-req 

- [Model Manipulation][2]
- [Plugin Tutorial][3]

## Setup

**Source**: [gazebo/examples/plugins/world_edit][4]

### Create [file][4] called `~/gazebo_plugin_tutorial/world_edit.world` w/ following contents:

```xml
<?xml version ='1.0'?>
<sdf version ='1.4'>
  <world name='default'>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <plugin filename="libworld_edit.so" name="world_edit"/>
  </world>
</sdf>
```

### Create [file][5] called `~/gazebo_plugin_tutorial/world_edit.cc`. Code explained below.

###### Creates a new pointer & initialize to using the world name. World name allows node to communicate w/ one specific world

```
// Create a new transport node
transport::NodePtr node(new transport::Node());

// Initialize the node with the world name
node->Init(_parent->GetName());
```

###### Creates publisher to send messages on the "~/physics" topic

```
// Create a publisher on the ~/physics topic
transport::PublisherPtr physicsPub = node->Advertise<msgs::Physics>("~/physics");
```

###### Creates a physics message & the step time and gravity are altered. Message is then published to the "~/physics" topic.

```
msgs::Physics physicsMsg;
physicsMsg.set_type(msgs::Physics::ODE);

// Set the step time
physicsMsg.set_max_step_size(0.01);

// Change gravity
msgs::Set(physicsMsg.mutable_gravity(),
    ignition::math::Vector3d(0.01, 0, 0.1));
physicsPub->Publish(physicsMsg);
```

## Build

###### Add the following to CMakeLists.txt

```
add_library(world_edit SHARED world_edit.cc)
target_link_libraries(world_edit ${GAZEBO_LIBRARIES})
```

###### Compiling this code will result in a shared library,`~/gazebo_plugin_tutorial/build/libworld_edit.so`, that can be inserted in a Gazebo simulation.

```
$ cd ~/gazebo_plugin_tutorial/build
$ cmake ../
$ make
```

- For my setup: `$ cd ~/ROS-Tutorials/gazebo_plugin_tutorial/build`

## Run Tutorial

###### Add folder to the `GAZEBO_PLUGIN_PATH` environment variable:

```
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_plugin_tutorial/build/
```

- For my setup: `$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/ROS-Tutorials/gazebo_plugin_tutorial/build/`

###### Start the Gazebo world

```
$ cd ~/gazebo_plugin_tutorial
$ gazebo world_edit.world
```

##### Now add a box to the world. The box should float up & away from the camera.

[1]: http://gazebosim.org/tutorials?tut=plugins_world_properties&cat=write_plugin
[2]: model_plugin.md
[3]: plugins.md
[4]: https://bitbucket.org/osrf/gazebo/src/gazebo7/examples/plugins/world_edit
[4]: ../gazebo_plugin_tutorial/world_edit.world
[5]: ../gazebo_plugin_tutorial/world_edit.cc
