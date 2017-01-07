# World Plugins

[**Link to Tutorials**][1]

**Description**: This tutorial demonstrates how to insert predefined and custom models into Gazebo. 

## Pre-req 

- [Model Manipulation][2]
- [Plugin Tutorial][3]

## Code 

**Source**: [gazebo/examples/plugins/factory][4]

#### Create source file [factory.cc][5] in `/gazebo_plugin_tutorial`. The code is explained below.

##### First part of the code creates a world plugin

```c++
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class Factory : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
```

##### `Load` function are 3 different methods for model insertion

- 1st method uses a World method to load a model based on a file in the resource path defined by `GAZEBO_MODEL_PATH` environment variable.

```c++
    // Option 1: Insert model from file via function call.
    // The filename must be in the GAZEBO_MODEL_PATH environment variable.
    _parent->InsertModelFile("model://box");
```

- 2nd method uses a World method to load a model based on string data

```c++
    // Option 2: Insert model from string via function call.
    // Insert a sphere model from string
    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(
       "<sdf version ='1.4'>\
          <model name ='sphere'>\
            <pose>1 0 0 0 0 0</pose>\
            <link name ='link'>\
              <pose>0 0 .5 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>");
    // Demonstrate using a custom model name.
    sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
    model->GetAttribute("name")->SetFromString("unique_sphere");
    _parent->InsertModelSDF(sphereSDF);
```

- 3rd method uses the message mechanism to insert model.
  - most useful for stand alone applications that communicate w/ Gazebo over a network connection.

```c++
    // Option 3: Insert model from file via message passing.
    {
      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->GetName());

      // Create a publisher on the ~/factory topic
      transport::PublisherPtr factoryPub =
      node->Advertise<msgs::Factory>("~/factory");

      // Create the message
      msgs::Factory msg;

      // Model file to load
      msg.set_sdf_filename("model://cylinder");

      // Pose to initialize the model to
      msgs::Set(msg.mutable_pose(),
          ignition::math::Pose3d(
            ignition::math::Vector3d(1, -2, 0),
            ignition::math::Quaterniond(0, 0, 0)));

      // Send the message
      factoryPub->Publish(msg);
```

## Compile

#### Add following to [CMakeLists.txt][6] 

```
add_library(factory SHARED factory.cc)
target_link_libraries(factory
  ${GAZEBO_LIBRARIES}
)
```

#### Compiling this code will result in a shared library, `~/gazebo_plugin_tutorial/build/libfactory.so`, that can be inserted in a Gazebo simulation.

```
$ cd ~/gazebo_plugin_tutorial/build
$ cmake ../
$ make
```

## Make the shapes

#### Make model directory w/ a box and a cylinder

```
$ mkdir ~/gazebo_plugin_tutorial/models
$ cd ~/gazebo_plugin_tutorial/models
$ mkdir box cylinder
```

#### In box directory create model.sdf and model.config

###### Create box [model.sdf][7] w/ following code:

```xml
<?xml version='1.0'?>
<sdf version ='1.6'>
  <model name ='box'>
    <pose>1 2 0 0 0 0</pose>
    <link name ='link'>
      <pose>0 0 .5 0 0 0</pose>
      <collision name ='collision'>
        <geometry>
          <box><size>1 1 1</size></box>
        </geometry>
      </collision>
      <visual name ='visual'>
        <geometry>
          <box><size>1 1 1</size></box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

###### Create [model.config][8] file w/ following code:

```xml
<?xml version='1.0'?>
<model>
  <name>box</name>
  <version>1.0</version>
  <sdf >model.sdf</sdf>

  <author>
    <name>me</name>
    <email>somebody@somewhere.com</email>
  </author>

  <description>
    A simple Box.
  </description>
</model>
```

#### In cylinder directory create model.sdf and model.config

###### Create [model.sdf][7] w/ following code:

```xml
<?xml version='1.0'?>
<sdf version ='1.6'>
  <model name ='cylinder'>
    <pose>1 2 0 0 0 0</pose>
    <link name ='link'>
      <pose>0 0 .5 0 0 0</pose>
      <collision name ='collision'>
        <geometry>
          <cylinder><radius>0.5</radius><length>1</length></cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder><radius>0.5</radius><length>1</length></cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

###### Create [model.config][8] file w/ following code:

```xml
<?xml version='1.0'?>

<model>
  <name>cylinder</name>
  <version>1.0</version>
  <sdf>model.sdf</sdf>

  <author>
    <name>me</name>
    <email>somebody@somewhere.com</email>
  </author>

  <description>
    A simple cylinder.
  </description>
</model>
```

## Run the code

#### Make sure your `$GAZEBO_MODEL_PATH` refers to your new models directory:

```
$ export GAZEBO_MODEL_PATH=$HOME/gazebo_plugin_tutorial/build:$GAZEBO_MODEL_PATH
```

- in my setup: `$ export GAZEBO_MODEL_PATH=$HOME/ROS-Tutorials/gazebo_plugin_tutorial/build:$GAZEBO_MODEL_PATH`

#### Add your library path to the `GAZEBO_PLUGIN_PATH`:

```
$ export GAZEBO_PLUGIN_PATH=$HOME/gazebo_plugin_tutorial/build:$GAZEBO_PLUGIN_PATH
```

- in my setup: `$ export GAZEBO_PLUGIN_PATH=$HOME/ROS-Tutorials/gazebo_plugin_tutorial/build:$GAZEBO_PLUGIN_PATH`

#### Create [world SDF file][9] called `~/gazebo_plugin_tutorial/factory.world` w/ following code:

```xml
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <plugin name="factory" filename="libfactory.so"/>
  </world>
</sdf>
```

#### Run Gazebo

```
$ gazebo ~/ROS-Tutorials/gazebo_plugin_tutorial/factory.world
```

#### The Gazebo window should show an environment w/ a sphere, box, and cylinder arranged in a row.

[1]: http://gazebosim.org/tutorials?tut=plugins_world&cat=write_plugin
[2]: model_plugin.md
[3]: plugins.md
[4]: https://bitbucket.org/osrf/gazebo/src/gazebo7/examples/plugins/factory
[5]: ../gazebo_plugin_tutorial/factory.cc
[6]: ../gazebo_plugin_tutorial/CMakeLists.txt
[7]: ../gazebo_plugin_tutorial/model.sdf
[8]: ../gazebo_plugin_tutorial/model.config
[9]: ../gazebo_plugin_tutorial/factory.world
