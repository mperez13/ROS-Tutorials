# World Plugins

[**Link to Tutorials**][1]

**Description**: This tutorial demonstrates how to insert predefined and custom models into Gazebo. 

## Pre-req 

- [Model Manipulation][2]
- [Plugin Tutorial][3]

## Code 

**Source**: [gazebo/examples/plugins/factory][4]

### Create source file [factory.cc][5] in `/gazebo_plugin_tutorial`. The code is explained below.

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

```
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

#### Create box [model.sdf][7] w/ following code:

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







[1]: http://gazebosim.org/tutorials?tut=plugins_world&cat=write_plugin
[2]: model_plugin.md
[3]: plugins.md
[4]: https://bitbucket.org/osrf/gazebo/src/gazebo7/examples/plugins/factory
[5]: ../gazebo_plugin_tutorial/factory.cc
[6]: ../gazebo_plugin_tutorial/CMakeLists.txt
[7]: ../gazebo_plugin_tutorial/model.sdf
