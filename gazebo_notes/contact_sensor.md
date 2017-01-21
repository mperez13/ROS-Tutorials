# [Contact Sensor][9]

**Description of Tutorial**: This tutorial demonstrates the process of creating a contact sensor, and getting the contact data via a plugin or a message. A contact sensor detects collisions between two object and reports the location of the contact associated forces.

## Setup Tutorial

Start by creating a work directory:

```
$ mkdir ~/gazebo_contact_tutorial
$ cd ~/gazebo_contact_tutorial
```

- for my setup:
  
```
$ mkdir ~/ROS-Tutorials/gazebo_contact_tutorial
$ cd ~/ROS-Tutorials/gazebo_contact_tutorial
```

Make a [SDF world file][1] w/ a box that has a contact sensor. 

```
gedit contact.world
```

- The contact sensor is attached to a link within a box model.
- It will report collisions between the `box_collision` object and any other object on the world.

## Print Contact Values

Run `contact.world` using Gazebo:

```
gazebo contact.world
```

In a new terminal list the topics published by Gazebo:

```
gz topic -l
```

- output should look like:

    ```
    /gazebo/default/pose/info
    /gazebo/default/gui
    /gazebo/default/log/status
    /gazebo/default/response
    /gazebo/default/world_stats
    /gazebo/default/selection
    /gazebo/default/model/info
    /gazebo/default/light
    /gazebo/default/physics/contacts
    /gazebo/default/visual
    /gazebo/default/request
    /gazebo/default/joint
    /gazebo/default/sensor
    /gazebo/default/box/link/my_contact
    /gazebo/default/box/link/my_contact/contacts
    /gazebo/world/modify
    /gazebo/default/diagnostics
    /gazebo/default/factory
    /gazebo/default/model/modify
    /gazebo/default/scene
    /gazebo/default/physics
    /gazebo/default/world_control
    /gazebo/server/control
    ```

    - When I ran this I got:
        
        ![contact values][2]
    
    - topic we are interested: `/gazebo/default/box/link/my_contact`
        - `my_contact` contact sensor publishes on this topic

Print value of the contact sensors to the screen:

```
gz topic -e /gazebo/default/box/link/my_contact
```

- When I ran this I got these values [do not know what values mean]:
    
    ```
    contact {
      collision1: "box::link::box_collision"
      collision2: "ground_plane::link::collision"
      position {
        x: -0.50000000000000233
        y: -0.49999999999999745
        z: -4.90002483033436e-12
      }
      position {
        x: 0.49999999999999767
        y: -0.50000000000000033
        z: -4.89997839318797e-12
      }
      position {
        x: -0.4999999999999995
        y: 0.50000000000000255
        z: -4.8999319863921415e-12
      }
      position {
        x: 0.50000000000000044
        y: 0.49999999999999967
        z: -4.8998855492457509e-12
      }
      normal {
        x: 0
        y: 0
        z: 1
      }
      normal {
        x: 0
        y: 0
        z: 1
      }
      normal {
        x: 0
        y: 0
        z: 1
      }
      normal {
        x: 0
        y: 0
        z: 1
      }
      depth: 4.9000803414855909e-12
      depth: 4.9000339043392012e-12
      depth: 4.8999874975433727e-12
      depth: 4.8999410603969822e-12
      wrench {
        body_1_name: "box::link::box_collision"
        body_1_id: 12
        body_2_name: "ground_plane::link::collision"
        body_2_id: 6
        body_1_wrench {
          force {
            x: -6.7876614242007092e-16
            y: 7.2379382647167134e-17
            z: 2.4254816002579651
          }
          torque {
            x: -1.2127408001289826
            y: 1.212740800128983
            z: -3.7557276253361908e-16
          }
        }
        body_2_wrench {
          force {
            x: 0
            y: 0
            z: 0
          }
          torque {
            x: 0
            y: 0
            z: 0
          }
        }
      }
      wrench {
        body_1_name: "box::link::box_collision"
        body_1_id: 12
        body_2_name: "ground_plane::link::collision"
        body_2_id: 6
        body_1_wrench {
          force {
            x: -1.1024941811452518e-15
            y: -5.6715395743487394e-16
            z: 2.4255183997420335
          }
          torque {
            x: -1.2127591998710172
            y: -1.2127591998710161
            z: -8.34824069290063e-16
          }
        }
        body_2_wrench {
          force {
            x: 0
            y: 0
            z: 0
          }
          torque {
            x: 0
            y: 0
            z: 0
          }
        }
      }
      wrench {
        body_1_name: "box::link::box_collision"
        body_1_id: 12
        body_2_name: "ground_plane::link::collision"
        body_2_id: 6
        body_1_wrench {
          force {
            x: -1.1171430502329111e-15
            y: -3.7486829177618766e-16
            z: 2.425518399742026
          }
          torque {
            x: 1.2127591998710128
            y: 1.2127591998710139
            z: 7.4600567100454934e-16
          }
        }
        body_2_wrench {
          force {
            x: 0
            y: 0
            z: 0
          }
          torque {
            x: 0
            y: 0
            z: 0
          }
        }
      }
      wrench {
        body_1_name: "box::link::box_collision"
        body_1_id: 12
        body_2_name: "ground_plane::link::collision"
        body_2_id: 6
        body_1_wrench {
          force {
            x: -1.8584962931228495e-15
            y: -9.3572422334182954e-16
            z: 2.4254816002579864
          }
          torque {
            x: 1.2127408001289925
            y: -1.2127408001289921
            z: 4.6138603489051006e-16
          }
        }
        body_2_wrench {
          force {
            x: 0
            y: 0
            z: 0
          }
          torque {
            x: 0
            y: 0
            z: 0
          }
        }
      }
      time {
        sec: 580
        nsec: 630000000
      }
      world: "default"
    }
    ```

- Above command will dump all the contacts to the terminal. Stop it using `ctrl-c`
- Note: if it doesn't work add between `</contact>` and `</sensor>` tags in order to output on the terminal:

    ```
    <update_rate> 5 </update_rate>
    ```
    - rate "5" can be changed to a different frequency if wanted.

## Contact Sensor Plugin
    
A plugin created for the contact sensor can get the collision data, manipulate it and output it to an arbitrary destination (for example ROS topic)

- Note: For Gazebo version 3.0 and above, will need to have the Gazebo dev packages installed (something `libgazebo*-dev`).  Check [installation tutorials][3] for further instructions

1. Modify `contact.world` SDF file.  Add following line directly below `<sensor name='my_contact' type=`contact`>`:

```c++
<plugin name="my_plugin" filename="libcontact.so"/>
```

    - This line tell Gazebo to load `libcontact.so` sensor plugin.
2. Create a header file for the plugin, [ContactPlugin.hh][4], and paste following:

```c++
#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo{
    /// \brief An example plugin for a contact sensor.
    class ContactPlugin : public SensorPlugin {
        /// \brief Constructor.
        public: ContactPlugin();

        /// \brief Destructor.
        public: virtual ~ContactPlugin();

        /// \brief Load the sensor plugin.
        /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
        /// \param[in] _sdf SDF element that describes the plugin.
        public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

        /// \brief Callback that receives the contact sensor's update signal.
        private: virtual void OnUpdate();

        /// \brief Pointer to the contact sensor
        private: sensors::ContactSensorPtr parentSensor;

        /// \brief Connection that maintains a link between the contact sensor's
        /// updated signal and the OnUpdate callback.
        private: event::ConnectionPtr updateConnection;
    };
}
#endif
```

3. Create source file, [ContactPlugin.cc][5] with the following:

```c++
#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin(){
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin(){
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
```

The following code from the `Load` function gets pointer to the contact sensor through the `_sensor` parameter. 
Then we test to make sure the pointer is valid and create a connection to contact sensor's `updated` event.
Last line guarantees that the sensor is initialized.
     
```c++
  // Get the parent sensor.
  this->parentSensor =
  std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor) {
  gzerr << "ContactPlugin requires a ContactSensor.\n";
  return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
```

```
}
```

The `OnUpdate` function is called whenever the contact sensor is updated. This print out the contact values.

```c++
/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
// Get all the contacts.
msgs::Contacts contacts;
contacts = this->parentSensor->Contacts();
for (unsigned int i = 0; i < contacts.contact_size(); ++i)
{
  std::cout << "Collision between[" << contacts.contact(i).collision1() << "] and [" << contacts.contact(i).collision2() << "]\n";

  for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
  {
    std::cout << j << "  Position:"
              << contacts.contact(i).position(j).x() << " "
              << contacts.contact(i).position(j).y() << " "
              << contacts.contact(i).position(j).z() << "\n";
    std::cout << "   Normal:"
              << contacts.contact(i).normal(j).x() << " "
              << contacts.contact(i).normal(j).y() << " "
              << contacts.contact(i).normal(j).z() << "\n";
    std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
  }
}
}
```

## Compiling the code

1. Create [`CMakeLists.txt`][6] file:

```
cd ~/gazebo_contact_tutorial; gedit CMakeLists.txt
```
    
  - in my setup:

    ```
    $ cd ~/ROS-Tutorials/gazebo_contact_tutorial
    $ gedit CMakeLists.txt
    ```
        
2. Add the following to the `CMakeLists.txt` file:

    ```
    cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

    find_package(gazebo REQUIRED)

    include_directories(${GAZEBO_INCLUDE_DIRS})
    link_directories(${GAZEBO_LIBRARY_DIRS})
    list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

    add_library(contact SHARED ContactPlugin.cc)
    target_link_libraries(contact ${GAZEBO_libraries})
    ```
    
3. Create a build directory and make plugin:

  ```
  $ mkdir build
  $ cd build
  $ cmake ../
  $ make
  ```

## Running the code

1. Modify `LD_LIBRARY_PATH` so the library loader can find your library:
    
  ```
  $ export LD_LIBRARY_PATH=~/gazebo_contact_tutorial/build:$LD_LIBRARY_PATH
  ```
    
2. In the build directory (`cd ~/ROS-Tutorials/gazebo_contact_tutorial/build`) run `gzserver`:
    
  ```
  gzserver ../contact.world
  ```
    
  - The data that I got back was:
        
    ![collision contacts][7]
        
**Gazebo API** - [ContactSensor Class Reference][8]

Return to [Gazebo Categories][10]

[1]: ../gazebo_contact_tutorial/contact.world 
[2]: images/contact_values.png
[3]: http://gazebosim.org/tutorials?cat=install
[4]: ../gazebo_contact_tutorial/ContactPlugin.hh
[5]: ../gazebo_contact_tutorial/ContactPlugin.cc
[6]: ../gazebo_contact_tutorial/CMakeLists.txt
[7]: images/collision_contacts.png
[8]: https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1ContactSensor.html#details
[9]: http://gazebosim.org/tutorials?tut=contact_sensor#Introduction
[10]: ../gazebo_notes.md
