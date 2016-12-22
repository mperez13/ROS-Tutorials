# Contact Sensor

- Link To Tutorial - http://gazebosim.org/tutorials?tut=contact_sensor#Introduction

This tutorial demonstrates the process of creating a contact sensor, and getting the contact data via a plugin or a message. A contact sensor detects collisions between two object and reports the location of the contact associated forces.

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
    
    - topic we are interested: `/gazebo/default/box/link/my_contact`
        - `my_contact` contact sensor publishes on this topic

Print value of the contact sensors to the screen:

```
gz topic -e /gazebo/default/box/link/my_contact
```

- Above command will dump all the contacts to the terminal. Stop it using `ctrl-c`
- Note: if it doesn't work add between `</contact>` and `</sensor>` tags in order to output on the terminal:

    ```
    <update_rate> 5 </update_rate>
    ```
    - rate "5" can be changed to a different frequency if wanted.

## Contact Sensor Plugin
    


    
[1]:  ../gazebo_contact_tutorial/contact.world 
