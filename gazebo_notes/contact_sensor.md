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


    
[1]:  ../gazebo_contact_tutorial/contact.world 
