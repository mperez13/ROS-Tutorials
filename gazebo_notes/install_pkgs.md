# [Installing gazebo_ros_pkgs][1]

Set of ROS packages for interfacing w/ Gazebo are contained w/in a new meta package (catkin's version of stacks) named `gazebo_ros_pkgs`

Following instructions are for Gazebo integration using ROS Kinetic, ROS Jade, and ROS Indigo.

Assuming ROS and Gazebo is already installed.

### Test that stand alone Gazebo works first

```
gazebo
```

- should see the GUI w/ an empty world

### Test that you have the right version of Gazebo

```
which gzserver
which gzclient
```

- if installed from source, should get
    
    ```
    /usr/local/bin/gzserver
    /usr/local/bin/gzclient
    ```

- if installed from debian, should get

    ```
    /usr/bin/gzserver
    /usr/bin/gzclient
    ```

## Install gazebo_ros_pkgs

### 2 ways to install: (1) from packages, which is easier and faster, or (2) from source, which easier to debug and submit bug patches

#### Install Pre-Built Debians

```
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```











[1]: http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros
