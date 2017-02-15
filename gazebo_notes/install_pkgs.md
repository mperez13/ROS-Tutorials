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

#### Install from Source (Ubuntu)

###### Setup a Catkin Workspace

if you don't have a catkin workspace setup, do:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```

Add `.bashrc` file a source to the setup scripts:

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

For more details [Create Catkin Workspace][2] tutorial

##### Clone Github Repositories

Make sure `git` is installed

```
sudo apt-get install git
```

Kinetic uses gazebo 7.x series, start by installing it:

```
sudo apt-get install -y libgazebo7-dev
```

Download source code from `gazebo_ros_pkgs` [github repository][3]:

```
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b kinetic-devel
```

Check for missing dependencies using rosdep:

```
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro kinetic
```

Can automatically install missing dependencies using rosdep via debian install:

```
rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
```

##### Build gazebo_ros_pkgs

```
cd ~/catkin_ws/
catkin_make
```

For issues or questions see [answers.gazebosim.org][4]

#### Testing Gazebo w/ ROS Integration





[1]: http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros
[2]: http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace
[3]: https://github.com/ros-simulation/gazebo_ros_pkgs
[4]: http://answers.gazebosim.org/questions/
