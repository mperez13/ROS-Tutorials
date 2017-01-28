# [Create ROS Workspace][1]

## create catkin workspace

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws
$ catkin_make
```
    
## catkin_make command

- current directory holds build and devel folder
    - devel 
		- several setup.*sh files
		- sourcing any of these files will overlay this workspace on top of your environment
    - catkin documentation
	- source setup file
	
    ```
    $ source devel/setup.bash
    ```

- to make sure workspace is properly overlayed by setup script, make sure ROS_PACKAGE_PATH environment variable includes directory you are in

    ```
    $ echo $ROS_PACKAGE_PATH
    /home/youruser/catkin_ws/src:/opt/ros/kinetic/share
    ```

[1]: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
