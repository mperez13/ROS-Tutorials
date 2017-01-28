# [ROS Filesystem][1]

#### Pre-req 

- for this tutorial, we'll inspect package in ros-tutorials, so install
    
    ```
    $ sudo apt-get install ros-<distro>-ros-tutorials
    ```

## Concepts

#### packages

- organization unit
- each contain libraries, executable, scripts or other artifacts

#### Manifest ([package.xml][2])

- description of a package
- serves to define dependencies between packages 
- serves to capture meta info about the package like version, maintainer, license, etc.

#### Stacks

- stacks where removed w/ catkin to simplify the growing code base & to support better distribution of packages
- in catkin, can define meta packages to collect similar packages and multiple packages can reside in a simple VCS repository
    - those 2 features replace the functionality of stacks

## Tools

#### [rospack][3]

- commandline tool for retrieving info about packages available
- [rospack documentation][4]
    
    ```
    $ rospack find package_name
    ```

#### roscd

- part of [rosbash][5]
- allows you to change directory directly to a package or a stack

    ```
    $ roscd locationname/subdir
    ```

- roscd log
    - will take you where ROS stores log files

#### rosls

- part of [rosbash][5] suite
- allows to use `ls` directly in package by name rather than by absolute path
    
    ```
    $ rosls locationname/subdir
    ```

[1]: http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem
[2]: http://wiki.ros.org/catkin/package.xml
[3]: http://wiki.ros.org/rospack
[4]: http://docs.ros.org/independent/api/rospkg/html/rospack.html
[5]: http://wiki.ros.org/rosbash
