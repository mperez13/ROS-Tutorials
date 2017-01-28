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
those 2 features replace the functionality of stacks

[1]: http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem
[2]: http://wiki.ros.org/catkin/package.xml
