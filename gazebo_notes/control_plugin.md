#Control Plugin 

Link to tutorial - http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5

- Plugin 
  - is a C++ library that is loaded by Gazebo at runtime
  - has access to Gazebo's API, which allows a plugin to perform tasks inlcuding moving objects, adding/removing objects & accessing sensor data
  - [Tutorials on plugins](http://gazebosim.org/tutorials?cat=write_plugin)

##Write a plugin

**Overview**: will craete the plugin in a new directory.  The contents of this directory will include the plugin source code, and CMake build script.

###Step 1: Create a [velodyne_plugin](https://github.com/mperez13/ROS-Tutorials/velodyne_plugin) workspace

###Step 2: Create the plugin source file

1. Create [velodyne_plugin.cc](https://github.com/mperez13/ROS-Tutorials/velodyne_plugin/velodyne_plugin.cc)

###Step 3: Create CMake build script

1. Create [CMakeLists.txt](https://github.com/mperez13/ROS-Tutorials/velodyne_plugin/CMakeLists.txt) 

###Step 4: Attach the plugin to the Velodyne sensor

- use SDF's `\<inlcude>` capability to test out plugin w/out touching the main Velodyne SDF file

1. Create a new world file inside your workspace: [velodyne.world](https://github.com/mperez13/ROS-Tutorials/velodyne_plugin/velodyne.world)

###Step 5: Build and test

1. W/in your workspace, create build directory and compile the plugin:

    ```
    mkdir build
    cd build
    cmake ..
    make
    ```
2. Run the world. Make sure to run gazebo within the `build` directory so Gazebo can find the plugin library.

    ```
    cd ~/velodyne_plugin/build
    gazebo ../velodyne.world
    ```
3. Check your terminal, it should say:
    
    ```
    The velodyne plugin is attached to model[my_velodyne]
    ```
  
