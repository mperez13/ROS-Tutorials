# Overview of Gazebo Plugins 

Link to Tutorial: http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin

**Plugin**: chunk of code that is compiled as a shared library & inserted into the simulation

- has a direct access to all functionality of Gazebo through the standard C++ classes
- are useful because:
    - let developers control almost any aspect of Gazebo
    - are self-contained routines that are easily shared
    - can be inserted & removed from a running system
- allow to choose what functionality to include in their simulations
- should use a plugin when you want to programmatically alter a simulation
- example: move models, respond to events, insert new models given a set of preconditions
    - you want fast interface to gazebo, w/out the overhead of the transport layer
- example: No serialization & deserialization of messages
    - you have some code that could benefit others & want to share it

## Plugin Types

6 types of plugins:

1. World
2. Model
3. Sensor
4. System
5. Visual
6. GUI

Each plugin type is managed by a different component of Gazebo.

- ex: Model plugin is attached to & controls a specific model in Gazebo
- ex: World plugin is attached to a world
- ex: Sensor plugin to a specific sensor
- ex: System plugin is specified on the command line & loads first during a Gazebo startup.
    - this plugin gives the user control over the startup process
    
## Hello WorldPlugin!

A bare bones world plugin contains a class w/ a few member functions.

1. If Gazebo is installed from debians, make sure Gazebo development files. If installed from source, ignore this step. 

    ```
    sudo apt-get install libgazebo6-dev
    ```
    
2. Make a directory & .cc file for the new plugin:
    
    ```
    $ mkdir ~/gazebo_plugin_tutorial
    $ cd ~/gazebo_plugin_tutorial
    $ gedit hello_world.cc
    ```
    
3. Add following code to [hello_world.cc][1]; code below is also located in Gazebo source: [examples/plugins/hello_world/hello_world.cc][2] along w/ CMakeLists.txt file
    1.  The [gazebo/gazebo.hh][3] file includes a core set of basic gazebo functions.
        
        ```c++
        #include <gazebo/gazebo.hh>

        namespace gazebo {
        ```
        
        - doesn't include: `gazebo/physics/physics.hh`, `gazebo/rendering/rendering.hh`, or `gazebo/sensors/sensors.hh` as those should be included on a case by case basis. All plugins must be in the gazebo namespace
    
    2. Each plugin must inherit from a plugin type; in this case `WorldPlugin` class
    
        ```c++
        class WorldPluginTutorial : public WorldPlugin {
            public: WorldPluginTutorial() : WorldPlugin() {
                  printf("Hello World!\n");
            }
        ```
    
    3. `Load` function is a mandatory function. It receives an SDF element that contains the elements & attributes specified in loaded SDF file    
    
    ```c++
        public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

        }
    };
    ```
    
    4. Plugin must be registered w/ the simulation using `GZ_REGISTER_WORLD_PLUGIN` macro
        1. only parameter to this macro is the name of the plugin class
        2. There are matching register macros for each plugin type:
            - `GZ_REGISTER_MODEL_PLUGIN`
            - `GZ_REGISTER_SENSOR_PLUGIN`
            - `GZ_REGISTER_SYSTEM_PLUGIN`
            - `GZ_REGISTER_VISUAL_PLUGIN`
    
    ``` c++
        GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
    }
    ```

## Compiling the Plugin

1. Create the plugin: [CMakeLists.txt][4]

    ```
    $ gedit ~/gazebo_plugin_tutorial/CMakeLists.txt
    ```
    
2. Add the code below:

    ```
    cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

    find_package(gazebo REQUIRED)
    include_directories(${GAZEBO_INCLUDE_DIRS})
    link_directories(${GAZEBO_LIBRARY_DIRS})
    list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

    add_library(hello_world SHARED hello_world.cc)
    target_link_libraries(hello_world ${GAZEBO_LIBRARIES})
    ```
    
    1. new in `gazebo6`: c++11 flags are now required for all downstream software to compile against gazebo, which is done with:
        
        ```
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
        ```
        
3. Create build directory

    ```
    $ mkdir ~/gazebo_plugin_tutorial/build
    $ cd ~/gazebo_plugin_tutorial/build
    ```
    
4. Compile the code
    1. compiling the code will result in a shared library `~/gazebo_plugin_tutorial/build/libhello_world.so`, that can be inserted in a Gazebo simulation

    ```
    $ cmake ../
    $ make
    ```
    
5. Add your library path to the Gazebo library path to the `GAZEBO_PLUGIN_PATH`:

    ```
    $ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_plugin_tutorial/build
    ```

## Using a Plugin

- Once you have a plugin compiled as a shared library (see above), can attach it to a world or model in an SDF file (see [SDF documentation][5] for more info). 
- On startup, Gazebo parses the SDF file, locates the plugin, and loads the code. 
- It is important that Gazebo is capable of finding the plugin. Either the full path to the plugin is specified, or the plugin exists in one of the paths in the GAZEBO_PLUGIN_PATH environment variable.

- Example [world file][6]
    
    ```html
    <?xml version="1.0"?>
    <sdf version="1.4">
        <world name="default">
            <plugin name="hello_world" filename="libhello_world.so"/>
        </world>
    </sdf>
    ```

1. Make a copy of the `hello.world` in `~/gazebo_plugin_tutorial/hello.world` and open it w/ `gzserver`
    
    ```
    $ gzserver ~/gazebo_plugin_tutorial/hello.world --verbose
    ```

- output should be similar to:
    
    ```
    Gazebo multi-robot simulator, version 6.1.0
    Copyright (C) 2012-2015 Open Source Robotics Foundation.
    Released under the Apache 2 License.
    http://gazebosim.org

    [Msg] Waiting for master.
    [Msg] Connected to gazebo master @ http://127.0.0.1:11345
    [Msg] Publicized address: 172.23.1.52
    Hello World!
    ```

[1]: helloworld.cc
[2]: http://bitbucket.org/osrf/gazebo/src/gazebo6/examples/plugins/hello_world
[3]: https://bitbucket.org/osrf/gazebo/src/gazebo6/gazebo/gazebo_core.hh
[4]: CMakeLists.txt
[5]: http://gazebosim.org/sdf.html
[6]: https://bitbucket.org/osrf/gazebo/src/gazebo6/examples/plugins/hello_world/hello.world
[7]: hello.world  
