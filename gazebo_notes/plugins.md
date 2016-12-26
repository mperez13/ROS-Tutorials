# Overview of Gazebo Plugins

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
3. Add following code to [hello_world.cc][1]
    - code below is also located in Gazebo source: [examples/plugins/hello_world/hello_world.cc][2] along w/ CMakeLists.txt file

    ```
    #include <gazebo/gazebo.hh>

    namespace gazebo {
    ```
    
    The [gazebo/gazebo.hh][3] file includes a core set of basic gazebo functions.
        - doesn't include: `gazebo/physics/physics.hh`, `gazebo/rendering/rendering.hh`, or `gazebo/sensors/sensors.hh` as those should be included on a case by case basis. All plugins must be in the gazebo namespace
    
    ```
        class WorldPluginTutorial : public WorldPlugin {
            public: WorldPluginTutorial() : WorldPlugin() {
                  printf("Hello World!\n");
            }
    ```
    
    
    
    
    ```
            public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
            
            }
        };
        GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
    }
    ```


[1]: helloworld.cc
[2]: http://bitbucket.org/osrf/gazebo/src/gazebo6/examples/plugins/hello_world
[3]: https://bitbucket.org/osrf/gazebo/src/gazebo6/gazebo/gazebo_core.hh
    
    
    
    
    
    
    
    
    
    
