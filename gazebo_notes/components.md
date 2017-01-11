# Components

**Description to [Tutorial][1]**: This page describes each of the items involved in running a Gazebo simulation. The items are World Files, Model Files, Environment Variables, Gazebo Server, Graphical Client, Server + Graphical Client in one, Plugins.

## World Files

- contains all elements in a simulation, including robots, lights, sensors, and static objects
- is formatted using [SDF (Simulation Description Format)][2] and has typically `.world` extension
  - Gazebo server reads this files to generate and populate world
- number of example worlds are shipped w/ Gazebo 
  - located in `<install_path>/share/gazebo-<version>/worlds`

## Model Files

- uses the same [SDF][2] format as world files, but should only contain a single `<model> ... </model>`
- file purpose is to facilitate model reuse and simplify world files

- Once file is created, it can be included in a world file using the following SDF syntax
  
  ```
  <include>
    <uri>model://model_file_name</uri>
  </include>
  ```

- [online model database][3] contains number of models
- If internet access is available, you can insert any model from the database & necessary content will be downloaded at runtime.

## Environment Variables

- uses a number of environment variables to locate files & set up communcation between the server and clients
- Starting Gazebo 1.9.0, default values that work for most cases are compiled in. This means you don't need to set any variables.

#### List of Variables

`GAZEBO_MODEL_PATH`: colon-separated set of directories where Gazebo will search for models

`GAZEBO_RESOURCE_PATH`: colon-separated set of directories where Gazebo will search for other resources such as world and media files

`GAZEBO_MASTER_URI`: URI of the Gazebo master; specifies IP and port where server will be started & tells the clients where to connect to

`GAZEBO_PLUGIN_PATH`: colon-separated set of directories where Gazebo will search for the plugin shared libraries at runtime

`GAZEBO_MODEL_DATABASE_URI`: URI of the online model database where Gazebo will download models from

**These defaults are also included in a shell script**:
  
  ```
  source <install_path>/share/gazebo/setup.sh
  ```

- To modify Gazebo's behavior, should first source the shell script listed above, then modify the variables that it sets.

## Gazebo Server

- parses a world description file given on the command line & then simulates the world using a physics and sensor engine
- can be started using the following command (Note: server doesn't include any graphics; it's meant to run headless)

  ```
  gzserver <world_filename>
  ```
  
##### `<world_filename>` can be:

1. relative to the current directory
2. an absolute path
3. relative to a path component in `GAZEBO_RESOURCE_PATH`

##### Worlds that are shipped w/ Gazebo are located in `<install_path>/share/gazebo-<version_number>/worlds`

- example: to use `empty.world` (which is shipped w/ Gazebo), use following command:

  ```
  gzserver worlds/empty.world
  ```
  
## Graphical Client

- connects to a running `gzserver` & visualize the elements
- is a tool that allows you to modify the running simulation

##### The graphical client is run using:

  ```
  gzclient
  ```

## Server + Graphical Client in One

- `gazebo` command combines server and client in one executable.

##### Instead of running `gzserver worlds/empty.world` and then `gzclient`, you can:

  ```
  gazebo worlds/empty.world
  ```

## Plugins

- provides a simple & convenient mechanism to interface w/ Gazebo
- can either be loaded on the command line, or specified in a world/model file (see the [SDF][2] format)

##### Plugins specified on the command line are loaded first, then plugins specified in the world/model files are loaded.
 
##### Most plugins are loaded by the server, but can also be loaded by the graphical client to facilitate custom GUI generation.

##### Example of loading a plugin on the command line:

```
gzserver -s <plugin_filename> <world_file>
```

##### The same mechanism is used by the graphical client:

```
gzclient -g <plugin_filename>
```

###### For more information refer to the [plugins overview][4] page.

[1]: http://gazebosim.org/tutorials?tut=components&cat=get_started
[2]: http://gazebosim.org/sdf.html
[3]: http://bitbucket.org/osrf/gazebo_models
[4]: plugins.md 
