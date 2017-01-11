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


## Graphical Client


## Server + Graphical Client in One

## Plugins



















[1]: http://gazebosim.org/tutorials?tut=components&cat=get_started
[2]: http://gazebosim.org/sdf.html
[3]: http://bitbucket.org/osrf/gazebo_models
