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

###### `GAZEBO_MODEL_PATH`

###### `GAZEBO_RESOURCE_PATH`

###### `GAZEBO_MASTER_URI`

###### `GAZEBO_PLUGIN_PATH`

###### `GAZEBO_MODEL_DATABASE_URI`


## Gazebo Server


## Graphical Client


## Server + Graphical Client in One

## Plugins



















[1]: http://gazebosim.org/tutorials?tut=components&cat=get_started
[2]: http://gazebosim.org/sdf.html
