# Architecture

**Description to [Tutorial][1]**: Architecture of Gazebo.

**Gazebo**:

- uses distributed architecture w/ separate libraries for physics simulation, rendering, user interface, communication, and sensor generation.
- provides 2 executable programs for running simulations:
  - `gzserver`: simulating the physics, rendering and sensors
  - `gzclient`: provides a GUI to visualize & interact w/ the simulation

###### The Client and server communicate using the gazebo communication library.

## Communication Between Processes

- Communication library currently uses the open source `Google Protobuf` for the message serialization and `boost::ASIO` for the transport mechanism
  - supports publish/subscribe communication paradigm
  - **example**: simulated world publishes body pose updates and sensor generation & GUI will consume these messages to produce output.

###### This mechanism allows running simulation & controlling aspects of Gazebo.

## System

### Gazebo Master

- topic name server
- provides namelookup & topic management

###### Single master can handle multiple physics simulations, sensor generators, and GUIs.


||Communication|Physics|Rendering|Sensor Generation|GUI|
|-----|-----|-----|-----|-----|-----|
|**Dependencies**|Protobuf and boost::ASIO|Dynamics engine (with internal collision detection)|OGRE|Rendering Library, Physics Library|Rendering Library, Qt|
|**External API**||Provides a simple and generic interface to physics simulation|Allows for loading, initialization, and scene creation|Provide functionality to initialize and run a set of sensors|None|
|**Internal API**|None|Defines a fundamental interface to the physics library for 3rd party dynamic engines.|Store metadata for visualization, call the OGRE API for rendering.|TBD|None|
|**Advertised Topics**|None|||||
|**Subscribed Topics**|None|||||

**Communication Library**:

- Acts as the communication and transport mechanism for Gazebo
- currently supports only publish/subscribe, but it's possible to use [RPC][2] w/ minimal effort. This library is used by almost all subsequent libraries

**Physics Library**:

- provides simple and generic interface to fundamental simulation components, including rigid bodies, collision shapes and joints for representing articulation constraints
- interface has been integrated w/ four open-source physics engines:
  - [Open Dynamics Engine(ODE)][2]  
  - [Bullet][3] 
  - [Simbody][4]
  - [Dynamic Animation and Robotics Toolkit (DART)][5]
  
  Each of these physics engine can load a model described in the [SDF][6] using XML

**Rendering Library**:

- uses OGRE to provide a simple interface for rendering 3D scenes to both the GUI and sensor libraries
- includes lighting, textures, and sky simulation

It is possible to write plugins for the rendering engine

**Sensor Generation**:

- implements all the various types of sensors, listens to world state updates from physics simulator & produces output specified by the instantiated sensors

**GUI**:

- uses Qt to create graphical widgets for users to interact w/ the simulation
- user may control the flow of time by pausing or changing time step size via GUI widgets
- user may also modify the scene by adding, modifying, or removing models
- there are some tools for visualizing and logging simulated sensor data

#### Plugins

- physics, sensor, and rendering libraries support plugins

Plugins provide users w/ access to the respective libraries w/out using the communication system.

Return to [Get Started][7]

[1]: http://gazebosim.org/tutorials?tut=architecture&cat=get_started
[2]: http://ode.org/
[3]: http://bulletphysics.org/
[4]: https://simtk.org/home/simbody
[5]: http://dartsim.github.io/
[6]: http://sdformat.org/
[7]: ../../gazebo_categories/get_started.md 



[2]: https://en.wikipedia.org/wiki/Remote_procedure_call
