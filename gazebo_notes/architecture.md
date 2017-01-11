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


|Library Name ->|Dependencies|External API|Internal API|Advertised Topics|Subscribed Topics|Notes|
|-----|-----|-----|-----|-----|-----|||
|**Communication**|Protobuf and boost::ASIO||None|None|None|Acts as the communication & transport mechanism for Gazebo; currently supports only publish/subscribe, but it's possible to use [RPC][2] w/ minimal effort. This library is used by almost all subsequent libraries|
|**Physics**|Dynamics engine (with internal collision detection)|provides a simple & generic interface to physics simulation|Defines a fundamental interface to the physics library for 3rd party dynamic engines||||
|**Rendering**|||||||
|**Sensor Generation**|||||||
|**GUI**|||||||

### Physics Library

### Rendering Library

### Sensor Generation

### GUI

### Plugins

[1]: http://gazebosim.org/tutorials?tut=architecture&cat=get_started
[2]: https://en.wikipedia.org/wiki/Remote_procedure_call
