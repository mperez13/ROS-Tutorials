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


|Library Name ->|Communication|Physics|Rendering|Sensor Generation|GUI|
|-----|-----|-----|-----|
|**Dependencies**||||||
|**External API**||||||
|**Internal API**||||||
|**Advertised Topics**||||||
|**Subscribed Topics**||||||
|****||||||



### Communication Library

- **Dependencies**: Protobuf and boost::ASIO
- **External API**:
- **Internal API**: None
- **Advertised Topics**: None 
- **Subscribed Topics**: None

##### This library is used by almost all subsequent libraries

- acts as the communication & transport mechanism for Gazebo
- currently supports only publish/subscribe, but it's possible to use [RPC][2] w/ minimal effort

### Physics Library

### Rendering Library

### Sensor Generation

### GUI

### Plugins

[1]: http://gazebosim.org/tutorials?tut=architecture&cat=get_started
[2]: https://en.wikipedia.org/wiki/Remote_procedure_call
