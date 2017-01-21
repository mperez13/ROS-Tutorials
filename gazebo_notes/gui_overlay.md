# [GUI Overlay][1]

This tutorial describes how to create and use GUI overlay plugins to create custom interfaces for Gazebo.

Gazebo GUI overlay is like a transparent 2D layer that sits on top of the render window.

- QT widgets can be added to this layer through a plugin interface. 
- can show or hide all GUI overlays by clicking `View->GUI Overlays` on the main Gazebo menu bar

## Example 1: Spawn spheres

Creates a button that spawns a sphere 

1. Install the development debian
  
  ```
  sudo apt-get install libgazebo7-dev
  ```

2. Create a working directory
  
  ```
  mkdir ~/gazebo_gui_spawn
  cd ~/gazebo_gui_spawn
  ```

3. Download source code for the GUI overlay plugin

  ```
  wget https://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.hh
  wget https://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/gui_overlay_plugin_spawn/GUIExampleSpawnWidget.cc
  wget https://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/gui_overlay_plugin_spawn/CMakeLists.txt
  ```

4. Look at the header file - [GUIExampleSpawnWidget.hh][2]: 
## Example 2: Display Simulation Time

- Shows how to send data to Gazebo and receive data from Gazebo

[1]: http://gazebosim.org/tutorials?tut=gui_overlay&cat=user_input
[2]: ../gazebo_gui_spawn/GUIExampleSpawnWidget.hh
