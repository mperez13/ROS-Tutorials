# [GUI Overlay][1]

This tutorial describes how to create and use GUI overlay plugins to create custom interfaces for Gazebo.

Gazebo GUI overlay is like a transparent 2D layer that sits on top of the render window.

- QT widgets can be added to this layer through a plugin interface. 
- can show or hide all GUI overlays by clicking `View->GUI Overlays` on the main Gazebo menu bar

The first example shows how to create a button that adds a model with one click by using plugin.

The 2nd example shows how to add a timer to Gazebo.

## Example 1: Spawn spheres

Creates a button that spawns a sphere 

1. Install the development debian
  
  ```
  sudo apt-get install libgazebo7-dev
  ```

2. Create a working directory and add following file:
  - header file - [GUIExampleSpawnWidget.hh][2]
  - source file - [GUIExampleSpawnWidget.cc][3]
  - [CMakeList.txt][12]

3. Compile the plugin
  
  ```
  cd ~/gazebo_gui_spawn
  mkdir build
  cd build
  cmake ../
  make
  ```
  
4. Make sure Gazebo can find the plugin by appending the `build` directory to the `GAZEBO_PLUGIN_PATH` environment variable:

  ```
  cd ~/gazebo_gui_spawn/build
  export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
  ```
  Previous command will only work in current shell, so you will have to export it everytime you run it.  Can be helpful to create a shell script file.
  
  - I created [boot_spawn.sh][10] and [gz_setup.sh][11] to do this( remember to allow access w/ `chmod +x file_name`) 
  
  - bootgui.sh: 
    - source the gz_setup.sh
    - can also add command to run world file
    
  - gz_setup.sh: 
      - add all environment variables necessary
  
  - To run: use commands `./boot_spawn.sh` inside the `gazebo_gui_spawn` directory
    
5. Need to tell Gazebo that it should load the overlay plugin
  - There are 2 methods to accomplish this:
    - **SDF world file:** modify a world SDF file to contain GUI plugin
      
      ```xml
      <?xml version="1.0" ?>
      <sdf version="1.5">
        <world name="default">

          <gui>
            <plugin name="sample" filename="libgui_example_spawn_widget.so"/>
          </gui>

          <!-- A global light source -->
          <include>
            <uri>model://sun</uri>
          </include>
          <!-- A ground plane -->
          <include>
            <uri>model://ground_plane</uri>
          </include>
        </world>
      </sdf>
      ```
      
    - **GUI INI file:** Modify `~/.gazebo/gui.ini` file by adding the following lines so the plugin is loaded every time Gazebo is run
      
      ```c++
      [overlay_plugins]
      filenames=libgui_example_spawn_widget.so
      ```

6. When Gazebo is running, a button should appear in the upper left if the render window
  - If a SDF world file w/ GUI plugin:
    
    ```
    gazebo spawn_widget_example.world
    ```
    
  - or if you modified `~/.gazebo/gui.ini`
    
    ```
    gazebo
    ```
    
    ![image gui overlay][4]

7. Click on the button to spawn spheres.

  ![image gui overlay 2][5]

## Example 2: Display Simulation Time

- Shows how to send data to Gazebo and receive data from Gazebo

1. Create working directory 'gazebo_gui_time' w/ following files:
  - header file - [GUIExampleTimeWidget.hh][6]
  - source file - [GUIExampleTimeWidget.cc][7]
  - [CMakeLists.txt][13]
  
2. Follow same steps as in the first example to compile the plugin, tell Gazebo where to find it & load it via `gui.ini` or SDF world file
  - **tip**: can add both plugin to `gui.ini`, which will load both spawn sphere plugin and time plugin 
    
    ```
    [overlay_plugins]
    filenames=libgui_example_spawn_widget.so:libgui_example_time_widget.so
    ```
    
    - a new text box to the right of the spawn button should show the simulation time.
  
      ![image ex 2][8]

[1]: http://gazebosim.org/tutorials?tut=gui_overlay&cat=user_input
[2]: ../gazebo_gui_spawn/GUIExampleSpawnWidget.hh
[3]: ../gazebo_gui_spawn/GUIExampleSpawnWidget.cc
[4]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/gui_overlay/files/spawn_button.png
[5]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/gui_overlay/files/spawn_sphere.png
[6]: ../gazebo_gui_time/GUIExampleTimeWidget.hh
[7]: ../gazebo_gui_time/GUIExampleTimeWidget.cc
[8]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/gui_overlay/files/time.png
[9]: ../gazebo_categories/user_input.md
[10]: ../gazebo_gui_spawn/boot_spawn.sh
[11]: ../gazebo_gui_spawn/gz_setup.sh
[12]: ../gazebo_gui_spawn/CMakeLists.txt
[13]: ../gazebo_gui_time/CMakeLists.txt
