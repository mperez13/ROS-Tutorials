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

2. Created a working directory in order to combine both examples:
  
  ```
  mkdir ~/gazebo_gui
  cd ~/gazebo_gui
  ```

3. For example \#1 create the following files:
  - header file - [GUIExampleSpawnWidget.hh][2] 
  - source file - [GUIExampleSpawnWidget.cc][3] 

4. For example \#2, create the following files:
  - header file - [GUIExampleTimeWidget.hh][6]
  - source file - [GUIExampleTimeWidget.cc][7]

5. Create a [CMakeList.txt][] file including both example.

6. Compile the plugin
  
  ```
  cd ~/gazebo_gui
  mkdir build
  cd build
  cmake ../
  make
  ```
  
7. Make sure Gazebo can find the plugin by appending the `build` directory to the `GAZEBO_PLUGIN_PATH` environment variable:
  - I created [bootgui.sh][10] and [gz_setup.sh][11] to do this
    - [remember to allow access w/ `chmod +x file_name`] 
  
  - bootgui.sh: source the gz_setup.sh
    - can also add command to run world file
    
  - gz_setup.sh: 
      - add all environment variables necessary
      
  - To run: use commands `./bootgui.sh`

8. Need to tell Gazebo that it should load the overlay plugin
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
5. Follow same steps as in the first example to compile the plugin, tell Gazebo where to find it & load it via `gui.ini` or SDF world file
  - **tip**: can add both plugin to `gui.ini`, which will load both spawn sphere plugin and time plugin 
    
    ```
    [overlay_plugins]
    filenames=libgui_example_spawn_widget.so:libgui_example_time_widget.so
    ```

6. a new text box to the right of the spawn button should show the simulation time.
  
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
[10]: ../gazebo_gui_spawn/bootgui.sh
[11]: ../gazebo_gui_spawn/gz_setup.sh
