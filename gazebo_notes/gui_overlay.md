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

5. Look at source file - [GUIExampleSpawnWidget.cc][3]

6. Compile the plugin
  
  ```
  cd ~/gazebo_gui_spawn
  mkdir build
  cd build
  cmake ../
  make
  ```
  
7. Make sure Gazebo can find the plugin by appending the `build` directory to the `GAZEBO_PLUGIN_PATH` environment variable:
  - I created [bootgui.sh][10] and [gz_setup.sh][11] to do this( remember to allow access w/ `chmod +x file_name`) 
  
  - bootgui.sh: 
    - source the gz_setup.sh
    - can also add command to run world file
    
  - gz_setup.sh: 
      - add all environment variables necessary
  
  - To run: use commands `./bootgui.sh` inside the `gazebo_gui_spawn` directory
    
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

9. When Gazebo is running, a button should appear in the upper left if the render window
  - If a SDF world file w/ GUI plugin:
    
    ```
    gazebo spawn_widget_example.world
    ```
    
  - or if you modified `~/.gazebo/gui.ini`
    
    ```
    gazebo
    ```
    
    ![image gui overlay][4]

10. Click on the button to spawn spheres.

  ![image gui overlay 2][5]

## Example 2: Display Simulation Time

- Shows how to send data to Gazebo and receive data from Gazebo

1. Create working directory
  
  ```
  mkdir ~/gazebo_gui_time
  cd ~/gazebo_gui_time
  ```

2. Download source code for the GUI overlay plugin
  
  ```
  wget https://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.hh
  wget https://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/gui_overlay_plugin_time/GUIExampleTimeWidget.cc
  wget https://bitbucket.org/osrf/gazebo/raw/gazebo7/examples/plugins/gui_overlay_plugin_time/CMakeLists.txt
  ```

3. Look at the header file - [GUIExampleTimeWidget.hh][6]
  - plugin inherits the GUIPlugin class & use Qt's `Q_OBJECT` macro
    
    ```c++
    class GAZEBO_VISIBLE GUIExampleTimeWidget : public GUIPlugin
    {
      Q_OBJECT
    ```
  
  - use `SetSimTime` signal as a thread safe mechanism to update the displayed simulation time
    
    ```c++
    // A signal used to set the sim time line edit
    // _string -> String represention of sim time
    signals: void SetSimTime(QString _string);
    ```
    
  - `OnStats` callback is used to receive information from Gazebo
  
    ```c++
    // Callback that received world statistics messages.
    // _msg World statistics message that is received.
    protected: void OnStats(ConstWorldStatisticsPtr &_msg);
    ```
  
  - use Gazebo's transport mechanism to receive messages from Gazebo
    
    ```
    // Node used to establish communication w/ gzserver
    private: transport::NodePtr node;
    
    // Subscriber to world statistics messages
    private: transport::SubscriberPtr statsSub;
    ```

4. Look at the source file - [GUIExampleTimeWidget.cc][7]
  - in the constructor, create QLabel to display the time & connect it to the `SetSimeTime` signal
    
    ```c++
    // Create a time label
    QLabel *timeLabel = new QLabel(tr("00:00:00.00"));

    // Add the label to the frame's layout
    frameLayout->addWidget(label);
    frameLayout->addWidget(timeLabel);
    connect(this, SIGNAL(SetSimTime(QString)), timeLabel, SLOT(setText(QString)), Qt::QueuedConnection);
    ```
  
  - constructor also connect to Gazebo's `~/world_stats` topic
    
    ```c++
    // Create a node for transportation
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init("default");
    this->statsSub = this->node->Subscribe("~/world_stats", &GUIExampleTimeWidget::OnStats, this);
    ```
    
  - a msg is received, `OnStats` function is called & dislayed time is updated
    
    ```c++
    void GUIExampleTimeWidget::OnStats(ConstWorldStatisticsPtr &_msg)
    {
      this->SetSimTime(QString::fromStdString(this->FormatTime(_msg->sim_time())));
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
