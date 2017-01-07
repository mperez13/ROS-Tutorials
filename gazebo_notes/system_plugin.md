# System Plugin

[**Link to Tutorial**][1]

**Description**: This tutorial will create a source file that is a system plugin for gzclient designed to save images into the directory `/tmp/gazebo_frames`

## Pre-req

[Overview of Plugins][2]

## Code

[source code][3]

##### Create a file `system_gui.cc`:

  ```
  $ cd ~/gazebo_plugin_tutorial
  $ gedit system_gui.cc
  ```

##### Add following into `system_gui.cc`:

```c++
#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

namespace gazebo {
    class SystemGUI : public SystemPlugin 
    {
        // \brief Destructor
        public: virtual ~SystemGUI() {
            this->connections.clear();
            if (this->userCam)
                this->userCam->EnableSaveFrame(false);
            this->userCam.reset();
        }

        // Load and Init functions must not block and are called at startup, before Gazebo is loaded

        // \brief Called after the plugin has been constructed.
        public: void Load(int /*_argc*/, char ** /*_argv*/) 
        {
            this->connections.push_back(
                event::Events::ConnectPreRender(boost::bind(&SystemGUI::Update, this)));
        }

        // \brief Called once after Load
        private: void Init() {
        }

        /*On first Update, we get a pointer to the user camera and enable saving of frames*/
        // \brief Called every PreRender event. See the Load function.
        private: void Update() 
        {
            if (!this->userCam) 
            {
                // Get a pointer to the active user camera
                this->userCam = gui::get_active_camera();

                // Enable saving frames
                this->userCam->EnableSaveFrame(true);

                // Specify the path to save frames into
                this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");
            }

            // Get scene pointer
            rendering::ScenePtr scene = rendering::get_scene();

            // Wait until the scene is initialized.
            if (!scene || !scene->GetInitialized())
                return;

            // Look for a specific visual by name.
            if (scene->GetVisual("ground_plane"))
                std::cout << "Has ground plane visual\n";
        }

        /// Pointer the user camera.
        private: rendering::UserCameraPtr userCam;

        /// All the event connections.
        private: std::vector<event::ConnectionPtr> connections;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}
```

## Compiling Camera Plugin

##### Add following to `~/gazebo_plugin_tutorial/CMakeLists.txt`

```
add_library(system_gui SHARED system_gui.cc)
target_link_libraries(system_gui ${GAZEBO_LIBRARIES})
```

##### Rebuild & you should end up w/ a libsystem_gui.so library
    
```
$ cd ~/gazebo_plugin_tutorial/build
$ cmake ../
$ make
```

> When the plugin is build, you might get a warning.
  
  ![running system plugin][4]

##### Remember to set your library path to the GAZEBO_PLUGIN_PATH. For my setup:

```
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/ROS-Tutorials/gazebo_plugin_tutorial/build
```

## Run the Plugin

##### Start gzserver in the background:
    
```
$ gzserver &
```

> when you run this, you should get something like this:
    
  ![running gzserver][5]

##### Run the client w/ plugin:

```
$ gzclient -g libsystem_gui.so
```

##### Inside `/tmp/gazebo_frames` you should see many saved images from the current plugin

> The saved images are a snapshot of what is done in gazebo while gzserver is running.

> Tried to changed `/tmp/gazebo_frames` to `~/ROS-Tutorials/gazebo_plugin_tutorial/tmp/gazebo_frames`; Did not save images to this folder. Will try later to work around this


##### To terminate press `Ctrl-C`.

##### Remember to also terminate the background server process after you quit the client.

- In the same terminal, bring last process to foreground:

      ```
      $ fg
      ```
  - Press `Ctrl-C` to abort the process. 
- Alternatively, just kill `gzserver` process:
    
    ```
    $ killall gzserver
    ```

[1]: http://gazebosim.org/tutorials?tut=system_plugin&cat=write_plugin
[2]: gazebo_notes/plugins.md
[3]: https://bitbucket.org/osrf/gazebo/src/gazebo5/examples/plugins/system_gui_plugin
[4]: images/run_system_gui_cc.png 
[5]: images/start_gzserver.png
