# [Razer Hydra][1]

Gazebo supports the [Razer Hydra controller][3]. You will be able to use this motion and orientation detection controller to interact with your models in Gazebo.

## Razer Hydra Configuration

Open terminal and run commnand:

```
echo -e "ATTRS{idProduct}==\"0300\",ATTRS{idVendor}==\"1532\",ATTR{bInterfaceNumber}==\"00\",TAG=\"hydra-tracker\"\nSUBSYSTEM==\"hidraw\",TAGS==\"hydra-tracker\", MODE=\"0666\", SYMLINK+=\"hydra\"" > 90-hydra.rules
```

- This will create the file `90-hydra.rules`

We need to access to the controller w/out root access

```
sudo cp 90-hydra.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

## Gazebo Compilation w/ Razer Hydra Support

Need to install the optional `libusb` dependency

```
sudo apt-get install libusb-1.0-0-dev
```

Once Hydra is configured and extra dependency satisfied, should be able to compile Gazebo from source w/ Hydra support.

During execution of `cmake` command, should see this msg confirming that SDK is found:

```
-- Looking for libusb-1.0 - found. Razer Hydra support enabled.
```

## Using Hydra w/in Gazebo

1. Load Hydra plugin in your world file

```
<!-- Load the plugin for Razer Hydra -->
<plugin name="hydra" filename="libHydraPlugin.so"></plugin>
```

- plugin will automatically publish msgs on the topic `/hydra`

2. write a plugin that subscribes to the hydra topic & make something interesting
    - For this tutorial: going to move a sphere by using the right joystick of Hydra
    - A `HydraDemoPlugin` is available in Gazebo in the `plugins/` directory
    - Plugin Code:
    
        ```c++
        #include <boost/bind.hpp>
        #include <gazebo/gazebo.hh>
        #include <gazebo/physics/physics.hh>
        #include "gazebo/transport/transport.hh"
        #include "HydraDemoPlugin.hh"

        using namespace gazebo;

        GZ_REGISTER_MODEL_PLUGIN(HydraDemoPlugin)

        /////////////////////////////////////////////////
        HydraDemoPlugin::HydraDemoPlugin()
        {
        }

        /////////////////////////////////////////////////
        HydraDemoPlugin::~HydraDemoPlugin()
        {
        }

        /////////////////////////////////////////////////
        void HydraDemoPlugin::OnHydra(ConstHydraPtr &_msg)
        {
          boost::mutex::scoped_lock lock(this->msgMutex);
          this->hydraMsgPtr = _msg;
        }

        /////////////////////////////////////////////////
        void HydraDemoPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
          // Get the world name.
          this->model = _parent;
          this->world = this->model->GetWorld();

          // Subscribe to Hydra updates by registering OnHydra() callback.
          this->node = transport::NodePtr(new transport::Node());
          this->node->Init(this->world->GetName());
          this->hydraSub = this->node->Subscribe("~/hydra",
              &HydraDemoPlugin::OnHydra, this);

          // Listen to the update event. This event is broadcast every
          // simulation iteration.
          this->updateConnection = event::Events::ConnectWorldUpdateBegin(
              boost::bind(&HydraDemoPlugin::Update, this, _1));
        }

        /////////////////////////////////////////////////
        void HydraDemoPlugin::Update(const common::UpdateInfo & /*_info*/)
        {
          boost::mutex::scoped_lock lock(this->msgMutex);

          // Return if we don't have messages yet
          if (!this->hydraMsgPtr)
            return;

          // Read the value of the right joystick.
          double joyX = this->hydraMsgPtr->right().joy_x();
          double joyY = this->hydraMsgPtr->right().joy_y();

          // Move the sphere.
          this->model->SetLinearVel(math::Vector3(-joyX * 0.2, joyY * 0.2, 0));

          // Remove the message that has been processed.
          this->hydraMsgPtr.reset();
        }
        ```
    - Include our model plugin in a world file. (available under `worlds/hydra_demo.world`)
        
        ```xml
        <?xml version="1.0" ?>
        <sdf version="1.4">
          <world name="default">

            <!-- A ground plane -->
            <include>
              <uri>model://ground_plane</uri>
            </include>

            <!-- A global light source -->
            <include>
              <uri>model://sun</uri>
            </include>

            <!-- Load the plugin for Razer Hydra -->
            <plugin name="hydra" filename="libHydraPlugin.so">
              <pivot>0.04 0 0</pivot>
              <grab>0.12 0 0</grab>
            </plugin>

            <!-- A sphere controlled by Hydra-->
            <model name="sphere">
              <pose>0 0 0 0 0 0</pose>
              <link name="link">
                <collision name="collision">
                  <geometry>
                    <sphere>
                      <radius>0.5</radius>
                    </sphere>
                  </geometry>
                </collision>
                <visual name="visual">
                  <geometry>
                    <sphere>
                      <radius>0.5</radius>
                    </sphere>
                  </geometry>
                </visual>
              </link>

              <plugin name='sphere_ctroller' filename='libHydraDemoPlugin.so'></plugin>

            </model>

        </world>
        </sdf>
        ```    
3. Run Gazebo and use Hydra's right joystick to move the sphere.  Do not forget to plug your Hydra and then:
    
    ```
    gazebo worlds/hydra_demo.world
    ```

**Return [Gazebo Categories: User Input][2]

[1]: http://gazebosim.org/tutorials?tut=hydra&cat=user_input
[2]: ../gazebo_notes/user_input.md
[3]: http://en.wikipedia.org/wiki/Razer_Hydra
