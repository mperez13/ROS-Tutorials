# Quick Start 

**Description to [Tutorial][1]**: Shows how to run Gazebo, run Gazebo with a robot, location of the worlds, and client and server separation.

## Run Gazebo

Three steps will run Gazebo w/ a default world.

1. [Install][2] Gazebo
2. Open terminal
3. Start Gazebo by entering following at the command prompt.

  ```
  gazebo
  ```
## Run Gazebo w/ Robot

Open a terminal and enter the following command

```
gazebo worlds/pioneer2dx.world
```

## Where are the worlds located

World files are located in a versioned system directory. (`/usr/share/gazebo-7` on Ubuntu)

- For Gazebo 7.0 installation on OS X using Homebrew, type the following to see complete list of worlds.

  ```
  ls /usr/local/share/gazebo-7/worlds
  ```

## Client and Server Separation

#### gzserver

- `gzserver` executable runs the physics update-loop and sensor data generation
- core of Gazebo and can be used independently of a graphical interface
- phrase "run headless" (used in the forums) equates to running only the `gzserver`

#### gzclient

- `gzclient` executable runs a GT based user interface
  -  QT provides nice visualization of simulation and convenient controls over various simulation properties.

#### Run these executables

###### Open terminal and run the server:

```
gzserver
```

###### Open another terminal and run the graphical client:

```
gzclient
```

###### Should see the Gazebo user interface. Restart `gzclient` applicaition as often as you want & even run multiple interfaces.

[1]: http://gazebosim.org/tutorials?tut=quick_start&cat=get_started
[2]: http://gazebosim.org/tutorials?cat=install











