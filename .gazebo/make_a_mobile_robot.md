#Make a Mobile Robot

Link to Tutorial: http://gazebosim.org/tutorials?tut=build_robot

**Description**: Tutorial demonstrates Gazebo's basic model management & basic model representation inside the model databse by taking the user through the process of creating a two wheeled mobile robot that uses a differential drive mechanism for movement.

##Setup your model directory

1. Create model directory:
    `mkdir -p ~/.gazebo/models/my_robot`
2. Create a model config file: [model.config]()
    - file describes the robot w/ some extra meta data
3. Create a '~/.gazebo/models/my_robot/model.sdf' file: [model.sdf]()
    - file contains the necessary tags to instantiate model name `my_robot` using Gazebo linked against SDF version 1.4

