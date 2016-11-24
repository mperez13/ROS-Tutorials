#Make a Mobile Robot

Link to Tutorial: http://gazebosim.org/tutorials?tut=build_robot

**Description**: Tutorial demonstrates Gazebo's basic model management & basic model representation inside the model databse by taking the user through the process of creating a two wheeled mobile robot that uses a differential drive mechanism for movement.

##Setup your model directory

1. Create model directory:
    `mkdir -p ~/.gazebo/models/my_robot`
2. Create a model config file: [model.config](https://github.com/mperez13/ROS-Tutorials/blob/master/.gazebo/models/my_robot/model.config)
    - file describes the robot w/ some extra meta data
3. Create a '~/.gazebo/models/my_robot/model.sdf' file: [model.sdf](https://github.com/mperez13/ROS-Tutorials/blob/master/.gazebo/models/my_robot/model.sdf)
    - file contains the necessary tags to instantiate model name `my_robot` using Gazebo linked against SDF version 1.4

##Build the Model's Structure

- Create a rectangulat base w/ 2 wheels

The first step: layout the basic shape of the model

- make model `static`, which means it will be ignored by the physics engine; so model will stay in one place & allow us to properly align all the components

1. Make model static
2. Add rectangular base 
    - have created a `box` w/ size `0.4 x 0.2 x 0.1` meters
    - `collision` element specifies the shape used by the collision detection engine
    - `visual` element specifies the shape used by the rendering engine
    
    1. for most cases 'collision' and 'visual' elements are the same
        1. most common use for elements to be different is to have simplified  `collision` paired w/ `visual` element that uses a complex mesh, which would help performance 
3. Add a caster to the robot
4. Add Left Wheel
5. Add right wheel
6. Make the model dynamic by setting static to false
7. 

