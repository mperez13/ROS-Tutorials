# Building a World

**Link to tutorial:** - http://gazebosim.org/tutorials?tut=build_world&cat=build_world

## Terminology

- **World:** describes a collection of robots and objects(tables, building, etc.) and global parameters(sky, ambient light, physical properties) 

- **Static:** Entities marked as static (those having `<static>true</static>` in SDF) are objects which only have collision geometry
    - all objects not meant to move should be marked as static
- **Dynamic:** Entities marked as dynamic (either missing `<static>` or setting false in SDF) are objects which have both inertia and a collision geometry

## Setup

1. Make sure Gazebo is [installed][1]
2. Create a working directory for this tutorial
    
    ```
    $ mkdir ~/build_world_tutorial
    $ cd ~/build_world_tutorial
    ```
    
    - Notes: in my setup
        
        ```
        $ mkdir ~/ROS-Tutorials/build_world_tutorial
        $ cd ~/ROS-Tutorials/build_world_tutorial
        ```
3. Start Gazebo (make sure your inside the `build_world_tutorial` directory)
        
    ```
    $ gazebo
    ```
    
    - Should see a world w/ just a ground plane
    
## Adding Objects

There are two ways you can add objects to Gazebo

1. Simple shapes (located above the render window)
    
    ![simple shapes][2]

2. [Model Database][3] 

    ![model database][4]

### Adding Simple Shapes

Add shapes by simply clicking on the appropriate icon.
Each shape is of unit size:

- Box: 1 x 1 x 1 meter
- Sphere: 1 meter diameter
- Cylinder: 1 meter diameter, 1 meter length

To add:

1. Select the box icon and move mouse onto the render window. You should see a box that moves with your mouse. Left click to place in the position you want.
    - Repeat with sphere and cylinder

    ![adding a shape to gazebo][5]

### [Adding Model from the Model Database][6]

## Position Models

The pose of each model can be altered through the translate and rotate tools:

![position controls][7]

### Translation

Translation tools(1st of the three in the above image) allows you to move objects along the x, y, z axes.

1. Select the tool now and click on the object you want to move,
    - a three axes visual marker will appear over the object. Now you can move the object in the x, y, z directions
    - can also click on the object and drag it
    - can control which axis the object moves along by pressing and holding the x, y, or z key while dragging the object

    ![translation visual markers][8]
    
### Rotation

The rotate tool (middle of the three in the above image) allows you to orient a model around the x, y, z axes

1. Select the tool and click on the object you want to move.
    - three ring-shaped visual marker will appear over the object, which allows you to rotate the object around the x, y, and z axes.
    
    ![rotation model][9]

### Scale

The scale tool (left most in the above image) to resize a model in the x, y, z directions.  Currently scale tool only works w/ simple shapes (box, cylinder, sphere).

1. Select this tool now and click on a simple shape.
    - a three axes visual marker will appear over the object, which allows you to scale the x, y, and z dimensions of the object
    
    ![scale_position][10]


## Delete Models

Models can be deleted by selecting them and hitting the `Delete` key or by clicking the model and selecting `Delete`

## Saving a World

A world can be saved through `File` menu

1. Select `File` menu and choose `Save As`
    - a pop-up will appear you to enter a new filename.
    - Enter `my_world.sdf` and click okay

## Loading a world

A saved world may be loaded on the command line:

```
gazebo my_world.sdf
```

> You must be in current working directory or you must specify the complete path

[1]: http://gazebosim.org/tutorials?cat=install
[2]: images/empty_world_simple_shapes_highlighted.png 
[3]: https://bitbucket.org/osrf/gazebo_models
[4]: images/adding_model_database.png
[5]: images/add-shape-to-gazebo.png
[6]: add_model_from_model_database.md 
[7]: images/position_models.png 
[8]: images/translation_model.png 
[9]: images/rotation_model.png
[10]: images/scale_model.png
