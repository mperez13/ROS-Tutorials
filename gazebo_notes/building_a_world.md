# Building a World

**Link to tutorial:** - http://gazebosim.org/tutorials?tut=build_world&cat=build_world

## Terminology

- **World:** describes a collection of robots and objects(tables, building, etc.) and global parameters(sky, ambient light, physical properties) 

- **Static:** Entities marked as static (those having `<static>true</static>` in SDF) are objects which only have collision geometry
    - all objects not meant to move should be marked as static
- **Dynamic:** Entities marked as dynamic (either missing `<static>` or setting false in SDF) are objects which have both inertia and a collision geometry

## Setup

1. Make sure is [installed][1]
2. Create a working directory for this tutorial
    
    ```
    $ mkdir ~/build_world_tutorial
    $ cd ~/build_world_tutorial
    ```
    
    - in my setup:
        
        ```
        $ mkdir ~/ROS-Tutorials/build_world_tutorial
        $ cd ~/ROS-Tutorials/build_world_tutorial
        ```
3. Start Gazebo (make sure your inside the `build_world_tutorial` directory)
        
    ```
    $ gazebo
    ```
    
    - Should see a world w/ just a ground plane
    
As Kanye West said:

> We're living the future so
> the present is our past.




[1]: http://gazebosim.org/tutorials?cat=install
