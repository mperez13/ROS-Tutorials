# Modifying a World

**Link to tutorial:** http://gazebosim.org/tutorials/?tut=modifying_world

Start Gazebo:

```
$ gazebo
```

## Scene Proporties

In `World` tab, select `scene` item. A list of scene properties will be displayed in the list below. Click triangle to expand the properties.

![scene][1]

These properties allow you to change the ambient light. Note: backgrounf color will not change if the Sky is enabled.

## Physical Properties

- In `World` tab, select `physics` item.  A list of physics properties displayed below:
    
    ![physical][2]
    
    - 'enable physics' check box can be used to disable physics while allowing plugins and sensors to continue running.
    - 'real time update rate' parameter specifies in Hz the number of physics updates that will be attempted per second.
        - is number is set to 0, it will run as fast as it can
        - Note: product 'real time update rate' and 'max step size' represents the target 'real time factor' or ratio of simulation time to real-time
    - 'max step size' specifies the time duration in seconds of each physics update step
- in the gravity block:
    - 'x', 'y' and 'z' parameters set global gravity vector component in m/s<sup>2</sup>
- in solver block:
    - 'iterations' parameter specifies the number of iterations to use for iterative LCP solvers (used by ODE & bullet)
    - 'SOR' parameter stands for 
    



    

[1]: images/scene_prop.png
[2]: images/physics_prop.png
