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
    
    - `enable physics` check box can be used to disable physics while allowing plugins and sensors to continue running.
    - `real time update rate` parameter specifies in Hz the number of physics updates that will be attempted per second.
        - is number is set to 0, it will run as fast as it can
        - Note: product `real time update rate` and `max step size` represents the target `real time factor` or ratio of simulation time to real-time
    - `max step size` specifies the time duration in seconds of each physics update step
- in the gravity block:
    - `x`, `y` and `z` parameters set global gravity vector component in m/s<sup>2</sup>
- in solver block:
    - `iterations` parameter specifies the number of iterations to use for iterative LCP solvers (used by ODE & bullet)
    - `SOR` ([successive over-relaxation][3]) parameter can be used to try to speed the convergence of the iterative methods
- The constraints block contains several parameters related to solving constraints:
    - `CFM` ([Constraint Force Mixing][4]) and `ERP` ([Error Reduction Parameter][5]) parameters are used by ODE and bullet
        - CFM & ERP parameters [can be related to linear stiffness and damping coefficients][6]
    - `max velocity` & `surface layer` parameters are used to resolve contacts w/ a plit impulse method
    - contacts will not bounce if:
        - it penetrates deeper than a depth specified by `surface layer`
        - it has a normal velocity less than `max velocity`
- [SDF Physics Documentation][7] gives a description of these parameters.  

**Return to [Categories: Build a World][8]**

[1]: images/scene_prop.png
[2]: images/physics_prop.png
[3]: http://en.wikipedia.org/wiki/Successive_over-relaxation
[4]: http://ode-wiki.org/wiki/index.php?title=Manual:_Concepts#Constraint_Force_Mixing_.28CFM.29
[5]: http://ode-wiki.org/wiki/index.php?title=Manual:_Concepts#Joint_error_and_the_Error_Reduction_Parameter_.28ERP.29
[6]: http://ode-wiki.org/wiki/index.php?title=Manual:_Concepts#How_To_Use_ERP_and_CFM
[7]: http://osrf-distributions.s3.amazonaws.com/sdformat/api/dev.html#physics12
[8]: ../gazebo_categories/build_world.md 
