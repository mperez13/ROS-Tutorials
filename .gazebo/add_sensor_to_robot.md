#Add a Sensor to a Robot

Link to Tutorial - http://gazebosim.org/tutorials/?tut=add_laser

**Description**: This tutorial demonstrates how the user can create composite models directly from other models in the [Gazebo Model Database](https://bitbucket.org/osrf/gazebo_models/src) by using \<include> tags and \<joint> to connect different components of a composite model.

###Steps to Adding a Laser to `My_Robot`

1. Open `My_Robot`'s model.sdf file
2. Add the following directly before the \</model> tag at the end of your file.

    ```xhtml
    <include>
      <uri>model://hokuyo</uri>
      <pose>0.2 0 0.2 0 0 0</pose>
    </include>
    <joint name="hokuyo_joint" type="revolute">
      <child>hokuyo::link</child>
      <parent>chassis</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>
    ```
- [hokuyo model's SDF](https://bitbucket.org/osrf/gazebo_models/src/6cd587c0a30e/hokuyo/model.sdf?at=default&fileviewer=file-view-default)
- model name is `hokuyo`, so each link in the hokuyo model is prefaced w/ `hokuyo::`
- Listing of the [model database](http://models.gazebosim.org/) uri used by these tutorials

3. Start a Gazebo and add the robot to simulation using the Insert tab on the GUI. The robot should have a laser attached. 
4. (Optional) Try adding a camera to the robot. Camera's model URI is `model://camera`, it should have been locally catches for you in:

    ```
    ls  .../.gazebo/models/camera/
    ```


