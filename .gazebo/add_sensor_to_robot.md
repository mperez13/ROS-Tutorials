#Add a Sensor to a Robot

Link to Tutorial - http://gazebosim.org/tutorials/?tut=add_laser

**Description**: This tutorial demonstrates how the user can create composite models directly from other models in the [Gazebo Model Database](https://bitbucket.org/osrf/gazebo_models/src) by using \<include> tags and \<joint> to connect different components of a composite model.

###Steps to Adding a Laser to `My_Robot`

1. Open `My_Robot`'s model.sdf file
2. Add the following directly before the \</model> tag at the end of your file.
    ```html
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
3. 
