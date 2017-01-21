# [Camera Distortion][1]

Camera lenses have some degree of optical distortions, which results in warping of images.  

Tools such as Matlab or OpenCV to extract distortion coefficients along w/ other camera instrinsic parameters.

- w/ distortion coefficient, can create a distorted camera sensor in Gazebo

### Current Implementation

Gazebo supports simulation of camera based on [Brown's distortion model][2]

- The Brown–Conrady model corrects both for radial distortion and for tangential distortion caused by physical elements in a lens not being perfectly aligned.
- it expects 5 distortion `k1`, `k2`, `k3`, `p1`, `p2` (can get from camera calibration tools)
- `k` coefficients -> radial components of the distortion model
- `p` coefficients -> tangetial components

Few limitations w/ current implementation that needs to be taken into account:

1. Only barrel distortion is supported at the moment; typically has a negative `k1` value
2. Distortion is applied to the camera image texture
  1. This means taking the generated image daa and warping it
  2. This has the caveat that the final image (especially at the corners) has a narrower field of view than a real camera lens with barrel distortion
  3. One workaround to compensate for this effect is to increase the field of view of the camera sensor in Gazebo

## Creating a camera w/ distortion

1. Create [model.config][3] file
2. Create [model.sdf][4] file
3. Start Gazebo
4. Insert `Distorted Camera` model anywhere and a box in front of it 

  ![image distorted camera][5]

5. Topic Visualization (`Window->Topic Visualization` or `Ctrl-T`)

  ![image topic vis][6]

6. Click on `/gazebo/default/distorted_camera/link/camera/image` to get Camera View window that shows you the camera image

  ![image camera view][7]

The camera image should seem distorted; showing curve edges of the box.

 To adjust distortion, play w/ `k1`, `k2`, `k3`, `p1`, `p2` distortion coeffiecients in the [model.sdf][4]
  
**Return to [Gazebo Categories][8]**

[1]: http://gazebosim.org/tutorials?tut=camera_distortion&cat=sensors
[2]: http://en.wikipedia.org/wiki/Distortion_(optics)#Software_correction
[3]: ../.gazebo/models/distorted_camera/model.config
[4]: ../.gazebo/models/distorted_camera/model.sdf
[5]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/camera_distortion/files/distorted_camera_inserted.png
[6]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/camera_distortion/files/distorted_camera_topic_visualizer.png
[7]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/camera_distortion/files/distorted_camera_image_visualizer.png
[8]: ../gazebo_notes.md
