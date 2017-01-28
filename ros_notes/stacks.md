# Stacks

- ROS packages are organized into ROS stacks
- goal of stacks is to simplify the process of code sharing
- Each stack has associated version & can declare dependencies on other stacks
    - dependencies also declare a version number, which provides greater staility in development
	
- stacks is simply a directory descended from `ROS_ROOT` or `ROS_PACKAGE_PATH` that has [stack.xml][2]

- For realease purposes, [CMakeLists.txt][3] & [Makefile][3] should also be put into root of the stack
    - [roscreate-stack][4] tool can generate this file automatically

## Command-Line tools

- [rosstack][5] - primary tool ofr interacting w/ ROS stacks
    - stack-level equal of rospack tool for packages

- roscreate-stack helps automate process of creating a stack, including generating valid `stack.xml` file w/ correct dependencies
	
- rosbash
    - most commonly used for providing functionality to help you navigate ans use packages
	- provide ROS-variants of common Unix shell commands
	- most commonly used is roscd
    
	    ```
	    $ roscd package_name
	    ```

## Client Library Support

- in Python, can use `roslib.stacks` module in [roslib][7] package to get info about ROS stacks

[2]: http://wiki.ros.org/rosbuild/Stack%20Manifest
[3]: http://wiki.ros.org/StackBuildFiles
[4]: http://wiki.ros.org/roscreate
[5]: http://wiki.ros.org/rosstack
[6]: http://wiki.ros.org/rospack
[7]: http://wiki.ros.org/roslib
