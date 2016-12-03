#ROS Concepts

http://wiki.ros.org/ROS/Concepts

- ROS has 3 levels of concepts:
  - Filesystem  level
  - Computation Graph level
  - Community level
- ROS defines 2 types of [names](http://wiki.ros.org/Names):
  - Package Resource Names
  - Graph Resource Names

##ROS Filesystem Level

- [Packages](http://wiki.ros.org/Packages)
  - main unit for organizing software in ROS
  - may contain:
    - ROS runtime processes (nodes)
    - ROS-dependent library
    - datasets
    - configuration files
    - or anything else that is usefullt organized together
  -  most atomic build item & release item in ROS; meaning that the most granular thing you can build & release is a package
- [Metapackages](http://wiki.ros.org/Metapackages)
  - specialized Packages 
  - serve to represent a group of related other packages
  - most commonly metapackages are used as a backwards compatible place holder for converted [rosbuild Stacks](http://wiki.ros.org/rosbuild/ROS/Concepts#Stacks)
- [Package Manifests](http://wiki.ros.org/catkin/package.xml)
  - Manifest (package.xml) provide metadata about a package, including its name, version, description, license info, dependencies, & other meta info like exported packages
  - `package.xml` package manifest is defined in [REP-0127](http://www.ros.org/reps/rep-0127.html)
- Repositories
  - collection of packages which share a common VCS system
  - packages that share VCS share the same version & can be released together using the catkin release automation tool [bloom](http://wiki.ros.org/bloom)
  - often these repositories will map to converted [rosbuild](http://wiki.ros.org/rosbuild) [Stacks](http://wiki.ros.org/rosbuild/ROS/Concepts#Stacks)
  - Repositories can only contain one package
- [Message (msg) types](http://wiki.ros.org/msg)
  - message descriptions, stored in `my_package/msg/MyMessageType.msg` define the data structures for [messages](http://wiki.ros.org/Messages) sent in ROS
- [Service (srv) types](http://wiki.ros.org/srv)
  - Service desriptions, stored in `my_package/srv/MyServiceType.srv` define the request & response data structures for [services](http://wiki.ros.org/Services) in ROS
  
##ROS Computation Graph Level

- 













