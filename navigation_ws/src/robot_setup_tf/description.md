#Setting up your robot using tf
Link to tutorial (http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF)


#####To build, the code added following to CMakeLists.txt:

- add_executable(tf_broadcaster src/tf_broadcaster.cpp)
- add_executable(tf_listener src/tf_listener.cpp)
- target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
- target_link_libraries(tf_listener ${catkin_LIBRARIES})


