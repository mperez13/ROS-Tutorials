cdcmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(hello_world SHARED hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})

#################Added for Model Plugin################
add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
########################################################

#################Added for World Plugin################
add_library(factory SHARED factory.cc)
target_link_libraries(factory ${GAZEBO_LIBRARIES})
########################################################

#################Added for System Plugin################
add_library(system_gui SHARED system_gui.cc)
target_link_libraries(system_gui ${GAZEBO_LIBRARIES})
########################################################

#################Added for World Control################
add_library(world_edit SHARED world_edit.cc)
target_link_libraries(world_edit ${GAZEBO_LIBRARIES})
########################################################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
