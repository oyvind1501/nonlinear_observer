cmake_minimum_required(VERSION 3.5.1)

project(nonlinear_observer
        VERSION 0.1
        LANGUAGES CXX)


add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)

add_subdirectory(src)

find_package(roscpp QUIET)
if (roscpp_FOUND)
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs       
)


catkin_package(
        # INCLUDE_DIRS include
        # LIBRARIES 
        CATKIN_DEPENDS geometry_msgs roscpp message_runtime sensor_msgs 
        # DEPENDS eigen system_lib
)

include_directories(
	PRIVATE
        apps/ros
	PUBLIC
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(nlo apps/ros/ros_node.cpp)

target_link_libraries(nlo PRIVATE nonlinear_observer_lib ${catkin_LIBRARIES})

add_executable(nonlinear_observer_test apps/test/main.cpp)
target_link_libraries(nonlinear_observer_test PRIVATE nonlinear_observer_lib)

endif ()
