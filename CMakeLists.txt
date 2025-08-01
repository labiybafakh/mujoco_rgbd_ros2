cmake_minimum_required(VERSION 3.16)
project(mujoco_rgbd_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)

# Check ROS distribution and set compile definition
if(DEFINED ENV{ROS_DISTRO})
    if($ENV{ROS_DISTRO} STREQUAL "humble")
        add_compile_definitions(ROS_DISTRO_HUMBLE)
    endif()
endif()
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Find GLFW via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

# MuJoCo directories - using pre-built binary
set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco-3.3.4)
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_LIB_DIR ${MUJOCO_DIR}/lib)

# Define MuJoCo libraries
set(MUJOCO_LIBS
  ${MUJOCO_LIB_DIR}/libmujoco.so.3.3.4
  ${GLFW_LIBRARIES}
  GL
  GLEW
  m
  dl
)

# Find OpenCV and PCL for RGBD camera after MuJoCo
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

# Add executables
add_executable(mujoco_rgbd_node 
    src/mujoco_rgbd_ros2_node.cpp
    src/mujoco_rgbd_ros2_node_impl.cpp
    src/mujoco_rgbd_camera.cpp
)

# Link libraries
target_link_libraries(mujoco_rgbd_node
  ${MUJOCO_LIBS}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# Set ROS2 dependencies for mujoco_rgbd_node
ament_target_dependencies(mujoco_rgbd_node
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    pcl_ros
    pcl_conversions
    cv_bridge
    image_transport
    camera_info_manager
)

# Add library directories to linker path
target_link_directories(mujoco_rgbd_node PRIVATE 
    ${GLFW_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
)

# Include directories
include_directories(
  include
  ${MUJOCO_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${MUJOCO_LIB_DIR}
  ${PCL_LIBRARY_DIRS}
)

# Set RPATH for runtime library finding
set_target_properties(mujoco_rgbd_node PROPERTIES
  INSTALL_RPATH "${MUJOCO_LIB_DIR}"
  BUILD_RPATH "${MUJOCO_LIB_DIR}"
)

# Install targets
install(TARGETS
    mujoco_rgbd_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()