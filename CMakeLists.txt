cmake_minimum_required(VERSION 3.16)
project(mujoco_rgbd_zenoh)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Find OpenGL
find_package(OpenGL REQUIRED)

# Find GLFW via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

# MuJoCo directories - using pre-built binary
set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco-3.3.5)

# Platform-specific MuJoCo setup
if(APPLE)
    set(MUJOCO_FRAMEWORK_DIR ${MUJOCO_DIR}/mujoco.framework)
    set(MUJOCO_INCLUDE_DIR ${MUJOCO_FRAMEWORK_DIR}/Headers)
    set(MUJOCO_LIB_DIR ${MUJOCO_FRAMEWORK_DIR}/Versions/A)
    
    # Define MuJoCo libraries for macOS
    set(MUJOCO_LIBS
      ${MUJOCO_LIB_DIR}/libmujoco.3.3.5.dylib
      ${GLFW_LIBRARIES}
      "-framework OpenGL"
      "-framework Cocoa" 
      "-framework IOKit"
      "-framework CoreVideo"
    )
else()
    set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
    set(MUJOCO_LIB_DIR ${MUJOCO_DIR}/lib)
    
    # Define MuJoCo libraries for Linux
    set(MUJOCO_LIBS
      ${MUJOCO_LIB_DIR}/libmujoco.so.3.3.5
      ${GLFW_LIBRARIES}
      GL
      GLEW
      m
      dl
    )
endif()

# Find OpenCV for RGBD camera after MuJoCo
find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)  # Temporarily commented out due to VTK issue

# Add executables
add_executable(mujoco_rgbd_main 
    src/main.cpp
    src/mujoco_rgbd_camera.cpp
)

# Link libraries
target_link_libraries(mujoco_rgbd_main
  ${MUJOCO_LIBS}
  ${OpenCV_LIBS}
  # ${PCL_LIBRARIES}  # Temporarily commented out due to VTK issue
)

# Add library directories to linker path
target_link_directories(mujoco_rgbd_main PRIVATE 
    ${GLFW_LIBRARY_DIRS}
    # ${PCL_LIBRARY_DIRS}  # Temporarily commented out due to VTK issue
)

# Include directories
include_directories(
  include
  ${MUJOCO_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}  # Temporarily commented out due to VTK issue
)

# Link directories
link_directories(
  ${MUJOCO_LIB_DIR}
  # ${PCL_LIBRARY_DIRS}  # Temporarily commented out due to VTK issue
)

# Set RPATH for runtime library finding
set_target_properties(mujoco_rgbd_main PROPERTIES
  INSTALL_RPATH "${MUJOCO_LIB_DIR}"
  BUILD_RPATH "${MUJOCO_LIB_DIR}"
)

# Install targets
install(TARGETS
    mujoco_rgbd_main
    DESTINATION bin
)

# Install config files
install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
)
