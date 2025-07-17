cmake_minimum_required(VERSION 3.16)
project(3d_dwa_mujoco)

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

# Set MuJoCo paths
set(MUJOCO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco)
set(MUJOCO_LIB_PATH ${MUJOCO_ROOT}/build/lib)
set(MUJOCO_INCLUDE_PATH ${MUJOCO_ROOT}/include)

# Set library extension based on platform
if(APPLE)
    set(MUJOCO_LIB_EXT "dylib")
elseif(UNIX)
    set(MUJOCO_LIB_EXT "so")
else()
    set(MUJOCO_LIB_EXT "dll")
endif()

# Add executable
add_executable(mujoco_sim src/mujoco_main.cpp)

# Link libraries
target_link_libraries(mujoco_sim
    ${MUJOCO_LIB_PATH}/libmujoco.${MUJOCO_LIB_EXT}
    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES}
)

# Add GLFW library directory to linker path
target_link_directories(mujoco_sim PRIVATE ${GLFW_LIBRARY_DIRS})

# Include directories
target_include_directories(mujoco_sim PRIVATE
    ${MUJOCO_INCLUDE_PATH}
    ${OPENGL_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
)

# Compiler flags for macOS
if(APPLE)
    target_compile_options(mujoco_sim PRIVATE -Wno-deprecated-declarations)
    target_link_libraries(mujoco_sim "-framework OpenGL" "-framework Cocoa" "-framework IOKit")
endif()

# Set rpath for dynamic linking
set_target_properties(mujoco_sim PROPERTIES
    BUILD_RPATH "${MUJOCO_LIB_PATH}:${GLFW_LIBRARY_DIRS}"
    INSTALL_RPATH "${MUJOCO_LIB_PATH}:${GLFW_LIBRARY_DIRS}"
)