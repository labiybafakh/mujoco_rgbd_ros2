#!/bin/bash

# Build script for MuJoCo 3D DWA simulation

echo "Building MuJoCo 3D DWA simulation..."

# Detect system architecture and OS
OS=$(uname -s)
ARCH=$(uname -m)

# Set architecture flags based on system
CMAKE_ARCH_FLAGS=""
MUJOCO_LIB_EXT="so"
PARALLEL_JOBS=$(nproc 2>/dev/null || echo 4)

if [ "$OS" = "Darwin" ]; then
    echo "Detected macOS system"
    if [ "$ARCH" = "arm64" ]; then
        echo "Using ARM64 architecture"
        CMAKE_ARCH_FLAGS="-DCMAKE_OSX_ARCHITECTURES=arm64"
    else
        echo "Using x86_64 architecture"
        CMAKE_ARCH_FLAGS="-DCMAKE_OSX_ARCHITECTURES=x86_64"
    fi
    MUJOCO_LIB_EXT="dylib"
    PARALLEL_JOBS=$(sysctl -n hw.ncpu)
elif [ "$OS" = "Linux" ]; then
    echo "Detected Linux system"
    echo "Using default architecture: $ARCH"
    CMAKE_ARCH_FLAGS=""
    MUJOCO_LIB_EXT="so"
    PARALLEL_JOBS=$(nproc)
else
    echo "Unknown OS: $OS, using default settings"
    CMAKE_ARCH_FLAGS=""
fi

# Check if MuJoCo is already built
if [ ! -f "third_party/mujoco/build/lib/libmujoco.$MUJOCO_LIB_EXT" ]; then
    echo "MuJoCo not found. Building MuJoCo first..."
    cd third_party/mujoco
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release $CMAKE_ARCH_FLAGS -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_SIMULATE=ON -DMUJOCO_BUILD_TESTS=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    make -j$PARALLEL_JOBS mujoco
    cd ../../..
fi

# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake .. -DCMAKE_BUILD_TYPE=Release $CMAKE_ARCH_FLAGS

# Build
make -j$PARALLEL_JOBS

echo "Build complete. Executable: ./mujoco_sim"
echo "Model file: ../model.xml"