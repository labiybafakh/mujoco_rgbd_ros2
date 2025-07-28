#!/bin/bash

echo "Building Docker image for DWA 3D Simulation..."

# Build the Docker image
docker build -t dwa_3d_simulation:latest .

if [ $? -eq 0 ]; then
    echo "✅ Docker image built successfully!"
    echo "Image name: dwa_3d_simulation:latest"
else
    echo "❌ Docker build failed!"
    exit 1
fi