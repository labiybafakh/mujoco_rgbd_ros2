#pragma once

#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <memory>

struct CameraIntrinsics {
    double fx, fy;  // focal lengths
    double cx, cy;  // principal points
    int width, height;  // image dimensions
};

struct DepthParams {
    double z_near, z_far;  // clipping planes
    double extent;         // depth scale
};

class MujocoRGBDCamera {
public:
    MujocoRGBDCamera();
    ~MujocoRGBDCamera();

    // Initialize camera with MuJoCo model and camera ID
    bool initialize(const mjModel* model, int camera_id);
    
    // Update camera position (for mocap body cameras)
    bool updateCameraPosition(const mjModel* model, mjData* data, double x, double y, double z);
    
    // Set camera position directly (wrapper for updateCameraPosition)
    bool setCameraPosition(const mjModel* model, mjData* data, double x, double y, double z) {
        return updateCameraPosition(model, data, x, y, z);
    }
    
    // Capture RGBD data from current simulation state
    bool capture(const mjModel* model, mjData* data, const mjrContext* context);
    
    // Get captured images
    const cv::Mat& getColorImage() const { return color_image_; }
    const cv::Mat& getDepthImage() const { return depth_image_; }
    
    // Generate point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud() const;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateColorPointCloud() const;
    
    // Get camera parameters
    const CameraIntrinsics& getIntrinsics() const { return intrinsics_; }
    const DepthParams& getDepthParams() const { return depth_params_; }
    
    // Update viewport (call when window size changes)
    void updateViewport(int width, int height);

private:
    // Internal methods
    void setupCameraIntrinsics(const mjModel* model, const mjrRect& viewport);
    cv::Mat linearizeDepth(const cv::Mat& raw_depth) const;
    void allocateBuffers(int width, int height);
    void releaseBuffers();
    
    // Camera parameters
    CameraIntrinsics intrinsics_;
    DepthParams depth_params_;
    int camera_id_;
    
    // Image data
    cv::Mat color_image_;
    cv::Mat depth_image_;
    
    // OpenGL buffers
    std::unique_ptr<unsigned char[]> color_buffer_;
    std::unique_ptr<float[]> depth_buffer_;
    
    // Buffer management
    bool buffers_allocated_;
    int buffer_width_, buffer_height_;
};