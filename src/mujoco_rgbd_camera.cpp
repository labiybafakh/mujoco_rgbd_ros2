#include "mujoco_rgbd_camera.hpp"
#include <iostream>
#include <cmath>
#include <cassert>

MujocoRGBDCamera::MujocoRGBDCamera() 
    : camera_id_(-1), buffers_allocated_(false), buffer_width_(0), buffer_height_(0), 
      rgbd_scene_(nullptr), scene_initialized_(false) {
    // Initialize intrinsics to default values
    intrinsics_ = {0, 0, 0, 0, 0, 0};
    depth_params_ = {0, 0, 0};
}

MujocoRGBDCamera::~MujocoRGBDCamera() {
    cleanupScene();
    releaseBuffers();
}

bool MujocoRGBDCamera::initialize(const mjModel* model, int camera_id) {
    if (!model || camera_id < 0) {
        std::cerr << "Error: Invalid model or camera ID" << std::endl;
        return false;
    }
    
    // Temporarily bypass camera count check - MuJoCo may not count fixed cameras in ncam
    std::cout << "Debug: model->ncam = " << model->ncam << ", camera_id = " << camera_id << std::endl;
    
    // Check if camera exists by trying to access it
    if (camera_id < 0) {
        std::cerr << "Error: Invalid camera ID " << camera_id << std::endl;
        return false;
    }
    
    // For now, we'll trust the camera lookup from mj_name2id and proceed
    
    camera_id_ = camera_id;
    
    // Set depth parameters from model
    depth_params_.extent = model->stat.extent;
    depth_params_.z_near = model->vis.map.znear * depth_params_.extent;
    depth_params_.z_far = model->vis.map.zfar * depth_params_.extent;
    
    return true;
}

bool MujocoRGBDCamera::updateCameraPosition(const mjModel* model, mjData* data, double x, double y, double z) {
    if (!model || !data || camera_id_ < 0) {
        std::cerr << "Error: Invalid parameters for camera position update" << std::endl;
        return false;
    }
    
    // Find the camera body by name (assuming camera is attached to a body named "camera")
    int camera_body_id = mj_name2id(model, mjOBJ_BODY, "camera");
    if (camera_body_id < 0) {
        std::cerr << "Error: Camera body not found" << std::endl;
        return false;
    }
    
    // Check if this body has mocap
    int mocap_id = model->body_mocapid[camera_body_id];
    if (mocap_id < 0 || mocap_id >= model->nmocap) {
        std::cerr << "Error: Camera body is not a mocap body" << std::endl;
        return false;
    }
    
    // Update mocap position
    data->mocap_pos[mocap_id * 3 + 0] = x;
    data->mocap_pos[mocap_id * 3 + 1] = y;
    data->mocap_pos[mocap_id * 3 + 2] = z;
    
    return true;
}

bool MujocoRGBDCamera::capture(const mjModel* model, mjData* data, const mjrContext* context) {
    if (!model || !data || !context || camera_id_ < 0) {
        std::cerr << "Error: Invalid parameters for capture" << std::endl;
        return false;
    }
    
    // Initialize scene if not done yet
    if (!scene_initialized_) {
        initializeScene(model);
    }
    
    // Save current viewport to restore later
    int saved_viewport[4];
    glGetIntegerv(GL_VIEWPORT, saved_viewport);
    
    // Set up camera for rendering from the RGBD camera's perspective
    mjvCamera cam;
    mjv_defaultCamera(&cam);
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = camera_id_;
    
    // Set viewport for RGBD capture
    mjrRect viewport = {0, 0, 640, 480};
    glViewport(0, 0, viewport.width, viewport.height);
    
    // Allocate buffers if needed
    if (!buffers_allocated_ || buffer_width_ != viewport.width || buffer_height_ != viewport.height) {
        allocateBuffers(viewport.width, viewport.height);
        setupCameraIntrinsics(model, viewport);
    }
    
    // Update RGBD scene with camera view
    mjvOption opt;
    mjv_defaultOption(&opt);
    mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, rgbd_scene_);
    
    // Render and read pixels
    mjr_render(viewport, rgbd_scene_, context);
    mjr_readPixels(color_buffer_.get(), depth_buffer_.get(), viewport, context);
    
    // Restore original viewport
    glViewport(saved_viewport[0], saved_viewport[1], saved_viewport[2], saved_viewport[3]);
    
    // Convert to OpenCV matrices
    cv::Size img_size(viewport.width, viewport.height);
    
    // Color image (BGR format, flip vertically)
    cv::Mat rgb(img_size, CV_8UC3, color_buffer_.get());
    cv::flip(rgb, color_image_, 0);
    
    // Depth image (linearize and flip vertically)
    cv::Mat raw_depth(img_size, CV_32F, depth_buffer_.get());
    cv::Mat flipped_depth;
    cv::flip(raw_depth, flipped_depth, 0);
    depth_image_ = linearizeDepth(flipped_depth);
    
    return true;
}

void MujocoRGBDCamera::updateViewport(int width, int height) {
    if (width != buffer_width_ || height != buffer_height_) {
        allocateBuffers(width, height);
        // Update intrinsics for new viewport size
        if (camera_id_ >= 0) {
            mjrRect viewport = {0, 0, width, height};
            // Note: This requires model access, might need to store model reference
            // setupCameraIntrinsics(model, viewport);
        }
    }
}

// Temporarily commented out due to PCL dependency issues
std::vector<Point3D> MujocoRGBDCamera::generatePointCloud() const {

    std::vector<Point3D> cloud;

    if (depth_image_.empty()) {
        return cloud;
    }
    
    for (int i = 0; i < depth_image_.rows; i++) {
        for (int j = 0; j < depth_image_.cols; j++) {
            float depth = depth_image_.at<float>(i, j);
            // Debug: only print first few pixels to avoid console spam
            if (i < 2 && j < 5) {
                std::cout << "Depth at (" << i << ", " << j << "): " << depth << std::endl;
            }
            // Filter far points
            if (depth > 0 && depth < depth_params_.z_far) {
                
                Point3D point;
                point.x = (j - intrinsics_.cx) * depth / intrinsics_.fx;
                point.y = (i - intrinsics_.cy) * depth / intrinsics_.fy;
                point.z = depth;
                cloud.emplace_back(point);
            }
        }
    }
    
    return cloud;
}
/*

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MujocoRGBDCamera::generateColorPointCloud() const {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    
    if (depth_image_.empty() || color_image_.empty()) {
        return cloud;
    }
    
    assert(color_image_.size() == depth_image_.size());
    
    for (int i = 0; i < depth_image_.rows; i++) {
        for (int j = 0; j < depth_image_.cols; j++) {
            float depth = depth_image_.at<float>(i, j);
            
            // Filter far points
            if (depth > 0 && depth < depth_params_.z_far) {
                pcl::PointXYZRGB point;
                point.x = (j - intrinsics_.cx) * depth / intrinsics_.fx;
                point.y = (i - intrinsics_.cy) * depth / intrinsics_.fy;
                point.z = depth;
                
                // Get color (OpenCV uses BGR format)
                cv::Vec3b bgr = color_image_.at<cv::Vec3b>(i, j);
                point.b = bgr[0];
                point.g = bgr[1];
                point.r = bgr[2];
                
                cloud->push_back(point);
            }
        }
    }
    
    return cloud;
}
*/

void MujocoRGBDCamera::setupCameraIntrinsics(const mjModel* model, const mjrRect& viewport) {
    // Calculate intrinsics based on camera FOV
    double fovy = model->cam_fovy[camera_id_] * M_PI / 180.0;  // Convert to radians
    
    // Calculate focal length for Y direction from vertical FOV
    intrinsics_.fy = viewport.height / (2.0 * tan(fovy / 2.0));
    
    // Calculate aspect ratio and horizontal FOV
    double aspect = (double)viewport.width / viewport.height;
    double fovx = 2.0 * atan(tan(fovy / 2.0) * aspect);
    
    // Calculate focal length for X direction from horizontal FOV
    intrinsics_.fx = viewport.width / (2.0 * tan(fovx / 2.0));
    
    intrinsics_.cx = viewport.width / 2.0;
    intrinsics_.cy = viewport.height / 2.0;
    intrinsics_.width = viewport.width;
    intrinsics_.height = viewport.height;
}

cv::Mat MujocoRGBDCamera::linearizeDepth(const cv::Mat& raw_depth) const {
    cv::Mat depth_img(raw_depth.size(), CV_32F);
    
    for (int i = 0; i < raw_depth.rows; i++) {
        const float* raw_ptr = raw_depth.ptr<float>(i);
        float* depth_ptr = depth_img.ptr<float>(i);
        for (int j = 0; j < raw_depth.cols; j++) {
            if (raw_ptr[j] < 1.0f) {  // Valid depth range
                depth_ptr[j] = (2.0 * depth_params_.z_near * depth_params_.z_far) / 
                              (depth_params_.z_far + depth_params_.z_near - raw_ptr[j] * (depth_params_.z_far - depth_params_.z_near));
            } else {
                depth_ptr[j] = 0.0f;  // Invalid depth
            }
        }
    }

    return depth_img;
}

void MujocoRGBDCamera::allocateBuffers(int width, int height) {
    releaseBuffers();
    
    color_buffer_ = std::make_unique<unsigned char[]>(width * height * 3);
    depth_buffer_ = std::make_unique<float[]>(width * height);
    
    buffer_width_ = width;
    buffer_height_ = height;
    buffers_allocated_ = true;
}

void MujocoRGBDCamera::releaseBuffers() {
    color_buffer_.reset();
    depth_buffer_.reset();
    buffers_allocated_ = false;
    buffer_width_ = buffer_height_ = 0;
}

void MujocoRGBDCamera::initializeScene(const mjModel* model) {
    if (scene_initialized_) {
        cleanupScene();
    }
    
    rgbd_scene_ = new mjvScene();
    mjv_defaultScene(rgbd_scene_);
    mjv_makeScene(model, rgbd_scene_, 1000);  // Smaller scene for RGBD capture
    scene_initialized_ = true;
    
    std::cout << "RGBD scene initialized with capacity: " << rgbd_scene_->maxgeom << " geoms" << std::endl;
}

void MujocoRGBDCamera::cleanupScene() {
    if (scene_initialized_ && rgbd_scene_) {
        mjv_freeScene(rgbd_scene_);
        delete rgbd_scene_;
        rgbd_scene_ = nullptr;
        scene_initialized_ = false;
    }
}