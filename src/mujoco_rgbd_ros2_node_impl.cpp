#include "mujoco_rgbd_ros2/mujoco_rgbd_ros2_node.hpp"

MuJoCoRGBDNode::MuJoCoRGBDNode() : Node("mujoco_rgbd_node"), initialized_(false), frame_counter_(0),
    base_x_(0.0), base_y_(0.0), base_z_(1.0), base_roll_(0.0), base_pitch_(0.0), base_yaw_(0.0)
{
    // Parameters
    this->declare_parameter("model_file", "config/camera_environment.xml");
    this->declare_parameter("camera_name", "camera");
    this->declare_parameter("frame_id", "camera_link");
    this->declare_parameter("publish_rate", 30.0);
    this->declare_parameter("image_width", 640);
    this->declare_parameter("image_height", 480);
    this->declare_parameter("base_frame_z_offset", 0.0);
    this->declare_parameter("enable_visualizer", true);
    this->declare_parameter("visualizer_width", 1200);
    this->declare_parameter("visualizer_height", 900);

    model_file_ = this->get_parameter("model_file").as_string();
    camera_name_ = this->get_parameter("camera_name").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();
    base_frame_z_offset_ = this->get_parameter("base_frame_z_offset").as_double();
    enable_visualizer_ = this->get_parameter("enable_visualizer").as_bool();
    visualizer_width_ = this->get_parameter("visualizer_width").as_int();
    visualizer_height_ = this->get_parameter("visualizer_height").as_int();

    // Publishers
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/pointcloud", 10);
    color_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/color/image_raw", 10);
    depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/depth/image_raw", 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 10);

    // Transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Pose command subscriber
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/base_link/pose_command", 10,
        std::bind(&MuJoCoRGBDNode::pose_command_callback, this, std::placeholders::_1));

    // Initialize MuJoCo
    initialize_mujoco();

    // Create timer for publishing
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MuJoCoRGBDNode::publish_sensor_data, this)
    );

    // Start visualizer thread if enabled
    if (enable_visualizer_) {
        start_visualizer_thread();
    }

    RCLCPP_INFO(this->get_logger(), "MuJoCo RGBD Node initialized");
}

MuJoCoRGBDNode::~MuJoCoRGBDNode()
{
    stop_visualizer_thread();
    cleanup_mujoco();
}

void MuJoCoRGBDNode::initialize_mujoco()
{
    char error[1000] = "Fail to load model";
    model_ = mj_loadXML(model_file_.c_str(), 0, error, 1000);
    
    if (!model_) {
        RCLCPP_ERROR(this->get_logger(), "%s", error);
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "MuJoCo model loaded: %s", model_file_.c_str());
    data_ = mj_makeData(model_);

    // Initialize GLFW
    if (!glfwInit()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GLFW");
        rclcpp::shutdown();
        return;
    }

    // Create offscreen context for rendering
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ = glfwCreateWindow(image_width_, image_height_, "MuJoCo RGBD Offscreen", NULL, NULL);
    if (!window_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create GLFW window");
        glfwTerminate();
        rclcpp::shutdown();
        return;
    }

    glfwMakeContextCurrent(window_);

    // Find camera
    int camera_id = mj_name2id(model_, mjOBJ_CAMERA, camera_name_.c_str());
    if (camera_id == -1) {
        RCLCPP_ERROR(this->get_logger(), "Camera '%s' not found in model", camera_name_.c_str());
        rclcpp::shutdown();
        return;
    }

    // Initialize RGBD camera
    if (!rgbd_camera_.initialize(model_, camera_id)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize RGBD camera");
        rclcpp::shutdown();
        return;
    }

    // Initialize visualization structures
    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);

    mjv_makeScene(model_, &scene_, 1000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);

    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "MuJoCo rendering context initialized");
}

void MuJoCoRGBDNode::cleanup_mujoco()
{
    if (initialized_) {
        mjv_freeScene(&scene_);
        mjr_freeContext(&context_);
        
        if (window_) {
            glfwDestroyWindow(window_);
        }
        glfwTerminate();
        
        if (data_) mj_deleteData(data_);
        if (model_) mj_deleteModel(model_);
    }
}

void MuJoCoRGBDNode::publish_sensor_data()
{
    if (!initialized_) return;

    // Run simulation step
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        mj_step(model_, data_);
    }

    // Base_link pose will be updated via subscription to pose commands
    
    frame_counter_++;

    // Capture RGBD data
    if (!rgbd_camera_.capture(model_, data_, &context_)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "Failed to capture RGBD data");
        return;
    }

    // Get current time
    auto now = this->get_clock()->now();

    // Publish transforms
    publish_base_transform(now);
    publish_camera_transform(now);

    // Publish point cloud
    publish_pointcloud(now);

    // Publish images
    publish_images(now);

    // Publish camera info
    publish_camera_info(now);
}

void MuJoCoRGBDNode::publish_base_transform(const rclcpp::Time& timestamp)
{
    geometry_msgs::msg::TransformStamped base_transform;
    base_transform.header.stamp = timestamp;
    base_transform.header.frame_id = "world";
    base_transform.child_frame_id = "base_link";

    // Get camera pose from MuJoCo for base positioning
    int camera_id = mj_name2id(model_, mjOBJ_CAMERA, camera_name_.c_str());
    int camera_body_id = model_->cam_bodyid[camera_id];
    
    // Use stored base_link pose
    base_transform.transform.translation.x = base_x_;
    base_transform.transform.translation.y = base_y_;
    base_transform.transform.translation.z = base_z_;

    // Convert Euler angles to quaternion (ZYX order)
    tf2::Quaternion q;
    q.setRPY(base_roll_, base_pitch_, base_yaw_);
    base_transform.transform.rotation.w = q.w();
    base_transform.transform.rotation.x = q.x();
    base_transform.transform.rotation.y = q.y();
    base_transform.transform.rotation.z = q.z();

    tf_broadcaster_->sendTransform(base_transform);
}

void MuJoCoRGBDNode::publish_camera_transform(const rclcpp::Time& timestamp)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = timestamp;
    transform_stamped.header.frame_id = "base_link";
    transform_stamped.child_frame_id = frame_id_;

    // Get camera pose from MuJoCo
    int camera_id = mj_name2id(model_, mjOBJ_CAMERA, camera_name_.c_str());
    int camera_body_id = model_->cam_bodyid[camera_id];
    
    // 180-degree rotation around x-axis quaternion
    tf2::Quaternion flip_rotation(1.0, 0.0, 0.0, 0.0);  // 180 degrees around x-axis

    if (camera_body_id >= 0) {
        // Camera position relative to base_link (which moves with camera but has world orientation)
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = -base_frame_z_offset_;

        // Convert quaternion (MuJoCo uses w,x,y,z format)
        tf2::Quaternion original_rotation(
            data_->xquat[4 * camera_body_id + 1],  // x
            data_->xquat[4 * camera_body_id + 2],  // y
            data_->xquat[4 * camera_body_id + 3],  // z
            data_->xquat[4 * camera_body_id + 0]   // w
        );
        
        // Apply 180-degree flip around x-axis
        tf2::Quaternion final_rotation = original_rotation * flip_rotation;
        
        transform_stamped.transform.rotation.w = final_rotation.w();
        transform_stamped.transform.rotation.x = final_rotation.x();
        transform_stamped.transform.rotation.y = final_rotation.y();
        transform_stamped.transform.rotation.z = final_rotation.z();
    } else {
        // Default transform with 180-degree flip around x-axis
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = -base_frame_z_offset_;
        transform_stamped.transform.rotation.w = flip_rotation.w();
        transform_stamped.transform.rotation.x = flip_rotation.x();
        transform_stamped.transform.rotation.y = flip_rotation.y();
        transform_stamped.transform.rotation.z = flip_rotation.z();
    }

    tf_broadcaster_->sendTransform(transform_stamped);
}

void MuJoCoRGBDNode::publish_pointcloud(const rclcpp::Time& timestamp)
{
    auto color_cloud = rgbd_camera_.generateColorPointCloud();
    
    RCLCPP_DEBUG(this->get_logger(), "Generated point cloud with %zu points", color_cloud->size());
    
    if (color_cloud->size() > 0) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*color_cloud, cloud_msg);
        cloud_msg.header.stamp = timestamp;
        cloud_msg.header.frame_id = frame_id_;
        
        pointcloud_pub_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %zu points", color_cloud->size());
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "No points in generated cloud - check scene content");
    }
}

void MuJoCoRGBDNode::publish_images(const rclcpp::Time& timestamp)
{
    // Publish color image
    const cv::Mat& color_image = rgbd_camera_.getColorImage();
    if (!color_image.empty()) {
        auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_image).toImageMsg();
        color_msg->header.stamp = timestamp;
        color_msg->header.frame_id = frame_id_;
        color_image_pub_->publish(*color_msg);
    }

    // Publish depth image
    const cv::Mat& depth_image = rgbd_camera_.getDepthImage();
    if (!depth_image.empty()) {
        auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_image).toImageMsg();
        depth_msg->header.stamp = timestamp;
        depth_msg->header.frame_id = frame_id_;
        depth_image_pub_->publish(*depth_msg);
    }
}

void MuJoCoRGBDNode::publish_camera_info(const rclcpp::Time& timestamp)
{
    const auto& intrinsics = rgbd_camera_.getIntrinsics();
    
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id_;
    
    camera_info.width = intrinsics.width;
    camera_info.height = intrinsics.height;
    
    // Camera matrix
    camera_info.k[0] = intrinsics.fx;
    camera_info.k[2] = intrinsics.cx;
    camera_info.k[4] = intrinsics.fy;
    camera_info.k[5] = intrinsics.cy;
    camera_info.k[8] = 1.0;
    
    // Distortion (assuming no distortion)
    camera_info.d.resize(5, 0.0);
    camera_info.distortion_model = "plumb_bob";
    
    // Projection matrix
    camera_info.p[0] = intrinsics.fx;
    camera_info.p[2] = intrinsics.cx;
    camera_info.p[5] = intrinsics.fy;
    camera_info.p[6] = intrinsics.cy;
    camera_info.p[10] = 1.0;
    
    camera_info_pub_->publish(camera_info);
}

void MuJoCoRGBDNode::start_visualizer_thread()
{
    visualizer_running_ = true;
    visualizer_thread_ = std::thread(&MuJoCoRGBDNode::visualizer_loop, this);
    RCLCPP_INFO(this->get_logger(), "Visualizer thread started");
}

void MuJoCoRGBDNode::stop_visualizer_thread()
{
    if (visualizer_running_) {
        visualizer_running_ = false;
        if (visualizer_thread_.joinable()) {
            visualizer_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "Visualizer thread stopped");
    }
}

void MuJoCoRGBDNode::visualizer_loop()
{
    // Create visualizer window
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    GLFWwindow* vis_window = glfwCreateWindow(visualizer_width_, visualizer_height_, 
                                             "MuJoCo Simulation Visualizer", NULL, NULL);
    if (!vis_window) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create visualizer window");
        return;
    }

    glfwMakeContextCurrent(vis_window);
    glfwSwapInterval(1); // Enable vsync
    
    // Enable mouse interaction
    glfwSetWindowUserPointer(vis_window, this);
    glfwSetCursorPosCallback(vis_window, [](GLFWwindow* window, double x, double y) {
        static double lastx = x, lasty = y;
        static bool button_left = false, button_right = false;
        
        int state_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
        int state_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
        
        button_left = (state_left == GLFW_PRESS);
        button_right = (state_right == GLFW_PRESS);
        
        if (button_left || button_right) {
            MuJoCoRGBDNode* node = static_cast<MuJoCoRGBDNode*>(glfwGetWindowUserPointer(window));
            mjvCamera* cam = &node->vis_camera_;
            
            double dx = x - lastx;
            double dy = y - lasty;
            
            if (button_right) {
                // Right drag: rotate
                cam->azimuth += dx * 0.5;
                cam->elevation -= dy * 0.5;
                // Clamp elevation
                if (cam->elevation < -90) cam->elevation = -90;
                if (cam->elevation > 90) cam->elevation = 90;
            } else if (button_left) {
                // Left drag: zoom
                cam->distance *= (1.0 + 0.01 * dy);
                if (cam->distance < 0.1) cam->distance = 0.1;
                if (cam->distance > 100) cam->distance = 100;
            }
        }
        
        lastx = x;
        lasty = y;
    });

    // Initialize visualizer-specific rendering context
    mjrContext vis_context;
    mjr_defaultContext(&vis_context);
    mjr_makeContext(model_, &vis_context, mjFONTSCALE_150);

    // Initialize camera for visualization
    mjv_defaultCamera(&vis_camera_);
    vis_camera_.lookat[0] = 0.0;
    vis_camera_.lookat[1] = 0.0;
    vis_camera_.lookat[2] = 1.0;
    vis_camera_.distance = 5.0;
    vis_camera_.azimuth = 90.0;
    vis_camera_.elevation = -20.0;

    RCLCPP_INFO(this->get_logger(), "Visualizer window created and initialized");

    // Main visualization loop
    while (visualizer_running_ && !glfwWindowShouldClose(vis_window)) {
        glfwPollEvents();

        // Update scene
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (model_ && data_) {
            mjvScene vis_scene;
            mjv_defaultScene(&vis_scene);
            mjv_makeScene(model_, &vis_scene, 1000);
            mjv_updateScene(model_, data_, &option_, NULL, &vis_camera_, mjCAT_ALL, &vis_scene);

            // Render
            mjrRect viewport = {0, 0, visualizer_width_, visualizer_height_};
            mjr_render(viewport, &vis_scene, &vis_context);

            // Show base_link position and info
            char info_text[256];
            snprintf(info_text, sizeof(info_text), 
                    "Frame: %d\nBase_link Pose: [%.2f, %.2f, %.2f]\nRPY: [%.2f, %.2f, %.2f]", 
                    frame_counter_, base_x_, base_y_, base_z_, base_roll_, base_pitch_, base_yaw_);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, info_text, NULL, &vis_context);

            mjv_freeScene(&vis_scene);
        }

        glfwSwapBuffers(vis_window);
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    // Cleanup
    mjr_freeContext(&vis_context);
    glfwDestroyWindow(vis_window);
    RCLCPP_INFO(this->get_logger(), "Visualizer loop ended");
}

void MuJoCoRGBDNode::pose_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Extract position
    base_x_ = msg->pose.position.x;
    base_y_ = msg->pose.position.y;
    base_z_ = msg->pose.position.z;
    
    // Convert quaternion to Euler angles
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    m.getRPY(base_roll_, base_pitch_, base_yaw_);
    
    // Update base_link mocap body if it exists
    updateBaseLinkPose(base_x_, base_y_, base_z_, base_roll_, base_pitch_, base_yaw_);
    
    RCLCPP_DEBUG(this->get_logger(), "Updated base_link pose: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                base_x_, base_y_, base_z_, base_roll_, base_pitch_, base_yaw_);
}

bool MuJoCoRGBDNode::updateBaseLinkPose(double x, double y, double z, double roll, double pitch, double yaw)
{
    if (!model_ || !data_) {
        return false;
    }
    
    // Find base_link body
    int base_body_id = mj_name2id(model_, mjOBJ_BODY, "base_link");
    if (base_body_id < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                           "base_link body not found in model");
        return false;
    }
    
    // Check if this body has mocap
    int mocap_id = model_->body_mocapid[base_body_id];
    if (mocap_id < 0 || mocap_id >= model_->nmocap) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                           "base_link body is not a mocap body");
        return false;
    }
    
    // Update mocap position
    data_->mocap_pos[mocap_id * 3 + 0] = x;
    data_->mocap_pos[mocap_id * 3 + 1] = y;
    data_->mocap_pos[mocap_id * 3 + 2] = z;
    
    // Convert Euler angles to quaternion (ZYX order)
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    // Set quaternion (MuJoCo format: w,x,y,z)
    data_->mocap_quat[mocap_id * 4 + 0] = cr*cp*cy + sr*sp*sy; // w
    data_->mocap_quat[mocap_id * 4 + 1] = sr*cp*cy - cr*sp*sy; // x
    data_->mocap_quat[mocap_id * 4 + 2] = cr*sp*cy + sr*cp*sy; // y
    data_->mocap_quat[mocap_id * 4 + 3] = cr*cp*sy - sr*sp*cy; // z
    
    return true;
}