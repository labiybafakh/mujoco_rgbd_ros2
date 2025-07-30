#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include "mujoco_rgbd_camera.hpp"
#include <chrono>
#include <thread>
#include <memory>
#include <cmath>

class MuJoCoRGBDNode : public rclcpp::Node
{
public:
    MuJoCoRGBDNode() : Node("mujoco_rgbd_node"), initialized_(false), frame_counter_(0)
    {
        // Parameters
        this->declare_parameter("model_file", "config/camera_environment.xml");
        this->declare_parameter("camera_name", "camera");
        this->declare_parameter("frame_id", "camera_link");
        this->declare_parameter("publish_rate", 30.0);
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("enable_circular_motion", true);
        this->declare_parameter("circle_radius", 2.0);
        this->declare_parameter("circle_height", 1.0);

        model_file_ = this->get_parameter("model_file").as_string();
        camera_name_ = this->get_parameter("camera_name").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();
        enable_circular_motion_ = this->get_parameter("enable_circular_motion").as_bool();
        circle_radius_ = this->get_parameter("circle_radius").as_double();
        circle_height_ = this->get_parameter("circle_height").as_double();

        // Publishers
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/pointcloud", 10);
        color_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/color/image_raw", 10);
        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/depth/image_raw", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 10);

        // Transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initialize MuJoCo
        initialize_mujoco();

        // Create timer for publishing
        auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&MuJoCoRGBDNode::publish_sensor_data, this)
        );

        RCLCPP_INFO(this->get_logger(), "MuJoCo RGBD Node initialized");
    }

    ~MuJoCoRGBDNode()
    {
        cleanup_mujoco();
    }

private:
    void initialize_mujoco()
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

    void cleanup_mujoco()
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

    void publish_sensor_data()
    {
        if (!initialized_) return;

        // Run simulation step
        mj_step(model_, data_);

        // Update camera position with circular motion
        if (enable_circular_motion_) {
            double time = frame_counter_ / publish_rate_;  // Time in seconds
            double x = circle_radius_ * cos(time);
            double y = circle_radius_ * sin(time);
            double z = circle_height_;
            
            if (!rgbd_camera_.updateCameraPosition(model_, data_, x, y, z)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                    "Failed to update camera position");
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Camera position: [%.2f, %.2f, %.2f]", x, y, z);
            }
        }
        
        frame_counter_++;

        // Capture RGBD data
        if (!rgbd_camera_.capture(model_, data_, &context_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Failed to capture RGBD data");
            return;
        }

        // Get current time
        auto now = this->get_clock()->now();

        // Publish transform
        publish_camera_transform(now);

        // Publish point cloud
        publish_pointcloud(now);

        // Publish images
        publish_images(now);

        // Publish camera info
        publish_camera_info(now);
    }

    void publish_camera_transform(const rclcpp::Time& timestamp)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = timestamp;
        transform_stamped.header.frame_id = "world";
        transform_stamped.child_frame_id = frame_id_;

        // Get camera pose from MuJoCo
        int camera_id = mj_name2id(model_, mjOBJ_CAMERA, camera_name_.c_str());
        int camera_body_id = model_->cam_bodyid[camera_id];
        
        if (camera_body_id >= 0) {
            // Get body position and orientation
            transform_stamped.transform.translation.x = data_->xpos[3 * camera_body_id + 0];
            transform_stamped.transform.translation.y = data_->xpos[3 * camera_body_id + 1];
            transform_stamped.transform.translation.z = data_->xpos[3 * camera_body_id + 2];

            // Convert quaternion (MuJoCo uses w,x,y,z format)
            transform_stamped.transform.rotation.w = data_->xquat[4 * camera_body_id + 0];
            transform_stamped.transform.rotation.x = data_->xquat[4 * camera_body_id + 1];
            transform_stamped.transform.rotation.y = data_->xquat[4 * camera_body_id + 2];
            transform_stamped.transform.rotation.z = data_->xquat[4 * camera_body_id + 3];
        } else {
            // Default transform if camera is not attached to a body
            transform_stamped.transform.translation.x = 0.0;
            transform_stamped.transform.translation.y = 0.0;
            transform_stamped.transform.translation.z = 0.0;
            transform_stamped.transform.rotation.w = 1.0;
            transform_stamped.transform.rotation.x = 0.0;
            transform_stamped.transform.rotation.y = 0.0;
            transform_stamped.transform.rotation.z = 0.0;
        }

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    void publish_pointcloud(const rclcpp::Time& timestamp)
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

    void publish_images(const rclcpp::Time& timestamp)
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

    void publish_camera_info(const rclcpp::Time& timestamp)
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

    // Member variables
    std::string model_file_;
    std::string camera_name_;
    std::string frame_id_;
    double publish_rate_;
    int image_width_, image_height_;
    bool initialized_;
    
    // Camera movement parameters
    bool enable_circular_motion_;
    double circle_radius_;
    double circle_height_;
    int frame_counter_;

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    mjvOption option_;
    mjvScene scene_;
    mjrContext context_;
    GLFWwindow* window_ = nullptr;

    MujocoRGBDCamera rgbd_camera_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MuJoCoRGBDNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}