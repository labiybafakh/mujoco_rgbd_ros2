#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_DISTRO_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <image_transport/image_transport.hpp>

#include "mujoco_rgbd_camera.hpp"
#include <chrono>
#include <thread>
#include <memory>
#include <cmath>
#include <mutex>
#include <atomic>

class MuJoCoRGBDNode : public rclcpp::Node
{
public:
    MuJoCoRGBDNode();
    ~MuJoCoRGBDNode();

private:
    // Initialization and cleanup
    void initialize_mujoco();
    void cleanup_mujoco();

    // Main publishing loop
    void publish_sensor_data();

    // Transform publishing
    void publish_base_transform(const rclcpp::Time& timestamp);
    void publish_camera_transform(const rclcpp::Time& timestamp);

    // Base link control
    void pose_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    bool updateBaseLinkPose(double x, double y, double z, double roll, double pitch, double yaw);

    // Data publishing
    void publish_pointcloud(const rclcpp::Time& timestamp);
    void publish_images(const rclcpp::Time& timestamp);
    void publish_camera_info(const rclcpp::Time& timestamp);

    // Visualizer thread management
    void start_visualizer_thread();
    void stop_visualizer_thread();
    void visualizer_loop();

    // Node parameters
    std::string model_file_;
    std::string camera_name_;
    std::string frame_id_;
    double publish_rate_;
    int image_width_, image_height_;
    bool initialized_;
    
    // Base link pose storage
    double base_frame_z_offset_;
    int frame_counter_;
    double base_x_, base_y_, base_z_;
    double base_roll_, base_pitch_, base_yaw_;
    
    // Visualizer parameters
    bool enable_visualizer_;
    int visualizer_width_, visualizer_height_;
    
    // Threading and synchronization
    std::atomic<bool> visualizer_running_{false};
    std::thread visualizer_thread_;
    std::mutex data_mutex_;
    
    // Visualizer camera (public for mouse callback access)
public:
    mjvCamera vis_camera_;
private:

    // MuJoCo objects
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    mjvOption option_;
    mjvScene scene_;
    mjrContext context_;
    GLFWwindow* window_ = nullptr;

    // RGBD camera
    MujocoRGBDCamera rgbd_camera_;

    // ROS publishers and timer
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // ROS subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};