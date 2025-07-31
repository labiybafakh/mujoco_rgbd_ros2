#include "mujoco_rgbd_ros2/mujoco_rgbd_ros2_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MuJoCoRGBDNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}