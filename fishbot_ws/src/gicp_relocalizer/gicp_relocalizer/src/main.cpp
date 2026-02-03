#include <rclcpp/rclcpp.hpp>
#include "gicp_relocalizer/gicp_relocalizer_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // 节点选项配置
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(false);
    
    auto node = std::make_shared<gicp_relocalizer::GicpRelocalizerNode>(options);
    RCLCPP_INFO(node->get_logger(), "GICP Relocalizer Node started successfully");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}