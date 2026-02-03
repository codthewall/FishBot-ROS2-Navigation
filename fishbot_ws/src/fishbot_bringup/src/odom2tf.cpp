#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomTopic2TF : public rclcpp::Node {
public:
    OdomTopic2TF(const std::string& name) : Node(name) {
        // 创建odom话题订阅者，使用传感器数据的QoS
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 
            rclcpp::SensorDataQoS(),
            std::bind(&OdomTopic2TF::odom_callback, this, std::placeholders::_1)
        );

        // 创建TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "OdomTopic2TF节点已启动，监听odom话题...");
    }

private:
    // 成员变量
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 回调函数：处理odom消息并发布TF变换
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 创建变换消息
        geometry_msgs::msg::TransformStamped transform;
        
        // 填充变换信息
        transform.header = msg->header;
        transform.child_frame_id = msg->child_frame_id;
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation.x = msg->pose.pose.orientation.x;
        transform.transform.rotation.y = msg->pose.pose.orientation.y;
        transform.transform.rotation.z = msg->pose.pose.orientation.z;
        transform.transform.rotation.w = msg->pose.pose.orientation.w;

        // 广播坐标变换
        tf_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTopic2TF>("odom2tf");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}