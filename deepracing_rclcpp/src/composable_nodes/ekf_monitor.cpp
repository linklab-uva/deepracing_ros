#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <deepracing_msgs/msg/timestamped_packet_motion_data.hpp>

namespace deepracing
{
namespace composable_nodes
{

    class EkfMonitor : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC EkfMonitor(const rclcpp::NodeOptions & options) : 
                rclcpp::Node("ekf_monitor", options)
            {
                rclcpp::QoS qos = rclcpp::SystemDefaultsQoS().keep_last(10).durability_volatile();
                m_odom_measurement_subscription_ = create_subscription<nav_msgs::msg::Odometry>("odom", qos, 
                    std::bind(&EkfMonitor::odom_cb, this, std::placeholders::_1));
            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void odom_cb(const nav_msgs::msg::Odometry::ConstPtr& odom_measurement)
            {
            }
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_measurement_subscription_;

    };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::EkfMonitor)
