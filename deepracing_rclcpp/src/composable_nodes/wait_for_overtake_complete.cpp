#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>

namespace deepracing
{
namespace composable_nodes
{
//.automatically_declare_parameters_from_overrides(true)
    class WaitForOvertakeComplete : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC WaitForOvertakeComplete(const rclcpp::NodeOptions & options) :
                rclcpp::Node("wait_for_overtake_complete", rclcpp::NodeOptions(options).allow_undeclared_parameters(true))
            {
                rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
                m_subscription_ = create_subscription<builtin_interfaces::msg::Time>("overtake_end", qos, 
                    std::bind(&WaitForOvertakeComplete::topic_cb, this, std::placeholders::_1));
            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void topic_cb(const builtin_interfaces::msg::Time::ConstPtr& time_msg)
            {
                RCLCPP_INFO(get_logger(), "Overtake complete at time: %d.%d, shutting down node.", time_msg->sec, time_msg->nanosec);
                // // std::this_thread::sleep_for(std::chrono::seconds(1)); // wait a bit for ros2 bag to also log the message
                // RCLCPP_INFO(get_logger(), "Processing complete");
                m_subscription_.reset();
                exit(0); // Exit the node gracefully
            }
            rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr m_subscription_;

    };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::WaitForOvertakeComplete)
