#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/logging.hpp>
#include <deepracing_msgs/msg/timestamped_packet_session_data.hpp>
#include <mutex>
#include <deepracing/utils.hpp>
#include <rclcpp/exceptions.hpp>

namespace deepracing
{
namespace composable_nodes
{
    class SyntheticSessionDataNode : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC SyntheticSessionDataNode(const rclcpp::NodeOptions & options) : 
                rclcpp::Node("synthetic_session_data", options), names_map_(deepracing::Utils::trackNames())
            {
                for (const auto& pair : names_map_) {
                    inverse_names_map_[pair.second] = pair.first;
                }
                declare_parameter<std::string>("trackname");
                session_data_publisher_ = this->create_publisher<deepracing_msgs::msg::TimestampedPacketSessionData>("session_data", 1);
                timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0), 
                    std::bind(&SyntheticSessionDataNode::timer_cb, this));                
            }   
        private:
            std::map<std::int8_t, std::string> names_map_;
            std::map<std::string, std::int8_t> inverse_names_map_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<deepracing_msgs::msg::TimestampedPacketSessionData>::SharedPtr session_data_publisher_;
            void timer_cb()
            {
                std::string trackname = get_parameter("trackname").as_string();
                if (inverse_names_map_.find(trackname) == inverse_names_map_.end()) {
                    RCLCPP_ERROR(get_logger(), "Track name '%s' not found in inverse names map", trackname.c_str());
                    return;
                }
                deepracing_msgs::msg::TimestampedPacketSessionData session_data;
                session_data.udp_packet.track_id = inverse_names_map_.at(trackname);
                session_data.header.stamp = this->now();
                session_data_publisher_->publish(session_data);
            }
    };
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::SyntheticSessionDataNode)
