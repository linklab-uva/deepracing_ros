#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#include <deepracing_msgs/msg/timestamped_packet_motion_data.hpp>

namespace deepracing
{
namespace composable_nodes
{

    class ReceiveMotionData : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC ReceiveMotionData(const rclcpp::NodeOptions & options) : rclcpp::Node("receive_motion_data", options)
            {
                rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
                m_udp_subscription_ = create_subscription<udp_msgs::msg::UdpPacket>("udp_in", qos, 
                    std::bind(&ReceiveMotionData::udp_cb, this, std::placeholders::_1));
            } 
        private:
            
            void DEEPRACING_RCLCPP_LOCAL udp_cb(const udp_msgs::msg::UdpPacket::ConstPtr& udp_packet)
            {
            }
            
            rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr m_udp_subscription_;
            rclcpp::Publisher<deepracing_msgs::msg::TimestampedPacketMotionData>::SharedPtr m_motion_data_publisher_;
    };
    
}
}

RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::ReceiveMotionData)
