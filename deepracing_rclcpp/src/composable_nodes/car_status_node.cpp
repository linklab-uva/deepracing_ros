#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#include <deepracing_msgs/msg/timestamped_packet_car_status_data.hpp>
#include <f1_datalogger/car_data/f1_2018/timestamped_car_data.h>
#include <deepracing_ros/utils/f1_msg_utils.h>

namespace deepracing
{
namespace composable_nodes
{

    class ReceiveCarStatusData : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC ReceiveCarStatusData(const rclcpp::NodeOptions & options) : 
                rclcpp::Node("receive_car_status_data", options)
            {
                // rclcpp::QoS qos = rclcpp::SystemDefaultsQoS().history();
                m_car_status_data_publisher_ = create_publisher<deepracing_msgs::msg::TimestampedPacketCarStatusData>("car_status_data", 10);
                m_udp_subscription_ = create_subscription<udp_msgs::msg::UdpPacket>("car_status_data/raw_udp", 10, 
                    std::bind(&ReceiveCarStatusData::udp_cb, this, std::placeholders::_1));
                m_time_start_ = get_clock()->now();
                m_all_cars_param_ = declare_parameter<bool>("all_cars", false);
            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void udp_cb(const udp_msgs::msg::UdpPacket::ConstPtr& udp_packet)
            {
                deepf1::twenty_eighteen::PacketCarStatusData* udp_data = reinterpret_cast<deepf1::twenty_eighteen::PacketCarStatusData*>((void*)&(udp_packet->data.at(0)));
                deepracing_msgs::msg::TimestampedPacketCarStatusData rosdata;
                rosdata.udp_packet = deepracing_ros::F1MsgUtils::toROS(*udp_data, m_all_cars_param_); 
                rosdata.header.set__stamp(udp_packet->header.stamp).set__frame_id(deepracing_ros::F1MsgUtils::world_coordinate_name);
                m_car_status_data_publisher_->publish(std::make_unique<deepracing_msgs::msg::TimestampedPacketCarStatusData>(rosdata));
            }
            
            rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr m_udp_subscription_;
            rclcpp::Publisher<deepracing_msgs::msg::TimestampedPacketCarStatusData>::SharedPtr m_car_status_data_publisher_;

            rclcpp::Time m_time_start_;
            bool m_all_cars_param_;
    };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::ReceiveCarStatusData)