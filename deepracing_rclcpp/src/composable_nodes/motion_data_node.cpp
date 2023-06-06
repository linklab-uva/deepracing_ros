#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#include <deepracing_msgs/msg/timestamped_packet_motion_data.hpp>
#include <f1_datalogger/car_data/f1_2018/timestamped_car_data.h>
#include <deepracing_ros/utils/f1_msg_utils_2020.h>

namespace deepracing
{
namespace composable_nodes
{

    class ReceiveMotionData : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC ReceiveMotionData(const rclcpp::NodeOptions & options) : 
                rclcpp::Node("receive_motion_data", options)
            {
                // rclcpp::QoS qos = rclcpp::SystemDefaultsQoS().history();
                m_motion_data_publisher_ = create_publisher<deepracing_msgs::msg::TimestampedPacketMotionData>("motion_data", 10);
                m_udp_subscription_ = create_subscription<udp_msgs::msg::UdpPacket>("motion_data/raw_udp", 10, 
                    std::bind(&ReceiveMotionData::udp_cb, this, std::placeholders::_1));
                m_time_start_ = get_clock()->now();
                m_all_cars_param_ = declare_parameter<bool>("all_cars", false);
            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void udp_cb(const udp_msgs::msg::UdpPacket::ConstPtr& udp_packet)
            {
                deepf1::twenty_twenty::PacketMotionData* udp_data = reinterpret_cast<deepf1::twenty_twenty::PacketMotionData*>((void*)&(udp_packet->data.at(0)));
                deepracing_msgs::msg::TimestampedPacketMotionData rosdata;
                rosdata.udp_packet = deepracing_ros::F1MsgUtils2020::toROS(*udp_data, m_all_cars_param_); 
                rosdata.header.set__stamp(udp_packet->header.stamp).set__frame_id(deepracing_ros::F1MsgUtils2020::world_coordinate_name);
                if (rosdata.udp_packet.header.player_car_index<22)
                {
                    deepracing_msgs::msg::CarMotionData& ego_motion_data = rosdata.udp_packet.car_motion_data.at(rosdata.udp_packet.header.player_car_index);
                    ego_motion_data.world_forward_dir.header.stamp =
                    ego_motion_data.world_position.header.stamp = 
                    ego_motion_data.world_right_dir.header.stamp =
                    ego_motion_data.world_up_dir.header.stamp = 
                    ego_motion_data.world_velocity.header.stamp = rosdata.header.stamp;
                }
                if (m_all_cars_param_)
                {
                    for(deepracing_msgs::msg::CarMotionData & motion_data : rosdata.udp_packet.car_motion_data)
                    {
                        motion_data.world_forward_dir.header.stamp =
                        motion_data.world_position.header.stamp = 
                        motion_data.world_right_dir.header.stamp =
                        motion_data.world_up_dir.header.stamp = 
                        motion_data.world_velocity.header.stamp = rosdata.header.stamp;
                    }
                }
                RCLCPP_INFO(get_logger(), "%s", "Publishing on motion data parser");
                m_motion_data_publisher_->publish(std::make_unique<deepracing_msgs::msg::TimestampedPacketMotionData>(rosdata));
            }
            
            rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr m_udp_subscription_;
            rclcpp::Publisher<deepracing_msgs::msg::TimestampedPacketMotionData>::SharedPtr m_motion_data_publisher_;

            rclcpp::Time m_time_start_;
            bool m_all_cars_param_;
    };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::ReceiveMotionData)
