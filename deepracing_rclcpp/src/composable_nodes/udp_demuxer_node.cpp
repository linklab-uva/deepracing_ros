#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#include <deepracing_msgs/msg/timestamped_packet_motion_data.hpp>
#include <f1_datalogger/car_data/f1_2023/timestamped_car_data.h>
#include <deepracing_ros/utils/f1_msg_utils_2023.h>

namespace deepracing
{
namespace composable_nodes
{

    class UdpDemuxer : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC UdpDemuxer(const rclcpp::NodeOptions & options) : 
                rclcpp::Node("udp_demuxer_node", options)
            {
                rclcpp::QoS qos = rclcpp::SystemDefaultsQoS().keep_last(10).durability_volatile();
                m_motion_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("motion_data/raw_udp", qos);
                m_telemetry_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("telemetry_data/raw_udp", qos);
                m_car_setup_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("car_setup_data/raw_udp", qos);
                m_car_status_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("car_status_data/raw_udp", qos);
                m_lap_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("lap_data/raw_udp", qos);
                m_session_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("session_data/raw_udp", qos);
                m_participants_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("participants_data/raw_udp", qos);
                m_udp_subscription_ = create_subscription<udp_msgs::msg::UdpPacket>("udp_in", qos.keep_last(100), 
                    std::bind(&UdpDemuxer::udp_cb, this, std::placeholders::_1));
                m_time_start_ = get_clock()->now();
            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void udp_cb(udp_msgs::msg::UdpPacket::UniquePtr udp_packet)
            {
                if(udp_packet->data.size()<sizeof(deepf1::twenty_twentythree::PacketEventData))
                {
                    RCLCPP_ERROR(get_logger(), 
                        "Received a packet with only %llu bytes. Smallest packet that should ever be received (Event Data) has %llu bytes",
                             udp_packet->data.size(), sizeof(deepf1::twenty_twentythree::PacketEventData));
                    return;
                }
                uint16_t* packet_format = reinterpret_cast<uint16_t*>(&(udp_packet->data[0]));
                if(!((*packet_format)==2020))
                {
                    std::stringstream error_stream;
                    error_stream << "Received UDP packet formatted for " << *packet_format;
                    error_stream << ", but deepracing_ros is compiled for packet format 2020";
                    std::string error_msg = error_stream.str();
                    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
                    throw std::runtime_error(error_msg);
                }
                deepf1::twenty_twentythree::PacketHeader* header = reinterpret_cast<deepf1::twenty_twentythree::PacketHeader*>(&(udp_packet->data[0]));
                switch (header->packetId)
                {
                    case deepf1::twenty_twentythree::PacketID::MOTION:
                    {
                        m_motion_data_udp_publisher_->publish(std::move(udp_packet));
                        break;
                    }
                    case deepf1::twenty_twentythree::PacketID::CARTELEMETRY:
                    {
                        m_telemetry_data_udp_publisher_->publish(std::move(udp_packet));
                        break;
                    }
                    case deepf1::twenty_twentythree::PacketID::CARSETUPS:
                    {
                        m_car_setup_data_udp_publisher_->publish(std::move(udp_packet));
                        break;
                    }
                    case deepf1::twenty_twentythree::PacketID::CARSTATUS:
                    {
                        m_car_status_data_udp_publisher_->publish(std::move(udp_packet));
                        break;
                    }
                    case deepf1::twenty_twentythree::PacketID::LAPDATA:
                    {
                        m_lap_data_udp_publisher_->publish(std::move(udp_packet));
                        break;
                    }
                    case deepf1::twenty_twentythree::PacketID::SESSION:
                    {
                        m_session_data_udp_publisher_->publish(std::move(udp_packet));
                        break;
                    }
                    case deepf1::twenty_twentythree::PacketID::PARTICIPANTS:
                    {
                        m_participants_data_udp_publisher_->publish(std::move(udp_packet));
                        break;
                    }
                    default:
                    {
                        return;
                    }
                }
                
            }
            
            rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr m_udp_subscription_;
            rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr m_motion_data_udp_publisher_;
            rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr m_telemetry_data_udp_publisher_;
            rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr m_car_setup_data_udp_publisher_;
            rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr m_car_status_data_udp_publisher_;
            rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr m_lap_data_udp_publisher_;
            rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr m_session_data_udp_publisher_;
            rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr m_participants_data_udp_publisher_;

            rclcpp::Time m_time_start_;
    };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::UdpDemuxer)
