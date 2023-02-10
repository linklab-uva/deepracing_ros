#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#include <deepracing_msgs/msg/timestamped_packet_motion_data.hpp>
#include <f1_datalogger/car_data/f1_2018/timestamped_car_data.h>
#include <deepracing_ros/utils/f1_msg_utils.h>

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
                m_motion_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("motion_data/raw_udp", 10);
                m_telemetry_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("telemetry_data/raw_udp", 10);
                m_car_setup_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("car_setup_data/raw_udp", 10);
                m_car_status_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("car_status_data/raw_udp", 10);
                m_lap_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("lap_data/raw_udp", 10);
                m_session_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("session_data/raw_udp", 1);
                m_participants_data_udp_publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("participants_data/raw_udp", 1);
                m_udp_subscription_ = create_subscription<udp_msgs::msg::UdpPacket>("udp_in", 10, 
                    std::bind(&UdpDemuxer::udp_cb, this, std::placeholders::_1));
                m_time_start_ = get_clock()->now();
            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void udp_cb(const udp_msgs::msg::UdpPacket::UniquePtr& udp_packet)
            {
                deepf1::twenty_eighteen::PacketHeader* header = reinterpret_cast<deepf1::twenty_eighteen::PacketHeader*>((void*)&(udp_packet->data.at(0)));
                switch (header->m_packetId)
                {
                    case deepf1::twenty_eighteen::PacketID::MOTION:
                    {
                        m_motion_data_udp_publisher_->publish(std::make_unique<udp_msgs::msg::UdpPacket>(*udp_packet));
                        break;
                    }
                    case deepf1::twenty_eighteen::PacketID::CARTELEMETRY:
                    {
                        m_telemetry_data_udp_publisher_->publish(std::make_unique<udp_msgs::msg::UdpPacket>(*udp_packet));
                        break;
                    }
                    case deepf1::twenty_eighteen::PacketID::CARSETUPS:
                    {
                        m_car_setup_data_udp_publisher_->publish(std::make_unique<udp_msgs::msg::UdpPacket>(*udp_packet));
                        break;
                    }
                    case deepf1::twenty_eighteen::PacketID::CARSTATUS:
                    {
                        m_car_status_data_udp_publisher_->publish(std::make_unique<udp_msgs::msg::UdpPacket>(*udp_packet));
                        break;
                    }
                    case deepf1::twenty_eighteen::PacketID::LAPDATA:
                    {
                        m_lap_data_udp_publisher_->publish(std::make_unique<udp_msgs::msg::UdpPacket>(*udp_packet));
                        break;
                    }
                    case deepf1::twenty_eighteen::PacketID::SESSION:
                    {
                        m_session_data_udp_publisher_->publish(std::make_unique<udp_msgs::msg::UdpPacket>(*udp_packet));
                        break;
                    }
                    case deepf1::twenty_eighteen::PacketID::PARTICIPANTS:
                    {
                        m_participants_data_udp_publisher_->publish(std::make_unique<udp_msgs::msg::UdpPacket>(*udp_packet));
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
