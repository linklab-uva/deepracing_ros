#ifndef DEEPRACING_ROS_F1_MSG_UTILS_2020_H
#define DEEPRACING_ROS_F1_MSG_UTILS_2020_H
#include <deepracing_ros/utils/visibility_control.hpp>
#include "f1_datalogger/car_data/f1_2020/car_data.h"
#include "deepracing_msgs/msg/packet_header.hpp"
#include "deepracing_msgs/msg/packet_motion_data.hpp"
#include "deepracing_msgs/msg/packet_car_setup_data.hpp"
#include "deepracing_msgs/msg/packet_car_status_data.hpp"
#include "deepracing_msgs/msg/packet_car_telemetry_data.hpp"
#include "deepracing_msgs/msg/packet_lap_data.hpp"
#include "deepracing_msgs/msg/packet_session_data.hpp"
#include "deepracing_msgs/msg/weather_forecast_sample.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>
#include <array>

namespace deepracing_ros
{
    class F1MsgUtils2020
    {
    public:
        DEEPRACING_RCLCPP_UTILS_PUBLIC F1MsgUtils2020() = default;
        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::PacketHeader toROS(const deepf1::twenty_twenty::PacketHeader& header_data);
        
        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::CarSetupData toROS(const deepf1::twenty_twenty::CarSetupData& setup_data);
        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::PacketCarSetupData toROS(const deepf1::twenty_twenty::PacketCarSetupData& packet_setup_data, bool copy_all_cars);

        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::LapData toROS(const deepf1::twenty_twenty::LapData& lap_data);
        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::PacketLapData toROS(const deepf1::twenty_twenty::PacketLapData& lap_data, bool copy_all_cars);

        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::CarTelemetryData toROS(const deepf1::twenty_twenty::CarTelemetryData& telemetry_data);
        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::PacketCarTelemetryData toROS(const deepf1::twenty_twenty::PacketCarTelemetryData& telemetry_data, bool copy_all_cars);

        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::CarStatusData toROS(const deepf1::twenty_twenty::CarStatusData& status_data);
        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::PacketCarStatusData toROS(const deepf1::twenty_twenty::PacketCarStatusData& packet_status_data, bool copy_all_cars);

        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::CarMotionData toROSMotionData(const deepf1::twenty_twenty::CarMotionData& motion_data);
        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::PacketMotionData toROS(const deepf1::twenty_twenty::PacketMotionData& motion_data, bool copy_all_cars);

        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::MarshalZone toROS(const deepf1::twenty_twenty::MarshalZone& marshal_zone);
        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::PacketSessionData toROS(const deepf1::twenty_twenty::PacketSessionData& session_data);

        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::WeatherForecastSample toROS(const deepf1::twenty_twenty::WeatherForecastSample& weather_forecast);

        static inline DEEPRACING_RCLCPP_UTILS_PUBLIC const std::array<std::string, 4> wheel_order()
        {
            return std::array<std::string, 4>({"RearLeft, RearRight, FrontLeft, FrontRight"});
        }
        static inline DEEPRACING_RCLCPP_UTILS_PUBLIC const std::array<std::string, 25> track_names()
        {
            return std::array<std::string, 25>({"Australia", "France", "China", "Bahrain", "Spain", "Monaco",
                                              "Canada", "Britain", "Germany", "Hungary", "Belgium", "Italy",
                                               "Singapore", "Japan", "Abu_Dhabi", "USA", "Brazil", "Austria",
                                               "Russia", "Mexico", "Azerbaijan", "Bahrain_short", "Britan_short",
                                               "USA_short", "Japan_short"});
        }
        static DEEPRACING_RCLCPP_UTILS_PUBLIC const std::string world_coordinate_name;
        static DEEPRACING_RCLCPP_UTILS_PUBLIC const std::string car_coordinate_name;
    private:

    };
}
#endif