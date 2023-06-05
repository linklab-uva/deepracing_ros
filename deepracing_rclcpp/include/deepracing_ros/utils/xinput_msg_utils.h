#ifndef DEEPRACING_ROS_JOY_MSG_UTILS_H
#define DEEPRACING_ROS_JOY_MSG_UTILS_H
#include <deepracing_ros/utils/visibility_control.hpp>
#include <deepracing_msgs/msg/xinput_state.hpp>
#include <windows.h>
#include <Xinput.h>
namespace deepracing_ros
{
    class XinputMsgUtils
    {
    public:
        DEEPRACING_RCLCPP_UTILS_PUBLIC XinputMsgUtils() = default;
        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::XinputState toMsg(const XINPUT_STATE& xinput_gamepad);
        static DEEPRACING_RCLCPP_UTILS_PUBLIC XINPUT_STATE toXinput(const deepracing_msgs::msg::XinputState& xinput_msg);

    };
}
#endif