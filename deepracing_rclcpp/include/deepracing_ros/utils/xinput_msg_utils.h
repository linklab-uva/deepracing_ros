#ifndef DEEPRACING_ROS_JOY_MSG_UTILS_H
#define DEEPRACING_ROS_JOY_MSG_UTILS_H
#include <deepracing_ros/utils/visibility_control.hpp>
#include <deepracing_msgs/msg/xinput_state.hpp>
#include <f1_datalogger/controllers/f1_interface.h>
#include <windows.h>
#include <Xinput.h>
namespace deepracing_ros
{
    class XinputMsgUtils
    {
    public:
        DEEPRACING_RCLCPP_UTILS_PUBLIC XinputMsgUtils() = default;
        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::XinputState toMsg(const XINPUT_STATE& xinput_state);
        static DEEPRACING_RCLCPP_UTILS_PUBLIC XINPUT_STATE toXinput(const deepracing_msgs::msg::XinputState& xinput_msg);

        
        static DEEPRACING_RCLCPP_UTILS_PUBLIC deepracing_msgs::msg::XinputGamepad toMsg(const XINPUT_GAMEPAD& xinput_gamepad);
        static DEEPRACING_RCLCPP_UTILS_PUBLIC XINPUT_GAMEPAD toXinput(const deepracing_msgs::msg::XinputGamepad& xinput_msg);
        static DEEPRACING_RCLCPP_UTILS_PUBLIC XINPUT_GAMEPAD toXinput(const deepf1::F1ControlCommand& f1_interface_command);
        static DEEPRACING_RCLCPP_UTILS_PUBLIC void toXinputInplace(const deepf1::F1ControlCommand& f1_interface_command, XINPUT_GAMEPAD& rtn);

        

    };
}
#endif