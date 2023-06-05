#include <deepracing_ros/utils/xinput_msg_utils.h>
namespace deepracing_ros
{
    deepracing_msgs::msg::XinputState XinputMsgUtils::toMsg(const XINPUT_STATE& xinput_gamepad)
    {
        deepracing_msgs::msg::XinputState rtn;

        return rtn;
    }
    XINPUT_STATE XinputMsgUtils::toXinput(const deepracing_msgs::msg::XinputState& joy_msg)
    {
        XINPUT_STATE rtn;

        return rtn;
    }
}