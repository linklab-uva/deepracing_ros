#include <deepracing_ros/utils/xinput_msg_utils.h>
namespace deepracing_ros
{
    XINPUT_GAMEPAD XinputMsgUtils::toXinput(const deepracing_msgs::msg::XinputGamepad& xinput_msg)
    {
        XINPUT_GAMEPAD rtn;
        rtn.wButtons = xinput_msg.buttons;
        rtn.bLeftTrigger = xinput_msg.left_trigger;
        rtn.bRightTrigger = xinput_msg.right_trigger;
        rtn.sThumbLX = xinput_msg.thumb_lx;
        rtn.sThumbLY = xinput_msg.thumb_ly;
        rtn.sThumbRX = xinput_msg.thumb_rx;
        rtn.sThumbRY = xinput_msg.thumb_ry;
        return rtn;
    }
    XINPUT_STATE XinputMsgUtils::toXinput(const deepracing_msgs::msg::XinputState& xinput_msg)
    {
        XINPUT_STATE rtn;
        rtn.dwPacketNumber = xinput_msg.packet_number;
        rtn.Gamepad = toXinput(xinput_msg.gamepad);
        return rtn;
    }

    deepracing_msgs::msg::XinputState XinputMsgUtils::toMsg(const XINPUT_STATE& xinput_state)
    {
        deepracing_msgs::msg::XinputState rtn;
        rtn.packet_number=xinput_state.dwPacketNumber;
        rtn.gamepad = toMsg(xinput_state.Gamepad);
        return rtn;
    }
    deepracing_msgs::msg::XinputGamepad XinputMsgUtils::toMsg(const XINPUT_GAMEPAD& xinput_gamepad)
    {
        deepracing_msgs::msg::XinputGamepad rtn;
        rtn.buttons = xinput_gamepad.wButtons;
        rtn.left_trigger = xinput_gamepad.bLeftTrigger;
        rtn.right_trigger = xinput_gamepad.bRightTrigger;
        rtn.thumb_lx = xinput_gamepad.sThumbLX;
        rtn.thumb_ly = xinput_gamepad.sThumbLY;
        rtn.thumb_rx = xinput_gamepad.sThumbRX;
        rtn.thumb_ry = xinput_gamepad.sThumbRY;
        return rtn;
    }

}