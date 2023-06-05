#include <deepracing_ros/utils/xinput_msg_utils.h>
namespace deepracing_ros
{
    XINPUT_GAMEPAD XinputMsgUtils::toXinput(const deepracing_msgs::msg::XinputGamepad& joy_msg)
    {
        XINPUT_GAMEPAD rtn;
        rtn.wButtons = joy_msg.buttons;
        rtn.bLeftTrigger = joy_msg.left_trigger;
        rtn.bRightTrigger = joy_msg.right_trigger;
        rtn.sThumbLX = joy_msg.thumb_lx;
        rtn.sThumbLY = joy_msg.thumb_ly;
        rtn.sThumbRX = joy_msg.thumb_rx;
        rtn.sThumbRY = joy_msg.thumb_ry;
        return rtn;
    }
    XINPUT_STATE XinputMsgUtils::toXinput(const deepracing_msgs::msg::XinputState& joy_msg)
    {
        XINPUT_STATE rtn;
        rtn.dwPacketNumber = joy_msg.packet_number;
        rtn.Gamepad = toXinput(joy_msg.gamepad);
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
    XINPUT_GAMEPAD XinputMsgUtils::toXinput(const deepf1::F1ControlCommand& f1_interface_command)
    {
        XINPUT_GAMEPAD rtn;
        toXinputInplace(f1_interface_command, rtn);
        return rtn;
    }
    void XinputMsgUtils::toXinputInplace(const deepf1::F1ControlCommand& f1_interface_command, XINPUT_GAMEPAD& rtn)
    {
        //short goes from -32,768 to 32,767
        //but left (negative) on the joystick is steering to the left, which is a positive steering angle in our convention.
        double steering = std::clamp<double>(f1_interface_command.steering, -1.0, 1.0);
        if (steering>0.0)
        {
            rtn.sThumbLX = (SHORT)std::round(-32768.0*steering);
        }
        else
        {
            rtn.sThumbLX = (SHORT)std::round(-32767.0*steering);
        }
        rtn.sThumbLY=rtn.sThumbRX=rtn.sThumbRY=0;
        //BYTE (unsigned char) goes from 0 to 255
        double throttle = std::clamp<double>(f1_interface_command.throttle, -1.0, 1.0);
        double brake = std::clamp<double>(f1_interface_command.brake, -1.0, 1.0);
        rtn.bRightTrigger = (BYTE)std::round(255.0*throttle);
        rtn.bLeftTrigger = (BYTE)std::round(255.0*brake);
    }

}