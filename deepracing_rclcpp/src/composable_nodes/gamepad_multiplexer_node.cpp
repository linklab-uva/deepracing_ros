#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <sstream>
#include <functional>
#include <deepracing_msgs/msg/xinput_state.hpp>
#include <deepracing_ros/utils/xinput_msg_utils.h>
#include <windows.h>
#include <Xinput.h>
namespace deepracing
{
namespace composable_nodes
{
class GamepadMultiplexerNode : public rclcpp::Node 
{
  public:
    GamepadMultiplexerNode( const rclcpp::NodeOptions & options )
     : rclcpp::Node("gamepad_multiplexer", options)
    {
      double frequency = declare_parameter<double>("frequency", 100.0);
      timer_ = rclcpp::create_timer(this,get_clock(), 
        rclcpp::Duration::from_seconds(1.0/frequency), std::bind(&GamepadMultiplexerNode::timerCallback, this));
      device_index_= (unsigned long)declare_parameter<int>("device_index", 0);
      driver_names_ = declare_parameter<std::vector<std::string>>("driver_names", std::vector<std::string>());
      if(driver_names_.empty() || driver_names_.size()>2)
      {
        throw std::runtime_error("Must specify either 1 or 2 driver names");
      }
      for(const std::string& driver : driver_names_)
      {
        std::string direct_topic_name = "/" + driver + "/controller_input";
        publishers_[driver] = create_publisher<deepracing_msgs::msg::XinputState>(direct_topic_name, rclcpp::QoS{1});
      }
    }
  private:
    inline void timerCallback()
    {
      XINPUT_STATE state;
      try
      {
        state = XINPUT_STATE();
        XInputGetState(device_index_, &state);
        if(state.Gamepad.sThumbRY>30000)
        {
          deepracing_msgs::msg::XinputState msg = deepracing_ros::XinputMsgUtils::toMsg(state)
            .set__header(std_msgs::msg::Header().set__stamp(get_clock()->now()));
          msg.gamepad.thumb_rx = msg.gamepad.thumb_ry = 0;
          publishers_[driver_names_.at(0)]->publish(msg);
        }
        else if(driver_names_.size()>1 && (state.Gamepad.sThumbRY<-30000))
        {
          deepracing_msgs::msg::XinputState msg = deepracing_ros::XinputMsgUtils::toMsg(state)
            .set__header(std_msgs::msg::Header().set__stamp(get_clock()->now()));
          msg.gamepad.thumb_rx = msg.gamepad.thumb_ry = 0;
          publishers_[driver_names_.at(1)]->publish(msg);
        }
      }
      catch(std::runtime_error& ex)
      {
        RCLCPP_ERROR(get_logger(), "Could not get state from index: %d", device_index_);
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned long device_index_;
    std::map<std::string, rclcpp::Publisher<deepracing_msgs::msg::XinputState>::SharedPtr> publishers_;
    std::vector<std::string> driver_names_;

};
}
}
#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::GamepadMultiplexerNode);