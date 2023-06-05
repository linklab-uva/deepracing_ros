#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <functional>
#include <f1_datalogger/controllers/multiagent_f1_interface_factory.h>
#include <deepracing_msgs/msg/xinput_state.hpp>
#include <deepracing_ros/utils/xinput_msg_utils.h>
#include <std_msgs/msg/float64.hpp>
#include <memory>
#include <unordered_map>

#define WIN32_LEAN_AND_MEAN
#define NOGDI
#include <windows.h>
#include <Xinput.h>
#include <f1_datalogger/controllers/vigem_interface.h>


#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
namespace deepracing
{
namespace composable_nodes
{
class MultiagentControlNode : public rclcpp::Node 
{
  public:
    MultiagentControlNode( const rclcpp::NodeOptions & options )
     : rclcpp::Node("multiagent_control_node", options)
    {
      std::vector<std::string> driver_names = declare_parameter<std::vector<std::string>>("driver_names", std::vector<std::string>());
      try
      {
        for(const std::string& driver : driver_names)
        {
          interfaces_[driver] = std::static_pointer_cast<deepf1::VigemInterface>(factory_.createInterface());
          std::string direct_topic_name = "/" + driver + "/controller_override";

          std::function<void(const deepracing_msgs::msg::XinputState::ConstPtr&)> func_override
          = std::bind(&MultiagentControlNode::setStateDirect_, this, std::placeholders::_1,  driver);
          direct_subscriptions_[driver] = create_subscription<deepracing_msgs::msg::XinputState>(direct_topic_name, rclcpp::SensorDataQoS(), func_override);

          std::string steer_topic_name = "/" + driver + "/steering";
          std::function<void(const std_msgs::msg::Float64::ConstPtr&)> func_steer
          = std::bind(&MultiagentControlNode::setSteering_, this, std::placeholders::_1,  driver);
          steer_subscriptions_[driver] = create_subscription<std_msgs::msg::Float64>(steer_topic_name, rclcpp::SensorDataQoS(), func_steer);

          std::string accel_topic_name = "/" + driver + "/acceleration";
          std::function<void(const std_msgs::msg::Float64::ConstPtr&)> func_accel
          = std::bind(&MultiagentControlNode::setAcceleration_, this, std::placeholders::_1,  driver);
          accel_subscriptions_[driver] = create_subscription<std_msgs::msg::Float64>(accel_topic_name, rclcpp::SensorDataQoS(), func_accel);

          last_direct_input_[driver] = deepracing_msgs::msg::XinputState();
        }
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(get_logger(), "Could not allocated interface, error message: %s", e.what());
      }
    }
  private:
    deepf1::MultiagentF1InterfaceFactory factory_;
    std::unordered_map<std::string, std::shared_ptr<deepf1::VigemInterface>> interfaces_;
    std::unordered_map<std::string, rclcpp::Subscription<deepracing_msgs::msg::XinputState>::SharedPtr> direct_subscriptions_;
    std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> steer_subscriptions_;
    std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> accel_subscriptions_;
    std::unordered_map<std::string, deepracing_msgs::msg::XinputState> last_direct_input_;
    rclcpp::TimerBase::SharedPtr timer_;

    void setStateDirect_(const deepracing_msgs::msg::XinputState::ConstPtr& state, const std::string& driver)
    {
      RCLCPP_INFO(get_logger(), "Setting state directly for driver %s", driver.c_str());
      interfaces_[driver]->setStateDirectly(deepracing_ros::XinputMsgUtils::toXinput(*state));
      last_direct_input_[driver] = *state;
    }
    void setSteering_(const std_msgs::msg::Float64::ConstPtr& steering_angle, const std::string& driver)
    {
      rclcpp::Time now = get_clock()->now();
      rclcpp::Duration dt = now - last_direct_input_[driver].header.stamp;
      if (dt<rclcpp::Duration::from_seconds(0.25))
      {
        return;
      }
      deepf1::F1ControlCommand cmd;
      cmd.brake = cmd.throttle = std::nan("");
      cmd.steering = steering_angle->data;
      interfaces_[driver]->setCommands(cmd);
    }
    void setAcceleration_(const std_msgs::msg::Float64::ConstPtr& accel, const std::string& driver)
    {
      rclcpp::Time now = get_clock()->now();
      rclcpp::Duration dt = now - last_direct_input_[driver].header.stamp;
      if (dt<rclcpp::Duration::from_seconds(0.25))
      {
        return;
      }
      deepf1::F1ControlCommand cmd;
      cmd.steering = std::nan("");
      if(accel->data>0.0)
      {
        cmd.throttle = accel->data;
        cmd.brake = 0.0;
      }
      else
      {
        cmd.throttle = 0.0;
        cmd.brake = accel->data;
      }
      interfaces_[driver]->setCommands(cmd);
    }
};

}
}
#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::MultiagentControlNode);