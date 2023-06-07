#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <functional>
#include <memory>
#include <algorithm>
#include <f1_datalogger/controllers/multiagent_f1_interface_factory.h>
#include <deepracing_msgs/msg/xinput_state.hpp>
#include <deepracing_ros/utils/xinput_msg_utils.h>
#include <std_msgs/msg/float64.hpp>
#include <unordered_map>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <control_msgs/msg/pid_state.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef _MSC_VER
  #include <windows.h>
  #include <Xinput.h>
  #include <f1_datalogger/controllers/vigem_interface.h>
  typedef deepf1::VigemInterface InterfaceType;
#else
  #error "This node is only supported on Windows for now"
#endif

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
      if(driver_names.size()<1 || driver_names.size()>4)
      {
        throw std::runtime_error("Must declare between 1 and 4 driver names");
      }
      std::vector<double> full_lock_left_vec = declare_parameter<std::vector<double>>("full_lock_left", std::vector<double>());
      if(full_lock_left_vec.size()!=driver_names.size())
      {
        std::stringstream error_stream;
        error_stream <<"Declared" << driver_names.size() << "drivers, but " << full_lock_left_vec.size() << " full-lock-left values.";
        throw std::runtime_error(error_stream.str());
      }
      std::vector<double> full_lock_right_vec = declare_parameter<std::vector<double>>("full_lock_right", std::vector<double>());
      if(full_lock_right_vec.size()!=driver_names.size())
      {
        std::stringstream error_stream;
        error_stream <<"Declared" << driver_names.size() << "drivers, but " << full_lock_right_vec.size() << " full-lock-right values.";
        throw std::runtime_error(error_stream.str());
      }
      try
      {
        for(std::size_t i = 0; i < driver_names.size(); i++)
        {
          std::string driver = driver_names.at(i);
          double full_lock_left = full_lock_left_vec.at(i);
          double full_lock_right = full_lock_right_vec.at(i);
          interfaces_[driver] = std::static_pointer_cast<InterfaceType>(factory_.createInterface());
          std::string direct_topic_name = "/" + driver + "/controller_override";

          std::function<void(const deepracing_msgs::msg::XinputState::ConstPtr&)> func_override
          = std::bind(&MultiagentControlNode::setStateDirect_, this, std::placeholders::_1,  driver);
          direct_subscriptions_[driver] = create_subscription<deepracing_msgs::msg::XinputState>(direct_topic_name, rclcpp::SensorDataQoS(), func_override);

          std::string steer_topic_name = "/" + driver + "/steering";
          std::function<void(const ackermann_msgs::msg::AckermannDriveStamped::ConstPtr&)> func_steer
          = std::bind(&MultiagentControlNode::setSteering_, this, std::placeholders::_1, driver, full_lock_left, full_lock_right);
          steer_subscriptions_[driver] = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(steer_topic_name, rclcpp::SensorDataQoS(), func_steer);

          std::string accel_topic_name = "/" + driver + "/velocity_pid_state";
          std::function<void(const control_msgs::msg::PidState::ConstPtr&)> func_accel
          = std::bind(&MultiagentControlNode::setAcceleration_, this, std::placeholders::_1,  driver);
          accel_subscriptions_[driver] = create_subscription<control_msgs::msg::PidState>(accel_topic_name, rclcpp::SensorDataQoS(), func_accel);

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
    std::unordered_map<std::string, std::shared_ptr<InterfaceType>> interfaces_;
    std::unordered_map<std::string, rclcpp::Subscription<deepracing_msgs::msg::XinputState>::SharedPtr> direct_subscriptions_;
    std::unordered_map<std::string, rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr> steer_subscriptions_;
    std::unordered_map<std::string, rclcpp::Subscription<control_msgs::msg::PidState>::SharedPtr> accel_subscriptions_;
    std::unordered_map<std::string, deepracing_msgs::msg::XinputState> last_direct_input_;
    rclcpp::TimerBase::SharedPtr timer_;

    void setStateDirect_(const deepracing_msgs::msg::XinputState::ConstPtr& state, const std::string& driver)
    {
      RCLCPP_DEBUG(get_logger(), "Setting state directly for driver %s", driver.c_str());
      interfaces_[driver]->setStateDirectly(deepracing_ros::XinputMsgUtils::toXinput(*state));
      last_direct_input_[driver] = *state;
    }
    void setSteering_(const ackermann_msgs::msg::AckermannDriveStamped::ConstPtr& ackermann_cmd, 
      const std::string& driver, const double& full_lock_left, const double& full_lock_right)
    {
      const rclcpp::Time& now = get_clock()->now();
      const rclcpp::Duration& dt = now - last_direct_input_[driver].header.stamp;
      if (dt<rclcpp::Duration::from_seconds(0.25))
      {
        return;
      }
      deepf1::F1ControlCommand cmd;
      cmd.brake = cmd.throttle = std::nan("");
      const double& desired_steering = ackermann_cmd->drive.steering_angle;
      if(desired_steering>=0.0)
      {
        cmd.steering = std::clamp<double>(desired_steering/full_lock_left, 0.0, 1.0);
      }
      else
      {
        cmd.steering = std::clamp<double>(desired_steering/full_lock_right, -1.0, 0.0);
      }
      interfaces_[driver]->setCommands(cmd);
    }
    void setAcceleration_(const control_msgs::msg::PidState::ConstPtr& pid_state, const std::string& driver)
    {
      const rclcpp::Time& now = get_clock()->now();
      const rclcpp::Duration& dt = now - last_direct_input_[driver].header.stamp;
      if (dt<rclcpp::Duration::from_seconds(0.25))
      {
        return;
      }
      deepf1::F1ControlCommand cmd;
      cmd.steering = std::nan("");
      const double& accel = pid_state->output;
      if(accel>0.0)
      {
        cmd.throttle = accel;
        cmd.brake = 0.0;
      }
      else
      {
        cmd.throttle = 0.0;
        cmd.brake = accel;
      }
      interfaces_[driver]->setCommands(cmd);
    }
};

}
}
#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::MultiagentControlNode);