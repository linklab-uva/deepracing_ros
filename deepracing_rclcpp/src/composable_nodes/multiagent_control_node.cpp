#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <functional>
#include <memory>
#include <algorithm>
#include <deepracing_msgs/msg/xinput_state.hpp>
#include <deepracing_ros/utils/xinput_msg_utils.h>
#include <std_msgs/msg/float64.hpp>
#include <unordered_map>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <control_msgs/msg/pid_state.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>

#ifdef _MSC_VER
  #include <windows.h>
  #include <Xinput.h>
  #include <f1_datalogger/controllers/multiagent_f1_interface_factory.h>
  // #include <f1_datalogger/controllers/vigem_interface.h>
  // typedef deepf1::VigemInterface InterfaceType;
  #include <f1_datalogger/controllers/f1_interface_factory.h>
  #include <f1_datalogger/controllers/vjoy_interface.h>
  // typedef deepf1::VJoyInterface deepf1::F1Interface;
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
      deadzone_ratio_= declare_parameter<double>("deadzone_ratio", 0.0);
      driver_names_ = declare_parameter<std::vector<std::string>>("driver_names", std::vector<std::string>());
      if(driver_names_.size()<1 || driver_names_.size()>4)
      {
        throw std::runtime_error("Must declare between 1 and 4 driver names");
      }
      std::vector<double> full_lock_left_vec = declare_parameter<std::vector<double>>("full_lock_left", std::vector<double>());
      if(full_lock_left_vec.size()!=driver_names_.size())
      {
        std::stringstream error_stream;
        error_stream <<"Declared" << driver_names_.size() << "drivers, but " << full_lock_left_vec.size() << " full-lock-left values.";
        throw std::runtime_error(error_stream.str());
      }
      std::vector<double> full_lock_right_vec = declare_parameter<std::vector<double>>("full_lock_right", std::vector<double>());
      if(full_lock_right_vec.size()!=driver_names_.size())
      {
        std::stringstream error_stream;
        error_stream <<"Declared" << driver_names_.size() << "drivers, but " << full_lock_right_vec.size() << " full-lock-right values.";
        throw std::runtime_error(error_stream.str());
      }
      try
      {
        for(std::size_t i = 0; i < driver_names_.size(); i++)
        {
          std::string driver = driver_names_.at(i);
          double full_lock_left = full_lock_left_vec.at(i);
          double full_lock_right = full_lock_right_vec.at(i);
          vigem_interfaces_[driver] = factory_.createInterface();
          // vjoy_interfaces_[driver] = deepf1::F1InterfaceFactory::getDefaultInterface(i+1);
          std::string direct_topic_name = "/" + driver + "/controller_override";

          std::function<void(const deepracing_msgs::msg::XinputState::ConstPtr&)> func_override
          = std::bind(&MultiagentControlNode::setStateDirect_, this, std::placeholders::_1,  driver);
          direct_subscriptions_[driver] = create_subscription<deepracing_msgs::msg::XinputState>(direct_topic_name, rclcpp::QoS{1}, func_override);

          std::string steer_topic_name = "/" + driver + "/ctrl_cmd";
          std::function<void(ackermann_msgs::msg::AckermannDriveStamped::UniquePtr&)> func_steer
          = std::bind(&MultiagentControlNode::setSteering_, this, std::placeholders::_1, driver, full_lock_left, full_lock_right);
          steer_subscriptions_[driver] = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(steer_topic_name, rclcpp::QoS{1}, func_steer);

          std::string accel_topic_name = "/" + driver + "/velocity_pid_state";
          std::function<void(const control_msgs::msg::PidState::ConstPtr&)> func_accel
          = std::bind(&MultiagentControlNode::setAcceleration_, this, std::placeholders::_1,  driver, full_lock_left, full_lock_right);
          accel_subscriptions_[driver] = create_subscription<control_msgs::msg::PidState>(accel_topic_name, rclcpp::SensorDataQoS(), func_accel);

          last_direct_input_[driver] = deepracing_msgs::msg::XinputState();

          std::string state_out_name = "/" + driver + "/controller_state";
          state_publishers_[driver] = create_publisher<deepracing_msgs::msg::XinputState>(state_out_name, rclcpp::QoS{1});
        }
        // std::function<void(const std::string&)> func_timer = ;
        timer_ = rclcpp::create_timer(get_node_base_interface(), get_node_timers_interface(), 
          get_clock(), rclcpp::Duration::from_seconds(0.05), std::bind(&MultiagentControlNode::timerCB_, this));
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(get_logger(), "Could not allocated interface, error message: %s", e.what());
      }
    }
  private:
    ackermann_msgs::msg::AckermannDriveStamped::UniquePtr current_steering_cmd_;
    deepf1::MultiagentF1InterfaceFactory factory_;
    std::unordered_map<std::string, std::shared_ptr<deepf1::F1Interface>> vigem_interfaces_;
    std::unordered_map<std::string, std::shared_ptr<deepf1::F1Interface>> vjoy_interfaces_;
    std::unordered_map<std::string, rclcpp::Subscription<deepracing_msgs::msg::XinputState>::SharedPtr> direct_subscriptions_;
    std::unordered_map<std::string, rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr> steer_subscriptions_;
    std::unordered_map<std::string, rclcpp::Subscription<control_msgs::msg::PidState>::SharedPtr> accel_subscriptions_;
    std::unordered_map<std::string, rclcpp::Publisher<deepracing_msgs::msg::XinputState>::SharedPtr> state_publishers_;
    std::unordered_map<std::string, deepracing_msgs::msg::XinputState> last_direct_input_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex set_steering_mutex_;
    std::vector<std::string> driver_names_;
    double deadzone_ratio_;
    void timerCB_()
    {
      const rclcpp::Time time = get_clock()->now();
      for(const std::string& driver : driver_names_)
      {
        state_publishers_[driver]->publish(
          deepracing_ros::XinputMsgUtils::toMsg(
            vigem_interfaces_[driver]->getCurrentState()).set__header(std_msgs::msg::Header().set__stamp(time))
          );
      }
    }
    void setStateDirect_(const deepracing_msgs::msg::XinputState::ConstPtr& state, const std::string& driver)
    {
      RCLCPP_INFO(get_logger(), "Setting state directly for driver %s", driver.c_str());
      vigem_interfaces_[driver]->setStateDirectly(deepracing_ros::XinputMsgUtils::toXinput(*state));
      //vjoy_interfaces_[driver]->setStateDirectly(deepracing_ros::XinputMsgUtils::toXinput(*state));
      last_direct_input_[driver] = *state;
    }
    void setSteering_(ackermann_msgs::msg::AckermannDriveStamped::UniquePtr& ackermann_cmd, 
      const std::string& driver, const double& full_lock_left, const double& full_lock_right)
    {
      std::scoped_lock lock(set_steering_mutex_);
      current_steering_cmd_ = std::move(ackermann_cmd);
    }
    void setAcceleration_(const control_msgs::msg::PidState::ConstPtr& pid_state, 
    const std::string& driver, const double& full_lock_left, const double& full_lock_right)
    {
      {
        std::scoped_lock lock(set_steering_mutex_);
        if(!current_steering_cmd_)
        {
          return;
        }
      }
      const rclcpp::Time& now = get_clock()->now();
      const rclcpp::Duration& dt = now - last_direct_input_[driver].header.stamp;
      if (dt<rclcpp::Duration::from_seconds(0.25))
      {
        return;
      }
      deepf1::F1ControlCommand cmd;
      const double& accel = std::clamp<double>(pid_state->output, -1.0, 1.0);
      if(accel>=0.0)
      {
        cmd.throttle = accel;
        cmd.brake = 0.0;
      }
      else
      {
        cmd.throttle = 0.0;
        cmd.brake = -accel;
      }
      double desired_steering;
      {
        std::scoped_lock lock(set_steering_mutex_);
        desired_steering = std::clamp<double>(current_steering_cmd_->drive.steering_angle, full_lock_right, full_lock_left);
      } 
      if(desired_steering>0.0)
      {
        cmd.steering = std::clamp<double>((1-deadzone_ratio_)*desired_steering/full_lock_left + deadzone_ratio_, -1.0, 1.0);
      }
      else if(desired_steering<0.0)
      {
        cmd.steering =  std::clamp<double>(-(1-deadzone_ratio_)*desired_steering/full_lock_right - deadzone_ratio_, -1.0, 1.0);
      }
      else
      {
        cmd.steering = 0.0;
      }
      // RCLCPP_INFO(get_logger(), "Applying steering ratio: %f", cmd.steering);
      // RCLCPP_INFO(get_logger(), "Setting vigem value to : %f", cmd.steering);
      // vjoy_interfaces_[driver]->setCommands(cmd);
      vigem_interfaces_[driver]->setCommands(cmd);
    }
};

}
}
#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::MultiagentControlNode);