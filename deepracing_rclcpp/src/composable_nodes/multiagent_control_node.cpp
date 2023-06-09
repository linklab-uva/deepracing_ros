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
      MultiagentControlNode(const rclcpp::NodeOptions &options)
          : rclcpp::Node("multiagent_control_node", options)
      {
        driver_names_ = declare_parameter<std::vector<std::string>>("driver_names", std::vector<std::string>());
        if (driver_names_.size() < 1 || driver_names_.size() > 4)
        {
          throw std::runtime_error("Must declare between 1 and 4 driver names");
        }
        std::vector<double> full_lock_left_vec = declare_parameter<std::vector<double>>("full_lock_left", std::vector<double>());
        if (full_lock_left_vec.size() != driver_names_.size())
        {
          std::stringstream error_stream;
          error_stream << "Declared" << driver_names_.size() << "drivers, but " << full_lock_left_vec.size() << " full-lock-left values.";
          throw std::runtime_error(error_stream.str());
        }
        std::vector<double> full_lock_right_vec = declare_parameter<std::vector<double>>("full_lock_right", std::vector<double>());
        if (full_lock_right_vec.size() != driver_names_.size())
        {
          std::stringstream error_stream;
          error_stream << "Declared" << driver_names_.size() << "drivers, but " << full_lock_right_vec.size() << " full-lock-right values.";
          throw std::runtime_error(error_stream.str());
        }
        try
        {
          const rclcpp::Time& now = get_clock()->now();
          for (std::size_t i = 0; i < driver_names_.size(); i++)
          {
            std::string driver = driver_names_.at(i);
            double full_lock_left = full_lock_left_vec.at(i);
            double full_lock_right = full_lock_right_vec.at(i);
            interfaces_[driver] = factory_.createInterface();
            // vjoy_interfaces_[driver] = deepf1::F1InterfaceFactory::getDefaultInterface(i+1);
            std::string state_in_topic = "/" + driver + "/controller_input";
            std::string state_out_topic = "/" + driver + "/controller_state";
            std::function<void(const deepracing_msgs::msg::XinputState::UniquePtr &)> func_setstate
              = std::bind(&MultiagentControlNode::setState_, this, std::placeholders::_1, driver);
            xinput_subscriptions_[driver] = create_subscription<deepracing_msgs::msg::XinputState>(state_in_topic, rclcpp::QoS{1}, func_setstate);
            xinput_publishers_[driver] = create_publisher<deepracing_msgs::msg::XinputState>(state_out_topic, rclcpp::QoS{1});
            last_input_[driver] = deepracing_msgs::msg::XinputState().set__header(std_msgs::msg::Header().set__stamp(now));

          }
          // std::function<void(const std::string&)> func_timer = ;
          timer_ = rclcpp::create_timer(get_node_base_interface(), get_node_timers_interface(),
                                        get_clock(), rclcpp::Duration::from_seconds(0.05), std::bind(&MultiagentControlNode::timerCB_, this));
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(get_logger(), "Could not allocated interface, error message: %s", e.what());
        }
      }

    private:
      ackermann_msgs::msg::AckermannDriveStamped::UniquePtr current_steering_cmd_;
      deepf1::MultiagentF1InterfaceFactory factory_;
      std::unordered_map<std::string, std::shared_ptr<deepf1::F1Interface>> interfaces_;
      std::unordered_map<std::string, rclcpp::Subscription<deepracing_msgs::msg::XinputState>::SharedPtr> xinput_subscriptions_;
      std::unordered_map<std::string, std::mutex> xinput_mutexes_;
      std::unordered_map<std::string, rclcpp::Publisher<deepracing_msgs::msg::XinputState>::SharedPtr> xinput_publishers_;
      std::unordered_map<std::string, deepracing_msgs::msg::XinputState> last_input_;

      rclcpp::TimerBase::SharedPtr timer_;
      std::mutex set_steering_mutex_;
      std::vector<std::string> driver_names_;
      void timerCB_()
      {
        for (const std::string &driver : driver_names_)
        {
          xinput_publishers_[driver]->publish(last_input_[driver]);
        }
      }
      std::mutex& mutexGetter_(const std::string& driver)
      {
          return xinput_mutexes_[driver];
      }
      void setState_(const deepracing_msgs::msg::XinputState::UniquePtr &state, const std::string &driver)
      {
        RCLCPP_DEBUG(get_logger(), "Setting state for driver %s", driver.c_str());
        const rclcpp::Time& time = get_clock()->now();
        { 
          std::lock_guard lock(mutexGetter_(driver));
          last_input_[driver] = state->set__header(std_msgs::msg::Header().set__stamp(time));
          interfaces_[driver]->setStateDirectly(deepracing_ros::XinputMsgUtils::toXinput(last_input_[driver]));
        }
      }
    };

  }
}
#include <rclcpp_components/register_node_macro.hpp> // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::MultiagentControlNode);