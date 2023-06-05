#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <functional>
#include <f1_datalogger/controllers/multiagent_f1_interface_factory.h>
#include <deepracing_msgs/msg/xinput_state.hpp>
#include <deepracing_ros/utils/xinput_msg_utils.h>
#include <memory>

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
          std::function<void(const deepracing_msgs::msg::XinputState::ConstPtr&)> func
          = std::bind(&MultiagentControlNode::setStateDirect_, this, std::placeholders::_1,  driver);
          direct_subscriptions_[driver] = create_subscription<deepracing_msgs::msg::XinputState>(direct_topic_name, rclcpp::SensorDataQoS(), func);
        }
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(get_logger(), "Could not allocated interface, error message: %s", e.what());
      }
    }
  private:
    deepf1::MultiagentF1InterfaceFactory factory_;
    std::map<std::string, std::shared_ptr<deepf1::VigemInterface>> interfaces_;
    std::map<std::string, rclcpp::Subscription<deepracing_msgs::msg::XinputState>::SharedPtr> direct_subscriptions_;
    rclcpp::TimerBase::SharedPtr timer_;

    void setStateDirect_(const deepracing_msgs::msg::XinputState::ConstPtr& state, const std::string& driver)
    {
      RCLCPP_INFO(get_logger(), "Setting state directly for driver %s", driver.c_str());
      interfaces_[driver]->setStateDirectly(deepracing_ros::XinputMsgUtils::toXinput(*state));
    }
};

}
}
#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::MultiagentControlNode);