#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <functional>
#include <f1_datalogger/controllers/multiagent_f1_interface_factory.h>

#define WIN32_LEAN_AND_MEAN
#define NOGDI
#include <windows.h>
#include <Xinput.h>
#include <f1_datalogger/controllers/vigem_interface.h>

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
      try
      {
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(get_logger(), "Could not allocated interface, error message: %s", e.what());
      }

    }
  private:
    deepf1::MultiagentF1InterfaceFactory factory_;
    std::shared_ptr<deepf1::F1Interface> interface1_, interface2_;
};

}
}
#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::MultiagentControlNode)