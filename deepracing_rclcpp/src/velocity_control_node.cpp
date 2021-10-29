#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <functional>
#include <deepracing_msgs/msg/timestamped_packet_motion_data.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <std_msgs/msg/float64.hpp>


class VelocityControlNode : public rclcpp::Node 
{

  public:
    VelocityControlNode( const rclcpp::NodeOptions & options )
     : rclcpp::Node("velocity_control_node", options),
      m_current_speed_(0.0),
      m_setpoint_(0.0)
    {
      setpoint_listener = create_subscription<std_msgs::msg::Float64>("setpoint_in", rclcpp::QoS{1}, std::bind(&VelocityControlNode::setpointCallback, this, std::placeholders::_1));
      odom_listener = create_subscription<nav_msgs::msg::Odometry>("odom_in", rclcpp::QoS{1}, std::bind(&VelocityControlNode::odomCallback, this, std::placeholders::_1));
    }
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_listener;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_listener;

    double getError()
    {
      return m_setpoint_ - m_current_speed_;
    }

  private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr new_odom)
    {
      RCLCPP_DEBUG(get_logger(),"Got some odom");
      m_current_speed_=new_odom->twist.twist.linear.x;
    }
    void setpointCallback(const std_msgs::msg::Float64::SharedPtr new_setpoint)
    {
      RCLCPP_DEBUG(get_logger(),"Got a new setpoint");
      m_setpoint_=new_setpoint->data;
    }

    double m_current_speed_, m_setpoint_;
};
int main(int argc, char *argv[]) {
  rclcpp::init(argc,argv);
  std::shared_ptr<VelocityControlNode> node(new VelocityControlNode(rclcpp::NodeOptions()));
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor( new rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 3) );
  executor->add_node(node);
  rclcpp::Rate rate(node->declare_parameter<double>("frequency", 10.0));
  std::shared_ptr<control_toolbox::PidROS> pid_controller(new control_toolbox::PidROS(node));
  pid_controller->initPid(0.075, 0.0075, 0.005, 0.75, -0.75, true);
  std::thread spinthread = std::thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, executor));
  pid_controller->setCurrentCmd(0.0);
  pid_controller->computeCommand(node->getError(), rclcpp::Duration::from_seconds(0.0));
  rclcpp::Time t0 = node->now();
  rclcpp::Time t1;
  while(rclcpp::ok())
  {
    rate.sleep();
    double error = node->getError();
    t1 = node->now();
    pid_controller->computeCommand(error, t1 - t0);
    t0 = t1;
  }
  spinthread.join();
  
}