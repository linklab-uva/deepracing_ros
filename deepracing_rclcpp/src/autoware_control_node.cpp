#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <functional>
#include <deepracing_msgs/msg/timestamped_packet_motion_data.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <f1_datalogger/controllers/f1_interface_factory.h>

class AutowareControlNode : public rclcpp::Node 
{

  public:
    AutowareControlNode( const rclcpp::NodeOptions & options )
     : rclcpp::Node("autoware_control_node", options), m_current_speed_(0.0)
    {
    }
    void init(std::shared_ptr<control_toolbox::PidROS> pid)
    {
      m_velocity_pid_ = pid;
      m_velocity_pid_->initPid(1.25, 0.025, 0.005, 0.2, -0.2, true);
      m_velocity_pid_->setCurrentCmd(0.0);
      m_velocity_pid_->computeCommand(0.0, rclcpp::Duration::from_seconds(0.0));
      m_game_interface_ = deepf1::F1InterfaceFactory::getDefaultInterface();
      command_listener = create_subscription<autoware_auto_msgs::msg::VehicleControlCommand>("ctrl_cmd", rclcpp::QoS{1},
        std::bind(&AutowareControlNode::commandCallback, this, std::placeholders::_1));
      odom_listener = create_subscription<nav_msgs::msg::Odometry>("/ego_vehicle/odom", rclcpp::QoS{1},
        std::bind(&AutowareControlNode::odomCallback, this, std::placeholders::_1));
    }
    inline void controlLoop(const rclcpp::Duration& dt)
    {
      double error = m_setpoints.velocity_mps-m_current_speed_;
      double throttlecommand = m_velocity_pid_->computeCommand(m_setpoints.velocity_mps-m_current_speed_, dt);
      // double throttlecommand;
      // if (error>0.0)
      // {
      //   throttlecommand = 1.0;
      // }
      // else
      // {
      //   throttlecommand = -1.0;
      // }
      double steercommand = m_setpoints.front_wheel_angle_rad;
      deepf1::F1ControlCommand cmd;
      if (steercommand>=0.0)
      {
        cmd.steering=steercommand/0.2986730635166168;
      }
      else
      {
        cmd.steering=steercommand/0.26346784830093384;
      }
      if(throttlecommand>=0.0)
      {
        cmd.throttle=throttlecommand;
        cmd.brake=0.0;
      }
      else
      {
        cmd.throttle=0.0;
        cmd.brake=-throttlecommand;
      }
      m_game_interface_->setCommands(cmd);
    }
  private:

    inline void commandCallback(const autoware_auto_msgs::msg::VehicleControlCommand::SharedPtr new_commands)
    {
      m_setpoints = *new_commands;
      // m_setpoints.velocity_mps*=0.9;
    }
    
    inline void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
      m_current_speed_ = odom->twist.twist.linear.x;
    }
    double m_current_speed_;
    std::shared_ptr<control_toolbox::PidROS> m_velocity_pid_;
    std::shared_ptr<deepf1::F1Interface> m_game_interface_;
    autoware_auto_msgs::msg::VehicleControlCommand m_setpoints;
    rclcpp::Subscription<autoware_auto_msgs::msg::VehicleControlCommand>::SharedPtr command_listener;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_listener;

};
int main(int argc, char *argv[]) {
  rclcpp::init(argc,argv);
  std::shared_ptr<AutowareControlNode> node(new AutowareControlNode(rclcpp::NodeOptions()));
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor( new rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 3) );
  executor->add_node(node);
  rclcpp::Rate rate(node->declare_parameter<double>("frequency", 25.0));
  std::shared_ptr<control_toolbox::PidROS> pid_controller(new control_toolbox::PidROS(node));
  node->init(pid_controller);
  std::thread spinthread = std::thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, executor));
  rclcpp::Time t0 = node->now();
  rclcpp::Time t1;
  while(rclcpp::ok())
  {
    rate.sleep();
    t1 = node->now();
    node->controlLoop(t1 - t0);
    t0 = t1;
  }
  spinthread.join();
  
}