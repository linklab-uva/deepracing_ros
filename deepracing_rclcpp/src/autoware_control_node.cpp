#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <functional>
#include <deepracing_msgs/msg/timestamped_packet_car_telemetry_data.hpp>
#include <deepracing_msgs/msg/timestamped_packet_car_status_data.hpp>
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
      m_safe_vel_ = declare_parameter<double>("safe_vel", 20.0);

      m_full_lock_left_ = declare_parameter<double>("full_lock_left", 0.2986730635166168);
      m_full_lock_right_ = declare_parameter<double>("full_lock_right", -0.26346784830093384);
      m_safe_steer_max_ = declare_parameter<double>("safe_steer_max", m_full_lock_left_);
      m_safe_steer_min_ = declare_parameter<double>("safe_steer_min", m_full_lock_right_);

      m_velocity_pid_ = pid;
      m_velocity_pid_->initPid(0.5, 0.05, 0.00, 1.0, -1.0, true);
      m_velocity_pid_->setCurrentCmd(0.0);
      m_velocity_pid_->computeCommand(0.0, rclcpp::Duration::from_seconds(0.0));

      m_game_interface_ = deepf1::F1InterfaceFactory::getDefaultInterface();
      rclcpp::SubscriptionOptions listner_options;
      listner_options.callback_group=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      command_listener = create_subscription<autoware_auto_msgs::msg::VehicleControlCommand>("ctrl_cmd", rclcpp::QoS{1},
        std::bind(&AutowareControlNode::commandCallback, this, std::placeholders::_1), listner_options);
      status_listener = create_subscription<deepracing_msgs::msg::TimestampedPacketCarStatusData>("/f1_game/status_data", rclcpp::QoS{1},
        std::bind(&AutowareControlNode::statusCallback, this, std::placeholders::_1), listner_options);
      telemetry_listener = create_subscription<deepracing_msgs::msg::TimestampedPacketCarTelemetryData>("/f1_game/telemetry_data", rclcpp::QoS{1},
        std::bind(&AutowareControlNode::telemetryCallback, this, std::placeholders::_1), listner_options);
      odom_listener = create_subscription<nav_msgs::msg::Odometry>("/ego_vehicle/odom", rclcpp::QoS{1},
        std::bind(&AutowareControlNode::odomCallback, this, std::placeholders::_1));
      
    }
    inline void controlLoop(const rclcpp::Duration& dt)
    {
      double steercommand = m_setpoints.front_wheel_angle_rad;
      double error;
      if (steercommand<=m_safe_steer_max_ && steercommand>=m_safe_steer_min_)
      {
        error = m_setpoints.velocity_mps-m_current_speed_;
      }
      else
      {
        error = m_safe_vel_-m_current_speed_;
      }

      double throttlecommand = m_velocity_pid_->computeCommand(error, dt);
      deepf1::F1ControlCommand cmd;
      if (steercommand>=0.0)
      {
        cmd.steering=steercommand/m_full_lock_left_;
      }
      else
      {
        cmd.steering=-steercommand/m_full_lock_right_;
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
      if( (!m_drs_enabled_) && m_drs_allowed_ )
      {
        m_game_interface_->pushDRS();
      }
    }
  private:

    inline void telemetryCallback(const deepracing_msgs::msg::TimestampedPacketCarTelemetryData::SharedPtr telemetry_data)
    {
      m_drs_enabled_ = telemetry_data->udp_packet.car_telemetry_data.at(telemetry_data->udp_packet.header.player_car_index).drs>0;
    }
    inline void statusCallback(const deepracing_msgs::msg::TimestampedPacketCarStatusData::SharedPtr status_data)
    {
      m_drs_allowed_ = status_data->udp_packet.car_status_data.at(status_data->udp_packet.header.player_car_index).drs_allowed>0;
    }
    inline void commandCallback(const autoware_auto_msgs::msg::VehicleControlCommand::SharedPtr new_commands)
    {
      autoware_auto_msgs::msg::VehicleControlCommand newsetpoints(*new_commands);
      // newsetpoints.front_wheel_angle_rad*=2.0;
      // newsetpoints.velocity_mps*=0.95;
      m_setpoints = newsetpoints;
    }
    
    inline void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
      // m_current_speed_ = std::sqrt( odom->twist.twist.linear.x*odom->twist.twist.linear.x + 
      //                               odom->twist.twist.linear.y*odom->twist.twist.linear.y +
      //                               odom->twist.twist.linear.z*odom->twist.twist.linear.z );
      m_current_speed_ =  odom->twist.twist.linear.x;
    }
    double m_current_speed_, m_safe_steer_max_, m_safe_steer_min_, m_safe_vel_, m_full_lock_left_, m_full_lock_right_;
    bool m_drs_allowed_, m_drs_enabled_;
    std::shared_ptr<control_toolbox::PidROS> m_velocity_pid_;
    std::shared_ptr<deepf1::F1Interface> m_game_interface_;
    autoware_auto_msgs::msg::VehicleControlCommand m_setpoints;
    rclcpp::Subscription<autoware_auto_msgs::msg::VehicleControlCommand>::SharedPtr command_listener;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_listener;
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketCarStatusData>::SharedPtr status_listener;
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketCarTelemetryData>::SharedPtr telemetry_listener;
    
    

};
int main(int argc, char *argv[]) {
  rclcpp::init(argc,argv);
  std::shared_ptr<AutowareControlNode> node(new AutowareControlNode(rclcpp::NodeOptions()));
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor( new rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 3) );
  executor->add_node(node);
  // rclcpp::Clock::SharedPtr clock = node->get_clock();
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