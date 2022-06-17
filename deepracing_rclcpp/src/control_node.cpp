#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <functional>
#include <deepracing_msgs/msg/timestamped_packet_car_telemetry_data.hpp>
#include <deepracing_msgs/msg/timestamped_packet_car_status_data.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <f1_datalogger/controllers/f1_interface_factory.h>

class ControlNode : public rclcpp::Node 
{

  public:
    ControlNode( const rclcpp::NodeOptions & options )
     : rclcpp::Node("control_node", options), m_current_speed_(0.0), m_external_setpoint_(0.0)
    {
      m_safe_vel_ = declare_parameter<double>("safe_vel", 23.0);
      m_full_lock_left_ = declare_parameter<double>("full_lock_left", 0.295);
      m_full_lock_right_ = declare_parameter<double>("full_lock_right", -0.2659);
      m_safe_steer_max_ = declare_parameter<double>("safe_steer_max", m_full_lock_left_);
      m_safe_steer_min_ = declare_parameter<double>("safe_steer_min", m_full_lock_right_);
      m_frequency_ = declare_parameter<double>("frequency", 25.0);

      rcl_interfaces::msg::ParameterDescriptor use_external_setpoint_description;
      use_external_setpoint_description.name="use_external_setpoint";
      use_external_setpoint_description.read_only=false;
      use_external_setpoint_description.dynamic_typing=false;
      use_external_setpoint_description.type=rclcpp::PARAMETER_BOOL;
      m_use_external_setpoint_ = declare_parameter<bool>(use_external_setpoint_description.name, false, use_external_setpoint_description);    
    }
    void initParameterCallbacks()
    {
      cb_handle_ = add_on_set_parameters_callback(std::bind(&ControlNode::setParamsCB_, this, std::placeholders::_1));
    }
    void init(std::shared_ptr<control_toolbox::PidROS> pid)
    {

      m_velocity_pid_ = pid;
      m_velocity_pid_->initPid(0.5, 0.05, 0.00, 1.0, -1.0, true);
      m_velocity_pid_->setCurrentCmd(0.0);
      m_velocity_pid_->computeCommand(0.0, rclcpp::Duration::from_seconds(0.0));

      m_game_interface_ = deepf1::F1InterfaceFactory::getDefaultInterface();
      rclcpp::SubscriptionOptions listner_options;
      listner_options.callback_group=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      command_listener = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("ctrl_cmd", rclcpp::QoS{1},
        std::bind(&ControlNode::commandCallback, this, std::placeholders::_1), listner_options);
      listner_options = rclcpp::SubscriptionOptions();
      listner_options.callback_group=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      status_listener = create_subscription<deepracing_msgs::msg::TimestampedPacketCarStatusData>("status_data", rclcpp::QoS{1},
        std::bind(&ControlNode::statusCallback, this, std::placeholders::_1), listner_options);
      listner_options = rclcpp::SubscriptionOptions();
      listner_options.callback_group=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      telemetry_listener = create_subscription<deepracing_msgs::msg::TimestampedPacketCarTelemetryData>("telemetry_data", rclcpp::QoS{1},
        std::bind(&ControlNode::telemetryCallback, this, std::placeholders::_1), listner_options);
      listner_options = rclcpp::SubscriptionOptions();
      listner_options.callback_group=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      odom_listener = create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::QoS{1},
        std::bind(&ControlNode::odomCallback, this, std::placeholders::_1), listner_options);
      if(m_use_external_setpoint_)
      {
        listner_options = rclcpp::SubscriptionOptions();
        listner_options.callback_group=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        setpoint_listener = create_subscription<std_msgs::msg::Float64>("external_setpoint", rclcpp::QoS{1},
          std::bind(&ControlNode::longitudinalErrorCB, this, std::placeholders::_1), listner_options);
      }
    }
    inline void controlLoop(const rclcpp::Duration& dt)
    {
      double steercommand = m_setpoints.drive.steering_angle;
      double error;
      if(m_use_external_setpoint_)
      {
        error = m_external_setpoint_ - m_current_speed_;
      }
      else
      {
        if (steercommand<=m_safe_steer_max_ && steercommand>=m_safe_steer_min_)
        {
          error = m_setpoints.drive.speed-m_current_speed_;
        }
        else
        {
          error = m_safe_vel_-m_current_speed_;
        }
      }
      double throttlecommand = m_velocity_pid_->computeCommand(error, dt);
      if (m_setpoints.drive.speed>=86.0)
      {
        throttlecommand=1.0;
      }
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
    rcl_interfaces::msg::SetParametersResult setParamsCB_(const std::vector<rclcpp::Parameter> & parameters)
    {
      RCLCPP_INFO(get_logger(), "Setting parameters of PID control node.");
      rcl_interfaces::msg::SetParametersResult result;
      control_toolbox::Pid::Gains gains = m_velocity_pid_->getGains();
      bool update_gains = false;
      for (const rclcpp::Parameter & parameter : parameters) {
        const std::string param_name = parameter.get_name();
        if (param_name == "p") {
          gains.p_gain_ = parameter.get_value<double>();
          update_gains = true;
        } else if (param_name == "i") {
          gains.i_gain_ = parameter.get_value<double>();
          update_gains = true;
        } else if (param_name == "d") {
          gains.d_gain_ = parameter.get_value<double>();
          update_gains = true;
        } else if (param_name == "i_clamp_max") {
          gains.i_max_ = parameter.get_value<double>();
          update_gains = true;
        } else if (param_name == "i_clamp_min") {
          gains.i_min_ = parameter.get_value<double>();
          update_gains = true;
        } else if (param_name == "antiwindup") {
          gains.antiwindup_ = parameter.get_value<bool>();
          update_gains = true;
        } else if (param_name == "use_external_setpoint") {
          bool new_use_external_setpoint = parameter.get_value<bool>();
          RCLCPP_INFO(get_logger(), "Yay: %d", new_use_external_setpoint);         
          if (new_use_external_setpoint && !setpoint_listener){
            rclcpp::SubscriptionOptions listener_options;
            listener_options.callback_group=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            setpoint_listener = create_subscription<std_msgs::msg::Float64>("external_setpoint", rclcpp::QoS{1},
              std::bind(&ControlNode::longitudinalErrorCB, this, std::placeholders::_1), listener_options);
          }         
          if (!new_use_external_setpoint && bool(setpoint_listener)){
            setpoint_listener.reset();
          }
          m_use_external_setpoint_ = new_use_external_setpoint;
        }  
      }
      if (update_gains)
      {
        m_velocity_pid_->setGains(gains);
      }
      result.set__successful(true);
      result.set__reason("You");

      return result;
    }
    inline void longitudinalErrorCB(const std_msgs::msg::Float64::SharedPtr error_value)
    {
      m_external_setpoint_ = error_value->data;
    }
    inline void telemetryCallback(const deepracing_msgs::msg::TimestampedPacketCarTelemetryData::SharedPtr telemetry_data)
    {
      m_drs_enabled_ = telemetry_data->udp_packet.car_telemetry_data.at(telemetry_data->udp_packet.header.player_car_index).drs>0;
    }
    inline void statusCallback(const deepracing_msgs::msg::TimestampedPacketCarStatusData::SharedPtr status_data)
    {
      m_drs_allowed_ = status_data->udp_packet.car_status_data.at(status_data->udp_packet.header.player_car_index).drs_allowed>0;
    }
    inline void commandCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr new_commands)
    {
      ackermann_msgs::msg::AckermannDriveStamped newsetpoints(*new_commands);
      // if (newsetpoints.drive.speed>=79.5)
      // {
      //   newsetpoints.drive.speed=89.5;
      // }
      m_setpoints = newsetpoints;
    }
    
    inline void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
      // m_current_speed_ = std::sqrt( odom->twist.twist.linear.x*odom->twist.twist.linear.x + 
      //                               odom->twist.twist.linear.y*odom->twist.twist.linear.y +
      //                               odom->twist.twist.linear.z*odom->twist.twist.linear.z );
      m_current_speed_ = odom->twist.twist.linear.x;
    }
    double m_current_speed_, m_safe_steer_max_, m_safe_steer_min_, m_safe_vel_, m_full_lock_left_, m_full_lock_right_, m_external_setpoint_, m_frequency_;
    bool m_drs_allowed_, m_drs_enabled_, m_use_external_setpoint_;
    std::shared_ptr<control_toolbox::PidROS> m_velocity_pid_;
    std::shared_ptr<deepf1::F1Interface> m_game_interface_;
    ackermann_msgs::msg::AckermannDriveStamped m_setpoints;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_listener;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_listener;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_listener;
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketCarStatusData>::SharedPtr status_listener;
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketCarTelemetryData>::SharedPtr telemetry_listener;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;

};
int main(int argc, char *argv[]) {
  rclcpp::init(argc,argv);
  std::shared_ptr<ControlNode> node(new ControlNode(rclcpp::NodeOptions()));
  rclcpp::Rate rate(node->get_parameter("frequency").as_double());
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor( new rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 3) );
  executor->add_node(node);
  rclcpp::Clock::SharedPtr clock = node->get_clock();
  std::shared_ptr<control_toolbox::PidROS> pid_controller(new control_toolbox::PidROS(node));
  node->init(pid_controller);
  node->remove_on_set_parameters_callback(pid_controller->getParametersCallbackHandle().get());
  double p, i, d, i_clamp_min, i_clamp_max;
  bool antiwindup;
  node->get_parameter_or<double>("p", p, 0.5);
  node->get_parameter_or<double>("i", i, 0.0);
  node->get_parameter_or<double>("d", d, 0.0);
  node->get_parameter_or<double>("i_clamp_min", i_clamp_min, -1.0);
  node->get_parameter_or<double>("i_clamp_max", i_clamp_max, 1.0);
  node->get_parameter_or<bool>("antiwindup", antiwindup, true);
  pid_controller->setGains(p, i, d, i_clamp_min, i_clamp_max, antiwindup);
  node->initParameterCallbacks();
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