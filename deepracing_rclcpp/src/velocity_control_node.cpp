#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include <sstream>
#include <functional>
#include <deepracing_msgs/msg/timestamped_packet_motion_data.hpp>
#include <deepracing_msgs/msg/timestamped_packet_car_telemetry_data.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

class VelocityControlNode : public rclcpp::Node 
{

  public:
    VelocityControlNode( const rclcpp::NodeOptions & options )
     : rclcpp::Node("velocity_control_node", options),
      m_current_speed_(0.0),
      m_error_rate_(0.0),
      m_current_accel_(0.0),
      m_pid_controller_(get_node_base_interface(),
          get_node_logging_interface(),
          get_node_parameters_interface(),
          get_node_topics_interface())
    {
      setpoint_listener = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("ctrl_cmd", rclcpp::QoS{1}, std::bind(&VelocityControlNode::setpointCallback, this, std::placeholders::_1));
      telemetry_listener = create_subscription<deepracing_msgs::msg::TimestampedPacketCarTelemetryData>("telemetry_data", rclcpp::QoS{1}, std::bind(&VelocityControlNode::telemetryCallback, this, std::placeholders::_1));
      m_with_acceleration_ = declare_parameter<bool>("with_acceleration", false);
      m_setpoint_scale_factor_ = declare_parameter<double>("setpoint_scale_factor", 1.0);
      if (m_with_acceleration_)
      {
        odom_synch_listener.subscribe(this, "odom_in", setpoint_listener->get_actual_qos().get_rmw_qos_profile() );
        accel_synch_listener.subscribe(this, "accel_in", setpoint_listener->get_actual_qos().get_rmw_qos_profile() );
        odom_accel_synchronizer.reset(new message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, geometry_msgs::msg::AccelWithCovarianceStamped>(odom_synch_listener, accel_synch_listener, 4));
        odom_accel_synchronizer->registerCallback(std::bind(&VelocityControlNode::synchCallback, this, std::placeholders::_1, std::placeholders::_2));
      }
      else
      {
        odom_listener = create_subscription<nav_msgs::msg::Odometry>("odom_in", rclcpp::QoS{1}, std::bind(&VelocityControlNode::odomCallback, this, std::placeholders::_1));
      }
      m_pid_controller_.initPid(1.0, 0.0, 0.0, 100.0, -100.0, true);
    }

    inline double getError()
    {
      return m_setpoint_scale_factor_*m_setpoint_.drive.speed - m_current_speed_;
    }
    inline bool withAcceleration()
    {
      return m_with_acceleration_;
    }
    inline void timerCB()
    {
      rclcpp::Time now = get_clock()->now();
      if (current_time_.nanoseconds()==0)
      {
        current_time_= now;
        m_pid_controller_.setCurrentCmd(0.0);
        m_pid_controller_.computeCommand(0.0, rclcpp::Duration::from_seconds(0.0));
        return;
      }
      if(m_with_acceleration_)
      {
        double error = getError(), error_dot;
        if(error>0)
        {
          error_dot = -m_current_accel_;
        }
        else
        {
          error_dot = m_current_accel_;
        }
        m_pid_controller_.computeCommand(error, error_dot, now - current_time_);
      }
      else
      {
        m_pid_controller_.computeCommand(getError(), now - current_time_);
      }
      
      current_time_= now;
    }

    rclcpp::Time current_time_;

  private:
    bool m_with_acceleration_;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr setpoint_listener;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_listener;
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketCarTelemetryData>::SharedPtr telemetry_listener;


    std::shared_ptr< message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, geometry_msgs::msg::AccelWithCovarianceStamped> > odom_accel_synchronizer;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_synch_listener;
    message_filters::Subscriber<geometry_msgs::msg::AccelWithCovarianceStamped> accel_synch_listener;
    inline void synchCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& new_odom, const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr& new_accel)
    {
      RCLCPP_DEBUG(get_logger(),"Got a synchronized pair of odom and acceleration");
      m_current_speed_=new_odom->twist.twist.linear.x;
      m_current_accel_=new_accel->accel.accel.linear.x;
    }
    inline void odomCallback(const nav_msgs::msg::Odometry::SharedPtr new_odom)
    {
      RCLCPP_DEBUG(get_logger(),"Got some odom");
      m_current_speed_=new_odom->twist.twist.linear.x;
    }
    inline void setpointCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr new_setpoint)
    {
      RCLCPP_DEBUG(get_logger(),"Got a new setpoint");
      m_setpoint_=*new_setpoint;
    }
    inline void telemetryCallback(const deepracing_msgs::msg::TimestampedPacketCarTelemetryData::SharedPtr current_telemetry)
    {
      RCLCPP_DEBUG(get_logger(),"Got a new telemetry msg");
      m_current_telemetry_=*current_telemetry;
    }

    double m_current_speed_, m_current_accel_, m_error_rate_, m_setpoint_scale_factor_;
    ackermann_msgs::msg::AckermannDriveStamped m_setpoint_;

    deepracing_msgs::msg::TimestampedPacketCarTelemetryData m_current_telemetry_;

    control_toolbox::PidROS m_pid_controller_;
};
int main(int argc, char *argv[]) {
  rclcpp::init(argc,argv);
  std::shared_ptr<VelocityControlNode> node(new VelocityControlNode(rclcpp::NodeOptions()));
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor( new rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 3) );
  executor->add_node(node);
  double frequency = node->declare_parameter<double>("frequency", 50.0);
  // pid_controller->initPid(0.075, 0.0075, 0.005, 0.75, -0.75, true);
  // pid_controller->setCurrentCmd(0.0);
  rclcpp::TimerBase::SharedPtr timer = rclcpp::create_timer(node, node->get_clock(), rclcpp::Duration::from_seconds(1.0/frequency),
    std::bind(&VelocityControlNode::timerCB, node));
  executor->spin();
  // std::thread spinthread = std::thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, ));
  // pid_controller->computeCommand(node->getError(), rclcpp::Duration::from_seconds(0.0));
  // rclcpp::Time t0 = node->now();
  // rclcpp::Time t1;
  // while(rclcpp::ok())
  // {
  //   rate.sleep();
  //   double error = node->getError();
  //   t1 = node->now();
  //   pid_controller->computeCommand(error, t1 - t0);
  //   t0 = t1;
  // }
  // spinthread.join();
  
}