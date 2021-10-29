#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <functional>
#include <deepracing_msgs/msg/timestamped_packet_motion_data.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <std_msgs/msg/float64.hpp>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

class VelocityControlNode : public rclcpp::Node 
{

  public:
    VelocityControlNode( const rclcpp::NodeOptions & options )
     : rclcpp::Node("velocity_control_node", options),
      m_current_speed_(0.0),
      m_setpoint_(0.0),
      m_error_rate_(0.0)
    {
      setpoint_listener = create_subscription<std_msgs::msg::Float64>("setpoint_in", rclcpp::QoS{1}, std::bind(&VelocityControlNode::setpointCallback, this, std::placeholders::_1));
      m_with_acceleration_ = declare_parameter<bool>("with_acceleration", false);
      if (m_with_acceleration_)
      {
        odom_synch_listener.subscribe(this, "odom_in", setpoint_listener->get_actual_qos().get_rmw_qos_profile() );
        accel_synch_listener.subscribe(this, "accel_in", setpoint_listener->get_actual_qos().get_rmw_qos_profile() );
        odom_accel_synchronizer.reset(new message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, geometry_msgs::msg::AccelStamped>(odom_synch_listener, accel_synch_listener, 4));
        odom_accel_synchronizer->registerCallback(std::bind(&VelocityControlNode::synchCallback, this, std::placeholders::_1, std::placeholders::_2));
      }
      else
      {
        odom_listener = create_subscription<nav_msgs::msg::Odometry>("odom_in", rclcpp::QoS{1}, std::bind(&VelocityControlNode::odomCallback, this, std::placeholders::_1));
      }
    }

    inline double getError()
    {
      return m_setpoint_ - m_current_speed_;
    }
    inline bool withAcceleration()
    {
      return m_with_acceleration_;
    }

  private:
    bool m_with_acceleration_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_listener;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_listener;

    std::shared_ptr< message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, geometry_msgs::msg::AccelStamped> > odom_accel_synchronizer;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_synch_listener;
    message_filters::Subscriber<geometry_msgs::msg::AccelStamped> accel_synch_listener;
    inline void synchCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& new_odom, const geometry_msgs::msg::AccelStamped::ConstSharedPtr& new_accel)
    {
      RCLCPP_DEBUG(get_logger(),"Got a synchronized pair of odom and acceleration");
      geometry_msgs::msg::Quaternion rotation = new_odom->pose.pose.orientation;
      Eigen::Quaterniond rotationeig(rotation.w, rotation.x, rotation.y, rotation.z);
      geometry_msgs::msg::Vector3 accelglobal = new_accel->accel.linear;
      Eigen::Vector3d accelglobaleig(accelglobal.x, accelglobal.y, accelglobal.z);
      Eigen::Vector3d accellocaleig = rotationeig.inverse()*accelglobaleig;
      m_current_speed_=new_odom->twist.twist.linear.x;
    }
    inline void odomCallback(const nav_msgs::msg::Odometry::SharedPtr new_odom)
    {
      RCLCPP_DEBUG(get_logger(),"Got some odom");
      m_current_speed_=new_odom->twist.twist.linear.x;
    }
    inline void setpointCallback(const std_msgs::msg::Float64::SharedPtr new_setpoint)
    {
      RCLCPP_DEBUG(get_logger(),"Got a new setpoint");
      m_setpoint_=new_setpoint->data;
    }

    double m_current_speed_, m_setpoint_, m_error_rate_;
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