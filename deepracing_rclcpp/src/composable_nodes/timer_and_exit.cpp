#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>

namespace deepracing
{
namespace composable_nodes
{
//.automatically_declare_parameters_from_overrides(true)
    class TimerAndExit : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC TimerAndExit(const rclcpp::NodeOptions & options) :
                rclcpp::Node("timer_and_exit", rclcpp::NodeOptions(options).allow_undeclared_parameters(true))
            {
                m_start_time_ = std::chrono::steady_clock::now();
                m_ros_start_time_ = this->get_clock()->now();
                m_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TimerAndExit::timer_cb, this));
                declare_parameter<int>("exit_after_seconds", 300);
                declare_parameter<int>("exit_after_ros_seconds", 80);
            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void timer_cb()
            {
                int exit_after_seconds = get_parameter("exit_after_seconds").as_int();
                int exit_after_ros_seconds = get_parameter("exit_after_ros_seconds").as_int();
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - m_start_time_).count();
                if (elapsed >= exit_after_seconds) {
                    RCLCPP_INFO(this->get_logger(), "Exiting because wall time since start exceeded %d seconds", exit_after_seconds);
                    exit(0);
                }
                rclcpp::Time ros_time = this->get_clock()->now();
                rclcpp::Duration ros_duration = ros_time - m_ros_start_time_;
                if (ros_duration >= rclcpp::Duration::from_seconds(float(exit_after_ros_seconds))) {
                    RCLCPP_INFO(this->get_logger(), "Exiting because ROS time exceeded %d seconds", exit_after_ros_seconds);
                    exit(0);
                }

            }
            rclcpp::TimerBase::SharedPtr m_timer_;
            std::chrono::steady_clock::time_point m_start_time_;
            rclcpp::Time m_ros_start_time_;

    };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::TimerAndExit)
