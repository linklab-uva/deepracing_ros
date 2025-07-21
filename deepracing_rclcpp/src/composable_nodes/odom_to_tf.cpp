#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace deepracing
{
namespace composable_nodes
{
//.automatically_declare_parameters_from_overrides(true)
    class OdomToTF : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC OdomToTF(const rclcpp::NodeOptions & options) : 
                m_tf_broadcaster_(this),
                rclcpp::Node("odom_to_tf", rclcpp::NodeOptions(options).allow_undeclared_parameters(true))
            {
                m_tf_frame_ = this->declare_parameter<std::string>("tf_frame");

                rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
                m_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("odom", qos, 
                    std::bind(&OdomToTF::odom_cb, this, std::placeholders::_1));
            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void odom_cb(const nav_msgs::msg::Odometry::ConstPtr& odom_msg)
            {
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header = odom_msg->header;
                tf_msg.child_frame_id = m_tf_frame_;
                tf_msg.transform.translation.x = odom_msg->pose.pose.position.x;
                tf_msg.transform.translation.y = odom_msg->pose.pose.position.y;
                tf_msg.transform.translation.z = odom_msg->pose.pose.position.z;
                tf_msg.transform.rotation = odom_msg->pose.pose.orientation;
                m_tf_broadcaster_.sendTransform(tf_msg);
            }
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscription_;
            tf2_ros::TransformBroadcaster m_tf_broadcaster_;
            std::string m_tf_frame_;

    };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::OdomToTF)
