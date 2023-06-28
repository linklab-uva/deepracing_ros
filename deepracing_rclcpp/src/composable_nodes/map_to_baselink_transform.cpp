#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace deepracing
{
namespace composable_nodes
{

  class MapToBaselinkTransform : public rclcpp::Node
  {
    public:
      DEEPRACING_RCLCPP_PUBLIC MapToBaselinkTransform(const rclcpp::NodeOptions & options) : 
          rclcpp::Node("map_to_odom_transform", options)
      {
        tf_broadcaster_ =  std::make_unique<tf2_ros::TransformBroadcaster>(this);
        m_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("odom/filtered", 1, 
            std::bind(&MapToBaselinkTransform::udp_cb, this, std::placeholders::_1));
      } 
    private:
      inline DEEPRACING_RCLCPP_LOCAL void udp_cb(const nav_msgs::msg::Odometry::ConstPtr& map_to_bl_odom)
      {
        const geometry_msgs::msg::Point & position_msg = map_to_bl_odom->pose.pose.position;
        const geometry_msgs::msg::Quaternion & quaternion_msg = map_to_bl_odom->pose.pose.orientation;
        geometry_msgs::msg::TransformStamped transform_out;
        transform_out.transform.translation.x = position_msg.x;
        transform_out.transform.translation.y = position_msg.y;
        transform_out.transform.translation.z = position_msg.z;
        transform_out.transform.rotation = quaternion_msg;
        transform_out.set__header(map_to_bl_odom->header);
        transform_out.set__child_frame_id(map_to_bl_odom->child_frame_id);
        tf_broadcaster_->sendTransform(transform_out);
      }
      
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscription_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::MapToBaselinkTransform)
