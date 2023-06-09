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

  class MapToOdomTransform : public rclcpp::Node
  {
    public:
      DEEPRACING_RCLCPP_PUBLIC MapToOdomTransform(const rclcpp::NodeOptions & options) : 
          rclcpp::Node("map_to_odom_transform", options)
      {

        odom_frame_ = declare_parameter<std::string>("odom_frame");
        base_link_frame_ = declare_parameter<std::string>("base_link_frame");
        map_frame_ = declare_parameter<std::string>("map_frame", "map");

        std::chrono::nanoseconds cache_length(20000000000);
        tf_buffer_ =  std::make_unique<tf2_ros::Buffer>(this->get_clock(), cache_length);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ =  std::make_unique<tf2_ros::TransformBroadcaster>(this);
          // rclcpp::QoS qos = rclcpp::SystemDefaultsQoS().history();
        m_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("odom/filtered", 1, 
            std::bind(&MapToOdomTransform::udp_cb, this, std::placeholders::_1));


      } 
    private:
      inline DEEPRACING_RCLCPP_LOCAL void udp_cb(const nav_msgs::msg::Odometry::ConstPtr& map_to_bl_odom)
      {

        const geometry_msgs::msg::Point & position_msg = map_to_bl_odom->pose.pose.position;
        Eigen::Vector3d position_eigen(position_msg.x, position_msg.y, position_msg.z);

        const geometry_msgs::msg::Quaternion & quaternion_msg = map_to_bl_odom->pose.pose.orientation;
        Eigen::Quaterniond quaternion_eigen(quaternion_msg.w, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z);

        Eigen::Isometry3d map_to_bl_eigen;
        map_to_bl_eigen.fromPositionOrientationScale(position_eigen, quaternion_eigen, Eigen::Vector3d::Ones());

        geometry_msgs::msg::TransformStamped odom_to_bl;
        Eigen::Isometry3d odom_to_bl_eigen;
        try{
          odom_to_bl = tf_buffer_->lookupTransform(odom_frame_, base_link_frame_, map_to_bl_odom->header.stamp, rclcpp::Duration::from_seconds(0.1));
          odom_to_bl_eigen= tf2::transformToEigen(odom_to_bl);
        }catch(std::exception& ex)
        {
          return;
        }

        Eigen::Isometry3d map_to_odom_eigen = map_to_bl_eigen*odom_to_bl_eigen.inverse();

        geometry_msgs::msg::TransformStamped transform_out = tf2::eigenToTransform(map_to_odom_eigen);
        transform_out.header.set__stamp(map_to_bl_odom->header.stamp).set__frame_id(map_frame_);
        transform_out.set__child_frame_id(odom_frame_);
        tf_broadcaster_->sendTransform(transform_out);

      }
      
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscription_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};

      std::string base_link_frame_, odom_frame_, map_frame_;
  };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::MapToOdomTransform)
