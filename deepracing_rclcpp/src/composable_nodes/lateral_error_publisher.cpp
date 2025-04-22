#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <deepracing/pcl_types.hpp>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace deepracing
{
namespace composable_nodes
{

    class LateralErrorPublisher : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC LateralErrorPublisher(const rclcpp::NodeOptions & options) : 
                rclcpp::Node("lateral_error_publisher", options)
            {

                rclcpp::QoS qos = rclcpp::SystemDefaultsQoS().keep_last(10).durability_volatile();
                m_raceline_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>("/optimal_raceline", qos, 
                    std::bind(&LateralErrorPublisher::raceline_cb, this, std::placeholders::_1));
                m_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("odom/filtered", qos, 
                    std::bind(&LateralErrorPublisher::odom_cb, this, std::placeholders::_1));
                m_error_publisher_ = create_publisher<std_msgs::msg::Float64>("lateral_error", qos);
                m_refpoint_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("reference_point", qos);
            

            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void odom_cb(const nav_msgs::msg::Odometry::ConstPtr& odom_msg)
            {
                if(!m_raceline_)
                {
                    RCLCPP_WARN(get_logger(), "Raceline not received yet, skipping odom callback");
                    return;
                }
                Eigen::Isometry3d odom_eigen_d; 
                tf2::fromMsg(odom_msg->pose.pose, odom_eigen_d);
                Eigen::Translation3d offset(-1.925, 0.0, 0.0);
                odom_eigen_d = odom_eigen_d*Eigen::Translation3d(-1.925, 0.0, 0.0);;
                Eigen::Isometry3f odom_eigen = odom_eigen_d.cast<float>();
                pcl::PointCloud<deepracing::PointXYZTALS> raceline_local;
                {
                    std::lock_guard<std::mutex> lock(m_mutex_);
                    pcl::transformPointCloud(*m_raceline_, raceline_local, odom_eigen.inverse());
                }
                std::size_t closest_index = 0;
                float closest_distance = raceline_local.at(0).getVector3fMap().norm();
                for (std::size_t i = 1; i < raceline_local.size(); i++)
                {
                    float current_distance = raceline_local.at(i).getVector3fMap().norm();
                    if (current_distance < closest_distance)
                    {
                        closest_distance = current_distance;
                        closest_index = i;
                    }
                }
                const deepracing::PointXYZTALS& closest_point = raceline_local.at(closest_index);
                
                std_msgs::msg::Float64 error_msg;
                error_msg.data = closest_point.x;
                m_error_publisher_->publish(error_msg);

                geometry_msgs::msg::PointStamped refpoint_msg;
                refpoint_msg.header.stamp = odom_msg->header.stamp;
                refpoint_msg.header.frame_id = odom_msg->child_frame_id;
                refpoint_msg.point.x = closest_point.x;
                refpoint_msg.point.y = closest_point.y;
                refpoint_msg.point.z = closest_point.z;
                m_refpoint_publisher_->publish(refpoint_msg);
            }
            inline DEEPRACING_RCLCPP_LOCAL void raceline_cb(const sensor_msgs::msg::PointCloud2::ConstPtr& raceline_msg)
            {
                {
                    std::lock_guard<std::mutex> lock(m_mutex_);
                    m_raceline_.reset(new pcl::PointCloud<deepracing::PointXYZTALS>());
                    pcl::fromROSMsg(*raceline_msg, *m_raceline_);
                }

            }
            
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_raceline_subscription_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscription_;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_error_publisher_;
            rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_refpoint_publisher_;

            std::shared_ptr<pcl::PointCloud<deepracing::PointXYZTALS>> m_raceline_;
            std::mutex m_mutex_;
    };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::LateralErrorPublisher)
