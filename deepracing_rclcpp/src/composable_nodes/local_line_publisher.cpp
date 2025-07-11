#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <deepracing/pcl_types.hpp>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

namespace deepracing
{
namespace composable_nodes
{

    class LocalLinePublisher : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC LocalLinePublisher(const rclcpp::NodeOptions & options) : 
                rclcpp::Node("local_line_publisher", options)
            {

                rclcpp::QoS qos = rclcpp::SystemDefaultsQoS().keep_last(10).durability_volatile();
                m_ib_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("inner_boundary_local", qos);
                m_ob_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("outer_boundary_local", qos);
                m_rl_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("optimal_raceline_local", qos);
    
                m_rl_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>("/optimal_raceline", qos, 
                    std::bind(&LocalLinePublisher::rl_cb, this, std::placeholders::_1));
                m_ib_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>("/inner_boundary", qos, 
                    std::bind(&LocalLinePublisher::ib_cb, this, std::placeholders::_1));
                m_ob_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>("/outer_boundary", qos, 
                    std::bind(&LocalLinePublisher::ob_cb, this, std::placeholders::_1));

                m_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("odom/filtered", qos, 
                    std::bind(&LocalLinePublisher::odom_cb, this, std::placeholders::_1));

            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void odom_cb(const nav_msgs::msg::Odometry::ConstPtr& odom_msg)
            {
                if ((!m_ib_) || (!m_ob_) || (!m_raceline_)){
                    return;
                }
                Eigen::Isometry3d odom_eigen_d; 
                tf2::fromMsg(odom_msg->pose.pose, odom_eigen_d);
                Eigen::Isometry3d odom_eigen_inverse_d = odom_eigen_d.inverse(); 
                Eigen::Isometry3f odom_eigen_inv = odom_eigen_inverse_d.cast<float>();
                pcl::PointCloud<deepracing::PointXYZTALS> rl_local;
                pcl::PointCloud<deepracing::PointXYZLapdistance> ib_local, ob_local;
                {
                    std::lock_guard<std::mutex> lock(m_rl_mutex_);
                    pcl::transformPointCloud(*m_raceline_, rl_local, odom_eigen_inv);
                }
                {
                    std::lock_guard<std::mutex> lock(m_ib_mutex_);
                    pcl::transformPointCloud(*m_ib_, ib_local, odom_eigen_inv);
                }
                {
                    std::lock_guard<std::mutex> lock(m_ob_mutex_);
                    pcl::transformPointCloud(*m_ob_, ob_local, odom_eigen_inv);
                }
                
                sensor_msgs::msg::PointCloud2 rl_local_msg, ib_local_msg, ob_local_msg;
                pcl::toROSMsg<deepracing::PointXYZTALS>(rl_local, rl_local_msg);
                pcl::toROSMsg<deepracing::PointXYZLapdistance>(ib_local, ib_local_msg);
                pcl::toROSMsg<deepracing::PointXYZLapdistance>(ob_local, ob_local_msg);

                rl_local_msg.set__header(odom_msg->header);
                rl_local_msg.header.set__frame_id(odom_msg->child_frame_id);
                ib_local_msg.set__header(rl_local_msg.header);
                ob_local_msg.set__header(rl_local_msg.header);

                m_ib_publisher_->publish(ib_local_msg);
                m_ob_publisher_->publish(ob_local_msg);
                m_rl_publisher_->publish(rl_local_msg);


            }
            inline DEEPRACING_RCLCPP_LOCAL void ib_cb(const sensor_msgs::msg::PointCloud2::ConstPtr& ib_msg)
            {
                {
                    std::lock_guard<std::mutex> lock(m_ib_mutex_);
                    m_ib_.reset(new pcl::PointCloud<deepracing::PointXYZLapdistance>());
                    pcl::fromROSMsg(*ib_msg, *m_ib_);
                }

            }
            inline DEEPRACING_RCLCPP_LOCAL void ob_cb(const sensor_msgs::msg::PointCloud2::ConstPtr& ob_msg)
            {
                {
                    std::lock_guard<std::mutex> lock(m_ob_mutex_);
                    m_ob_.reset(new pcl::PointCloud<deepracing::PointXYZLapdistance>());
                    pcl::fromROSMsg(*ob_msg, *m_ob_);
                }

            }
            inline DEEPRACING_RCLCPP_LOCAL void rl_cb(const sensor_msgs::msg::PointCloud2::ConstPtr& raceline_msg)
            {
                {
                    std::lock_guard<std::mutex> lock(m_rl_mutex_);
                    m_raceline_.reset(new pcl::PointCloud<deepracing::PointXYZTALS>());
                    pcl::fromROSMsg(*raceline_msg, *m_raceline_);
                }

            }
            
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_ib_subscription_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_ob_subscription_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_rl_subscription_;

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscription_;
            
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_ib_publisher_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_ob_publisher_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_rl_publisher_;


            std::shared_ptr<pcl::PointCloud<deepracing::PointXYZTALS>> m_raceline_;
            std::shared_ptr<pcl::PointCloud<deepracing::PointXYZLapdistance>> m_ib_;
            std::shared_ptr<pcl::PointCloud<deepracing::PointXYZLapdistance>> m_ob_;
            
            std::mutex m_ib_mutex_, m_ob_mutex_, m_rl_mutex_;
    };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::LocalLinePublisher)
