#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp>

namespace deepracing
{
namespace composable_nodes
{
//.automatically_declare_parameters_from_overrides(true)
    class PolygonPublisher : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC PolygonPublisher(const rclcpp::NodeOptions & options) : 
                rclcpp::Node("polygon_publisher", rclcpp::NodeOptions(options).allow_undeclared_parameters(true))
            {

                rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
                m_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("odom", qos, 
                    std::bind(&PolygonPublisher::odom_cb, this, std::placeholders::_1));
                rclcpp::QoS qos_out = rclcpp::SensorDataQoS();
                m_polygon_publisher_ = create_publisher<geometry_msgs::msg::PolygonStamped>("polygon", qos_out);

                double car_width = declare_parameter<double>("car_width");
                double car_length = declare_parameter<double>("car_length");


                m_points_local_.push_back(0.5*Eigen::Vector3d(-car_length, -car_width, 0.0));
                m_points_local_.push_back(0.5*Eigen::Vector3d(car_length, -car_width, 0.0));
                m_points_local_.push_back(0.5*Eigen::Vector3d(car_length, car_width, 0.0));
                m_points_local_.push_back(0.5*Eigen::Vector3d(-car_length, car_width, 0.0));
            
            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void odom_cb(const nav_msgs::msg::Odometry::ConstPtr& odom_msg)
            {
                Eigen::Isometry3d odom_eigen;
                tf2::fromMsg(odom_msg->pose.pose, odom_eigen);
                geometry_msgs::msg::PolygonStamped polygon_out;
                polygon_out.header=odom_msg->header;
                for (const Eigen::Vector3d& point_local : m_points_local_){
                    const Eigen::Vector3d point_global = odom_eigen*point_local;
                    geometry_msgs::msg::Point32 point_global_msg;
                    point_global_msg.set__x(point_global.x()).set__y(point_global.y()).set__z(point_global.z());
                    polygon_out.polygon.points.push_back(point_global_msg);
                }
                m_polygon_publisher_->publish(polygon_out);
            }
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscription_;
            rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr m_polygon_publisher_;

            std::vector<Eigen::Vector3d> m_points_local_;
    };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::PolygonPublisher)
