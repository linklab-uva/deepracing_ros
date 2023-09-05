#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <robot_localization/srv/set_state.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/logging.hpp>

namespace deepracing
{
namespace composable_nodes
{

    class EkfMonitor : public rclcpp::Node
    {
        public:
            typedef message_filters::sync_policies::ExactTime
                <nav_msgs::msg::Odometry, sensor_msgs::msg::Imu> SyncPolicy;
            DEEPRACING_RCLCPP_PUBLIC EkfMonitor(const rclcpp::NodeOptions & options) : 
                rclcpp::Node("ekf_monitor", options), initialized_(false)
            {
                rclcpp::QoS qos = rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile();

                m_state_setter_=create_client<robot_localization::srv::SetState>("set_state");

                m_imu_measurement_cache_.reset(new message_filters::Cache<sensor_msgs::msg::Imu>(10));
                m_odom_measurement_cache_.reset(new message_filters::Cache<nav_msgs::msg::Odometry>(10));

                m_imu_measurement_subscription_.reset(new message_filters::Subscriber<sensor_msgs::msg::Imu>(this, "imu", qos.get_rmw_qos_profile()));
                m_odom_measurement_subscription_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(this, "odom", qos.get_rmw_qos_profile()));

                m_sync_.reset(new message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>
                    (*m_odom_measurement_subscription_, *m_imu_measurement_subscription_, 10));
                m_sync_->registerCallback(std::bind(&EkfMonitor::sync_cb, this, std::placeholders::_1, std::placeholders::_2));
                

            } 
        private:
            inline DEEPRACING_RCLCPP_LOCAL void handle_setstate_response(rclcpp::Client<robot_localization::srv::SetState>::SharedFuture future)
            {
                RCLCPP_INFO(get_logger(), "%s", "Set the initial state");
            }

            inline DEEPRACING_RCLCPP_LOCAL void sync_cb(const nav_msgs::msg::Odometry::ConstSharedPtr odom_measurement, const sensor_msgs::msg::Imu::ConstSharedPtr imu_measurement)
            {
                m_imu_measurement_cache_->add(imu_measurement);
                m_odom_measurement_cache_->add(odom_measurement);
                if(!initialized_)
                {
                    robot_localization::srv::SetState::Request::SharedPtr req(new robot_localization::srv::SetState::Request);
                    req->state.update_vector.fill(true);
                    req->state.update_vector[9]=req->state.update_vector[10]=req->state.update_vector[11]=false;

                    req->state.pose.pose.pose=odom_measurement->pose.pose;
                    req->state.pose.pose.covariance[0]=req->state.pose.pose.covariance[7]=req->state.pose.pose.covariance[14]=
                        req->state.pose.pose.covariance[21]=req->state.pose.pose.covariance[28]=req->state.pose.pose.covariance[35]=1.0;
                    req->state.pose.header=odom_measurement->header;

                    const geometry_msgs::msg::Point& position_global_msg=odom_measurement->pose.pose.position;
                    const geometry_msgs::msg::Quaternion& quaternion_global_msg=odom_measurement->pose.pose.orientation;

                    Eigen::Vector3d position_global(position_global_msg.x, position_global_msg.y, position_global_msg.z);
                    Eigen::Quaterniond quaternion_global(quaternion_global_msg.w, quaternion_global_msg.x, quaternion_global_msg.y, quaternion_global_msg.z);
                    quaternion_global.normalize();

                    Eigen::Isometry3d pose_global_eigen;
                    pose_global_eigen.fromPositionOrientationScale(position_global, quaternion_global, Eigen::Vector3d::Ones());
               
                    const geometry_msgs::msg::Vector3& linear_vel_local_msg=odom_measurement->twist.twist.linear;
                    Eigen::Vector3d linear_vel_local_eigen(linear_vel_local_msg.x, linear_vel_local_msg.y, linear_vel_local_msg.z);
                    Eigen::Vector3d linear_vel_global_eigen = pose_global_eigen.rotation()*linear_vel_local_eigen;
                    req->state.twist.twist.twist.linear.x=linear_vel_global_eigen.x();
                    req->state.twist.twist.twist.linear.y=linear_vel_global_eigen.y();
                    req->state.twist.twist.twist.linear.z=linear_vel_global_eigen.z();
                    req->state.twist.twist.covariance[0]=req->state.twist.twist.covariance[7]=req->state.twist.twist.covariance[14]=
                        req->state.twist.twist.covariance[21]=req->state.twist.twist.covariance[28]=req->state.twist.twist.covariance[35]=1.0;
                    req->state.twist.header=odom_measurement->header;

                    const geometry_msgs::msg::Vector3& linear_accel_local_msg=imu_measurement->linear_acceleration;
                    Eigen::Vector3d linear_accel_local_eigen(linear_accel_local_msg.x, linear_accel_local_msg.y, linear_accel_local_msg.z);
                    Eigen::Vector3d linear_accel_global_eigen = pose_global_eigen.rotation()*linear_accel_local_eigen;

                    req->state.accel.accel.accel.linear.x=linear_accel_global_eigen.x();
                    req->state.accel.accel.accel.linear.y=linear_accel_global_eigen.y();
                    req->state.accel.accel.accel.linear.z=linear_accel_global_eigen.z();
                    req->state.accel.accel.covariance[0]=req->state.accel.accel.covariance[7]=req->state.accel.accel.covariance[14]=
                        req->state.accel.accel.covariance[21]=req->state.accel.accel.covariance[28]=req->state.accel.accel.covariance[35]=1.0;
                    req->state.accel.header=odom_measurement->header;

                    m_state_setter_->async_send_request(req, std::bind(&EkfMonitor::handle_setstate_response, this, std::placeholders::_1));
                    initialized_=true;
                }
            }
            std::shared_ptr<message_filters::Cache<nav_msgs::msg::Odometry>> m_odom_measurement_cache_;
            std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> m_odom_measurement_subscription_;


            std::shared_ptr<message_filters::Cache<sensor_msgs::msg::Imu>> m_imu_measurement_cache_;
            std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> m_imu_measurement_subscription_;

            std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>> m_sync_;   

            rclcpp::Client<robot_localization::srv::SetState>::SharedPtr m_state_setter_;

            bool initialized_;

    };
    
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::EkfMonitor)
