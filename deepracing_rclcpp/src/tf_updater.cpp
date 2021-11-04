#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <Eigen/Geometry>
#include <tf2/buffer_core.h>
#include <functional>
#include <deepracing_msgs/msg/timestamped_packet_motion_data.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <boost/math/constants/constants.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "deepracing_ros/utils/f1_msg_utils.h"
#include <tf2_eigen/tf2_eigen.h>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <random_numbers/random_numbers.h>


class NodeWrapperTfUpdater_ 
{

  public:
    NodeWrapperTfUpdater_( const rclcpp::NodeOptions & options = 
      rclcpp::NodeOptions()
      )
    {
     node = rclcpp::Node::make_shared("f1_tf_updater");//,"",options);
     m_tf_from_odom_ = node->declare_parameter<bool>("tf_from_odom", false);
     statictfbroadcaster.reset(new tf2_ros::StaticTransformBroadcaster(node));
     tfbroadcaster.reset(new tf2_ros::TransformBroadcaster(node));
     mapToTrack.header.frame_id = "map";
     mapToTrack.child_frame_id = deepracing_ros::F1MsgUtils::world_coordinate_name;

     carToBaseLink.header.frame_id = deepracing_ros::F1MsgUtils::car_coordinate_name;
     carToBaseLink.child_frame_id = "base_link";

     tf2::Quaternion quat;
     quat.setRPY( boost::math::constants::half_pi<double>(), 0.0, 0.0 );
     mapToTrack.transform.translation.x = 0.0;
     mapToTrack.transform.translation.y = 0.0;
     mapToTrack.transform.translation.z = 0.0;
     mapToTrack.transform.rotation.x = quat.x();
     mapToTrack.transform.rotation.y = quat.y();
     mapToTrack.transform.rotation.z = quat.z();
     mapToTrack.transform.rotation.w = quat.w();

     std::vector<double> car_to_base_translation = node->declare_parameter< std::vector<double> >("centroid_to_base_translation", std::vector<double>{-1.85, 0.0, 0.0});
     if (car_to_base_translation.size()!=3)
     {
       throw rclcpp::exceptions::InvalidParameterValueException("\"centroid_to_base_translation\" must have exactly 3 values");
     }
     carToBaseLink.transform.translation.x = car_to_base_translation.at(0);
     carToBaseLink.transform.translation.y = car_to_base_translation.at(1);
     carToBaseLink.transform.translation.z = car_to_base_translation.at(2);
     carToBaseLink.transform.rotation.x = 0.0;
     carToBaseLink.transform.rotation.y = 0.0;
     carToBaseLink.transform.rotation.z = 0.0;
     carToBaseLink.transform.rotation.w = 1.0;
    
     carToBaseLinkEigen = tf2::transformToEigen(carToBaseLink);
     mapToTrackEigen = tf2::transformToEigen(mapToTrack);
     baseLinkToCarEigen = carToBaseLinkEigen.inverse();


     this->listener = this->node->create_subscription<deepracing_msgs::msg::TimestampedPacketMotionData>("motion_data", 1, std::bind(&NodeWrapperTfUpdater_::packetCallback, this, std::placeholders::_1));
     this->pose_publisher = this->node->create_publisher<geometry_msgs::msg::PoseStamped>("/ego_vehicle/pose", 1);
     this->twist_publisher = this->node->create_publisher<geometry_msgs::msg::TwistStamped>("/ego_vehicle/velocity", 1);
     this->twist_local_publisher = this->node->create_publisher<geometry_msgs::msg::TwistStamped>("/ego_vehicle/velocity_local", 1);
     this->odom_publisher = this->node->create_publisher<nav_msgs::msg::Odometry>("/ego_vehicle/odom", 1);
     this->autoware_state_publisher = this->node->create_publisher<autoware_auto_msgs::msg::VehicleKinematicState>("/ego_vehicle/state", 1);
     
     if( m_tf_from_odom_ )
     {
       odom_listener = this->node->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 1, std::bind(&NodeWrapperTfUpdater_::odomCallback, this, std::placeholders::_1));
     }
     
    }  
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_listener;
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketMotionData>::SharedPtr listener;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_local_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    rclcpp::Publisher<autoware_auto_msgs::msg::VehicleKinematicState>::SharedPtr autoware_state_publisher;

    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfbroadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> statictfbroadcaster;
    geometry_msgs::msg::TransformStamped mapToTrack, carToBaseLink;    
    Eigen::Isometry3d mapToTrackEigen, carToBaseLinkEigen, baseLinkToCarEigen;
    inline void publishStatic()
    {
      this->statictfbroadcaster->sendTransform(carToBaseLink);
      this->statictfbroadcaster->sendTransform(mapToTrack);
    }
  private:
    random_numbers::RandomNumberGenerator m_rng_;
    bool m_tf_from_odom_;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
      geometry_msgs::msg::TransformStamped transformMsg;
      transformMsg.set__header(odom_msg->header);
      transformMsg.set__child_frame_id(odom_msg->child_frame_id);
      transformMsg.transform.translation.set__x(odom_msg->pose.pose.position.x);
      transformMsg.transform.translation.set__y(odom_msg->pose.pose.position.y);
      transformMsg.transform.translation.set__z(odom_msg->pose.pose.position.z);
      transformMsg.transform.set__rotation(odom_msg->pose.pose.orientation);
      this->tfbroadcaster->sendTransform(transformMsg);
    }
    void packetCallback(const deepracing_msgs::msg::TimestampedPacketMotionData::SharedPtr motion_data_packet)
    {
     // std::cout << "Got some data" << std::endl;
     // RCLCPP_INFO(node->get_logger(), "Got some data");
      uint8_t idx;
      if( motion_data_packet->udp_packet.header.player_car_index<20 )
      {
        idx = motion_data_packet->udp_packet.header.player_car_index;
      }
      else
      {
        idx = 0;
      }

      const deepracing_msgs::msg::CarMotionData &motion_data = motion_data_packet->udp_packet.car_motion_data[idx];
      const geometry_msgs::msg::Vector3Stamped &velocityROS = motion_data.world_velocity;
      const geometry_msgs::msg::Vector3Stamped &upROS = motion_data.world_up_dir;
      const geometry_msgs::msg::Vector3Stamped &forwardROS = motion_data.world_forward_dir;
      const geometry_msgs::msg::Vector3Stamped &rightROS = motion_data.world_right_dir;

      Eigen::Vector3d leftEigen(-rightROS.vector.x, -rightROS.vector.y, -rightROS.vector.z);
      leftEigen.normalize();
      Eigen::Vector3d forwardEigen(forwardROS.vector.x, forwardROS.vector.y, forwardROS.vector.z);
      forwardEigen.normalize();
      Eigen::Vector3d upEigen = forwardEigen.cross(leftEigen);
      upEigen.normalize();
      Eigen::Matrix3d rotmat;
      rotmat.col(0) = forwardEigen;
      rotmat.col(1) = leftEigen;
      rotmat.col(2) = upEigen;
      Eigen::Quaterniond rotationEigen(rotmat);
      rotationEigen.normalize();
      geometry_msgs::msg::TransformStamped transformMsg;
      transformMsg.set__header(motion_data_packet->header);
      transformMsg.set__child_frame_id(deepracing_ros::F1MsgUtils::car_coordinate_name);
      transformMsg.transform.rotation.x = rotationEigen.x();
      transformMsg.transform.rotation.y = rotationEigen.y();
      transformMsg.transform.rotation.z = rotationEigen.z();
      transformMsg.transform.rotation.w = rotationEigen.w();

      const geometry_msgs::msg::PointStamped& positionROS = motion_data.world_position;
      transformMsg.transform.translation.set__x(positionROS.point.x);
      transformMsg.transform.translation.set__y(positionROS.point.y);
      transformMsg.transform.translation.set__z(positionROS.point.z);

      
      Eigen::Isometry3d trackToCarEigen = tf2::transformToEigen(transformMsg);
      Eigen::Isometry3d mapToCarEigen = mapToTrackEigen * trackToCarEigen;
      Eigen::Isometry3d mapToBLEigen = mapToCarEigen * carToBaseLinkEigen;

      Eigen::Vector3d mapToBL_translation(mapToBLEigen.translation());
      Eigen::Quaterniond mapToBL_quaternion(mapToBLEigen.rotation());

      geometry_msgs::msg::PoseStamped pose;
      pose.header.set__frame_id("map");
      pose.header.set__stamp(transformMsg.header.stamp);
      pose.pose.position.set__x(mapToBL_translation.x());
      pose.pose.position.set__y(mapToBL_translation.y());
      pose.pose.position.set__z(mapToBL_translation.z());

      pose.pose.orientation.set__x(mapToBL_quaternion.x());
      pose.pose.orientation.set__y(mapToBL_quaternion.y());
      pose.pose.orientation.set__z(mapToBL_quaternion.z());
      pose.pose.orientation.set__w(mapToBL_quaternion.w());

      Eigen::Vector3d centroidVelEigenGlobal(velocityROS.vector.z, velocityROS.vector.x, velocityROS.vector.y);
      Eigen::Vector3d centroidVelEigenLocal(velocityROS.vector.z, motion_data_packet->udp_packet.local_velocity.x, motion_data_packet->udp_packet.local_velocity.y);
      
     
      geometry_msgs::msg::TwistStamped car_velocity_local;
      car_velocity_local.header.set__stamp(transformMsg.header.stamp);
      car_velocity_local.header.set__frame_id(transformMsg.child_frame_id);
      car_velocity_local.twist.linear.set__x(motion_data_packet->udp_packet.local_velocity.z);
      car_velocity_local.twist.linear.set__y(motion_data_packet->udp_packet.local_velocity.x);
      car_velocity_local.twist.linear.set__z(motion_data_packet->udp_packet.local_velocity.y);
      car_velocity_local.twist.angular.set__x(motion_data_packet->udp_packet.angular_velocity.z);
      car_velocity_local.twist.angular.set__y(motion_data_packet->udp_packet.angular_velocity.x);
      car_velocity_local.twist.angular.set__z(motion_data_packet->udp_packet.angular_velocity.y);

      nav_msgs::msg::Odometry odom;
      odom.set__header(pose.header);
      odom.set__child_frame_id(car_velocity_local.header.frame_id);
      odom.pose.set__pose(tf2::toMsg(mapToCarEigen));

      double extra_position_noise=0.15;
      double extra_rot_noise=0.0001;
      double extra_vel_noise=0.05;      
      Eigen::Quaterniond random_quat = Eigen::Quaterniond::UnitRandom();
      odom.pose.pose.position.x+=extra_position_noise*m_rng_.gaussian01();
      odom.pose.pose.position.y+=extra_position_noise*m_rng_.gaussian01();
      odom.pose.pose.position.z+=extra_position_noise*m_rng_.gaussian01();
      odom.pose.pose.orientation.x+=extra_rot_noise*random_quat.x();
      odom.pose.pose.orientation.y+=extra_rot_noise*random_quat.y();
      odom.pose.pose.orientation.z+=extra_rot_noise*random_quat.z();
      odom.pose.pose.orientation.w+=extra_rot_noise*random_quat.w();

      odom.pose.covariance[0]=odom.pose.covariance[7]=odom.pose.covariance[14]=0.0025 + extra_position_noise;
      odom.pose.covariance[21]=odom.pose.covariance[28]=odom.pose.covariance[35]=1.0E-4 + extra_rot_noise;

      odom.twist.set__twist(car_velocity_local.twist);
      odom.twist.twist.linear.x+=extra_vel_noise*m_rng_.gaussian01();
      odom.twist.twist.linear.y+=extra_vel_noise*m_rng_.gaussian01();
      odom.twist.twist.linear.z+=extra_vel_noise*m_rng_.gaussian01();
      odom.twist.covariance[0]=odom.pose.covariance[7]=odom.pose.covariance[14]=0.000225+extra_vel_noise;
      odom.twist.covariance[21]=odom.pose.covariance[28]=odom.pose.covariance[35]=2.0E-5;

      carToBaseLink.header.set__stamp(motion_data.world_position.header.stamp);
      mapToTrack.header.set__stamp(motion_data.world_position.header.stamp);

      autoware_auto_msgs::msg::VehicleKinematicState state;
      state.set__header(pose.header);
      state.state.x=pose.pose.position.x;
      state.state.y=pose.pose.position.y;
      state.state.z=pose.pose.position.z;
      state.state.longitudinal_velocity_mps=odom.twist.twist.linear.x;
      state.state.front_wheel_angle_rad=-motion_data_packet->udp_packet.front_wheels_angle;
      state.state.rear_wheel_angle_rad=0.0;
      mapToBL_quaternion.x()=0.0;
      mapToBL_quaternion.y()=0.0;
      mapToBL_quaternion.normalize();
      state.state.heading.imag=mapToBL_quaternion.z();
      state.state.heading.real=mapToBL_quaternion.w();

      publishStatic();
      if (!m_tf_from_odom_)
      {
        this->tfbroadcaster->sendTransform(transformMsg);
      }
      this->odom_publisher->publish(odom);
      this->autoware_state_publisher->publish(state);
    }
};
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  NodeWrapperTfUpdater_ nw;
  std::shared_ptr<rclcpp::Node> node = nw.node;
  RCLCPP_INFO(node->get_logger(), "Updating TF data");
  int num_threads_ = node->declare_parameter<int>("num_threads", 0);
  size_t num_threads;
  if (num_threads_<=0){
    num_threads = 0;
    RCLCPP_INFO(node->get_logger(), "Spinning with the number of detected CPU cores");
  }
  else{
    num_threads = (size_t)num_threads_;
    RCLCPP_INFO(node->get_logger(), "Spinning with %zu threads", num_threads);
  }
 // node->create_wall_timer(std::chrono::seconds(1), std::bind(&NodeWrapperTfUpdater_::publishStatic, &nw), node->create_callback_group(rclcpp::CallbackGroupType::Reentrant));

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), num_threads);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}