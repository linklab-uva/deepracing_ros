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


class NodeWrapperTfUpdater_ 
{

  public:
    NodeWrapperTfUpdater_( const rclcpp::NodeOptions & options = 
      rclcpp::NodeOptions()
      )
    {
     node = rclcpp::Node::make_shared("f1_tf_updater");//,"",options);
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
     
     
    }  
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketMotionData>::SharedPtr listener;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_local_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;

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
      Eigen::Isometry3d trackToBLEigen = trackToCarEigen * carToBaseLinkEigen;

      Eigen::Vector3d trackToBL_translation(trackToBLEigen.translation());
      Eigen::Quaterniond trackToBL_quaternion(trackToBLEigen.rotation());


      geometry_msgs::msg::PoseStamped pose;
      pose.set__header(transformMsg.header);
      pose.pose.position.set__x(trackToBL_translation.x());
      pose.pose.position.set__y(trackToBL_translation.y());
      pose.pose.position.set__z(trackToBL_translation.z());
      pose.pose.orientation.set__x(trackToBL_quaternion.x());
      pose.pose.orientation.set__y(trackToBL_quaternion.y());
      pose.pose.orientation.set__z(trackToBL_quaternion.z());
      pose.pose.orientation.set__w(trackToBL_quaternion.w());

      
      //Need to come back to this.  It's not clear exactly what angular velocity is coming off the UDP stream. Will follow up with CodeMasters on that.
      // Eigen::Vector3d 
      Eigen::Vector3d centroidVelEigenLocal(motion_data_packet->udp_packet.local_velocity.z, motion_data_packet->udp_packet.local_velocity.x, motion_data_packet->udp_packet.local_velocity.y);
      
      Eigen::Vector3d centroidAngVelEigenGlobal(motion_data_packet->udp_packet.angular_velocity.x, motion_data_packet->udp_packet.angular_velocity.y, motion_data_packet->udp_packet.angular_velocity.z);
      Eigen::Vector3d centroidAngVelEigenLocal = trackToBLEigen.rotation().inverse()*centroidAngVelEigenGlobal;
      // if (centroidAngVelEigenLocal.z()>=0)
      // {
      //   centroidAngVelEigenLocal = Eigen::Vector3d(0.0,0.0, centroidAngVelEigenLocal.norm());
      // }
      // else
      // {
      //   centroidAngVelEigenLocal = Eigen::Vector3d(0.0,0.0, -centroidAngVelEigenLocal.norm());
      // }
     
      Eigen::Vector3d baseLinkToCarVec(baseLinkToCarEigen.translation());
      Eigen::Vector3d localVelEigen = centroidVelEigenLocal - centroidAngVelEigenLocal.cross(baseLinkToCarVec);
      localVelEigen = Eigen::Vector3d(localVelEigen.norm(), 0.0, 0.0);

      geometry_msgs::msg::TwistStamped car_velocity;
      car_velocity.set__header(transformMsg.header);
      car_velocity.twist.set__linear(motion_data.world_velocity.vector);
      car_velocity.twist.set__angular(motion_data_packet->udp_packet.angular_velocity);

      geometry_msgs::msg::TwistStamped car_velocity_local;
      car_velocity_local.header.set__stamp(transformMsg.header.stamp);
      car_velocity_local.header.set__frame_id("base_link");
      car_velocity_local.twist.linear.set__x(localVelEigen.x());
      car_velocity_local.twist.linear.set__y(localVelEigen.y());
      car_velocity_local.twist.linear.set__z(localVelEigen.z());
      car_velocity_local.twist.angular.set__x(centroidAngVelEigenLocal.x());
      car_velocity_local.twist.angular.set__y(centroidAngVelEigenLocal.y());
      car_velocity_local.twist.angular.set__z(centroidAngVelEigenLocal.z());

      Eigen::Isometry3d mapToBLEigen = mapToTrackEigen * trackToBLEigen;
      nav_msgs::msg::Odometry odom;
      odom.header.set__stamp(transformMsg.header.stamp);
      odom.header.set__frame_id(mapToTrack.header.frame_id);
      odom.set__child_frame_id(car_velocity_local.header.frame_id);
      Eigen::Vector3d mapToBLTranslation(mapToBLEigen.translation());
      odom.pose.pose.position.set__x(mapToBLTranslation.x());
      odom.pose.pose.position.set__y(mapToBLTranslation.y());
      odom.pose.pose.position.set__z(mapToBLTranslation.z());
      Eigen::Quaterniond mapToBLQuat(mapToBLEigen.rotation());
      odom.pose.pose.orientation.set__x(mapToBLQuat.x());
      odom.pose.pose.orientation.set__y(mapToBLQuat.y());
      odom.pose.pose.orientation.set__z(mapToBLQuat.z());
      odom.pose.pose.orientation.set__w(mapToBLQuat.w());


      odom.pose.covariance[0]=odom.pose.covariance[7]=odom.pose.covariance[14]=0.0025;
      odom.pose.covariance[21]=odom.pose.covariance[28]=odom.pose.covariance[35]=1.0E-4;

      odom.twist.set__twist(car_velocity_local.twist);
      odom.twist.covariance[0]=odom.pose.covariance[7]=odom.pose.covariance[14]=0.000225;
      odom.twist.covariance[21]=odom.pose.covariance[28]=odom.pose.covariance[35]=2.0E-5;

      carToBaseLink.header.set__stamp(motion_data.world_position.header.stamp);
      mapToTrack.header.set__stamp(motion_data.world_position.header.stamp);

      this->tfbroadcaster->sendTransform(transformMsg);
      this->twist_publisher->publish(car_velocity);
      this->twist_local_publisher->publish(car_velocity_local);
      this->pose_publisher->publish(pose);
      this->odom_publisher->publish(odom);
      publishStatic();
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