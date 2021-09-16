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
#include <tf2/convert.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "deepracing_ros/utils/f1_msg_utils.h"


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
     mapToTrack.header.stamp = this->node->now();
     mapToTrack.child_frame_id = deepracing_ros::F1MsgUtils::world_coordinate_name;

     carToBaseLink.header.frame_id = deepracing_ros::F1MsgUtils::car_coordinate_name;
     carToBaseLink.header.stamp = this->node->now();
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

     node->declare_parameter("car_to_base_translation", std::vector<double>{0.0, 0.0, 0.0});
     node->declare_parameter("car_to_base_quaternion", std::vector<double>{0.0, 0.0, 0.0, 1.0});
     std::vector<double> car_to_base_translation = node->get_parameter("car_to_base_translation").as_double_array();
     std::vector<double> car_to_base_quaternion = node->get_parameter("car_to_base_quaternion").as_double_array();
     carToBaseLink.transform.translation.x = car_to_base_translation.at(0);
     carToBaseLink.transform.translation.y = car_to_base_translation.at(1);
     carToBaseLink.transform.translation.z = car_to_base_translation.at(2);
     carToBaseLink.transform.rotation.x = car_to_base_quaternion.at(0);
     carToBaseLink.transform.rotation.y = car_to_base_quaternion.at(1);
     carToBaseLink.transform.rotation.z = car_to_base_quaternion.at(2);
     carToBaseLink.transform.rotation.w = car_to_base_quaternion.at(3);


     this->statictfbroadcaster->sendTransform(mapToTrack);
     this->listener = this->node->create_subscription<deepracing_msgs::msg::TimestampedPacketMotionData>("motion_data", 1, std::bind(&NodeWrapperTfUpdater_::packetCallback, this, std::placeholders::_1));
     this->pose_publisher = this->node->create_publisher<geometry_msgs::msg::PoseStamped>("/ego_vehicle/pose", 1);
     this->twist_publisher = this->node->create_publisher<geometry_msgs::msg::TwistStamped>("/ego_vehicle/velocity", 1);
     
     
    }  
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketMotionData>::SharedPtr listener;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;

    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfbroadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> statictfbroadcaster;
    geometry_msgs::msg::TransformStamped mapToTrack, carToBaseLink;    
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
      // rotmat.col(0) = leftEigen;
      // rotmat.col(1) = upEigen;
      // rotmat.col(2) = forwardEigen;
      rotmat.col(0) = forwardEigen;
      rotmat.col(1) = leftEigen;
      rotmat.col(2) = upEigen;
      Eigen::Quaterniond rotationEigen(rotmat);
      rotationEigen.normalize();
      geometry_msgs::msg::TransformStamped transformMsg;
      transformMsg.header.set__frame_id(deepracing_ros::F1MsgUtils::world_coordinate_name);
      transformMsg.header.set__stamp(motion_data.world_position.header.stamp);
      transformMsg.set__child_frame_id(deepracing_ros::F1MsgUtils::car_coordinate_name);
      transformMsg.transform.rotation.x = rotationEigen.x();
      transformMsg.transform.rotation.y = rotationEigen.y();
      transformMsg.transform.rotation.z = rotationEigen.z();
      transformMsg.transform.rotation.w = rotationEigen.w();

      const geometry_msgs::msg::PointStamped& positionROS = motion_data.world_position;
      transformMsg.transform.translation.set__x(positionROS.point.x);
      transformMsg.transform.translation.set__y(positionROS.point.y);
      transformMsg.transform.translation.set__z(positionROS.point.z);
      this->tfbroadcaster->sendTransform(transformMsg);
      this->statictfbroadcaster->sendTransform(mapToTrack);

      carToBaseLink.header.set__stamp(motion_data.world_position.header.stamp);
      this->statictfbroadcaster->sendTransform(carToBaseLink);
      

      geometry_msgs::msg::PoseStamped pose;
      pose.set__header(transformMsg.header);
      pose.pose.position.set__x(transformMsg.transform.translation.x);
      pose.pose.position.set__y(transformMsg.transform.translation.y);
      pose.pose.position.set__z(transformMsg.transform.translation.z);
      pose.pose.set__orientation(transformMsg.transform.rotation);

      this->pose_publisher->publish(pose);

      geometry_msgs::msg::TwistStamped car_velocity;
      car_velocity.set__header(transformMsg.header);
      car_velocity.twist.set__linear(motion_data.world_velocity.vector);
      car_velocity.twist.set__angular(motion_data_packet->udp_packet.angular_velocity);
      this->twist_publisher->publish(car_velocity);
      


    }
};
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  NodeWrapperTfUpdater_ nw;
  std::shared_ptr<rclcpp::Node> node = nw.node;
  RCLCPP_INFO(node->get_logger(), "Updating TF data");
  int num_threads_ = node->declare_parameter<int>("num_threads", 3);
  size_t num_threads;
  if (num_threads_<=0){
    num_threads = 0;
    RCLCPP_INFO(node->get_logger(), "Spinning with the number of detected CPU cores");
  }
  else{
    num_threads = (size_t)num_threads_;
    RCLCPP_INFO(node->get_logger(), "Spinning with %zu threads", num_threads);
  }
  

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), num_threads);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}