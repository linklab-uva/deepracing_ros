#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <Eigen/Geometry>
#include <tf2/buffer_core.h>
#include <functional>
#include <deepracing_msgs/msg/timestamped_packet_motion_data.hpp>
#include <deepracing_msgs/msg/timestamped_packet_session_data.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <boost/math/constants/constants.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "deepracing_ros/utils/f1_msg_utils.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random_numbers/random_numbers.h>
#include <json/json.h>
#include <deepracing_ros/utils/f1_msg_utils.h>
#include <deepracing_ros/utils/file_utils.h>
#include <fstream>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>


class NodeWrapperTfUpdater_ 
{

  public:
    NodeWrapperTfUpdater_( const rclcpp::NodeOptions & options = 
      rclcpp::NodeOptions()
      )
    {
     current_track_id = -1;
     node = rclcpp::Node::make_shared("f1_tf_updater");//,"",options);
     m_tf_from_odom_ = node->declare_parameter<bool>("tf_from_odom", false);
     carname = node->declare_parameter<std::string>("carname", "");
     statictfbroadcaster.reset(new tf2_ros::StaticTransformBroadcaster(node));
     tfbroadcaster.reset(new tf2_ros::TransformBroadcaster(node));
     mapToTrack.header.frame_id = "map";
     mapToTrack.child_frame_id = deepracing_ros::F1MsgUtils::world_coordinate_name;
     std::filesystem::path covariance_file_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("deepracing_launch")) / std::filesystem::path("data") / std::filesystem::path("covariances.json");

     Json::Value root;
     Json::CharReaderBuilder builder;
     builder["collectComments"] = true;
     JSONCPP_STRING errs;
     std::ifstream ifs;
     std::ifstream covariance_file;
     covariance_file.open(covariance_file_path.string().c_str());
     bool success = parseFromStream(builder, covariance_file, &root, &errs);
     covariance_file.close();
     if(!success)
     {
       RCLCPP_ERROR(node->get_logger(), "Unable to parse json file at %s. Error message: %s", covariance_file_path.string().c_str(), errs.c_str());
       return;
     }
     Json::Value position_cov_json = root["position_cov"];
     Json::Value linear_vel_cov_json = root["linear_vel_cov"];
     Json::Value angular_vel_cov_json = root["angular_vel_cov"];
     Json::Value linear_accel_cov_json = root["linear_accel_cov"];
     Json::Value euler_angles_cov_json = root["euler_angles_cov"];
     std::array<double, 9> position_cov, linear_vel_cov;
     for (unsigned int i = 0; i < imu_msg_.angular_velocity_covariance.size(); i++)
     {
       imu_msg_.angular_velocity_covariance[i] = angular_vel_cov_json[i].asDouble();
       imu_msg_.orientation_covariance[i] = euler_angles_cov_json[i].asDouble();
       imu_msg_.linear_acceleration_covariance[i] = linear_accel_cov_json[i].asDouble();
       position_cov[i] = position_cov_json[i].asDouble();
       linear_vel_cov[i] = linear_vel_cov_json[i].asDouble();
     }
     Eigen::MatrixXd pose_cov_matrix(6,6);
     pose_cov_matrix.setZero();
     pose_cov_matrix.block<3,3>(0,0) = Eigen::Matrix3d(position_cov.data());
     pose_cov_matrix.block<3,3>(3,3) = Eigen::Matrix3d(imu_msg_.orientation_covariance.data());
     Eigen::MatrixXd vel_cov_matrix(6,6);
     vel_cov_matrix.setZero();
     vel_cov_matrix.block<3,3>(0,0) = Eigen::Matrix3d(linear_vel_cov.data());
     vel_cov_matrix.block<3,3>(3,3) = Eigen::Matrix3d(imu_msg_.angular_velocity_covariance.data());
     std::copy_n(pose_cov_matrix.data(), odom_msg_.pose.covariance.size(), odom_msg_.pose.covariance.begin());
     std::copy_n(vel_cov_matrix.data(), odom_msg_.twist.covariance.size(), odom_msg_.twist.covariance.begin());
    



     tf2::Quaternion quat;
     quat.setRPY( boost::math::constants::half_pi<double>(), 0.0, 0.0 );
     tf2::Transform t(quat, tf2::Vector3(0.0,0.0,0.0));
     mapToTrack.transform = tf2::toMsg(t);
     node->declare_parameter< std::vector<double> >("centroid_to_base_translation");
     
     m_extra_position_noise = node->declare_parameter< double >("extra_position_noise", 0.0);
     m_extra_rot_noise = node->declare_parameter< double >("extra_rot_noise", 0.0);
     m_extra_vel_noise = node->declare_parameter< double >("extra_vel_noise", 0.0);
     m_extra_angvel_noise = node->declare_parameter< double >("extra_angvel_noise", 0.0);
     std::vector<double> car_to_base_translation;
     node->get_parameter<std::vector<double>>("centroid_to_base_translation", car_to_base_translation);
     if (car_to_base_translation.size()!=3)
     {
       throw rclcpp::exceptions::InvalidParameterValueException("\"centroid_to_base_translation\" must have exactly 3 values");
     }
     carToBaseLink.header.frame_id = deepracing_ros::F1MsgUtils::car_coordinate_name+"_"+carname;
     carToBaseLink.child_frame_id = "base_link_"+carname;
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

     this->session_listener = this->node->create_subscription<deepracing_msgs::msg::TimestampedPacketSessionData>("session_data", 1, std::bind(&NodeWrapperTfUpdater_::sessionCallback, this, std::placeholders::_1));
     this->listener = this->node->create_subscription<deepracing_msgs::msg::TimestampedPacketMotionData>("motion_data", 1, std::bind(&NodeWrapperTfUpdater_::packetCallback, this, std::placeholders::_1));
     this->twist_publisher = this->node->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 1);
     this->twist_local_publisher = this->node->create_publisher<geometry_msgs::msg::TwistStamped>("velocity_local", 1);
     this->odom_publisher = this->node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
     this->accel_publisher = this->node->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
     
     if( m_tf_from_odom_ )
     {
       odom_listener = this->node->create_subscription<nav_msgs::msg::Odometry>("odom/filtered", 1, std::bind(&NodeWrapperTfUpdater_::odomCallback, this, std::placeholders::_1));
     }
     
    }  
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_listener;
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketMotionData>::SharedPtr listener;
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketSessionData>::SharedPtr session_listener;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_local_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accel_publisher;
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
    random_numbers::RandomNumberGenerator m_rng_;
    bool m_tf_from_odom_;
    nav_msgs::msg::Odometry odom_msg_;
    sensor_msgs::msg::Imu imu_msg_;
    
    double m_extra_position_noise;
    double m_extra_rot_noise;
    double m_extra_vel_noise;   
    double m_extra_angvel_noise; 
    int8_t current_track_id;
    std::string carname;
    void sessionCallback(const deepracing_msgs::msg::TimestampedPacketSessionData::SharedPtr session_msg)
    {
      if((session_msg->udp_packet.track_id>=0) && (session_msg->udp_packet.track_id!=current_track_id))
      {
        
        std::vector<std::string> search_dirs = deepracing_ros::FileUtils::split(std::string(std::getenv("F1_TRACK_DIRS")));
        try
        {
          std::filesystem::path f1_datalogger_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("f1_datalogger"));
          search_dirs.push_back((f1_datalogger_path / std::filesystem::path("f1_tracks")).string());
          search_dirs.push_back((f1_datalogger_path / std::filesystem::path("f1_tracks") / std::filesystem::path("minimumcurvature")).string());
        }
        catch(const std::exception& e)
        {
          
        }
        
        std::array<std::string, 25> tracknames = deepracing_ros::F1MsgUtils::track_names();
        std::string trackname = tracknames[session_msg->udp_packet.track_id];
        std::string starting_pose_filepath = deepracing_ros::FileUtils::findFile(trackname+"_startingpose.json", search_dirs);
        if (starting_pose_filepath.empty())
        {
          RCLCPP_ERROR(node->get_logger(), "Could not find starting pose file for track: %s", trackname.c_str());
          return;
        }
        Json::Value root;
        Json::CharReaderBuilder builder;
        builder["collectComments"] = true;
        JSONCPP_STRING errs;
        std::ifstream ifs;
        ifs.open(starting_pose_filepath);
        bool success = parseFromStream(builder, ifs, &root, &errs);
        ifs.close();
        if(!success)
        {
          RCLCPP_ERROR(node->get_logger(), "Unable to parse json file at %s. Error message: %s", starting_pose_filepath.c_str(), errs.c_str());
          return;
        }
        Json::Value position_dict = root["position"];
        Json::Value quaternion_dict = root["quaternion"];

        tf2::Vector3 eigen_position(position_dict["x"].asDouble(), position_dict["y"].asDouble(), position_dict["z"].asDouble());
        tf2::Quaternion eigen_quaternion(quaternion_dict["x"].asDouble(), quaternion_dict["y"].asDouble(), quaternion_dict["z"].asDouble(), quaternion_dict["w"].asDouble());
        
        mapToTrack.transform = tf2::toMsg(tf2::Transform(eigen_quaternion, eigen_position).inverse());
        mapToTrackEigen = tf2::transformToEigen(mapToTrack);

        current_track_id = session_msg->udp_packet.track_id;
      }
      publishStatic();
    }  
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
      transformMsg.set__child_frame_id(deepracing_ros::F1MsgUtils::car_coordinate_name+"_"+carname);
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

      Eigen::Vector3d centroidVelEigenGlobal(velocityROS.vector.x, velocityROS.vector.y, velocityROS.vector.z);
      Eigen::Vector3d centroidVelEigenLocal = trackToCarEigen.rotation().inverse()*centroidVelEigenGlobal;    

      // Eigen::Vector3d centroidVelEigenGlobal(velocityROS.vector.x, velocityROS.vector.y, velocityROS.vector.z);
      // Eigen::Vector3d centroidVelEigenLocal(motion_data_packet->udp_packet.local_velocity.z, motion_data_packet->udp_packet.local_velocity.x, motion_data_packet->udp_packet.local_velocity.y); 
      // centroidVelEigenLocal+=m_extra_vel_noise*Eigen::Vector3d(m_rng_.gaussian01(), m_rng_.gaussian01(), m_rng_.gaussian01());


      Eigen::Vector3d centroidAngVelTrack(motion_data_packet->udp_packet.angular_velocity.x, motion_data_packet->udp_packet.angular_velocity.y, motion_data_packet->udp_packet.angular_velocity.z);
      Eigen::Vector3d centroidAngVel = trackToCarEigen.rotation().inverse()*centroidAngVelTrack;
      
      Eigen::Vector3d centroidAngAccelTrack(motion_data_packet->udp_packet.angular_acceleration.x, motion_data_packet->udp_packet.angular_acceleration.y, motion_data_packet->udp_packet.angular_acceleration.z);
      Eigen::Vector3d centroidAngAccel = trackToCarEigen.rotation().inverse()*centroidAngAccelTrack;

      Eigen::Vector3d centroidLinearAccel=9.81*Eigen::Vector3d(motion_data.g_force_longitudinal, motion_data.g_force_lateral, motion_data.g_force_vertical);



    
      odom_msg_.header.set__frame_id("map");
      odom_msg_.header.set__stamp(transformMsg.header.stamp);
      odom_msg_.set__child_frame_id(transformMsg.child_frame_id);
      odom_msg_.pose.pose = tf2::toMsg(mapToCarEigen);

      // odom.pose.covariance[0]=odom.pose.covariance[7]=odom.pose.covariance[14] = 0.0025;// + extra_position_variance;
      // odom.pose.covariance[21]=odom.pose.covariance[28]=odom.pose.covariance[35] = 1.0E-4;// + extra_rot_variance;
      odom_msg_.twist.twist.linear.x=centroidVelEigenLocal.x();// + m_extra_vel_noise*m_rng_.gaussian01();
      odom_msg_.twist.twist.linear.y=centroidVelEigenLocal.y();// + m_extra_vel_noise*m_rng_.gaussian01();
      odom_msg_.twist.twist.linear.z=centroidVelEigenLocal.z();// + m_extra_vel_noise*m_rng_.gaussian01();
      odom_msg_.twist.twist.angular.x=centroidAngVel.x();// + m_extra_angvel_noise*m_rng_.gaussian01();
      odom_msg_.twist.twist.angular.y=centroidAngVel.y();// + m_extra_angvel_noise*m_rng_.gaussian01();
      odom_msg_.twist.twist.angular.z=centroidAngVel.z();// + m_extra_angvel_noise*m_rng_.gaussian01();
      // odom.twist.covariance[0]=odom.twist.covariance[7]=odom.twist.covariance[14]=0.000225;//+extra_vel_variance;
      // odom.twist.covariance[21]=odom.twist.covariance[28]=odom.twist.covariance[35]=2.0E-5;//+extra_angvel_variance;

      
      imu_msg_.header.set__stamp(transformMsg.header.stamp);
      imu_msg_.header.set__frame_id(transformMsg.child_frame_id);
      imu_msg_.set__orientation(odom_msg_.pose.pose.orientation);
      // imu_msg.orientation_covariance[0]=imu_msg.orientation_covariance[4]=imu_msg.orientation_covariance[8]=odom.pose.covariance[21];
      imu_msg_.angular_velocity.set__x(centroidAngVel.x());
      imu_msg_.angular_velocity.set__y(centroidAngVel.y());
      imu_msg_.angular_velocity.set__z(centroidAngVel.z());
      // imu_msg.angular_velocity_covariance[0]=imu_msg.angular_velocity_covariance[4]=imu_msg.angular_velocity_covariance[8]=odom.twist.covariance[21];
      imu_msg_.linear_acceleration.set__x(centroidLinearAccel.x());
      imu_msg_.linear_acceleration.set__y(centroidLinearAccel.y());
      imu_msg_.linear_acceleration.set__z(centroidLinearAccel.z());
      // imu_msg.linear_acceleration_covariance[0]=imu_msg.linear_acceleration_covariance[4]=0.00225;
      // imu_msg.linear_acceleration_covariance[8]=0.005;

      carToBaseLink.header.set__stamp(motion_data.world_position.header.stamp);
      mapToTrack.header.set__stamp(motion_data.world_position.header.stamp);

      if (!m_tf_from_odom_)
      {
        this->tfbroadcaster->sendTransform(transformMsg);
      }
      this->odom_publisher->publish(odom_msg_);
      this->accel_publisher->publish(imu_msg_);
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