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
#include "deepracing_ros/utils/f1_msg_utils_2020.h"
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random_numbers/random_numbers.h>
#include <json/json.h>
#include <deepracing_ros/utils/file_utils.h>
#include <fstream>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace deepracing
{
namespace composable_nodes
{

class MeasurementPublisher 
{

  public:
    MeasurementPublisher( const rclcpp::NodeOptions & options = 
      rclcpp::NodeOptions()
      )
    {
     current_track_id = -1;
     node = rclcpp::Node::make_shared("f1_tf_updater");//,"",options);
     m_with_ekf_ = node->declare_parameter<bool>("with_ekf", false);
     carname = node->declare_parameter<std::string>("carname", "");
     statictfbroadcaster.reset(new tf2_ros::StaticTransformBroadcaster(node));
     mapToTrack.header.frame_id = "map";
     mapToTrack.child_frame_id = deepracing_ros::F1MsgUtils2020::world_coordinate_name;
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
          
     if( !m_with_ekf_ )
     {
        tfbroadcaster.reset(new tf2_ros::TransformBroadcaster(node));
        this->accel_publisher = this->node->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("accel/filtered", 1);
        this->odom_publisher = this->node->create_publisher<nav_msgs::msg::Odometry>("odom/filtered", 1);
     }
     else
     {
        this->odom_publisher = this->node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
     }  
     this->imu_publisher = this->node->create_publisher<sensor_msgs::msg::Imu>("imu", 1);


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
        imu_msg_.linear_acceleration_covariance[i] = accel_msg_.accel.covariance[i] = linear_accel_cov_json[i].asDouble();
        position_cov[i] = position_cov_json[i].asDouble();
        linear_vel_cov[i] = linear_vel_cov_json[i].asDouble();
     }
     Eigen::Matrix<double, 6, 6, Eigen::RowMajor> pose_cov_matrix;
     pose_cov_matrix.setZero();
     pose_cov_matrix.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(position_cov.data());
     pose_cov_matrix.block<3,3>(3,3) = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(imu_msg_.orientation_covariance.data());
     Eigen::Matrix<double, 6, 6, Eigen::RowMajor> vel_cov_matrix;
     vel_cov_matrix.setZero();
     vel_cov_matrix.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(linear_vel_cov.data());
     vel_cov_matrix.block<3,3>(3,3) = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(imu_msg_.angular_velocity_covariance.data()); 
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
     carToBaseLink.header.frame_id = deepracing_ros::F1MsgUtils2020::car_coordinate_name+"_"+carname;
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

     

     this->session_listener = this->node->create_subscription<deepracing_msgs::msg::TimestampedPacketSessionData>("session_data", 1, std::bind(&MeasurementPublisher::sessionCallback, this, std::placeholders::_1));
     
     this->listener = this->node->create_subscription<deepracing_msgs::msg::TimestampedPacketMotionData>("motion_data", 1, std::bind(&MeasurementPublisher::packetCallback, this, std::placeholders::_1));


     
    }  
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketMotionData>::SharedPtr listener;
    rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketSessionData>::SharedPtr session_listener;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_publisher;
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
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
    get_node_base_interface() const
    {
      return node->get_node_base_interface();
    }
  private:
    random_numbers::RandomNumberGenerator m_rng_;
    bool m_with_ekf_;
    nav_msgs::msg::Odometry odom_msg_;
    sensor_msgs::msg::Imu imu_msg_;
    geometry_msgs::msg::AccelWithCovarianceStamped accel_msg_;
    
    double m_extra_position_noise;
    double m_extra_rot_noise;
    double m_extra_vel_noise;   
    double m_extra_angvel_noise; 
    int8_t current_track_id;
    std::string carname;
    void sessionCallback(const deepracing_msgs::msg::TimestampedPacketSessionData::UniquePtr session_msg)
    {
      if((session_msg->udp_packet.track_id>=0) && (session_msg->udp_packet.track_id!=current_track_id))
      {
        char* envrtn = std::getenv("F1_TRACK_DIRS");
        std::vector<std::string> search_dirs;
        if (envrtn)
        {
          search_dirs = deepracing_ros::FileUtils::split(std::string(envrtn));
        }
        try
        {
          std::filesystem::path f1_datalogger_path = std::filesystem::path(ament_index_cpp::get_package_share_directory("f1_datalogger"));
          search_dirs.push_back((f1_datalogger_path / std::filesystem::path("f1_tracks")).string());
          search_dirs.push_back((f1_datalogger_path / std::filesystem::path("f1_tracks") / std::filesystem::path("minimumcurvature")).string());
        }
        catch(const std::exception& e)
        {
          
        }
        std::stringstream ss;
        for (const std::string& dir : search_dirs)
        {
          ss << dir <<", ";
        }
        RCLCPP_INFO(node->get_logger(), "Looking for starting pose file in: %s" , ss.str().c_str());
        
        std::array<std::string, 25> tracknames = deepracing_ros::F1MsgUtils2020::track_names();
        std::string trackname = tracknames[session_msg->udp_packet.track_id];
        std::string starting_pose_filepath = deepracing_ros::FileUtils::findFile(trackname+"_startingpose.json", search_dirs);
        RCLCPP_INFO(node->get_logger(), "Got starting pose file: %s", starting_pose_filepath.c_str());
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
    void packetCallback(const deepracing_msgs::msg::TimestampedPacketMotionData::UniquePtr motion_data_packet)
    {
     // std::cout << "Got some data" << std::endl;
      // RCLCPP_INFO(node->get_logger(), "Got some data");
      if(current_track_id<0)
      {
        RCLCPP_ERROR(node->get_logger(), "%s", "Haven't received track information from the session data stream yet");
        return;
      }
      uint8_t idx;
      if( motion_data_packet->udp_packet.header.player_car_index<22 )
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
      const geometry_msgs::msg::PointStamped& positionROS = motion_data.world_position;

      Eigen::Vector3d positioneigen_track(positionROS.point.x, positionROS.point.y, positionROS.point.z);
      Eigen::Vector3d leftEigen=Eigen::Vector3d(-rightROS.vector.x, -rightROS.vector.y, -rightROS.vector.z).normalized();
      Eigen::Vector3d forwardEigen=Eigen::Vector3d(forwardROS.vector.x, forwardROS.vector.y, forwardROS.vector.z).normalized();
      Eigen::Vector3d upEigen = forwardEigen.cross(leftEigen).normalized();
      Eigen::Matrix3d rotmat;
      rotmat.col(0) = forwardEigen;
      rotmat.col(1) = leftEigen;
      rotmat.col(2) = upEigen;
      Eigen::Quaterniond rotationEigen(rotmat);
      rotationEigen.normalize();
    
      Eigen::Isometry3d trackToCarEigen;
      trackToCarEigen.fromPositionOrientationScale(positioneigen_track, rotationEigen, Eigen::Vector3d::Ones());
      Eigen::Isometry3d mapToCarEigen = mapToTrackEigen * trackToCarEigen;
      Eigen::Isometry3d mapToBLEigen = mapToCarEigen * carToBaseLinkEigen;

      geometry_msgs::msg::TransformStamped transformMsg = tf2::eigenToTransform(trackToCarEigen);
      transformMsg.set__header(motion_data_packet->header);
      transformMsg.set__child_frame_id(deepracing_ros::F1MsgUtils2020::car_coordinate_name+"_"+carname);

      Eigen::Vector3d centroidVelEigenGlobal(velocityROS.vector.x, velocityROS.vector.y, velocityROS.vector.z);
      Eigen::Vector3d centroidVelEigenLocal = trackToCarEigen.rotation().inverse()*centroidVelEigenGlobal;    

      odom_msg_.header.set__frame_id("map");
      odom_msg_.header.set__stamp(transformMsg.header.stamp);
      odom_msg_.set__child_frame_id(transformMsg.child_frame_id);
      odom_msg_.pose.pose = tf2::toMsg(mapToCarEigen);
      odom_msg_.twist.twist.linear.x=centroidVelEigenLocal.x();
      odom_msg_.twist.twist.linear.y=centroidVelEigenLocal.y();
      odom_msg_.twist.twist.linear.z=centroidVelEigenLocal.z();

      imu_msg_.header.set__stamp(transformMsg.header.stamp);
      imu_msg_.header.set__frame_id(transformMsg.child_frame_id);
      imu_msg_.set__orientation(odom_msg_.pose.pose.orientation);
      Eigen::Vector3d centroidLinearAccel=9.81*Eigen::Vector3d(motion_data.g_force_longitudinal, motion_data.g_force_lateral, motion_data.g_force_vertical);
      imu_msg_.linear_acceleration.set__x(centroidLinearAccel.x());
      imu_msg_.linear_acceleration.set__y(centroidLinearAccel.y());
      imu_msg_.linear_acceleration.set__z(centroidLinearAccel.z());

      if(motion_data_packet->udp_packet.player_only_data)
      {
        Eigen::Vector3d centroidAngVelTrack(motion_data_packet->udp_packet.angular_velocity.x, motion_data_packet->udp_packet.angular_velocity.y, motion_data_packet->udp_packet.angular_velocity.z);
        Eigen::Vector3d centroidAngVel = trackToCarEigen.rotation().inverse()*centroidAngVelTrack;
        odom_msg_.twist.twist.angular.x=centroidAngVel.x();
        odom_msg_.twist.twist.angular.y=centroidAngVel.y();
        odom_msg_.twist.twist.angular.z=centroidAngVel.z();
        imu_msg_.set__angular_velocity(odom_msg_.twist.twist.angular);
      }
      
      carToBaseLink.header.set__stamp(motion_data.world_position.header.stamp);
      mapToTrack.header.set__stamp(motion_data.world_position.header.stamp);

      if (!m_with_ekf_)
      {
        this->tfbroadcaster->sendTransform(transformMsg);
        accel_msg_.header.set__stamp(odom_msg_.header.stamp).set__frame_id(transformMsg.child_frame_id);
        accel_msg_.accel.accel.set__linear(imu_msg_.linear_acceleration);
        this->accel_publisher->publish(std::make_unique<geometry_msgs::msg::AccelWithCovarianceStamped>(accel_msg_));
      }
      this->odom_publisher->publish(std::make_unique<nav_msgs::msg::Odometry>(odom_msg_));
      this->imu_publisher->publish(std::make_unique<sensor_msgs::msg::Imu>(imu_msg_));
    }
};

}
}
#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::MeasurementPublisher)