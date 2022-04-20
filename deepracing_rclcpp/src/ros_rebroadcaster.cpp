#include "rclcpp/rclcpp.hpp"
#include "f1_datalogger/udp_logging/utils/udp_stream_utils.h"
#include "f1_datalogger/f1_datalogger.h"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include "f1_datalogger/udp_logging/utils/eigen_utils.h"
#include "f1_datalogger/udp_logging/common/rebroadcast_handler_2018.h"
#include "f1_datalogger/image_logging/utils/opencv_utils.h"
#include "deepracing_ros/utils/f1_msg_utils.h"
#include "deepracing_msgs/msg/timestamped_image.hpp"
#include "deepracing_msgs/msg/timestamped_packet_motion_data.hpp"
#include "deepracing_msgs/msg/timestamped_packet_car_setup_data.hpp"
#include "deepracing_msgs/msg/timestamped_packet_car_status_data.hpp"
#include "deepracing_msgs/msg/timestamped_packet_car_telemetry_data.hpp"
#include "deepracing_msgs/msg/timestamped_packet_lap_data.hpp"
#include "deepracing_msgs/msg/timestamped_packet_session_data.hpp"
#include <std_srvs/srv/empty.hpp>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.h>
#include <rclcpp/executor.hpp>
#include <rclcpp/executor_options.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/time.hpp>
#include <chrono>
#include <cmath>
#include <rosgraph_msgs/msg/clock.hpp>

// #include <image_transport/camera_publisher.h>
// #include <image_transport/transport_hints.h>
namespace deepf1
{
inline rclcpp::Time fromDouble(const double& ts, rcl_clock_type_t clock_type = rcl_clock_type_t::RCL_SYSTEM_TIME)
{
  double fractpart, intpart;
  fractpart = modf(ts, &intpart);  
  int32_t sec =(int32_t)(intpart);
  uint32_t nanosec =(uint32_t)(fractpart*1E9);
  return rclcpp::Time(sec, nanosec, clock_type);
}
inline rclcpp::Time fromDeepf1Timestamp(const deepf1::TimePoint& timestamp, const deepf1::TimePoint& begin = deepf1::TimePoint())
{
  std::chrono::duration<uint64_t, std::nano> dt = std::chrono::duration_cast< std::chrono::duration<uint64_t, std::nano> >(timestamp - begin);
  uint64_t dtnsec = dt.count();
  return rclcpp::Time((int32_t)(dtnsec/static_cast<uint64_t>(1E9)), (uint32_t)(dtnsec%static_cast<uint64_t>(1E9)), rcl_clock_type_t::RCL_STEADY_TIME);
}
}
class ROSRebroadcaster_2018DataGrabHandler : public deepf1::IF12018DataGrabHandler
{
public:
  ROSRebroadcaster_2018DataGrabHandler(std::shared_ptr<rclcpp::Node> node)
  {
    node_ = node;
    rcl_interfaces::msg::ParameterDescriptor all_cars_description;
    all_cars_description.name = "publish_all_cars";
    all_cars_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    all_cars_description.description = "Whether to publish information about all 20 cars in the session, or just the ego vehicle. True means publish information about all cars";
    all_cars_description.read_only = false;
    all_cars_param_ = node_->declare_parameter<bool>("publish_all_cars", true, all_cars_description);
    node_->get_parameter<std::string>("time_source", this->time_source);
    // use_sim_time_ = node_->declare_parameter<bool>("use_sim_time");
    if (!(this->time_source=="ROS2"))
    {
      clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
    }
    
  }
  bool isReady() override
  {
    return ready_;
  }
  virtual inline void handleData(const deepf1::twenty_eighteen::TimestampedPacketCarSetupData& udp_data) override
  {
    deepracing_msgs::msg::TimestampedPacketCarSetupData rosdata;
    setStamp(rosdata.header.stamp, udp_data.data.m_header, udp_data.timestamp);
    rosdata.header.set__frame_id(deepracing_ros::F1MsgUtils::world_coordinate_name);
    rosdata.udp_packet = deepracing_ros::F1MsgUtils::toROS(udp_data.data, all_cars_param_);
    setup_data_publisher_->publish(rosdata);
  }
  virtual inline void handleData(const deepf1::twenty_eighteen::TimestampedPacketCarStatusData& udp_data) override
  {    
    deepracing_msgs::msg::TimestampedPacketCarStatusData rosdata;
    setStamp(rosdata.header.stamp, udp_data.data.m_header, udp_data.timestamp);
    rosdata.header.set__frame_id(deepracing_ros::F1MsgUtils::world_coordinate_name);
    rosdata.udp_packet = deepracing_ros::F1MsgUtils::toROS(udp_data.data, all_cars_param_);
    status_publisher_->publish(rosdata);
  }
  virtual inline void handleData(const deepf1::twenty_eighteen::TimestampedPacketCarTelemetryData& udp_data) override
  {
    
    deepracing_msgs::msg::TimestampedPacketCarTelemetryData rosdata;
    setStamp(rosdata.header.stamp, udp_data.data.m_header, udp_data.timestamp);
    rosdata.header.set__frame_id(deepracing_ros::F1MsgUtils::world_coordinate_name);
    rosdata.udp_packet = deepracing_ros::F1MsgUtils::toROS(udp_data.data, all_cars_param_);
    telemetry_publisher_->publish(rosdata);
  }
  virtual inline void handleData(const deepf1::twenty_eighteen::TimestampedPacketEventData& data) override
  {
  }
  virtual inline void handleData(const deepf1::twenty_eighteen::TimestampedPacketLapData& udp_data) override
  {
    deepracing_msgs::msg::TimestampedPacketLapData rosdata;
    setStamp(rosdata.header.stamp, udp_data.data.m_header, udp_data.timestamp);
    rosdata.header.set__frame_id(deepracing_ros::F1MsgUtils::world_coordinate_name);
    rosdata.udp_packet = deepracing_ros::F1MsgUtils::toROS(udp_data.data, all_cars_param_);
    lap_data_publisher_->publish(rosdata);
  }
  virtual inline void handleData(const deepf1::twenty_eighteen::TimestampedPacketMotionData& udp_data) override
  {
    deepracing_msgs::msg::TimestampedPacketMotionData rosdata;
    setStamp(rosdata.header.stamp, udp_data.data.m_header, udp_data.timestamp);
    if (clock_publisher_)
    {
      rosgraph_msgs::msg::Clock clockmsg;
      clockmsg.set__clock(rosdata.header.stamp);
      clock_publisher_->publish(clockmsg);
    }
    rosdata.header.set__frame_id(deepracing_ros::F1MsgUtils::world_coordinate_name);
    rosdata.udp_packet = deepracing_ros::F1MsgUtils::toROS(udp_data.data, all_cars_param_);
    deepracing_msgs::msg::CarMotionData& ego_motion_data = rosdata.udp_packet.car_motion_data.at(rosdata.udp_packet.header.player_car_index);
    ego_motion_data.world_forward_dir.header.stamp =
    ego_motion_data.world_position.header.stamp = 
    ego_motion_data.world_right_dir.header.stamp =
    ego_motion_data.world_up_dir.header.stamp = 
    ego_motion_data.world_velocity.header.stamp = rosdata.header.stamp;
    if (all_cars_param_)
    {
      for(deepracing_msgs::msg::CarMotionData & motion_data : rosdata.udp_packet.car_motion_data)
      {
        motion_data.world_forward_dir.header.stamp =
        motion_data.world_position.header.stamp = 
        motion_data.world_right_dir.header.stamp =
        motion_data.world_up_dir.header.stamp = 
        motion_data.world_velocity.header.stamp = rosdata.header.stamp;
      }
    }
    motion_publisher_->publish(rosdata);
  }
  virtual inline void handleData(const deepf1::twenty_eighteen::TimestampedPacketParticipantsData& data) override
  {
  }
  virtual inline void handleData(const deepf1::twenty_eighteen::TimestampedPacketSessionData& udp_data) override
  {
    deepracing_msgs::msg::TimestampedPacketSessionData rosdata;
    setStamp(rosdata.header.stamp, udp_data.data.m_header, udp_data.timestamp);
    rosdata.header.set__frame_id(deepracing_ros::F1MsgUtils::world_coordinate_name);
    rosdata.udp_packet = deepracing_ros::F1MsgUtils::toROS(udp_data.data);
    session_publisher_->publish(rosdata);
  }
  void init(const std::string& host, unsigned int port, const deepf1::TimePoint& begin) override
  {

    lap_data_publisher_ = node_->create_publisher<deepracing_msgs::msg::TimestampedPacketLapData>("lap_data", 1);
    motion_publisher_ = node_->create_publisher<deepracing_msgs::msg::TimestampedPacketMotionData>("motion_data", 1);
    session_publisher_ = node_->create_publisher<deepracing_msgs::msg::TimestampedPacketSessionData>("session_data", 1);
    setup_data_publisher_ = node_->create_publisher<deepracing_msgs::msg::TimestampedPacketCarSetupData>("setup_data", 1);
    status_publisher_ = node_->create_publisher<deepracing_msgs::msg::TimestampedPacketCarStatusData>("status_data", 1);
    telemetry_publisher_ = node_->create_publisher<deepracing_msgs::msg::TimestampedPacketCarTelemetryData>("telemetry_data", 1);

    ready_ = true;
    begin_ = begin;
    host_ = host;
    port_ = port;
  }
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Publisher <deepracing_msgs::msg::TimestampedPacketLapData> > lap_data_publisher_;
  std::shared_ptr<rclcpp::Publisher <deepracing_msgs::msg::TimestampedPacketMotionData> > motion_publisher_;
  std::shared_ptr<rclcpp::Publisher <deepracing_msgs::msg::TimestampedPacketSessionData> > session_publisher_;
  std::shared_ptr<rclcpp::Publisher <deepracing_msgs::msg::TimestampedPacketCarSetupData> > setup_data_publisher_;
  std::shared_ptr<rclcpp::Publisher <deepracing_msgs::msg::TimestampedPacketCarStatusData> > status_publisher_;
  std::shared_ptr<rclcpp::Publisher <deepracing_msgs::msg::TimestampedPacketCarTelemetryData> > telemetry_publisher_;
  std::shared_ptr<rclcpp::Publisher <rosgraph_msgs::msg::Clock> > clock_publisher_;
  std::string time_source;
private:
  bool ready_;
  std::chrono::high_resolution_clock::time_point begin_;
  std::string host_;
  unsigned int port_;
  bool all_cars_param_, use_sim_time_;
  void setStamp(builtin_interfaces::msg::Time& stamp, const deepf1::twenty_eighteen::PacketHeader& f1_header, const deepf1::TimePoint& deepracing_time)
  {
    if(this->time_source == "ROS2")
    {
      stamp = this->node_->get_clock()->now();
    }
    else if(this->time_source == "F1")
    {
      stamp = deepf1::fromDouble((double)f1_header.m_sessionTime, rcl_clock_type_t::RCL_STEADY_TIME);
    }
    else if(this->time_source == "DeepRacing")
    {
      stamp = deepf1::fromDeepf1Timestamp(deepracing_time, this->begin_);
    }
    else
    {
      throw std::runtime_error("\"time_source\" param set to invalid value: " + this->time_source);
    }
  }
};
class ROSRebroadcaster_FrameGrabHandler : public deepf1::IF1FrameGrabHandler
{
public:
  ROSRebroadcaster_FrameGrabHandler(std::shared_ptr<rclcpp::Node> node) : it(node)
  {
    std::cout<<"Loadable transports:"<<std::endl;
    for(const std::string& s : it.getLoadableTransports())
    {
      std::cout<<s<<std::endl;
    }
    rclcpp::ParameterValue resize_height_p = node->declare_parameter("resize_height",rclcpp::ParameterValue(-1));
    resize_height_ =  resize_height_p.get<int>();

    rclcpp::ParameterValue resize_width_p = node->declare_parameter("resize_width",rclcpp::ParameterValue(-1));
    resize_width_ =  resize_width_p.get<int>();

    rclcpp::ParameterValue resize_factor_p = node->declare_parameter("resize_factor",rclcpp::ParameterValue(1.0));
    resize_factor_ =  resize_factor_p.get<double>();


    rclcpp::ParameterValue top_left_row_p = node->declare_parameter("top_left_row",rclcpp::ParameterValue(0));
    top_left_row_ =  top_left_row_p.get<int>();
    RCLCPP_INFO(node->get_logger(),"Cropping from row %d\n", top_left_row_);


    rclcpp::ParameterValue top_left_col_p = node->declare_parameter("top_left_col",rclcpp::ParameterValue(0));
    top_left_col_ =  top_left_col_p.get<int>();

    rclcpp::ParameterValue crop_height_p = node->declare_parameter("crop_height",rclcpp::ParameterValue(0));
    crop_height_ =  crop_height_p.get<int>();

    rclcpp::ParameterValue crop_width_p = node->declare_parameter("crop_width",rclcpp::ParameterValue(0));
    crop_width_ =  crop_width_p.get<int>();

    this->node_ = node;
        
    // this->it_publisher_ = it.advertise("images", 1, true);
    this->it_publisher_ = it.advertiseCamera("images", 1, true);

    node_->get_parameter<std::string>("time_source", this->time_source);
  }
  virtual ~ROSRebroadcaster_FrameGrabHandler()
  {
  }
  bool isReady() override
  {
    return ready;
  }
  void handleData(const deepf1::TimestampedImageData& data) override
  {
    std_msgs::msg::Header header;
    
    if(time_source == "ROS2")
    {
      header.stamp = this->node_->get_clock()->now();
    }
    else if ((time_source == "F1") || (time_source == "DeepRacing"))
    {
      header.stamp = deepf1::fromDeepf1Timestamp(data.timestamp, this->begin_);
    }
    else
    {
      throw std::runtime_error("\"time_source\" param set to invalid value: " + time_source);
    }
    header.frame_id = deepracing_ros::F1MsgUtils::car_coordinate_name;
    const cv::Mat& imin = data.image;
   
    cv::Mat rgbimage;
    cv::Range rowrange, colrange;
    if (crop_height_>0 && crop_width_>0)
    {
      rowrange = cv::Range(top_left_row_, top_left_row_+crop_height_ - 1);
      colrange = cv::Range(top_left_col_, top_left_col_+crop_width_ - 1);
    }
    else
    {
      rowrange = cv::Range(top_left_row_, imin.rows - 1);
      colrange = cv::Range(top_left_col_, imin.cols - 1);
    }
    
    cv::cvtColor( imin(rowrange,colrange) , rgbimage , cv::COLOR_BGRA2RGB );
    if( resize_factor_!=1.0 )
    {
      cv::resize(rgbimage, rgbimage, cv::Size(),resize_factor_,resize_factor_,cv::INTER_AREA);
    }
    else if(resize_width_>0 && resize_height_>0)
    {
      cv::resize(rgbimage, rgbimage, cv::Size(resize_width_, resize_height_), 0.0, 0.0, cv::INTER_AREA);
    }

    cv_bridge::CvImage bridge_image(header, "rgb8", rgbimage);

    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.distortion_model="plumbob";
    camera_info.header = header;
    camera_info.height=static_cast<sensor_msgs::msg::CameraInfo::_height_type>(rgbimage.rows);
    camera_info.width=static_cast<sensor_msgs::msg::CameraInfo::_width_type>(rgbimage.cols);
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_ptr = std::make_shared<sensor_msgs::msg::CameraInfo const>(camera_info);
    this->it_publisher_.publish( bridge_image.toImageMsg(), camera_info_ptr);

    // this->it_publisher_.publish( bridge_image.toImageMsg() );
    

  }
  void init(const deepf1::TimePoint& begin, const cv::Size& window_size) override
  {
    this->begin_ = begin;
    ready = true;
  }
  double resize_factor_;
  int resize_width_;
  int resize_height_;
  int crop_height_;
  int crop_width_;
  int top_left_row_;
  int top_left_col_;
  std::string time_source;


private:
  double full_image_resize_factor_;
  bool ready;
  std::shared_ptr<rclcpp::Node> node_;
  image_transport::ImageTransport it;
  // image_transport::Publisher it_publisher_;
  image_transport::CameraPublisher it_publisher_;
  std::shared_ptr<rclcpp::Publisher <sensor_msgs::msg::Image> > publisher_;
  std::shared_ptr<rclcpp::Publisher <deepracing_msgs::msg::TimestampedImage> > timestamped_publisher_;
  deepf1::TimePoint begin_;
  
};
class NodeWrapper_ 
{

  public:
    NodeWrapper_()
     {
      node_ = rclcpp::Node::make_shared("f1_ros_rebroadcaster","");
    }  

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<ROSRebroadcaster_2018DataGrabHandler> datagrab_handler;
    std::shared_ptr<ROSRebroadcaster_FrameGrabHandler> image_handler;
    
};
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  NodeWrapper_ nw;
  const std::shared_ptr<rclcpp::Node>& node = nw.node_;
  rcl_interfaces::msg::ParameterDescriptor time_source_description;
  time_source_description.name="time_source";
  time_source_description.read_only=false;
  time_source_description.type=rclcpp::PARAMETER_STRING;
  std::stringstream tsdesc_stream;
  tsdesc_stream<<"What time source to use for populating the header timestamps of the various ROS2 messages."<<std::endl;
  tsdesc_stream<<"Acceptable values are:"<<std::endl;
  tsdesc_stream<<"\"ROS2\" (default). Both images and UDP types will use the standard ROS2 clock."<<std::endl;
  tsdesc_stream<<"\"DeepRacing\". Both images and UDP types will use the DeepRacing logger's internal C++ std::chrono clock."<<std::endl;
  tsdesc_stream<<"\"F1\". UDP types will use the session timestamp off the UDP stream. Images will use the DeepRacing logger clock."<<std::endl;
  time_source_description.description = tsdesc_stream.str();
  std::string time_source_p = node->declare_parameter<std::string>(time_source_description.name, "ROS2", time_source_description);
  std::set<std::string> acceptable_time_sources = {"ROS2", "DeepRacing", "F1"};
  if (acceptable_time_sources.find(time_source_p)==acceptable_time_sources.end())
  {
    throw rclcpp::exceptions::InvalidParameterValueException("time_source parameter must be on of: {\"ROS2\", \"DeepRacing\", \"F1\"}");
  }
  nw.datagrab_handler.reset(new ROSRebroadcaster_2018DataGrabHandler(node));
  nw.image_handler.reset(new ROSRebroadcaster_FrameGrabHandler(node));

  rcl_interfaces::msg::ParameterDescriptor capture_freq_description;
  capture_freq_description.floating_point_range.push_back(rcl_interfaces::msg::FloatingPointRange());
  capture_freq_description.floating_point_range[0].from_value=1.0;
  capture_freq_description.floating_point_range[0].to_value=60.0;
  capture_freq_description.floating_point_range[0].step=0.0;
  capture_freq_description.description="Frequency (in Hz) to capture images from the screen";

  rclcpp::ParameterValue search_string_p = node->declare_parameter("search_string",rclcpp::ParameterValue(""));
  rclcpp::ParameterValue capture_frequency_p = node->declare_parameter("capture_frequency",rclcpp::ParameterValue(35.0), capture_freq_description);
  rclcpp::ParameterValue hostname_p = node->declare_parameter("hostname",rclcpp::ParameterValue("127.0.0.1"));
  rclcpp::ParameterValue num_theads_p = node->declare_parameter("num_theads",rclcpp::ParameterValue(0));

  

  rcl_interfaces::msg::ParameterDescriptor port_description;
  port_description.integer_range.push_back(rcl_interfaces::msg::IntegerRange());
  port_description.integer_range[0].from_value=20777;
  port_description.integer_range[0].to_value=21000;
  port_description.integer_range[0].step=1;
  port_description.name="port";
  port_description.type=rclcpp::PARAMETER_INTEGER;
  port_description.description="What port number to listen on for UDP data";
  rclcpp::ParameterValue port_p = node->declare_parameter(port_description.name, rclcpp::ParameterValue(20777), port_description);

  rclcpp::ParameterValue rebroadcast_p = node->declare_parameter("rebroadcast",rclcpp::ParameterValue(false));
  std::string search_string = search_string_p.get<std::string>();
  deepf1::F1DataLogger dl(search_string, hostname_p.get<std::string>(), port_p.get<int>()); 
  std::shared_ptr<deepf1::RebroadcastHandler2018> rbh; 
  if(rebroadcast_p.get<bool>())
  {
    RCLCPP_INFO(node->get_logger(),"Rebroadcasting on port: %lld\n", port_p.get<int>()+1);
    rbh.reset(new deepf1::RebroadcastHandler2018());
    RCLCPP_INFO(node->get_logger(),"Created rebroadcast socket\n");
    dl.add2018UDPHandler(rbh);
    RCLCPP_INFO(node->get_logger(),"Added rebroadcast socket\n");
  }
  RCLCPP_INFO(node->get_logger(),
              "Listening for data from the game. \n"
              "Cropping an area (HxW)  (%d, %d)\n"
              "Resizing cropped area to (HxW)  (%d, %d)\n"
              ,nw.image_handler->crop_height_, nw.image_handler->crop_width_, nw.image_handler->resize_height_, nw.image_handler->resize_width_);
  
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb = node->add_on_set_parameters_callback([nw]
  (const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult rtn;
    for (const rclcpp::Parameter& parameter : params)
    {
      if(parameter.get_name()=="time_source")
      {
        nw.datagrab_handler->time_source=nw.image_handler->time_source=parameter.as_string();
      }
    }
    rtn.reason="To be? Or not to be?";
    rtn.successful=true;
    return rtn;
  });
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr shudownservice = node->create_service<std_srvs::srv::Empty>("deepracing_shutdown", 
  []
  (const std::shared_ptr<std_srvs::srv::Empty::Request> req,  std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    rclcpp::shutdown();
  }
  );
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), num_theads_p.get<int>());
  exec.add_node(node);
  dl.add2018UDPHandler(nw.datagrab_handler);
  std::shared_ptr<deepf1::IF1FrameGrabHandler> nullhandler;
  if (search_string.empty())
  {
    dl.start(capture_frequency_p.get<double>(), nullhandler);
  }
  else
  { 
    dl.start(capture_frequency_p.get<double>(), nw.image_handler);
  }
  exec.spin();
  rclcpp::shutdown();
  return 0;
}