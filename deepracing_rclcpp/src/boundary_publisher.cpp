#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <filesystem>
#include <fstream>
#include <streambuf>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <json/json.h>
#include <deepracing_msgs/msg/timestamped_packet_session_data.hpp>
#include <deepracing_msgs/msg/boundary_line.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "deepracing_ros/utils/f1_msg_utils.h"
#include "deepracing_ros/utils/file_utils.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

Json::Value readRacelineFile(std::shared_ptr<rclcpp::Node> node, std::string filepath)
{
    Json::Value rootval;
    std::ifstream file;
    file.open(filepath, std::fstream::in);
    RCLCPP_INFO(node->get_logger(), "Parsing json");
    Json::Reader reader;
    bool parsed = reader.parse(file,rootval);
    file.close();
    if(parsed)
    {
        RCLCPP_INFO(node->get_logger(), "Parsed json");
    }
    else
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to parse json");
    }
    Json::Value xarray = rootval["x"];
    if(  xarray.isNull() )
    {
        RCLCPP_FATAL(node->get_logger(), "no \"x\" key found in the json dictionary for a raceline file");
    }
    Json::Value yarray = rootval["y"];
    if(  yarray.isNull() )
    {
        RCLCPP_FATAL(node->get_logger(), "no \"y\" key found in the json dictionary for a raceline file");
    }
    Json::Value zarray = rootval["z"];
    if(  zarray.isNull() )
    {
        RCLCPP_FATAL(node->get_logger(), "no \"z\" key found in the json dictionary for a raceline file");
    }
    Json::Value tarray = rootval["t"];
    if(  tarray.isNull() )
    {
        RCLCPP_FATAL(node->get_logger(), "no \"t\" key found in the json dictionary for a raceline file");
    }   
    std::array<uint32_t,4> sizes = {(uint32_t)xarray.size(), (uint32_t)yarray.size(), (uint32_t)zarray.size(),(uint32_t)tarray.size()};
    if ( !std::all_of(sizes.begin(), sizes.end(), [xarray](uint32_t i){return i==xarray.size();}) )
    {
       std::stringstream ss;
       std::for_each(sizes.begin(), sizes.end(), [&ss](uint32_t i){ss<<i<<std::endl;});
       RCLCPP_FATAL(node->get_logger(), "All arrays are not the same size. Sizes: %s", ss.str().c_str()); 
    }
    return rootval;
}
Json::Value readBoundaryFile(std::shared_ptr<rclcpp::Node> node, std::string filepath)
{
    Json::Value rootval;
    std::ifstream file;
    file.open(filepath, std::fstream::in);
    RCLCPP_INFO(node->get_logger(), "Parsing json");
    Json::Reader reader;
    bool parsed = reader.parse(file,rootval);
    file.close();
    if(parsed)
    {
        RCLCPP_INFO(node->get_logger(), "Parsed json");
    }
    else
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to parse json");
    }
    Json::Value xarray = rootval["x"];
    if(  xarray.isNull() )
    {
        RCLCPP_FATAL(node->get_logger(), "no \"x\" key found in the json dictionary for a boundary file");
    }
    Json::Value yarray = rootval["y"];
    if(  yarray.isNull() )
    {
        RCLCPP_FATAL(node->get_logger(), "no \"y\" key found in the json dictionary for a boundary file");
    }
    Json::Value zarray = rootval["z"];
    if(  zarray.isNull() )
    {
        RCLCPP_FATAL(node->get_logger(), "no \"z\" key found in the json dictionary for a boundary file");
    }
    Json::Value rarray = rootval["r"];
    if(  rarray.isNull() )
    {
        RCLCPP_FATAL(node->get_logger(), "no \"r\" key found in the json dictionary for a boundary file");
    }   
    std::array<uint32_t,4> sizes = {(uint32_t)xarray.size(), (uint32_t)yarray.size(), (uint32_t)zarray.size(),(uint32_t)rarray.size()};
    if ( !std::all_of(sizes.begin(), sizes.end(), [xarray](uint32_t i){return i==xarray.size();}) )
    {
       std::stringstream ss;
       std::for_each(sizes.begin(), sizes.end(), [&ss](uint32_t i){ss<<i<<std::endl;});
       RCLCPP_FATAL(node->get_logger(), "All arrays are not the same size. Sizes: %s", ss.str().c_str()); 
    }
    return rootval;
}

void unpackDictionary(std::shared_ptr<rclcpp::Node> node, const Json::Value& boundary_dict, pcl::PointCloud<pcl::PointXYZI>& cloudpcl, std::string ikey )
{
    Json::Value x = boundary_dict["x"]; Json::Value y = boundary_dict["y"]; Json::Value z = boundary_dict["z"]; Json::Value I = boundary_dict[ikey]; 
    cloudpcl.clear();
    unsigned int imax = x.size();
    for (unsigned int i =0; i < imax; i++)
    {
        pcl::PointXYZI pointI(x[i].asDouble(), y[i].asDouble(), z[i].asDouble(), I[i].asDouble());
        cloudpcl.push_back(pointI);
    }
}
class NodeWrapper_
{
    public: 
        NodeWrapper_(std::shared_ptr<rclcpp::Node> node)
        {
            if(!bool(node))
            {
                this->node = rclcpp::Node::make_shared("f1_boundary_publisher","");
            }
            else
            {
                this->node=node;
            }
            listener =  this->node->create_subscription<deepracing_msgs::msg::TimestampedPacketSessionData>( "/session_data", 1, std::bind(&NodeWrapper_::sessionDataCallback, this, std::placeholders::_1)  );
            current_session_data.reset(new deepracing_msgs::msg::TimestampedPacketSessionData);
            current_session_data->udp_packet.track_id=-1;
        }
        std::shared_ptr<deepracing_msgs::msg::TimestampedPacketSessionData> current_session_data;
    private:
        std::shared_ptr<rclcpp::Node> node;
        std::shared_ptr< rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketSessionData> > listener;
        void sessionDataCallback(const deepracing_msgs::msg::TimestampedPacketSessionData::SharedPtr session_data_packet)
        {
            this->current_session_data.reset(new deepracing_msgs::msg::TimestampedPacketSessionData(*session_data_packet));
        }

};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    namespace fs = std::filesystem;
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("f1_boundary_publisher","");
    std::shared_ptr< rclcpp::Publisher<sensor_msgs::msg::PointCloud2> > innerpub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/inner_track_boundary/pcl",1);
    std::shared_ptr< rclcpp::Publisher<sensor_msgs::msg::PointCloud2> > outerpub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/outer_track_boundary/pcl",1);
    std::shared_ptr< rclcpp::Publisher<sensor_msgs::msg::PointCloud2> > racelinepub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/optimal_raceline/pcl",1);

    
    NodeWrapper_ nw(node);
    pcl::PointCloud<pcl::PointXYZI> innercloudPCL, outercloudPCL, racelinecloudPCL;
    sensor_msgs::msg::PointCloud2 innercloudMSG, outercloudMSG, racelinecloudMSG;
    Json::Value innerDict, outerDict, racelineDict;
    
    std::vector<std::string> search_dirs = node->declare_parameter<std::vector<std::string>>("track_search_dirs", std::vector<std::string>());
    search_dirs.insert(search_dirs.begin(),fs::current_path().string());
    try
    {
        std::string f1_datalogger_share_dir = ament_index_cpp::get_package_share_directory("f1_datalogger");
        RCLCPP_INFO(node->get_logger(), "f1_datalogger share directory: %s.", f1_datalogger_share_dir.c_str());
        std::string prefix_search_dir = (fs::path(f1_datalogger_share_dir)/fs::path("f1_tracks")).string();
        RCLCPP_INFO(node->get_logger(), "f1_datalogger share search directory: %s.", prefix_search_dir.c_str());
        search_dirs.push_back(prefix_search_dir);
    }
    catch(const ament_index_cpp::PackageNotFoundError& e)
    {   
        RCLCPP_WARN(node->get_logger(), "f1_datalogger was built as plain cmake (not ament), cannot locate it's install directory with ament");
    }
    if (search_dirs.empty())
    {
        RCLCPP_FATAL(node->get_logger(), "No directories specified to search for track files. Default location is the share directory of f1_datalogger or specified in the track_search_dirs ros parameter");
    }
    std::stringstream ss;
    std::for_each(search_dirs.begin(), search_dirs.end(), [&ss](const std::string dir){ss<<std::endl<<dir;});
    RCLCPP_INFO(node->get_logger(), "Searching in the following directories for track files (in order):%s", ss.str().c_str());
    std::string track_name = "";
    std::array<std::string,25> track_name_array = deepracing_ros::F1MsgUtils::track_names();

    while (rclcpp::is_initialized())
    {
        rclcpp::sleep_for(std::chrono::nanoseconds(int(1E9)));
        rclcpp::spin_some(node);
        innercloudMSG.header.stamp = node->now();
        RCLCPP_DEBUG(node->get_logger(), "Ran a spin.");
        std::string active_track="";
        int8_t track_index = nw.current_session_data->udp_packet.track_id;
        if(track_index>=0 && track_index<track_name_array.size())
        {
            active_track=track_name_array[track_index];
        }
        RCLCPP_DEBUG(node->get_logger(), "current_track: %s", track_name.c_str());
        RCLCPP_DEBUG(node->get_logger(), "active_track: %s", active_track.c_str());
        if(!active_track.empty() && track_name.compare(active_track)!=0)
        {            
            track_name = active_track;
            RCLCPP_INFO(node->get_logger(), "Detected new track  %s.", track_name.c_str());

            std::string inner_filename = deepracing_ros::FileUtils::findFile(track_name + "_innerlimit.json",search_dirs);
            RCLCPP_INFO(node->get_logger(), "Openning file %s.", inner_filename.c_str());
            innerDict = readBoundaryFile(node,inner_filename);
            unpackDictionary(node,innerDict, innercloudPCL, "r");
            RCLCPP_INFO(node->get_logger(), "Got %d points for the inner boundary.", innercloudPCL.size());

            std::string outer_filename = deepracing_ros::FileUtils::findFile(track_name + "_outerlimit.json",search_dirs);
            RCLCPP_INFO(node->get_logger(), "Openning file %s.", outer_filename.c_str());
            outerDict = readBoundaryFile(node,outer_filename);
            unpackDictionary(node,outerDict,outercloudPCL, "r");
            RCLCPP_INFO(node->get_logger(), "Got %d points for the outer boundary.", outercloudPCL.size());

            std::string raceline_filename = deepracing_ros::FileUtils::findFile(track_name + "_racingline.json",search_dirs);
            RCLCPP_INFO(node->get_logger(), "Openning file %s.", raceline_filename.c_str());
            racelineDict = readRacelineFile(node,raceline_filename);
            unpackDictionary(node,racelineDict,racelinecloudPCL,"t");
            RCLCPP_INFO(node->get_logger(), "Got %d points for the optimal raceline.", racelinecloudPCL.size());

            pcl::PCLPointCloud2 innercloudPC2, outercloudPC2, racelinecloudPC2;
            pcl::toPCLPointCloud2(pcl::PointCloud<pcl::PointXYZI>(innercloudPCL), innercloudPC2);
            pcl::toPCLPointCloud2(pcl::PointCloud<pcl::PointXYZI>(outercloudPCL), outercloudPC2);
            pcl::toPCLPointCloud2(pcl::PointCloud<pcl::PointXYZI>(racelinecloudPCL), racelinecloudPC2);
            pcl_conversions::moveFromPCL(innercloudPC2, innercloudMSG);
            pcl_conversions::moveFromPCL(outercloudPC2, outercloudMSG);
            pcl_conversions::moveFromPCL(racelinecloudPC2, racelinecloudMSG);
        }
        innercloudMSG.header.frame_id = deepracing_ros::F1MsgUtils::world_coordinate_name; 
        outercloudMSG.set__header( innercloudMSG.header );
        racelinecloudMSG.set__header( innercloudMSG.header );

        innerpub->publish(innercloudMSG);
        outerpub->publish(outercloudMSG);
        racelinepub->publish(racelinecloudMSG);
        
    }


    


}