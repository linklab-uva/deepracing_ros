#include <deepracing_ros/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <deepracing/track_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <deepracing_msgs/msg/timestamped_packet_session_data.hpp>
#include <deepracing_msgs/srv/get_line.hpp>
#include <mutex>
#include <deepracing/utils.hpp>


namespace deepracing
{
namespace composable_nodes
{

    class TrackmapPublisher : public rclcpp::Node
    {
        public:
            DEEPRACING_RCLCPP_PUBLIC TrackmapPublisher(const rclcpp::NodeOptions & options) : 
                rclcpp::Node("trackmap_publisher", options)
            {
                map_to_track_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
                rclcpp::QoS qos = rclcpp::SystemDefaultsQoS().keep_last(10).durability_volatile();
                search_dirs_ = declare_parameter<std::vector<std::string>>("search_dirs");
                
                get_line_srv_ = create_service<deepracing_msgs::srv::GetLine>("get_line", 
                    std::bind(&TrackmapPublisher::getline_cb, this, std::placeholders::_1, std::placeholders::_2));
                session_sub_ = create_subscription<deepracing_msgs::msg::TimestampedPacketSessionData>("session_data", qos,
                    std::bind(&TrackmapPublisher::session_cb, this, std::placeholders::_1));
                ib_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inner_boundary", qos);
                ob_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("outer_boundary", qos);
                raceline_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("optimal_raceline", qos);
                timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0), 
                    std::bind(&TrackmapPublisher::timer_cb, this));
                
            }   
        private:
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> map_to_track_broadcaster_;
            rclcpp::TimerBase::SharedPtr timer_;
            deepracing::TrackMap::Ptr track_map_;
            std::vector<std::string> search_dirs_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ib_pub_, ob_pub_, raceline_pub_, widthmap_pub_;
            rclcpp::Subscription<deepracing_msgs::msg::TimestampedPacketSessionData>::SharedPtr session_sub_;
            rclcpp::Service<deepracing_msgs::srv::GetLine>::SharedPtr get_line_srv_;
            std::mutex mutex_;
            void getline_cb(const deepracing_msgs::srv::GetLine::Request::SharedPtr request,
                deepracing_msgs::srv::GetLine::Response::SharedPtr response)
            {
                std::scoped_lock lock(mutex_);
                if(!track_map_)
                {
                    response->return_code=deepracing_msgs::srv::GetLine::Response::TRACKMAP_NOT_INITIALIZED;
                    return;
                }
                try{
                    const pcl::PCLPointCloud2& pc2 = track_map_->getCloud(request->key.data);
                    pcl_conversions::fromPCL(pc2, response->line);
                }catch(std::out_of_range& e){
                    response->return_code=deepracing_msgs::srv::GetLine::Response::KEY_NOT_FOUND;
                    return;
                }catch(std::exception& e){
                    response->return_code=deepracing_msgs::srv::GetLine::Response::UNKNOWN;
                    return;
                }
                response->return_code=deepracing_msgs::srv::GetLine::Response::SUCCESS;
            }
            // void session_cb(const deepracing_msgs::msg::TimestampedPacketSessionData::ConstPtr& session_data)
            void session_cb(deepracing_msgs::msg::TimestampedPacketSessionData::UniquePtr session_data)
            {
                if((session_data->udp_packet.track_id<0) || (session_data->udp_packet.track_id>34)){
                    return;
                }
                std::scoped_lock lock(mutex_);
                std::map<std::int8_t, std::string> names_map = deepracing::Utils::trackNames();
                std::string tracknamein = names_map.at(session_data->udp_packet.track_id);
                if(!track_map_ || track_map_->name()!=tracknamein)
                {
                    track_map_.reset();
                    track_map_ = deepracing::TrackMap::findTrackmap(tracknamein, search_dirs_);
                    if(track_map_){
                        RCLCPP_INFO(get_logger(), "Got map for track name %s", track_map_->name().c_str());
                        if(track_map_->widthMap() && !(widthmap_pub_))
                        {
                            widthmap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("width_map", rclcpp::SystemDefaultsQoS().keep_last(10).durability_volatile());
                        }
                        else if(!(track_map_->widthMap()) && widthmap_pub_)
                        {   
                            widthmap_pub_.reset();
                        }
                    }
                    else{
                        RCLCPP_FATAL(get_logger(), "%s", ":(");
                    }
                }

            }
            void timer_cb()
            {
                // RCLCPP_INFO(get_logger(), "%s", ":)");
                std::scoped_lock lock(mutex_);
                if(!track_map_)
                {
                    return;
                }
                const rclcpp::Time& now = get_clock()->now();
                geometry_msgs::msg::TransformStamped map_to_track_msg = tf2::eigenToTransform(track_map_->startinglinePose().inverse());
                map_to_track_msg.header.set__stamp(now);
                map_to_track_msg.header.frame_id="map";
                map_to_track_msg.child_frame_id="track";
                map_to_track_broadcaster_->sendTransform(map_to_track_msg);
                sensor_msgs::msg::PointCloud2 innerbound_msg, outerbound_msg, racline_msg, asdf;
                pcl::toROSMsg<deepracing::PointXYZLapdistance>(*track_map_->innerBound(), innerbound_msg);
                pcl::toROSMsg<deepracing::PointXYZLapdistance>(*track_map_->outerBound(), outerbound_msg);
                pcl::toROSMsg<deepracing::PointXYZTime>(*track_map_->raceline(), racline_msg);
                innerbound_msg.header.set__stamp(now);
                racline_msg.header.set__stamp(now);
                outerbound_msg.header.set__stamp(now);
                ib_pub_->publish(innerbound_msg);
                ob_pub_->publish(outerbound_msg);
                raceline_pub_->publish(racline_msg);
                const pcl::PointCloud<deepracing::PointWidthMap>::ConstPtr width_map = track_map_->widthMap();
                if(width_map && widthmap_pub_){
                    sensor_msgs::msg::PointCloud2 widthmap_msg;
                    pcl::toROSMsg<deepracing::PointWidthMap>(*width_map, widthmap_msg);
                    widthmap_msg.header.set__stamp(now);
                    widthmap_pub_->publish(widthmap_msg);
                }
            }

            
    };
}
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(deepracing::composable_nodes::TrackmapPublisher)
