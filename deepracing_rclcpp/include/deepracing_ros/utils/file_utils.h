#ifndef DEEPRACING_ROS_FILE_UTILS_H
#define DEEPRACING_ROS_FILE_UTILS_H
#include <deepracing_ros/utils/visibility_control.hpp>
#include <vector>
#include <string>


namespace deepracing_ros
{
    class FileUtils
    {
        public:
            static inline DEEPRACING_RCLCPP_UTILS_PUBLIC char pathsep()
            {
                #ifdef _MSC_VER
                    return ';';
                #else
                    return ':';
                #endif
            }
            static DEEPRACING_RCLCPP_UTILS_PUBLIC std::string findFile(const std::string& file, const std::vector<std::string> & search_dirs);
            static DEEPRACING_RCLCPP_UTILS_PUBLIC std::string findTrackmap(const std::string& trackname, const std::vector<std::string> & search_dirs);
            static DEEPRACING_RCLCPP_UTILS_PUBLIC std::vector<std::string> split(const std::string& str, char delim=pathsep());
    };
}

#endif