#ifndef DEEPRACING_ROS_FILE_UTILS_H
#define DEEPRACING_ROS_FILE_UTILS_H
#include <vector>
#include <string>
#ifndef DEEPRACING_ROS_PUBLIC
    #define DEEPRACING_ROS_PUBLIC
#endif
namespace deepracing_ros
{
    class FileUtils
    {
        public:
            static inline DEEPRACING_ROS_PUBLIC char pathsep()
            {
                #ifdef _MSC_VER
                    return ';';
                #else
                    return ':';
                #endif
            }
            static DEEPRACING_ROS_PUBLIC std::string findFile(const std::string& file, const std::vector<std::string> & search_dirs);
            static DEEPRACING_ROS_PUBLIC std::vector<std::string> split(const std::string& str, char delim=pathsep());
    };
}

#endif