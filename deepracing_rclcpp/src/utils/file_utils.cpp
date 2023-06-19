#include "deepracing_ros/utils/file_utils.h"
#ifdef DEEPRACING_RCLCPP_USE_BOOST_FILESYSTEM
  #include <boost/filesystem.hpp>
  namespace fs=boost::filesystem;
#else
  #include <filesystem>
  namespace fs=std::filesystem;
#endif
#include <algorithm>
#include <sstream>
std::string deepracing_ros::FileUtils::findTrackmap(const std::string& trackname, const std::vector<std::string> & search_dirs)
{
  for(const std::string& dir : search_dirs)
  {
    fs::path dirpath(dir);
    fs::path checkpath = dirpath/fs::path(trackname);
    fs::path metadata_checkpath = checkpath/fs::path("metadata.yaml");
    if(fs::exists(checkpath) && fs::is_directory(checkpath) && fs::exists(metadata_checkpath) && fs::is_regular_file(metadata_checkpath))
    {
      return checkpath.string();
    }
  }
  return std::string("");
}
std::string  deepracing_ros::FileUtils::findFile(const std::string& file, const std::vector<std::string> & search_dirs)
{
  const fs::path filepath(file);
  for(const std::string& dir : search_dirs)
  {
    fs::path dirpath(dir);
    fs::path checkpath = dirpath/filepath;
    if(fs::exists(checkpath) && fs::is_regular_file(checkpath))
    {
      return checkpath.string();
    }
  }
  
  return std::string("");
}

std::vector<std::string> deepracing_ros::FileUtils::split(const std::string& str, char delim)
{
    if (str.empty())
    {
      return std::vector<std::string>();
    }
    std::vector<std::string> cont;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delim)) {
        cont.push_back(token);
    }
    return cont;
}
