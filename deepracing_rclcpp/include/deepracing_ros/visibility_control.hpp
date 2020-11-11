// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef DEEPRACING_RCLCPP__VISIBILITY_CONTROL_HPP_
#define DEEPRACING_RCLCPP__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DEEPRACING_RCLCPP_EXPORT __attribute__ ((dllexport))
    #define DEEPRACING_RCLCPP_IMPORT __attribute__ ((dllimport))
  #else
    #define DEEPRACING_RCLCPP_EXPORT __declspec(dllexport)
    #define DEEPRACING_RCLCPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef DEEPRACING_RCLCPP_BUILDING_LIBRARY
    #define DEEPRACING_RCLCPP_PUBLIC DEEPRACING_RCLCPP_EXPORT
  #else
    #define DEEPRACING_RCLCPP_PUBLIC DEEPRACING_RCLCPP_IMPORT
  #endif
  #define DEEPRACING_RCLCPP_PUBLIC_TYPE DEEPRACING_RCLCPP_PUBLIC
  #define DEEPRACING_RCLCPP_LOCAL
#else
  #define DEEPRACING_RCLCPP_EXPORT __attribute__ ((visibility("default")))
  #define DEEPRACING_RCLCPP_IMPORT
  #if __GNUC__ >= 4
    #define DEEPRACING_RCLCPP_PUBLIC __attribute__ ((visibility("default")))
    #define DEEPRACING_RCLCPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DEEPRACING_RCLCPP_PUBLIC
    #define DEEPRACING_RCLCPP_LOCAL
  #endif
  #define DEEPRACING_RCLCPP_PUBLIC_TYPE
#endif

#endif  // DEEPRACING_RCLCPP__VISIBILITY_CONTROL_HPP_
