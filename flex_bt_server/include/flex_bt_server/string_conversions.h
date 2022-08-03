/*******************************************************************************
 *  Copyright (c) 2022
 *  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 *  Christopher Newport University
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *       POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef FLEX_BT_SERVER__STRING_CONVERSIONS_H_
#define FLEX_BT_SERVER__STRING_CONVERSIONS_H_

#include <iostream>
#include <string>
#include <math.h>

#include "rclcpp/time.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace BT
{

  /**
   * @brief Parse XML string to std_msgs::msg::Header
   * @param key XML string
   * @return std_msgs::msg::Header
   */
  template<>
  inline std_msgs::msg::Header convertFromString(const StringView key)
  {
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 2) {
      throw std::runtime_error("invalid number of fields for header attribute");
    } else {
      std_msgs::msg::Header header;
      header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[0]));
      header.frame_id = BT::convertFromString<std::string>(parts[1]);
      return header;
    }
  }

  template <typename T> inline
  T convertToString(StringView)
  {
      auto type_name = BT::demangle( typeid(T) );

      std::cerr << "You (maybe indirectly) called BT::convertFromString() for type [" <<
                   type_name <<"], but I can't find the template specialization.\n" << std::endl;

      throw LogicError(std::string("You didn't implement the template specialization of "
                                   "convertFromString for this type: ") + type_name );
  }

  /**
   * @brief Convert std_msgs::msg::Header to std::string
   * @param header std_msgs::msg::Header
   * @return std::string
   */
  inline std::string convertHeaderToString(std_msgs::msg::Header header)
  {
    std::string conversion = "";
    conversion = conversion + "stamp:" + "" + ";";
    conversion = conversion + "frame_id:" + header.frame_id + ";";
    return conversion;
  }

  // The follow templates are required when using these types as parameters
  // in our BT XML files. They parse the strings in the XML into their corresponding
  // data type.

  /**
   * @brief Parse XML string to geometry_msgs::msg::Point
   * @param key XML string
   * @return geometry_msgs::msg::Point
   */
  template<>
  inline geometry_msgs::msg::Point convertFromString(const StringView key)
  {
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 3) {
      throw std::runtime_error("invalid number of fields for point attribute)");
    } else {
      geometry_msgs::msg::Point position;
      position.x = BT::convertFromString<double>(parts[0]);
      position.y = BT::convertFromString<double>(parts[1]);
      position.z = BT::convertFromString<double>(parts[2]);
      return position;
    }
  }

  /**
   * @brief Convert geometry_msgs::msg::Point to std::string
   * @param position geometry_msgs::msg::Point
   * @return std::string
   */
  inline std::string convertPointToString(geometry_msgs::msg::Point position)
  {
    std::string conversion = "x:" + std::to_string(position.x) + ";" +
                             "y:" + std::to_string(position.y) + ";" +
                             "z:" + std::to_string(position.z) + ";";
    return conversion;
  }

  /**
   * @brief Parse XML string to geometry_msgs::msg::PointStamped
   * @param key XML string
   * @return geometry_msgs::msg::PointStamped
   */
  template<>
  inline geometry_msgs::msg::PointStamped convertFromString(const StringView key)
  {
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 5) {
      throw std::runtime_error("invalid number of fields for PointStamped attribute)");
    } else {
      geometry_msgs::msg::PointStamped point_stamped;
      point_stamped.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[0]));
      point_stamped.header.frame_id = BT::convertFromString<std::string>(parts[1]);
      point_stamped.point.x = BT::convertFromString<double>(parts[2]);
      point_stamped.point.y = BT::convertFromString<double>(parts[3]);
      point_stamped.point.z = BT::convertFromString<double>(parts[4]);
      return point_stamped;
    }
  }

  /**
   * @brief Convert geometry_msgs::msg::PointStamped to std::string
   * @param point_stamped geometry_msgs::msg::PointStamped
   * @return std::string
   */
  inline std::string convertPointStampedToString(geometry_msgs::msg::PointStamped point_stamped)
  {
    std::string conversion = "";
    conversion = conversion + convertHeaderToString(point_stamped.header) + ";";
    conversion = conversion + convertPointToString(point_stamped.point)+ ";";
    return conversion;
  }

  /**
   * @brief Parse XML string to geometry_msgs::msg::Quaternion
   * @param key XML string
   * @return geometry_msgs::msg::Quaternion
   */
  template<>
  inline geometry_msgs::msg::Quaternion convertFromString(const StringView key)
  {
    // four real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 4) {
      throw std::runtime_error("invalid number of fields for orientation attribute)");
    } else {
      geometry_msgs::msg::Quaternion orientation;
      orientation.x = BT::convertFromString<double>(parts[0]);
      orientation.y = BT::convertFromString<double>(parts[1]);
      orientation.z = BT::convertFromString<double>(parts[2]);
      orientation.w = BT::convertFromString<double>(parts[3]);
      return orientation;
    }
  }

  /**
   * @brief Convert geometry_msgs::msg::Quaternion to std::string
   * @param orientation geometry_msgs::msg::Quaternion
   * @return std::string
   */
  inline std::string convertQuaternionToString(geometry_msgs::msg::Quaternion orientation)
  {
    std::string conversion = "x:" + std::to_string(orientation.x) + ";" +
                             "y:" + std::to_string(orientation.y) + ";" +
                             "z:" + std::to_string(orientation.z) + ";" +
                             "w:" + std::to_string(orientation.w) + ";";
    return conversion;
  }

  /**
   * @brief Parse XML string to geometry_msgs::msg::PoseStamped
   * @param key XML string
   * @return geometry_msgs::msg::PoseStamped
   */
  template<>
  inline geometry_msgs::msg::PoseStamped convertFromString(const StringView key)
  {
    // 7 real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 9) {
      throw std::runtime_error("invalid number of fields for PoseStamped attribute");
    } else {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[0]));
      pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[1]);
      pose_stamped.pose.position.x = BT::convertFromString<double>(parts[2]);
      pose_stamped.pose.position.y = BT::convertFromString<double>(parts[3]);
      pose_stamped.pose.position.z = BT::convertFromString<double>(parts[4]);
      pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[5]);
      pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[6]);
      pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[7]);
      pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[8]);
      return pose_stamped;
    }
  }

  /**
   * @brief Convert geometry_msgs::msg::PoseStamped to std::string
   * @param pose_stamped geometry_msgs::msg::PoseStamped
   * @return std::string
   */
  inline std::string convertPoseStampedToString(geometry_msgs::msg::PoseStamped pose_stamped)
  {
    std::string conversion = "";
    conversion = conversion + convertHeaderToString(pose_stamped.header) + ";";
    conversion = conversion + convertPointToString(pose_stamped.pose.position) + ";";
    conversion = conversion + convertQuaternionToString(pose_stamped.pose.orientation) + ";";
    return conversion;
  }

  /**
   * @brief Parse XML string to geometry_msgs::msg::TwistStamped
   * @param key XML string
   * @return geometry_msgs::msg::TwistStamped
   */
  template<>
  inline geometry_msgs::msg::TwistStamped convertFromString(const StringView key)
  {
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 8) {
      throw std::runtime_error("invalid number of fields for TwistStamped attribute)");
    } else {
      geometry_msgs::msg::TwistStamped twist_stamped;
      twist_stamped.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[0]));
      twist_stamped.header.frame_id = BT::convertFromString<std::string>(parts[1]);
      twist_stamped.twist.linear.x = BT::convertFromString<double>(parts[2]);
      twist_stamped.twist.linear.y = BT::convertFromString<double>(parts[3]);
      twist_stamped.twist.linear.z = BT::convertFromString<double>(parts[4]);
      twist_stamped.twist.angular.x = BT::convertFromString<double>(parts[5]);
      twist_stamped.twist.angular.y = BT::convertFromString<double>(parts[6]);
      twist_stamped.twist.angular.z = BT::convertFromString<double>(parts[7]);
      return twist_stamped;
    }
  }

  /**
   * @brief Convert geometry_msgs::msg::TwistStamped to std::string
   * @param twist_stamped geometry_msgs::msg::TwistStamped
   * @return std::string
   */
  inline std::string convertTwistStampedToString(geometry_msgs::msg::TwistStamped twist_stamped)
  {
    std::string conversion = "";
    conversion = conversion + convertHeaderToString(twist_stamped.header) + ";";

    conversion = conversion +
      "x:" + std::to_string(twist_stamped.twist.linear.x) + ";" +
      "y:" + std::to_string(twist_stamped.twist.linear.y) + ";" +
      "z:" + std::to_string(twist_stamped.twist.linear.z) + ";;";

    conversion = conversion +
      "x:" + std::to_string(twist_stamped.twist.angular.x) + ";" +
      "y:" + std::to_string(twist_stamped.twist.angular.y) + ";" +
      "z:" + std::to_string(twist_stamped.twist.angular.z) + ";;";

    return conversion;
  }

  /**
   * @brief Parse XML string to std::chrono::milliseconds
   * @param key XML string
   * @return std::chrono::milliseconds
   */
  template<>
  inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(const StringView key)
  {
    return std::chrono::milliseconds(std::stoul(key.data()));
  }
}

#endif
