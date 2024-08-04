#ifndef MESSAGE_CONVERTER_HPP
#define MESSAGE_CONVERTER_HPP

#define PCL_NO_PRECOMPILE

#include <cstring>
#include "MessageTypes.hpp"
#include "delphi_esr_msgs/msg/esr_track.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class MessageConverter
{
public:
  MessageConverter();

  RadarMessageT * getRadarMessage();

  LidarMessageT * getLidarMessage();

  CameraMessageT * getCameraMessage();
  
  CameraMessageT * getCameraRawMessage();

  CameraCalibrationT getCameraCalibration();

  OdometryMessageT getOdometryMessage();

  OdometryMessageT * getTfMessage();

  void getROSMsg(
    std::string type,
    std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> message_wrapper);
  void getROSMsg(std::string type, rosbag2_storage::SerializedBagMessageSharedPtr message);
  // Needed to batch Radar detections
  RadarMessageT * getLastRadarMessage();

private:
  delphi_esr_msgs::msg::EsrTrack radar_ros_msg_;
  sensor_msgs::msg::PointCloud2 lidar_ros_msg_;
  sensor_msgs::msg::CompressedImage camera_ros_msg_;
  sensor_msgs::msg::CameraInfo camera_info_ros_msg_;
  nav_msgs::msg::Odometry odometry_ros_msg_;
  tf2_msgs::msg::TFMessage tf_msg_;
  sensor_msgs::msg::Image camera_ros_raw_msg_;

  // Needed to batch Radar detections
  unsigned long first_radar_timestamp;
  RadarMessageT * radar_msg;
  // Serialization
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf2_serialization_;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> lidar_serialization_;
  rclcpp::Serialization<sensor_msgs::msg::Image> camera_serialization_;
  rclcpp::Serialization<sensor_msgs::msg::CameraInfo> camera_info_serialization_;

};


#endif
