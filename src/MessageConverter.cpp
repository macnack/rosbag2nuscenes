#include "rosbag2nuscenes/MessageConverter.hpp"


MessageConverter::MessageConverter()
{
  first_radar_timestamp = 0;
}

RadarMessageT * MessageConverter::getRadarMessage()
{
  std::stringstream ss;
  ss << radar_ros_msg_.header.stamp.sec << radar_ros_msg_.header.stamp.nanosec;
  RadarPointT radar_point;
  radar_point.x = (float) (radar_ros_msg_.range * cos(radar_ros_msg_.angle * 3.14159 / 180));
  radar_point.y = (float) (radar_ros_msg_.range * sin(radar_ros_msg_.angle * 3.14159 / 180));
  radar_point.z = 0.0;
  radar_point.dyn_prop = (int8_t) 4;
  radar_point.id = (int16_t) radar_ros_msg_.id;
  radar_point.rcs = radar_ros_msg_.width;
  radar_point.vx = radar_ros_msg_.range_rate;
  radar_point.vy = radar_ros_msg_.lat_rate;
  radar_point.vx_comp = radar_ros_msg_.range_rate;
  radar_point.vy_comp = radar_ros_msg_.lat_rate;
  radar_point.is_quality_valid = 1;
  radar_point.ambig_state = 3;
  radar_point.x_rms = 0;
  radar_point.y_rms = 0;
  radar_point.invalid_state = 0;
  radar_point.pdh0 = 1;
  radar_point.vx_rms = 0;
  radar_point.vy_rms = 0;
  unsigned long message_timestamp = stoul(ss.str());
  if (!first_radar_timestamp) {
    radar_msg = new RadarMessageT();
    first_radar_timestamp = message_timestamp;
    radar_msg->timestamp = message_timestamp;
    radar_msg->frame_id = radar_ros_msg_.header.frame_id;
  }
  // If timestamp is within threshold for batch, add point to cloud and return nullptr
  if (message_timestamp - first_radar_timestamp < 10000000) {
    radar_msg->cloud.push_back(radar_point);
    return nullptr;
    // Else return the radar message and start a new batch
  } else {
    radar_msg->cloud.push_back(RadarPointT{});     // Nuscenes devkit expects binary data to have buffer after valid data
    radar_msg->cloud.width -= 1;
    RadarMessageT * return_msg = radar_msg;
    radar_msg = new RadarMessageT();
    radar_msg->timestamp = message_timestamp;
    radar_msg->frame_id = radar_ros_msg_.header.frame_id;
    radar_msg->cloud.push_back(radar_point);
    first_radar_timestamp = message_timestamp;
    return return_msg;
  }
}

LidarMessageT * MessageConverter::getLidarMessage()
{
  LidarMessageT * lidar_msg = new LidarMessageT();
  lidar_msg->frame_id = lidar_ros_msg_.header.frame_id;
  lidar_msg->timestamp = static_cast<uint64_t>(lidar_ros_msg_.header.stamp.sec) * 1000000000ULL +
    lidar_ros_msg_.header.stamp.nanosec;
  lidar_msg->timestamp = lidar_msg->timestamp / 1000;
  pcl::fromROSMsg(lidar_ros_msg_, lidar_msg->cloud);
  return lidar_msg;
}

CameraMessageT * MessageConverter::getCameraMessage()
{
  CameraMessageT * camera_msg = new CameraMessageT();
  camera_msg->frame_id = camera_ros_msg_.header.frame_id;
  camera_msg->timestamp = static_cast<uint64_t>(camera_ros_msg_.header.stamp.sec) * 1000000000ULL +
    camera_ros_msg_.header.stamp.nanosec;
  camera_msg->timestamp = camera_msg->timestamp / 1000;
  camera_msg->image = cv_bridge::toCvCopy(camera_ros_msg_, "rgb8")->image;
  return camera_msg;
}

CameraMessageT * MessageConverter::getCameraRawMessage()
{
  CameraMessageT * camera_msg = new CameraMessageT();
  camera_msg->frame_id = camera_ros_raw_msg_.header.frame_id;
  camera_msg->timestamp = static_cast<uint64_t>(camera_ros_raw_msg_.header.stamp.sec) *
    1000000000ULL + camera_ros_raw_msg_.header.stamp.nanosec;
  camera_msg->timestamp = camera_msg->timestamp / 1000;
  camera_msg->image = cv_bridge::toCvCopy(camera_ros_raw_msg_, "rgb8")->image;
  return camera_msg;
}

CameraCalibrationT MessageConverter::getCameraCalibration()
{
  CameraCalibrationT calibration_msg;
  calibration_msg.frame_id = camera_info_ros_msg_.header.frame_id;
  for (int i = 0; i < 3; i++) {
    std::vector<float> intrinsic_row;
    for (int j = 0; j < 3; j++) {
      intrinsic_row.push_back(camera_info_ros_msg_.k[3 * i + j]);
    }
    calibration_msg.intrinsic.push_back(intrinsic_row);
  }
  return calibration_msg;
}

OdometryMessageT MessageConverter::getOdometryMessage()
{
  OdometryMessageT odometry_msg;
  odometry_msg.timestamp = static_cast<uint64_t>(odometry_ros_msg_.header.stamp.sec) *
    1000000000ULL + odometry_ros_msg_.header.stamp.nanosec;
  odometry_msg.timestamp = odometry_msg.timestamp / 1000;
  odometry_msg.position =
  {odometry_ros_msg_.pose.pose.position.x, odometry_ros_msg_.pose.pose.position.y,
    odometry_ros_msg_.pose.pose.position.z};
  odometry_msg.orientation =
  {odometry_ros_msg_.pose.pose.orientation.w, odometry_ros_msg_.pose.pose.orientation.x,
    odometry_ros_msg_.pose.pose.orientation.y, odometry_ros_msg_.pose.pose.orientation.z};
  return odometry_msg;
}

OdometryMessageT * MessageConverter::getTfMessage()
{
  OdometryMessageT * odometry_msg = new OdometryMessageT();
  auto it = std::find_if(
    tf_msg_.transforms.begin(), tf_msg_.transforms.end(),
    [](const geometry_msgs::msg::TransformStamped & tf) {
      return tf.child_frame_id == "base_link";
    });
  if (it == tf_msg_.transforms.end()) {
    return nullptr;
  }
  odometry_msg->timestamp = static_cast<uint64_t>(it->header.stamp.sec) *
    1000000000ULL + it->header.stamp.nanosec;
  odometry_msg->timestamp = odometry_msg->timestamp / 1000;
  odometry_msg->position = {it->transform.translation.x, it->transform.translation.y,
    it->transform.translation.z};
  odometry_msg->orientation = {it->transform.rotation.w, it->transform.rotation.x,
    it->transform.rotation.y, it->transform.rotation.z};
  return odometry_msg;
}

void MessageConverter::getROSMsg(
  std::string type,
  std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> message_wrapper)
{
  if (type == "delphi_esr_msgs/msg/EsrTrack") {
    message_wrapper->message = &radar_ros_msg_;
  } else if (type == "sensor_msgs/msg/CompressedImage") {
    message_wrapper->message = &camera_ros_msg_;
  } else if (type == "sensor_msgs/msg/PointCloud2") {
    message_wrapper->message = &lidar_ros_msg_;
  } else if (type == "nav_msgs/msg/Odometry") {
    message_wrapper->message = &odometry_ros_msg_;
  } else if (type == "sensor_msgs/msg/CameraInfo") {
    message_wrapper->message = &camera_info_ros_msg_;
  } else {
    printf("Message type unknown, cannot deserialize.\n");
    exit(1);
  }
}

void MessageConverter::getROSMsg(
  std::string type,
  rosbag2_storage::SerializedBagMessageSharedPtr message)
{
  rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
  if (type == "tf2_msgs/msg/TFMessage") {
    tf2_serialization_.deserialize_message(&serialized_msg, &tf_msg_);
  } else if (type == "sensor_msgs/msg/PointCloud2") {
    lidar_serialization_.deserialize_message(&serialized_msg, &lidar_ros_msg_);
  } else if (type == "sensor_msgs/msg/Image") {
    camera_serialization_.deserialize_message(&serialized_msg, &camera_ros_raw_msg_);
  } else if (type == "sensor_msgs/msg/CameraInfo") {
    camera_info_serialization_.deserialize_message(&serialized_msg, &camera_info_ros_msg_);
  } else {
    printf("Message type unknown, cannot deserialize.\n");
    exit(1);
  }
}

RadarMessageT * MessageConverter::getLastRadarMessage()
{
  return radar_msg;
}
