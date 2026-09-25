// Copyright(c) 2025 AutoCore Technology (Nanjing) Co., Ltd. All rights reserved.
//
// Copyright 2025 TIER IV, Inc.
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

#include "detected_to_predicted_objects_converter.hpp"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
geometry_msgs::msg::Twist transform_twist(
  const geometry_msgs::msg::Twist & twist, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Twist transformed_twist = twist;

  tf2::Quaternion rotation;
  tf2::fromMsg(transform.transform.rotation, rotation);

  const tf2::Vector3 linear(twist.linear.x, twist.linear.y, twist.linear.z);
  const tf2::Vector3 angular(twist.angular.x, twist.angular.y, twist.angular.z);

  const tf2::Vector3 transformed_linear = tf2::quatRotate(rotation, linear);
  const tf2::Vector3 transformed_angular = tf2::quatRotate(rotation, angular);

  transformed_twist.linear.x = transformed_linear.x();
  transformed_twist.linear.y = transformed_linear.y();
  transformed_twist.linear.z = transformed_linear.z();
  transformed_twist.angular.x = transformed_angular.x();
  transformed_twist.angular.y = transformed_angular.y();
  transformed_twist.angular.z = transformed_angular.z();

  return transformed_twist;
}
}  // namespace

namespace autoware::perception_objects_converter
{
DetectedToPredictedObjectsConverter::DetectedToPredictedObjectsConverter(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("detected_to_predicted_objects_converter", options)
{
  output_frame_ = declare_parameter<std::string>("output_frame", "map");
  transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.2);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  detected_objects_sub_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "input/detected_objects", rclcpp::QoS{10},
    std::bind(
      &DetectedToPredictedObjectsConverter::detected_objects_callback, this,
      std::placeholders::_1));

  predicted_objects_pub_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
    "output/predicted_objects", rclcpp::QoS{10});
}

// Convert Boost UUID to unique_identifier_msgs::msg::UUID
unique_identifier_msgs::msg::UUID generateUUIDMsg()
{
  boost::uuids::random_generator gen;
  boost::uuids::uuid uuid = gen();

  unique_identifier_msgs::msg::UUID uuid_msg;
  std::copy(uuid.begin(), uuid.end(), uuid_msg.uuid.begin());

  return uuid_msg;
}

void DetectedToPredictedObjectsConverter::detected_objects_callback(
  const autoware_perception_msgs::msg::DetectedObjects::SharedPtr detected_objects_msg)
{
  auto predicted_objects_msg = std::make_unique<autoware_perception_msgs::msg::PredictedObjects>();
  const auto & source_frame = detected_objects_msg->header.frame_id;
  const bool needs_transform = !source_frame.empty() && source_frame != output_frame_;

  // Copy header
  predicted_objects_msg->header = detected_objects_msg->header;
  predicted_objects_msg->header.frame_id = output_frame_;

  geometry_msgs::msg::TransformStamped transform_stamped;
  if (needs_transform) {
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        output_frame_, source_frame, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform detected objects from %s to %s: %s", source_frame.c_str(),
        output_frame_.c_str(), ex.what());
      return;
    }
  }

  // Convert each detected object to predicted object
  for (const auto & detected_object : detected_objects_msg->objects) {
    autoware_perception_msgs::msg::PredictedObject predicted_object;

    // Generate UUID for the object using Boost
    predicted_object.object_id = generateUUIDMsg();

    // Copy fields from detected object
    predicted_object.existence_probability = detected_object.existence_probability;
    predicted_object.classification = detected_object.classification;
    predicted_object.shape = detected_object.shape;

    // Convert kinematics
    autoware_perception_msgs::msg::PredictedObjectKinematics predicted_kinematics;
    predicted_kinematics.initial_pose_with_covariance =
      detected_object.kinematics.pose_with_covariance;
    if (needs_transform) {
      geometry_msgs::msg::PoseStamped pose_in;
      geometry_msgs::msg::PoseStamped pose_out;
      pose_in.header = detected_objects_msg->header;
      pose_in.pose = detected_object.kinematics.pose_with_covariance.pose;
      tf2::doTransform(pose_in, pose_out, transform_stamped);
      predicted_kinematics.initial_pose_with_covariance.pose = pose_out.pose;
    }

    if (detected_object.kinematics.has_twist) {
      predicted_kinematics.initial_twist_with_covariance =
        detected_object.kinematics.twist_with_covariance;
      if (needs_transform) {
        predicted_kinematics.initial_twist_with_covariance.twist = transform_twist(
          detected_object.kinematics.twist_with_covariance.twist, transform_stamped);
      }
    }

    // Note: Acceleration and predicted paths would typically be empty or set to default values
    // as they are not available in the DetectedObject message

    predicted_object.kinematics = predicted_kinematics;

    // Add to objects array
    predicted_objects_msg->objects.push_back(predicted_object);
  }

  // Publish the converted message
  predicted_objects_pub_->publish(*predicted_objects_msg);
}
}  // namespace autoware::perception_objects_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception_objects_converter::DetectedToPredictedObjectsConverter)
