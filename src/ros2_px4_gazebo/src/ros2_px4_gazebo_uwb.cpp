// Copyright 2013 Open Source Robotics Foundation, Inc.
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

#include <ros2_px4_gazebo/ros2_px4_gazebo_uwb.hpp>

#include <ignition/math/Rand.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <ros2_px4_interfaces/msg/uwb_sensor.hpp>
//#include <ros2_px4_interfaces/msg/uwb_anchor.hpp>

#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
  class GazeboRosUwbPrivate
  {
  public:
    /// Callback to be called at every simulation iteration
    /// \param[in] info Updated simulation info
    void OnUpdate(const gazebo::common::UpdateInfo &info);

    /// Callback to be called at every anchor message received by the sensor
    /// \param[in] msg Incoming anchor message
    void AnchorCallback(const ros2_px4_interfaces::msg::UwbSensor::SharedPtr msg);

    /// Pointer to the link, model and world
    gazebo::physics::LinkPtr link_{nullptr};
    gazebo::physics::ModelPtr model_{nullptr};
    gazebo::physics::WorldPtr world_{nullptr};

    /// Pose of the link
    ignition::math::Pose3d link_pose_;

    /// The reference model and link to which calculate distances
    gazebo::physics::ModelPtr reference_model_{nullptr};
    gazebo::physics::LinkPtr reference_link_{nullptr};

    /// Pointer to ros node
    gazebo_ros::Node::SharedPtr ros_node_{nullptr};

    /// PubSub
    rclcpp::Publisher<ros2_px4_interfaces::msg::UwbSensor>::SharedPtr anchor_pub_{nullptr};
    rclcpp::Publisher<ros2_px4_interfaces::msg::UwbSensor>::SharedPtr sensor_pub_{nullptr};
    rclcpp::Subscription<ros2_px4_interfaces::msg::UwbSensor>::SharedPtr anchor_sub_{nullptr};

    // Topic names
    std::string anchor_topic_{"/uwb_anchors_broadcast"};
    std::string sensor_topic_{"/uwb_sensor_"};

    /// Keep track of the last update time.
    gazebo::common::Time last_time_;

    /// Publish rate in Hz.
    double update_rate_{0.0};

    /// Anchor unique ID
    uint64_t anchor_id_;

    /// Gaussian noise
    double gaussian_noise_;

    /// Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection_{nullptr};
  };

  GazeboRosUwb::GazeboRosUwb()
      : impl_(std::make_unique<GazeboRosUwbPrivate>())
  {
  }

  GazeboRosUwb::~GazeboRosUwb()
  {
  }

  // Load the plugin
  void GazeboRosUwb::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    std::string link_name;
    std::string reference_model_name;
    std::string reference_link_name;

    // Get model, world and time
    impl_->model_ = model;
    impl_->world_ = impl_->model_->GetWorld();
    impl_->last_time_ = impl_->world_->SimTime();

    // Configure the plugin from the SDF file
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

    // <update_rate> is the rate at which publish UWB packets
    if (!sdf->HasElement("update_rate"))
    {
      RCLCPP_DEBUG(impl_->ros_node_->get_logger(),
                   "UWB plugin missing <update_rate>, defaults to 0.0 (as fast as possible)");
    }
    else
    {
      impl_->update_rate_ = sdf->GetElement("update_rate")->Get<double>();
    }

    // <anchor_id> is an unique ID that represent the UWB tag
    if (!sdf->HasElement("anchor_id"))
    {
      impl_->anchor_id_ = rand();
      RCLCPP_DEBUG(impl_->ros_node_->get_logger(),
                   "UWB plugin missing <anchor_id>, assigning random ID: %lld",
                   (long long)impl_->anchor_id_);
    }
    else
    {
      impl_->anchor_id_ = sdf->GetElement("anchor_id")->Get<uint64_t>();
    }

    // <link_name> is the name of the link where the UWB tag is attached
    if (!sdf->HasElement("link_name"))
    {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Missing <link_name>, cannot proceed");
      return;
    }
    else
    {
      link_name = sdf->GetElement("link_name")->Get<std::string>();
    }

    // Checking if the link exists
    impl_->link_ = model->GetLink(link_name);
    if (!impl_->link_)
    {
      RCLCPP_ERROR(
          impl_->ros_node_->get_logger(), "link_name: %s does not exist\n",
          link_name.c_str());
      return;
    }

    // <reference_model_name> is the model to be taken as reference frame, defaults to world
    if (!sdf->HasElement("reference_model_name"))
    {
      RCLCPP_DEBUG(
          impl_->ros_node_->get_logger(), "Missing <reference_model_name>, defaults to world");
      reference_model_name = "world";
    }
    else
    {
      reference_model_name = sdf->GetElement("reference_model_name")->Get<std::string>();
    }

    // <reference_link_name> is the link of <reference_model_name> to be taken as reference frame
    if (!sdf->HasElement("reference_link_name"))
    {
      RCLCPP_DEBUG(
          impl_->ros_node_->get_logger(), "Missing <reference_link_name>, using model as reference");
    }
    else
    {
      reference_link_name = sdf->GetElement("reference_link_name")->Get<std::string>();
    }

    // Checking if custom reference frame exists
    if (reference_model_name != "/world" && reference_model_name != "world" &&
        reference_model_name != "/map" && reference_model_name != "map")
    {
      impl_->reference_model_ = impl_->world_->ModelByName(reference_model_name);
      if (!impl_->reference_model_)
      {
        RCLCPP_WARN(
            impl_->ros_node_->get_logger(), "<reference_model_name> [%s] does not exist.",
            reference_model_name.c_str());
      }

      impl_->reference_link_ = impl_->reference_model_->GetLink(reference_link_name);
      if (!impl_->reference_link_)
      {
        RCLCPP_WARN(
            impl_->ros_node_->get_logger(), "<reference_link_name> [%s] does not exist.",
            reference_link_name.c_str());
      }
    }

    // Setting up UWB tag position publisher
    impl_->anchor_pub_ = impl_->ros_node_->create_publisher<ros2_px4_interfaces::msg::UwbSensor>(impl_->anchor_topic_, 100);

    // This code is valid only for a Sensor UWB tag, that calculates the distances betweeen tags
    if (sdf->HasElement("is_sensor"))
    {
      // Setting up ranges publisher
      impl_->sensor_pub_ = impl_->ros_node_->create_publisher<ros2_px4_interfaces::msg::UwbSensor>(
          impl_->sensor_topic_ + std::to_string(impl_->anchor_id_), 100);

      // Setting up UWB tag position subscriber
      impl_->anchor_sub_ = impl_->ros_node_->create_subscription<ros2_px4_interfaces::msg::UwbSensor>(
          impl_->anchor_topic_, 100,
          std::bind(&GazeboRosUwbPrivate::AnchorCallback, impl_.get(), std::placeholders::_1));

      // <gaussian_noise> is the sigma value of gaussian noise to add to range readings
      if (!sdf->HasElement("gaussian_noise"))
      {
        RCLCPP_WARN(impl_->ros_node_->get_logger(), "Missing <gassian_noise>, defaults to 0.0");
        impl_->gaussian_noise_ = 0;
      }
      else
      {
        impl_->gaussian_noise_ = sdf->GetElement("gaussian_noise")->Get<double>();
      }
    }

    // Listen to the update event. This event is broadcast every simulation iteration
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosUwbPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
  }

  void GazeboRosUwbPrivate::OnUpdate(const gazebo::common::UpdateInfo &info)
  {
    // Check if link is destroyed
    if (!link_)
    {
      return;
    }

    gazebo::common::Time current_time = info.simTime;

    if (current_time < last_time_)
    {
      RCLCPP_WARN(ros_node_->get_logger(), "Negative update time difference detected.");
      last_time_ = current_time;
    }

    // Rate control
    if (update_rate_ > 0 &&
        (current_time - last_time_).Double() < (1.0 / update_rate_))
    {
      return;
    }

    // If we don't have any subscribers, don't bother composing and sending the message
    if (ros_node_->count_subscribers(anchor_topic_) == 0)
    {
      return;
    }

    // Get world pose of the linked model
    link_pose_ = link_->WorldPose();

    // Get relative pose of the linked model ...
    if (reference_model_)
    {
      // ... with the respect of the reference model ...
      auto reference_pose = reference_model_->WorldPose();
      if (reference_link_)
      {
        // or with the respect of the reference link
        reference_pose = reference_link_->WorldPose();
      }

      link_pose_.Pos() -= reference_pose.Pos();
      link_pose_.Pos() = reference_pose.Rot().RotateVectorReverse(link_pose_.Pos());
    }

    // Fill UWB tag message
    ros2_px4_interfaces::msg::UwbSensor anchor_msg;
    anchor_msg.timestamp = current_time.Double();
    anchor_msg.anchor_id = anchor_id_;
    anchor_msg.anchor_pos = gazebo_ros::Convert<geometry_msgs::msg::Point>(link_pose_.Pos());

    // Publish to ROS
    anchor_pub_->publish(anchor_msg);
    last_time_ = current_time;
  }

  void GazeboRosUwbPrivate::AnchorCallback(const ros2_px4_interfaces::msg::UwbSensor::SharedPtr msg)
  {
    double range;

    // Check if the sensor and the anchor are 2 separate entities
    if (msg->anchor_id != anchor_id_)
    {
      // Calculate the range between sensor and anchor
      ignition::math::Vector3d anchor_pos = gazebo_ros::Convert<ignition::math::Vector3d>(msg->anchor_pos);
      range = link_pose_.Pos().Distance(anchor_pos);

      // Fill UWB sensor message
      ros2_px4_interfaces::msg::UwbSensor sensor_msg;
      sensor_msg.timestamp = msg->timestamp;
      sensor_msg.anchor_id = msg->anchor_id;
      sensor_msg.anchor_pos = msg->anchor_pos;
      sensor_msg.range = range + ignition::math::Rand::DblNormal(0, gaussian_noise_);

      // Publish to ROS
      sensor_pub_->publish(sensor_msg);
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosUwb)

} // namespace gazebo
