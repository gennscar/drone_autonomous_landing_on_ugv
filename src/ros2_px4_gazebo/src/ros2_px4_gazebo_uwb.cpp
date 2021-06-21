#include <ros2_px4_gazebo/ros2_px4_gazebo_uwb.hpp>

#include <ignition/math/Rand.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo/transport/transport.hh>
#include <ros2_px4_interfaces/msg/uwb_sensor.hpp>

#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
  class RosPx4GazeboUwbPrivate
  {
  public:
    /// Callback to be called at every simulation iteration
    /// \param[in] info Updated simulation info
    void OnUpdate(const common::UpdateInfo &info);

    /// Callback to be called at every anchor message received by the sensor
    /// \param[in] msg Incoming anchor message
    void AnchorCallback(ConstPoseStampedPtr &_msg);

    /// Pointer to the link, model and world
    physics::LinkPtr link_{nullptr};
    physics::ModelPtr model_{nullptr};
    physics::WorldPtr world_{nullptr};

    /// Pose of the link
    ignition::math::Pose3d link_pose_;

    /// The reference model and link to which calculate the pose
    physics::ModelPtr reference_model_{nullptr};
    physics::LinkPtr reference_link_{nullptr};

    /// Gazebo node
    transport::NodePtr gazebo_node_{nullptr};

    /// Gazebo pub/sub to anchor broadcast
    transport::PublisherPtr anchor_pub_{nullptr};
    transport::SubscriberPtr anchor_sub_{nullptr};

    /// Pointer to ros node
    gazebo_ros::Node::SharedPtr ros_node_{nullptr};

    /// ROS publisher for sensor ranging data
    rclcpp::Publisher<ros2_px4_interfaces::msg::UwbSensor>::SharedPtr sensor_pub_{nullptr};

    // Topic names
    std::string anchor_topic_{"/uwb_anchors"};
    std::string sensor_topic_{"/uwb_sensor/"};

    /// Keep track of the last update time.
    common::Time last_time_;

    /// Publish rate in Hz.
    double update_rate_{0.0};

    /// Anchor unique ID
    std::string anchor_id_;

    /// Gaussian noise
    double gaussian_noise_;

    /// Pointer to the update event connection
    event::ConnectionPtr update_connection_{nullptr};
  };

  RosPx4GazeboUwb::RosPx4GazeboUwb()
      : impl_(std::make_unique<RosPx4GazeboUwbPrivate>())
  {
  }

  RosPx4GazeboUwb::~RosPx4GazeboUwb()
  {
    impl_->ros_node_.reset();
    if (impl_->gazebo_node_)
    {
      impl_->gazebo_node_->Fini();
    }
    impl_->gazebo_node_.reset();
  }

  // Load the plugin
  void RosPx4GazeboUwb::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    std::string link_name;
    std::string reference_model_name;
    std::string reference_link_name;

    // Get model, world and time
    impl_->model_ = model;
    impl_->world_ = impl_->model_->GetWorld();
    impl_->last_time_ = impl_->world_->SimTime();

    // Configure the Gazebo node
    impl_->gazebo_node_ = boost::make_shared<transport::Node>();
    impl_->gazebo_node_->Init(impl_->world_->Name());

    // Configure the ROS node from the SDF file
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

    // <update_rate> is the rate at which publish UWB packets
    if (!sdf->HasElement("update_rate"))
    {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
                  "UWB plugin missing <update_rate>, defaults to 0.0 (as fast as possible)");
    }
    else
    {
      impl_->update_rate_ = sdf->GetElement("update_rate")->Get<double>();
    }

    // <anchor_id> is an unique ID that represent the UWB tag
    if (!sdf->HasElement("anchor_id"))
    {
      impl_->anchor_id_ = std::to_string(rand());
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
                  "UWB plugin missing <anchor_id>, assigning random ID: %s",
                  impl_->anchor_id_.c_str());
    }
    else
    {
      impl_->anchor_id_ = sdf->GetElement("anchor_id")->Get<std::string>();
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

    // Setting up the publisher of the anchor pose
    impl_->anchor_pub_ = impl_->gazebo_node_->Advertise<msgs::PoseStamped>(impl_->anchor_topic_, 1);

    // This code is valid only if we want to publish ranging data
    if (sdf->HasElement("pub_range"))
    {
      // Setting up anchors pose subscriber
      impl_->anchor_sub_ = impl_->gazebo_node_->Subscribe(impl_->anchor_topic_, &RosPx4GazeboUwbPrivate::AnchorCallback, impl_.get());

      // Setting up ranges publisher
      impl_->sensor_pub_ = impl_->ros_node_->create_publisher<ros2_px4_interfaces::msg::UwbSensor>(
          impl_->sensor_topic_ + impl_->anchor_id_,
          impl_->ros_node_->get_qos().get_publisher_qos(impl_->sensor_topic_ + impl_->anchor_id_));

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
    impl_->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RosPx4GazeboUwbPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
  }

  void RosPx4GazeboUwbPrivate::OnUpdate(const common::UpdateInfo &info)
  {
    // Check if link is destroyed
    if (!link_)
    {
      return;
    }

    common::Time current_time = info.simTime;

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

    // Fill UWB message
    msgs::PoseStamped anchor_msg;
    msgs::Time *anchor_msg_time = new msgs::Time();
    msgs::Vector3d *anchor_msg_vec = new msgs::Vector3d();
    msgs::Quaternion *anchor_msg_qua = new msgs::Quaternion();
    msgs::Pose *anchor_msg_pose = new msgs::Pose();

    anchor_msg_time->set_sec(current_time.sec);
    anchor_msg_time->set_nsec(current_time.nsec);
    anchor_msg.set_allocated_time(anchor_msg_time);

    anchor_msg_vec->set_x(link_pose_.Pos().X());
    anchor_msg_vec->set_y(link_pose_.Pos().Y());
    anchor_msg_vec->set_z(link_pose_.Pos().Z());
    anchor_msg_pose->set_allocated_position(anchor_msg_vec);

    // Sending orientation for a future more complex noise model
    anchor_msg_qua->set_x(0.0);
    anchor_msg_qua->set_y(0.0);
    anchor_msg_qua->set_z(0.0);
    anchor_msg_qua->set_w(0.0);
    anchor_msg_pose->set_allocated_orientation(anchor_msg_qua);

    anchor_msg_pose->set_name(anchor_id_);
    anchor_msg.set_allocated_pose(anchor_msg_pose);

    anchor_pub_->Publish(anchor_msg);
    last_time_ = current_time;
  }

  void RosPx4GazeboUwbPrivate::AnchorCallback(ConstPoseStampedPtr &_msg)
  {
    double range;

    // Check if the sensor and the anchor are 2 separate entities
    if (_msg->pose().name() != anchor_id_)
    {
      // Calculate the range between sensor and anchor + gaussian noise
      range = link_pose_.Pos().Distance(_msg->pose().orientation().x(), _msg->pose().orientation().y(), _msg->pose().orientation().z());
      range += ignition::math::Rand::DblNormal(0, gaussian_noise_);

      // Fill ROS range message
      ros2_px4_interfaces::msg::UwbSensor sensor_msg;

      sensor_msg.anchor_pose.header.stamp.sec = _msg->time().sec(); // @todo: Simulate transmission delay anchor->sensor
      sensor_msg.anchor_pose.header.stamp.nanosec = _msg->time().nsec();

      sensor_msg.anchor_pose.header.frame_id = _msg->pose().name();

      sensor_msg.anchor_pose.pose.position.x = _msg->pose().position().x();
      sensor_msg.anchor_pose.pose.position.y = _msg->pose().position().y();
      sensor_msg.anchor_pose.pose.position.z = _msg->pose().position().z();

      sensor_msg.anchor_pose.pose.orientation.x = _msg->pose().orientation().x();
      sensor_msg.anchor_pose.pose.orientation.y = _msg->pose().orientation().y();
      sensor_msg.anchor_pose.pose.orientation.z = _msg->pose().orientation().z();
      sensor_msg.anchor_pose.pose.orientation.w = _msg->pose().orientation().w();

      sensor_msg.range = range;

      // Publish to ROS
      sensor_pub_->publish(sensor_msg);
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(RosPx4GazeboUwb)

} // namespace gazebo
