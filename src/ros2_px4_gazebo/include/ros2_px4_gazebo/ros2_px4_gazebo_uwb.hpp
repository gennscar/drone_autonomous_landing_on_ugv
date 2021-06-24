#ifndef ROS2_PX4_GAZEBO_UWB_HPP_
#define ROS2_PX4_GAZEBO_UWB_HPP_

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{

  class RosPx4GazeboUwbPrivate;

  class RosPx4GazeboUwb : public ModelPlugin
  {
  public:
    /// Constructor
    RosPx4GazeboUwb();

    /// Destructor
    virtual ~RosPx4GazeboUwb();

    // Documentation inherited
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

  private:
    /// Private data pointer
    std::unique_ptr<RosPx4GazeboUwbPrivate> impl_;
  };

} // namespace gazebo

#endif // ROS2_PX4_GAZEBO_UWB_HPP_
