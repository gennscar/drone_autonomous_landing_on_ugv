// Copyright 2012 Open Source Robotics Foundation, Inc.
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_UWB_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_UWB_HPP_

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{

  class GazeboRosUwbPrivate;

  class GazeboRosUwb : public gazebo::ModelPlugin
  {
  public:
    /// Constructor
    GazeboRosUwb();

    /// Destructor
    virtual ~GazeboRosUwb();

    // Documentation inherited
    virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  private:
    /// Private data pointer
    std::unique_ptr<GazeboRosUwbPrivate> impl_;
  };

} // namespace gazebo_plugins

#endif // GAZEBO_PLUGINS__GAZEBO_ROS_UWB_HPP_
