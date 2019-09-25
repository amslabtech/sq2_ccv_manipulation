// Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_STEER_DRIVE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_STEER_DRIVE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosSteerDrivePrivate;

/// A steer-differential drive plugin for gazebo. Based on the steer-diff drive plugin
/*
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_steer_drive" filename="libgazebo_ros_steer_drive.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/test</namespace>

      </ros>

      <!-- Update rate in Hz -->
      <update_rate>50</update_rate>

      <!--Options-->
      <steer_type>0</steer_type>
      <joint_status>0</joint_status>
      <steer_status>0</steer_status>


      <!-- wheels -->
      <left_joint>link_1_JOINT_0</left_joint>
      <right_joint>link_1_clone_JOINT_1</right_joint>
      <left_steer>link_0_JOINT_0</left_steer>
      <right_steer>link_0_JOINT_1</right_steer>

      <!-- kinematics -->
      <wheel_separation>0.8</wheel_separation>
      <wheel_diameter>0.4</wheel_diameter>

      <!-- limits -->
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <max_wheel_torque>20.0</max_wheel_torque>
      <max_wheel_acceleration>5.0</max_wheel_acceleration>
      <max_steer_acceleration>5.0</max_steer_acceleration>
      <max_steer_torque>20.0</max_steer_torque>
      <max_steer_vel>0.5</max_steer_vel>

      <!-- input -->
      <command_topic>cmd_vel</command_topic>

      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>

      <!-- odom -->
      <robot_base_frame>chassis</robot_base_frame>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <odometry_frame>odom_demo</odometry_frame>

    </plugin>
  \endcode
*/
class GazeboRosSteerDrive : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosSteerDrive();

  /// Destructor
  ~GazeboRosSteerDrive();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosSteerDrivePrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_STEER_DRIVE_HPP_
