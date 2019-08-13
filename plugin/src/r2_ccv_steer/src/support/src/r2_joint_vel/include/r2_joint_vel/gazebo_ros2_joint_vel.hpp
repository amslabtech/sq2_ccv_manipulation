
#ifndef GAZEBO_PLUGINS__GAZEBO_ROS2_JOINT_VEL_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS2_JOINT_VEL_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRos2JointVelPrivate;
/// A velocity plugin for gazebo. Based for the developed of the velocity accuracy for a hinge robot.
/*
 *
 * \author  Pritish Tripathy Debasis (pritish.debasis96@gmail.com)
 *
 * $ Id: 07/18/2019 Pritish9 $
 */
/**
  Example Usage:
  \code{.xml}
    <plugin name="libgazebo_ros2_joint_vel" filename="libgazebo_ros2_joint_vel.so">

    <ros>

      <!-- Add a namespace -->
      <namespace>/vel_ctrl</namespace>
      
      <argument>cmd_angle:=angle</argument>
      <argument>cmd_vel:=vel</argument>
      <argument>cmd_vel_ang:=angle</argument>
      
      <argument>j_status:=joint_state</argument>
    
    </ros>

    <!-- joint -->
    <base_joint>first_joint</base_joint>
    
    <!-- kinematics -->
    <joint_length>1.5</joint_length>
    
    <!-- limits -->
    <max_joint_acceleration>1</max_joint_acceleration>
    <max_joint_torque>1</max_joint_torque>

    <!-- output -->
    <publish_angle>1</publish_angle>
    <joint_status>1</joint_status>
    <publish_joint_tf>1</publish_joint_tf>

    <!-- input vel type (0:from sdf & 1:from topic)-->
    <vel_type>0</vel_type>

    </plugin>
  \endcode
*/
class GazeboRos2JointVel : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRos2JointVel();

  /// Destructor
  ~GazeboRos2JointVel();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRos2JointVelPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS2_JOINT_VEL_HPP_