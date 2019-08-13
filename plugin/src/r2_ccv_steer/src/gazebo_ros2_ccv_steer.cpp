
/*
new
 */

  // Populate message

/*
new
 */

#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <r2_ccv_steer/gazebo_ros2_ccv_steer.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sdf/sdf.hh>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <sensor_msgs/msg/joint_state.hpp>


#ifdef NO_ERROR
// NO_ERROR is a macro defined in Windows that's used as an enum in tf2
#undef NO_ERROR
#endif


double pi=M_PI;
namespace gazebo_plugins
{
class GazeboRos2CcvSteerPrivate
{
public:
  enum OdomSource
  {
    ENCODER = 0,
    WORLD = 1,
  };
  enum
  {
    STEER = 0,
    STEER_1 =1,
    LEFT = 2,
    RIGHT = 3,
  };



  void OnUpdate(const gazebo::common::UpdateInfo & _info);
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);
  void UpdateWheelVelocities();
  void UpdateOdometryEncoder(const gazebo::common::Time & _current_time);
  void UpdateOdometryWorld();
  void PublishOdometryTf(const gazebo::common::Time & _current_time);
  void PublishWheelsTf(const gazebo::common::Time & _current_time);
  void PublishOdometryMsg(const gazebo::common::Time & _current_time);
  void PublishAngleStatus();
  void OnSteerAngle(const std_msgs::msg::Float64::SharedPtr str_amsg);
  void OnSteerVel(const std_msgs::msg::Float64::SharedPtr str_vmsg);
  void OnSteerVelAngle(const std_msgs::msg::Float64::SharedPtr str_vamsg);
  gazebo::common::Time last_encoder_update_;
  gazebo::common::Time last_update_time_;
  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;
  gazebo::event::ConnectionPtr update_connection_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  std::mutex lock_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_angle_sub_; 
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_vel_sub_; 
  std::vector<gazebo::physics::JointPtr> joints_;
  sensor_msgs::msg::JointState joint_state;
  geometry_msgs::msg::Pose2D pose_encoder_;
  std::string odometry_frame_;
  OdomSource odom_source_;
  nav_msgs::msg::Odometry odom_;
  std::string robot_base_frame_;

  std::vector<double> wheel_separation_;
  std::vector<double> wheel_diameter_;
  std::vector<double> desired_wheel_speed_;
  std::vector<double> wheel_speed_instr_;
  
  double direc_;
  double travel_;
  double travel_steer;
  double max_steer_vel_; 
  double max_steer_accel_;
  double max_steer_torque_;
  double desired_steer_vel_;
  double desired_steer_angle_;
  double steer_vel_instr_;
  double init_steer_angle;
  bool steer_type_; 
  bool joint_status_; 
  double a_diff;
  double travel_up;
  double travel_ang;
  double C_vel;

  double max_wheel_torque_;
  double max_wheel_accel_;
  double target_x_{0.0};
  double target_rot_{0.0};
  double update_period_;
  bool publish_odom_;
  bool publish_wheel_tf_;
  bool publish_odom_tf_;
  unsigned int num_wheel_pairs_;
  double covariance_[3];

};


GazeboRos2CcvSteer::GazeboRos2CcvSteer():impl_(std::make_unique<GazeboRos2CcvSteerPrivate>())
{
}

GazeboRos2CcvSteer::~GazeboRos2CcvSteer()
{
}

void GazeboRos2CcvSteer::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;
// Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
  impl_->num_wheel_pairs_ = static_cast<unsigned int>(_sdf->Get<int>("num_wheel_pairs", 1).first);
  impl_->steer_type_ = _sdf->Get<bool>("steer_type", false).first;

  if (impl_->num_wheel_pairs_ < 1) {
    impl_->num_wheel_pairs_ = 1;
    RCLCPP_WARN(impl_->ros_node_->get_logger(),
      "Drive requires at least one pair of wheels. Setting [num_wheel_pairs] to 1");
  }
// Get joints
  impl_->joints_.resize(2);
  auto left_steer = _sdf->Get<std::string>("left_steer", "left_steer").first;
  impl_->joints_[GazeboRos2CcvSteerPrivate::STEER] = _model->GetJoint(left_steer);
  auto right_steer = _sdf->Get<std::string>("right_steer", "right_steer").first;
  impl_->joints_[GazeboRos2CcvSteerPrivate::STEER_1] = _model->GetJoint(right_steer);

// Check joint availability
  if (!impl_->joints_[GazeboRos2CcvSteerPrivate::STEER]||!impl_->joints_[GazeboRos2CcvSteerPrivate::STEER_1])
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
      "Joint [%s] or [%s] not found, plugin will not work.", left_steer.c_str(),right_steer.c_str());
   impl_->ros_node_.reset();
    return;
  }

//  Print Test


// Dynamic properties-------------------------------------------------------------------------------------------------------------------2
  impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration", 0.0).first;
  impl_->max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque", 5.0).first;
  
  impl_->max_steer_accel_ = _sdf->Get<double>("max_steer_acceleration", 0.0).first;
  impl_->max_steer_torque_ = _sdf->Get<double>("max_steer_torque", 5.0).first;
  impl_->max_steer_vel_ = _sdf->Get<double>("max_steer_vel", 0.5 ).first;
  impl_->joints_[GazeboRos2CcvSteerPrivate::STEER]->SetParam("fmax", 0, impl_->max_steer_torque_);
  impl_->joints_[GazeboRos2CcvSteerPrivate::STEER_1]->SetParam("fmax", 0, impl_->max_steer_torque_);


// Kinematic properties
  impl_->desired_steer_angle_ = 0;
  
  /*New */

  std::vector<gazebo::physics::JointPtr> left_joints, right_joints;

  for (auto left_joint_elem = _sdf->GetElement("left_joint"); left_joint_elem != nullptr;
    left_joint_elem = left_joint_elem->GetNextElement("left_joint"))
  {
    auto left_joint_name = left_joint_elem->Get<std::string>();
    auto left_joint = _model->GetJoint(left_joint_name);
    if (!left_joint) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(),
        "Joint [%s] not found, plugin will not work.", left_joint_name.c_str());
      impl_->ros_node_.reset();
      return;
    }
    left_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
    left_joints.push_back(left_joint);
  }

  for (auto right_joint_elem = _sdf->GetElement("right_joint"); right_joint_elem != nullptr;
    right_joint_elem = right_joint_elem->GetNextElement("right_joint"))
  {
    auto right_joint_name = right_joint_elem->Get<std::string>();
    auto right_joint = _model->GetJoint(right_joint_name);
    if (!right_joint) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(),
        "Joint [%s] not found, plugin will not work.", right_joint_name.c_str());
      impl_->ros_node_.reset();
      return;
    }
    right_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
    right_joints.push_back(right_joint);
  }

  if (left_joints.size() != right_joints.size() || left_joints.size() != impl_->num_wheel_pairs_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
      "Inconsistent number of joints specified. Plugin will not work.");
    impl_->ros_node_.reset();
    return;
  }

  unsigned int index;
  for (index = 0; index < impl_->num_wheel_pairs_; ++index) {
    impl_->joints_.push_back(right_joints[index]);
    impl_->joints_.push_back(left_joints[index]);
  }
  
  index = 0;
  impl_->wheel_separation_.assign(impl_->num_wheel_pairs_, 0.34);
  for (auto wheel_separation = _sdf->GetElement("wheel_separation"); wheel_separation != nullptr;
    wheel_separation = wheel_separation->GetNextElement("wheel_separation"))
  {
    if (index >= impl_->num_wheel_pairs_) {
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_separation>");
      break;
    }
    impl_->wheel_separation_[index] = wheel_separation->Get<double>();
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "Wheel pair %i separation set to [%fm]", index + 1, impl_->wheel_separation_[index]);
    index++;
  }

  index = 0;
  impl_->wheel_diameter_.assign(impl_->num_wheel_pairs_, 0.15);
  for (auto wheel_diameter = _sdf->GetElement("wheel_diameter"); wheel_diameter != nullptr;
    wheel_diameter = wheel_diameter->GetNextElement("wheel_diameter"))
  {
    if (index >= impl_->num_wheel_pairs_) {
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_diameter>");
      break;
    }
    impl_->wheel_diameter_[index] = wheel_diameter->Get<double>();
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "Wheel pair %i diameter set to [%fm]", index + 1, impl_->wheel_diameter_[index]);
    index++;
  }

  impl_->wheel_speed_instr_.assign(2 * impl_->num_wheel_pairs_, 0);
  impl_->desired_wheel_speed_.assign(2 * impl_->num_wheel_pairs_, 0);



// Set dependent values
  impl_->travel_ = pow(impl_->max_steer_vel_,2.0)/impl_->max_steer_accel_;
  impl_->joints_[GazeboRos2CcvSteerPrivate::STEER]->SetPosition(0,0.0000);

  impl_->joints_[GazeboRos2CcvSteerPrivate::STEER_1]->SetPosition(0,0.0000);

// Update rate  
  auto update_rate = _sdf->Get<double>("update_rate", 1000.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&GazeboRos2CcvSteerPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]",
    impl_->cmd_vel_sub_->get_topic_name());


// Joint state publisher
  impl_->joint_status_ = _sdf->Get<bool>("joint_status", false).first;
  if (impl_->joint_status_) {
    impl_->joint_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>("steer_status", rclcpp::QoS(rclcpp::KeepLast(1)));
    RCLCPP_INFO(impl_->ros_node_->get_logger(),"joint on [%s]", impl_->joint_state_pub_->get_topic_name());
    }
///// Manual
  if (impl_->steer_type_) {
////  Angle
// Steer subscriber
  impl_->steer_angle_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float64>("steer_angle",rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&GazeboRos2CcvSteerPrivate::OnSteerAngle, impl_.get(),std::placeholders::_1));
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publish steer angle on [%s]",impl_->steer_angle_sub_->get_topic_name());
//
////  Velocity
// Steer subscriber
  impl_->steer_vel_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float64>("steer_vel",rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&GazeboRos2CcvSteerPrivate::OnSteerVel, impl_.get(),std::placeholders::_1));
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publish steer vel on [%s]",impl_->steer_vel_sub_->get_topic_name());
//
  }
///// Auto
  else
  {
    impl_->steer_angle_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float64>("steer_vel_ang",rclcpp::QoS(rclcpp::KeepLast(1)), 
      std::bind(&GazeboRos2CcvSteerPrivate::OnSteerVelAngle, impl_.get(),std::placeholders::_1));
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publish steer vel on [%s]",impl_->steer_angle_sub_->get_topic_name());
  }

  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  impl_->odom_source_ = static_cast<GazeboRos2CcvSteerPrivate::OdomSource>(
    _sdf->Get<int>("odometry_source", 1).first);

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(rclcpp::KeepLast(1)));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  // Create TF broadcaster if needed
  impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
  if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_) {
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    if (impl_->publish_odom_tf_) {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
        "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
        impl_->robot_base_frame_.c_str());
    }
    
    for (index = 0; index < impl_->num_wheel_pairs_; ++index) {
      if (impl_->publish_wheel_tf_) {
        RCLCPP_INFO(impl_->ros_node_->get_logger(),
          "Publishing wheel transforms between [%s], [%s] and [%s]",
          impl_->robot_base_frame_.c_str(),
          impl_->joints_[2 * index + GazeboRos2CcvSteerPrivate::LEFT]->GetName().c_str(),
          impl_->joints_[2 * index + GazeboRos2CcvSteerPrivate::RIGHT]->GetName().c_str());
      }
    }

  }

  impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
  impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
  impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRos2CcvSteerPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(impl_->ros_node_->get_logger(),"First Ittration!!  Joints= %d", impl_->joints_.size());

}




// Incase of reset

void GazeboRos2CcvSteer::Reset()
{
  impl_->last_update_time_ =
    impl_->joints_[GazeboRos2CcvSteerPrivate::LEFT]->GetWorld()->SimTime();
  for (unsigned int i = 0; i < impl_->num_wheel_pairs_; ++i) {
    if (impl_->joints_[2 * i + GazeboRos2CcvSteerPrivate::LEFT] &&
      impl_->joints_[2 * i + GazeboRos2CcvSteerPrivate::RIGHT])
    {
      impl_->joints_[2 * i + GazeboRos2CcvSteerPrivate::LEFT]->SetParam(
        "fmax", 0, impl_->max_wheel_torque_);
      impl_->joints_[2 * i + GazeboRos2CcvSteerPrivate::RIGHT]->SetParam(
        "fmax", 0, impl_->max_wheel_torque_);
    }
  }
  impl_->pose_encoder_.x = 0;
  impl_->pose_encoder_.y = 0;
  impl_->pose_encoder_.theta = 0;
  impl_->target_x_ = 0;
  impl_->target_rot_ = 0;
}


//Every Update loop

void GazeboRos2CcvSteerPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  gazebo::common::Time current_time = _info.simTime;
  
  if (current_time < last_update_time_) {
    RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
    last_update_time_ = current_time;
  }
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
  if (seconds_since_last_update < update_period_) {
    return;
  }



  // Update odom message if using ground truth
  if (odom_source_ == WORLD) {
    UpdateOdometryWorld();
  }

  if (publish_odom_) {
    PublishOdometryMsg(_info.simTime);
  }

  if (publish_wheel_tf_) {
    PublishWheelsTf(_info.simTime);
  }

  if (publish_odom_tf_) {
    PublishOdometryTf(_info.simTime);
  }

// Update robot in case new velocities have been requested
  UpdateWheelVelocities();

/*
New
 */

// Current speed
  std::vector<double> current_speed(2 * num_wheel_pairs_);
  for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
    current_speed[2 * i + LEFT] =
      joints_[2 * i + LEFT]->GetVelocity(0) * (wheel_diameter_[i] / 2.0);
    current_speed[2 * i + RIGHT] =
      joints_[2 * i + RIGHT]->GetVelocity(0) * (wheel_diameter_[i] / 2.0);
  }

  // If max_accel == 0, or target speed is reached
  for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
    if (max_wheel_accel_ == 0 ||
      (fabs(desired_wheel_speed_[2 * i + LEFT] - current_speed[2 * i + LEFT]) < 0.01) ||
      (fabs(desired_wheel_speed_[2 * i + RIGHT] - current_speed[2 * i + RIGHT]) < 0.01))
    {
      joints_[2 * i + LEFT]->SetParam(
        "vel", 0, desired_wheel_speed_[2 * i + LEFT] / (wheel_diameter_[i] / 2.0));
      joints_[2 * i + RIGHT]->SetParam(
        "vel", 0, desired_wheel_speed_[2 * i + RIGHT] / (wheel_diameter_[i] / 2.0));
    } else {
      if (desired_wheel_speed_[2 * i + LEFT] >= current_speed[2 * i + LEFT]) {
        wheel_speed_instr_[2 * i + LEFT] += fmin(desired_wheel_speed_[2 * i + LEFT] -
            current_speed[2 * i + LEFT], max_wheel_accel_ * seconds_since_last_update);
      } else {
        wheel_speed_instr_[2 * i + LEFT] += fmax(desired_wheel_speed_[2 * i + LEFT] -
            current_speed[2 * i + LEFT], -max_wheel_accel_ * seconds_since_last_update);
      }

      if (desired_wheel_speed_[2 * i + RIGHT] > current_speed[2 * i + RIGHT]) {
        wheel_speed_instr_[2 * i + RIGHT] += fmin(desired_wheel_speed_[2 * i + RIGHT] -
            current_speed[2 * i + RIGHT], max_wheel_accel_ * seconds_since_last_update);
      } else {
        wheel_speed_instr_[2 * i + RIGHT] += fmax(desired_wheel_speed_[2 * i + RIGHT] -
            current_speed[2 * i + RIGHT], -max_wheel_accel_ * seconds_since_last_update);
      }

      joints_[2 * i + LEFT]->SetParam(
        "vel", 0, wheel_speed_instr_[2 * i + LEFT] / (wheel_diameter_[i] / 2.0));
      joints_[2 * i + RIGHT]->SetParam(
        "vel", 0, wheel_speed_instr_[2 * i + RIGHT] / (wheel_diameter_[i] / 2.0));
      }
    }

  

// Current values
  double current_steer_angle;
  double current_vel[3];
  double stop_angle;

  stop_angle = 0.0;
  current_steer_angle = joints_[STEER]->Position(0);
  current_vel[STEER] = fabs(joints_[STEER]->GetVelocity(0));
  if (fabs(desired_steer_angle_ - current_steer_angle) < 0.00005)
  {
    joints_[STEER]->SetParam("vel", 0, stop_angle);
    joints_[STEER]->SetPosition(0,desired_steer_angle_);
    joints_[STEER_1]->SetParam("vel", 0, stop_angle);
    joints_[STEER_1]->SetPosition(0,desired_steer_angle_);
    steer_vel_instr_=0.0;
  }

  else
  {
    if (fabs(init_steer_angle-current_steer_angle)<travel_steer)
    {
      if (fabs(desired_steer_vel_ - steer_vel_instr_) <= 0.001){
        steer_vel_instr_ =desired_steer_vel_;
      }
      else
      {
      steer_vel_instr_ += fmin(desired_steer_vel_ - current_vel[STEER],max_steer_accel_*seconds_since_last_update);
      }
    }
    else
    {
      desired_steer_vel_= 0.00000;
      if (fabs(steer_vel_instr_ - desired_steer_vel_) <= 0.001){
        steer_vel_instr_ =desired_steer_vel_;
      }
      else
      {
        steer_vel_instr_ += fmin(current_vel[STEER]-desired_steer_vel_,-max_steer_accel_*seconds_since_last_update);
      }
    }

    // RCLCPP_INFO(ros_node_->get_logger(),"%f / %f               %f            %f / %f",
    //   current_vel[BASE],current_angle[BASE]-init_angle[BASE],
    //   steer_vel_instr_[BASE],
    //   desired_steer_vel_[BASE],travel_up[BASE]);
    if (direc_<0.0){
      joints_[STEER]->SetParam("vel", 0, -steer_vel_instr_);
      joints_[STEER_1]->SetParam("vel", 0, -steer_vel_instr_);
    }
    else
    {
      joints_[STEER]->SetParam("vel", 0, steer_vel_instr_);
      joints_[STEER_1]->SetParam("vel", 0, steer_vel_instr_);
    }

    
  }

  RCLCPP_INFO(ros_node_->get_logger(),"First Ittration!!  Joints= %d",last_update_time_);

/*
End
 */

  joint_state.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  joint_state.name.resize(joints_.size());
  joint_state.position.resize(joints_.size());
  joint_state.velocity.resize(joints_.size());

  for (unsigned int i = 0; i < joints_.size(); ++i) {
    auto joint = joints_[i];
    double velocity = joint->GetVelocity(0);
    double position = joint->Position(0)*180/pi;
    joint_state.name[i] = joint->GetName();
    joint_state.position[i] = position;
    joint_state.velocity[i] = velocity;
  }

  if (joint_status_){
    PublishAngleStatus();
  }
  last_update_time_ = _info.simTime;
  RCLCPP_INFO(ros_node_->get_logger(),"First Ittration!!  Joints= %d",last_update_time_);
}

/* New */
// Set desired Position
void GazeboRos2CcvSteerPrivate::OnSteerAngle(const std_msgs::msg::Float64::SharedPtr str_amsg_)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  desired_steer_angle_ = (str_amsg_->data*pi)/180.00;
  init_steer_angle = joints_[STEER]->Position(0);
  a_diff=fabs(desired_steer_angle_-init_steer_angle);
  if (a_diff>=travel_steer)
    {
      travel_ang=travel_steer;
      C_vel=max_steer_vel_;
    }
  else
    {
      travel_ang=pow(a_diff,2.0) / (2.0*max_steer_accel_);
      C_vel=sqrt(travel_ang*max_steer_accel_);
    }
  travel_ang=pow(desired_steer_vel_,2.0)/(2.0*max_steer_accel_);
  travel_up=a_diff-travel_ang;

}
// Set desired velocity
void GazeboRos2CcvSteerPrivate::OnSteerVel(const std_msgs::msg::Float64::SharedPtr str_vmsg_)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  desired_steer_vel_ = str_vmsg_->data;;

  if (desired_steer_vel_<0)
  {
    direc_=-1.0;
    desired_steer_vel_=-desired_steer_vel_;
  }
  else
  {
    direc_=1.0;
  }
  if (desired_steer_vel_>max_steer_vel_){
    RCLCPP_INFO(ros_node_->get_logger(), "Desired vel is higher than max vel:  %f ", max_steer_vel_);
    desired_steer_vel_ = max_steer_vel_;
  }
  RCLCPP_INFO(ros_node_->get_logger(), "steer --- in --- ??? =  %f m/s", desired_steer_vel_);
}

// Set desired Position with auto vel
void GazeboRos2CcvSteerPrivate::OnSteerVelAngle(const std_msgs::msg::Float64::SharedPtr str_vamsg_){
  std::lock_guard<std::mutex> scoped_lock(lock_);
  desired_steer_angle_ = (str_vamsg_->data*pi)/180.00;
  init_steer_angle = joints_[STEER]->Position(0);
  a_diff=fabs(desired_steer_angle_-init_steer_angle);
  if (desired_steer_angle_<init_steer_angle)
  {
    direc_=-1.0;
  }
  else
  {
    direc_=1.0;
  }
  if (a_diff>=travel_)
    {
      travel_ang=travel_;
      desired_steer_vel_=max_steer_vel_;
    }
  else
    {
      //travel_ang[STEER]=pow(max_steer_vel_[STEER],2.0)/(2.0*max_steer_accel_[STEER]);
      travel_ang=a_diff/3.0;
      desired_steer_vel_=sqrt(travel_ang*max_steer_accel_);
    }
  //travel_ang[STEER]=travel_ang[STEER]/2;
  travel_steer=a_diff-travel_ang;
  RCLCPP_INFO(ros_node_->get_logger(), "[STEER]");
  RCLCPP_INFO(ros_node_->get_logger(), "Total Travel =  %f deg", a_diff);
  RCLCPP_INFO(ros_node_->get_logger(), "To Travel =  %f deg", travel_steer);
  RCLCPP_INFO(ros_node_->get_logger(), "Change time =  %f deg", travel_ang);
  RCLCPP_INFO(ros_node_->get_logger(), "Dogs --- in --- deg =  %f deg", str_vamsg_->data);
  RCLCPP_INFO(ros_node_->get_logger(), "Dogs --- in --- rad =  %f rad", desired_steer_angle_);
  RCLCPP_INFO(ros_node_->get_logger(), "Joint --- in --- ??? =  %f m/s", desired_steer_vel_);

}


/* New */



void GazeboRos2CcvSteerPrivate::UpdateWheelVelocities()
{
  std::lock_guard<std::mutex> scoped_lock(lock_);

  double vr = target_x_;
  double va = target_rot_;

  for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
    desired_wheel_speed_[2 * i + LEFT] = vr - va * wheel_separation_[i] / 2.0;
    desired_wheel_speed_[2 * i + RIGHT] = vr + va * wheel_separation_[i] / 2.0;
  }
}

void GazeboRos2CcvSteerPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_x_ = _msg->linear.x;
  target_rot_ = _msg->angular.z;
}

void GazeboRos2CcvSteerPrivate::UpdateOdometryEncoder(const gazebo::common::Time & _current_time)
{
  double vl = joints_[LEFT]->GetVelocity(0);
  double vr = joints_[RIGHT]->GetVelocity(0);

  double seconds_since_last_update = (_current_time - last_encoder_update_).Double();
  last_encoder_update_ = _current_time;

  double b = wheel_separation_[0];

  // Book: Sigwart 2011 Autonompus Mobile Robots page:337
  double sl = vl * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
  double sr = vr * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
  double ssum = sl + sr;

  double sdiff = sr - sl;

  double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dtheta = (sdiff) / b;

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

  double w = dtheta / seconds_since_last_update;
  double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

  tf2::Quaternion qt;
  tf2::Vector3 vt;
  qt.setRPY(0, 0, pose_encoder_.theta);
  vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  odom_.twist.twist.angular.z = w;
  odom_.twist.twist.linear.x = v;
  odom_.twist.twist.linear.y = 0;
}

void GazeboRos2CcvSteerPrivate::UpdateOdometryWorld()
{
  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // Get velocity in odom frame
  auto linear = model_->WorldLinearVel();
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint)
  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
}

void GazeboRos2CcvSteerPrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

void GazeboRos2CcvSteerPrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
{
  for (unsigned int i = 0; i < 2 * num_wheel_pairs_; ++i) {
    auto pose_wheel = joints_[i]->GetChild()->RelativePose();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = joints_[i]->GetParent()->GetName();
    msg.child_frame_id = joints_[i]->GetChild()->GetName();
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());

    transform_broadcaster_->sendTransform(msg);
  }
}

void GazeboRos2CcvSteerPrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
{
  // Set covariance
  odom_.pose.covariance[0] = covariance_[0];
  odom_.pose.covariance[7] = covariance_[1];
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = covariance_[2];

  odom_.twist.covariance[0] = covariance_[0];
  odom_.twist.covariance[7] = covariance_[1];
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = covariance_[2];

  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub_->publish(odom_);
}


//Angle Status Publish
void GazeboRos2CcvSteerPrivate::PublishAngleStatus()
{
  joint_state_pub_->publish(joint_state);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRos2CcvSteer)

}