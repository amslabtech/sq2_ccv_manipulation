
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


  void PublishJointTf(const gazebo::common::Time & _current_time);
  void OnSteerAngle(const std_msgs::msg::Float64::SharedPtr str_amsg);
  void OnSteerVel(const std_msgs::msg::Float64::SharedPtr str_vmsg);
  void OnSteerVelAngle(const std_msgs::msg::Float64::SharedPtr str_vamsg);
  void OnUpdate(const gazebo::common::UpdateInfo & _info);
  void UpdateJointVelocities();
  void PublishAngleStatus();
  void UpdatePosAngle();
  geometry_msgs::msg::Pose2D pose_encoder_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo::common::Time last_encoder_update_;
  gazebo::common::Time last_update_time_;
  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_angle_sub_; 
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_vel_sub_; 
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  std::vector<gazebo::physics::JointPtr> joints_;
  std::string robot_base_frame_;
  std::mutex lock_;
  sensor_msgs::msg::JointState joint_state;
  double target_vel_{0.0};
  double target_angle_{0.0};
  double update_period_;
  double wheel_length_;
  double max_wheel_torque_;

  std::vector<double> max_wheel_vel_;
  
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
  double a_diff;
  double travel_up;

  bool publish_angle_;
  bool publish_joint_tf_;
  bool joint_status_;
  bool acc_update_status_;
  double travel_ang[1];
  double C_vel[1];

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

  impl_->steer_type_ = _sdf->Get<bool>("steer_type", false).first;
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

  RCLCPP_INFO(impl_->ros_node_->get_logger(),"First Ittration!!  Joints= %d", impl_->joints_.size());

// Kinematic properties
  //auto joint_length = _sdf->Get<std::double_t>("joint_length", "joint_length").first;
  impl_->wheel_length_ = _sdf->Get<double>("wheel_length_", 100.00).first;
  impl_->desired_steer_angle_ = 0;
  impl_->desired_steer_angle_ = 0;
  RCLCPP_ERROR(impl_->ros_node_->get_logger(),"Wheel Length Specified : [%f] meters ", impl_->wheel_length_);

// Dynamic properties-------------------------------------------------------------------------------------------------------------------2
  impl_->max_steer_accel_ = _sdf->Get<double>("max_steer_acceleration", 0.0).first;
  impl_->max_steer_torque_ = _sdf->Get<double>("max_steer_torque", 5.0).first;
  impl_->max_steer_vel_ = _sdf->Get<double>("max_steer_vel", 0.5 ).first;
  impl_->joints_[GazeboRos2CcvSteerPrivate::STEER]->SetParam("fmax", 0, impl_->max_steer_torque_);
  impl_->joints_[GazeboRos2CcvSteerPrivate::STEER_1]->SetParam("fmax", 0, impl_->max_steer_torque_);


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
/*New */

///// Auto
  else
  {
    impl_->steer_angle_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float64>("steer_vel_ang",rclcpp::QoS(rclcpp::KeepLast(1)), 
      std::bind(&GazeboRos2CcvSteerPrivate::OnSteerVelAngle, impl_.get(),std::placeholders::_1));
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s] for FING Angle",impl_->steer_angle_sub_->get_topic_name());
  }

  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRos2CcvSteerPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

// Acceleration update type

  impl_->acc_update_status_ = _sdf->Get<bool>("acc_update_status", false).first;

}




// Incase of reset

void GazeboRos2CcvSteer::Reset()
{
  if (impl_->joints_[GazeboRos2CcvSteerPrivate::STEER])
  {
    impl_->last_update_time_ =
      impl_->joints_[GazeboRos2CcvSteerPrivate::STEER]->GetWorld()->SimTime();
    impl_->joints_[GazeboRos2CcvSteerPrivate::STEER]->SetParam("fmax", 0, impl_->max_wheel_torque_);
  }
  impl_->target_vel_ = 0;
  impl_->target_angle_ = 0;
}



//Every Update loop

void GazeboRos2CcvSteerPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  gazebo::common::Time current_time = _info.simTime;
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
  if (seconds_since_last_update < update_period_) {
    return;
  }

  if (publish_joint_tf_) {
    PublishJointTf(_info.simTime);
  }

  if (current_time < last_update_time_) {
    RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
    last_update_time_ = current_time;
  }


/*
New
 */

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
      travel_ang[STEER]=travel_steer;
      C_vel[STEER]=max_steer_vel_;
    }
  else
    {
      travel_ang[STEER]=pow(a_diff,2.0) / (2.0*max_steer_accel_);
      C_vel[STEER]=sqrt(travel_ang[STEER]*max_steer_accel_);
    }
  travel_ang[STEER]=pow(desired_steer_vel_,2.0)/(2.0*max_steer_accel_);
  travel_up=a_diff-travel_ang[STEER];

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
      travel_ang[STEER]=travel_/3.0;
      desired_steer_vel_=max_steer_vel_;
    }
  else
    {
      //travel_ang[STEER]=pow(max_steer_vel_[STEER],2.0)/(2.0*max_steer_accel_[STEER]);
      travel_ang[STEER]=a_diff/3.0;
      desired_steer_vel_=sqrt(travel_ang[STEER]*max_steer_accel_);
    }
  //travel_ang[STEER]=travel_ang[STEER]/2;
  travel_steer=a_diff-travel_ang[STEER];
  RCLCPP_INFO(ros_node_->get_logger(), "[STEER]");
  RCLCPP_INFO(ros_node_->get_logger(), "Total Travel =  %f deg", a_diff);
  RCLCPP_INFO(ros_node_->get_logger(), "To Travel =  %f deg", travel_steer);
  RCLCPP_INFO(ros_node_->get_logger(), "Change time =  %f deg", travel_ang[STEER]);
  RCLCPP_INFO(ros_node_->get_logger(), "Dogs --- in --- deg =  %f deg", str_vamsg_->data);
  RCLCPP_INFO(ros_node_->get_logger(), "Dogs --- in --- rad =  %f rad", desired_steer_angle_);
  RCLCPP_INFO(ros_node_->get_logger(), "Joint --- in --- ??? =  %f m/s", desired_steer_vel_);

}


// Joint TF Publish

void GazeboRos2CcvSteerPrivate::PublishJointTf(const gazebo::common::Time & _current_time)
{
  for (auto i : {STEER}) {
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

//Angle Status Publish


void GazeboRos2CcvSteerPrivate::PublishAngleStatus()
{
  joint_state_pub_->publish(joint_state);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRos2CcvSteer)

}