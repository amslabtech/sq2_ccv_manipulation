
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
#include <r2_joint_vel/gazebo_ros2_joint_vel.hpp>
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
class GazeboRos2JointVelPrivate
{
public:
  enum OdomSource
  {
    ENCODER = 0,
    WORLD = 1,
  };
  enum
  {
    BASE = 0,
    SHOLDER = 1,
    ELBOW = 2,
    WRIST = 3,
  };

  void PublishJointTf(const gazebo::common::Time & _current_time);
  void OnCmdAngle(const std_msgs::msg::Float64::SharedPtr _amsg);
  void OnCmdVel(const std_msgs::msg::Float64::SharedPtr _vmsg); 
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
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_angle_sub_; 
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_vel_sub_; 
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  std::vector<gazebo::physics::JointPtr> joints_;
  std::string robot_base_frame_;
  std::mutex lock_;
  sensor_msgs::msg::JointState joint_state;
  double target_vel_{0.0};
  double target_angle_{0.0};
  double update_period_;
  double joint_length_;
  double max_joint_torque_;
  double max_joint_accel_;
  double travel_; 
  double vel_;
  double direc_;
  double max_joint_vel_; 
  double desired_joint_vel_[1];
  double desired_joint_angle_[1];  
  double a_diff[1];
  double travel_up[1];
  double init_angle[1];
  double joint_vel_instr_[1];
  bool publish_angle_;
  bool publish_joint_tf_;
  bool joint_status_;
  bool acc_update_status_;
  double travel_ang[1];
  double C_vel[1];

};


GazeboRos2JointVel::GazeboRos2JointVel():impl_(std::make_unique<GazeboRos2JointVelPrivate>())
{
}

GazeboRos2JointVel::~GazeboRos2JointVel()
{
}

void GazeboRos2JointVel::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;
// Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
// Get joints
  impl_->joints_.resize(1);
  auto base_joint = _sdf->Get<std::string>("base_joint", "base_joint").first;
  impl_->joints_[GazeboRos2JointVelPrivate::BASE] = _model->GetJoint(base_joint);

// Check joint availability
  if (!impl_->joints_[GazeboRos2JointVelPrivate::BASE])
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
      "Joint [%s] not found, plugin will not work.", base_joint.c_str());
   impl_->ros_node_.reset();
    return;
  }

//  Print Test

  RCLCPP_INFO(impl_->ros_node_->get_logger(),"First Ittration!!  Joints= %d", impl_->joints_.size());

// Kinematic properties
  //auto joint_length = _sdf->Get<std::double_t>("joint_length", "joint_length").first;
  impl_->joint_length_ = _sdf->Get<double>("joint_length", 100.00).first;
  impl_->desired_joint_angle_[GazeboRos2JointVelPrivate::BASE] = 0;
  RCLCPP_ERROR(impl_->ros_node_->get_logger(),"Joint Length Specified : [%f] meters ", impl_->joint_length_);

// Dynamic properties-------------------------------------------------------------------------------------------------------------------2
  impl_->max_joint_accel_ = _sdf->Get<double>("max_joint_acceleration", 0.0).first;
  impl_->max_joint_torque_ = _sdf->Get<double>("max_joint_torque", 5.0).first;
  impl_->max_joint_vel_ = _sdf->Get<double>("max_joint_vel", 0.5 ).first;
  impl_->joints_[GazeboRos2JointVelPrivate::BASE]->SetParam("fmax", 0, impl_->max_joint_torque_);


// Set dependent values
  impl_->travel_ = pow(impl_->max_joint_vel_,2.0)/impl_->max_joint_accel_;
  impl_->vel_=impl_->max_joint_vel_/impl_->joint_length_;
  impl_->joints_[GazeboRos2JointVelPrivate::BASE]->SetPosition(0,0.0000);

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
    impl_->joint_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>("j_status", rclcpp::QoS(rclcpp::KeepLast(1)));
    RCLCPP_INFO(impl_->ros_node_->get_logger(),"joint on [%s]", impl_->joint_state_pub_->get_topic_name());
    }

////  Angle

// joint subscriber
  impl_->joint_angle_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float64>("cmd_angle",rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&GazeboRos2JointVelPrivate::OnCmdAngle, impl_.get(),std::placeholders::_1));
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s] for joint",impl_->joint_angle_sub_->get_topic_name());
//


////  Velocity

// joint subscriber
  impl_->joint_vel_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float64>("cmd_vel",rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&GazeboRos2JointVelPrivate::OnCmdVel, impl_.get(),std::placeholders::_1));
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s] for joint",impl_->joint_vel_sub_->get_topic_name());
//

  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRos2JointVelPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

// Acceleration update type

  impl_->acc_update_status_ = _sdf->Get<bool>("acc_update_status", false).first;

}



// Incase of reset

void GazeboRos2JointVel::Reset()
{
  if (impl_->joints_[GazeboRos2JointVelPrivate::BASE])
  {
    impl_->last_update_time_ =
      impl_->joints_[GazeboRos2JointVelPrivate::BASE]->GetWorld()->SimTime();
    impl_->joints_[GazeboRos2JointVelPrivate::BASE]->SetParam("fmax", 0, impl_->max_joint_torque_);
  }
  impl_->target_vel_ = 0;
  impl_->target_angle_ = 0;
}



//Every Update loop

void GazeboRos2JointVelPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
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
  double current_angle[1];
  double current_vel[1];
  double stop_angle[1];

  stop_angle[BASE] = 0.0;
  current_angle[BASE] = joints_[BASE]->Position(0);
  current_vel[BASE] = fabs(joints_[BASE]->GetVelocity(0));


  
  

  if (fabs(desired_joint_angle_[BASE] - current_angle[BASE]) < 0.00005)
  {
    joints_[BASE]->SetParam("vel", 0, stop_angle[BASE]);
    //joints_[BASE]->SetVelocity(0, stop_angle[BASE]);
    joints_[BASE]->SetPosition(0,desired_joint_angle_[BASE]);
    joint_vel_instr_[BASE]=0.0;
  }

  else
  {
    if (fabs(init_angle[BASE]-current_angle[BASE])<travel_up[BASE])
    {
      if (fabs(desired_joint_vel_[BASE] - joint_vel_instr_[BASE]) <= 0.001){
        joint_vel_instr_[BASE] =desired_joint_vel_[BASE];
      }
      else
      {
      joint_vel_instr_[BASE] += fmin(desired_joint_vel_[BASE] - current_vel[BASE],max_joint_accel_*seconds_since_last_update);
      }
    }
    else
    {
      desired_joint_vel_[BASE]= 0.00000;
      if (fabs(joint_vel_instr_[BASE] - desired_joint_vel_[BASE]) <= 0.001){
        joint_vel_instr_[BASE] =desired_joint_vel_[BASE];
      }
      else
      {
        joint_vel_instr_[BASE] += fmin(current_vel[BASE]-desired_joint_vel_[BASE],-max_joint_accel_*seconds_since_last_update);
      }
    }


    // RCLCPP_INFO(ros_node_->get_logger(),"%f / %f               %f            %f / %f",
    //   current_vel[BASE],current_angle[BASE]-init_angle[BASE],
    //   joint_vel_instr_[BASE],
    //   desired_joint_vel_[BASE],travel_up[BASE]);
    if (direc_<0.0){
      joints_[BASE]->SetParam("vel", 0, -joint_vel_instr_[BASE]);
      //joints_[BASE]->SetVelocity(0, joint_vel_instr_[BASE]);
    }
    else
    {
      joints_[BASE]->SetParam("vel", 0, joint_vel_instr_[BASE]);
      //joints_[BASE]->SetVelocity(0, joint_vel_instr_[BASE]);
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
    double velocity = joint->GetVelocity(0) * (joint_length_);
    double position = joint->Position(0);
    joint_state.name[i] = joint->GetName();
    joint_state.position[i] = position;
    joint_state.velocity[i] = velocity;
  }

  if (joint_status_){
    PublishAngleStatus();
  }
  last_update_time_ = _info.simTime;
}

// Set desired Position
void GazeboRos2JointVelPrivate::OnCmdAngle(const std_msgs::msg::Float64::SharedPtr _amsg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);

  target_angle_ = _amsg->data;
  desired_joint_angle_[BASE] = (target_angle_*pi)/180.00;
  init_angle[BASE] = joints_[BASE]->Position(0);
  a_diff[BASE]=fabs(desired_joint_angle_[BASE]-init_angle[BASE]);
  if (a_diff[BASE]>=travel_)
    {
      travel_ang[BASE]=travel_;
      C_vel[BASE]=vel_;
    }
  else
    {
      travel_ang[BASE]=pow(a_diff[BASE],2.0) / max_joint_accel_;
      C_vel[BASE]=sqrt(travel_ang[BASE]*max_joint_accel_);
    }
  travel_ang[BASE]=pow(desired_joint_vel_[BASE],2.0)/(2.0*max_joint_accel_);
  travel_up[BASE]=a_diff[BASE]-travel_ang[BASE];

  RCLCPP_INFO(ros_node_->get_logger(), "Path To Travel =  %f deg", travel_up[BASE]);
  RCLCPP_INFO(ros_node_->get_logger(), "New Position (deg) =  %f deg", target_angle_);
  RCLCPP_INFO(ros_node_->get_logger(), "New Position (rad) =  %f rad", desired_joint_angle_[BASE]);
}

// Set desired velocity
void GazeboRos2JointVelPrivate::OnCmdVel(const std_msgs::msg::Float64::SharedPtr _vmsg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_vel_ = _vmsg->data;
  //desired_joint_vel_[BASE] = (target_vel_*pi)/180;
  desired_joint_vel_[BASE] = target_vel_/joint_length_;

  RCLCPP_INFO(ros_node_->get_logger(), "Joint_Length =  %f m", joint_length_);
  RCLCPP_INFO(ros_node_->get_logger(), "Joint_Velocity =  %f m/s", desired_joint_vel_[BASE]);
  // if (desired_joint_vel_[BASE]<-max_joint_vel_){
  //   RCLCPP_INFO(ros_node_->get_logger(), "Desired vel is higher than max vel:  %f ", max_joint_vel_);
  //   desired_joint_vel_[BASE] = max_joint_vel_;
  // }
  if (desired_joint_vel_[BASE]<0)
  {
    direc_=-1.0;
    desired_joint_vel_[BASE]=-desired_joint_vel_[BASE];
  }
  else
  {
    direc_=1.0;
  }
  if (desired_joint_vel_[BASE]>max_joint_vel_){
    RCLCPP_INFO(ros_node_->get_logger(), "Desired vel is higher than max vel:  %f ", max_joint_vel_);
    desired_joint_vel_[BASE] = max_joint_vel_;
  }
  
  
  //RCLCPP_INFO(ros_node_->get_logger(), "PI =  %f deg", pi);
  // RCLCPP_INFO(ros_node_->get_logger(), "Joint_Length =  %f m", joint_length_);
  // RCLCPP_INFO(ros_node_->get_logger(), "Joint_Velocity=  %f m/s", desired_joint_vel_[BASE]);
}


// Joint TF Publish

void GazeboRos2JointVelPrivate::PublishJointTf(const gazebo::common::Time & _current_time)
{
  for (auto i : {BASE}) {
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


void GazeboRos2JointVelPrivate::PublishAngleStatus()
{
  joint_state_pub_->publish(joint_state);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRos2JointVel)

}