// Copyright 2023 Yunlong Feng
// Copyright 2025 Lihan Chen
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

#include "projectile_motion/projectile_motion_node.hpp"

#include "rmoss_projectile_motion/gaf_projectile_solver.hpp"
#include "rmoss_projectile_motion/gravity_projectile_solver.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace projectile_motion
{

ProjectileMotionNode::ProjectileMotionNode(rclcpp::NodeOptions options)
: Node("projectile_motion_node", options.use_intra_process_comms(true))
{
  offset_pitch_ = this->declare_parameter("projectile.offset_pitch", 0.0);
  offset_yaw_ = this->declare_parameter("projectile.offset_yaw", 0.0);
  offset_time_ = this->declare_parameter("projectile.offset_time", 0.0);
  shoot_speed_ = this->declare_parameter("projectile.initial_speed", 18.0);
  target_topic_ = this->declare_parameter("projectile.target_topic", "tracker/target");
  gimbal_cmd_topic_ = this->declare_parameter("projectile.gimbal_cmd_topic", "gimbal_cmd");
  shoot_cmd_topic_ = this->declare_parameter("projectile.shoot_cmd_topic", "cmd_shoot");
  shooter_frame_ = this->declare_parameter("projectile.target_frame", "shooter_link");
  solver_type_ = this->declare_parameter("projectile.solver_type", "gravity");

  RCLCPP_INFO(this->get_logger(), "Projectile motion solver type: %s", solver_type_.c_str());
  if (solver_type_ == "gravity") {
    solver_ = std::make_shared<rmoss_projectile_motion::GravityProjectileSolver>(shoot_speed_);
  } else if (solver_type_ == "gaf") {
    friction_ = this->declare_parameter("projectile.friction", 0.001);
    solver_ =
      std::make_shared<rmoss_projectile_motion::GafProjectileSolver>(shoot_speed_, friction_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown solver type: %s", solver_type_.c_str());
    return;
  }

  shoot_speed_ = get_parameter("projectile.initial_speed").as_double();
  if (solver_type_ == "gravity") {
    auto solver_ptr =
      std::dynamic_pointer_cast<rmoss_projectile_motion::GravityProjectileSolver>(solver_);
    solver_ptr->set_initial_vel(shoot_speed_);
  } else if (solver_type_ == "gaf") {
    auto solver_ptr =
      std::dynamic_pointer_cast<rmoss_projectile_motion::GafProjectileSolver>(solver_);
    solver_ptr->set_initial_vel(shoot_speed_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown solver type: %s", solver_type_.c_str());
  }

  gimbal_cmd_publisher_ =
    this->create_publisher<sensor_msgs::msg::JointState>(gimbal_cmd_topic_, 10);
  shoot_cmd_publisher_ =
    this->create_publisher<example_interfaces::msg::UInt8>(shoot_cmd_topic_, 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10.0));
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  target_sub_.subscribe(this, target_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  tf_filter_ = std::make_shared<tf2_filter>(
    target_sub_, *tf_buffer_, shooter_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  tf_filter_->registerCallback(&ProjectileMotionNode::targetCallback, this);

  RCLCPP_INFO(this->get_logger(), "Projectile motion node initialized.");
}

void ProjectileMotionNode::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  if (!msg->tracking) {
    // Stop shooting
    example_interfaces::msg::UInt8 shoot_cmd;
    shoot_cmd.data = 0;
    shoot_cmd_publisher_->publish(shoot_cmd);
    return;
  }

  if (!getCurrentGimbalAngle(
        msg->header.frame_id, msg->header.stamp, cur_roll_, cur_pitch_, cur_yaw_)) {
    return;
  }

  Eigen::Vector3d center_position(
    msg->position.x + offset_x_, msg->position.y + offset_y_, msg->position.z + offset_z_);
  Eigen::Vector3d center_velocity(msg->velocity.x, msg->velocity.y, msg->velocity.z);

  double hit_yaw = 0.0, hit_pitch = 0.0;
  calculateTargetPosition(msg, center_position, center_velocity, cur_yaw_, hit_yaw, hit_pitch);

  publishGimbalCommand(hit_pitch, hit_yaw, 1);
}

bool ProjectileMotionNode::getCurrentGimbalAngle(
  const std::string & frame_id, const rclcpp::Time & stamp, double & cur_roll, double & cur_pitch,
  double & cur_yaw)
{
  try {
    auto transform = tf_buffer_->lookupTransform(frame_id, shooter_frame_, stamp);
    tf2::Quaternion rot_q(
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w);
    tf2::Matrix3x3 rot_m(rot_q);
    rot_m.getRPY(cur_roll, cur_pitch, cur_yaw);
    offset_x_ = -transform.transform.translation.x;
    offset_y_ = -transform.transform.translation.y;
    offset_z_ = -transform.transform.translation.z;
    return true;
  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
    return false;
  }
}

void ProjectileMotionNode::calculateTargetPosition(
  const auto_aim_interfaces::msg::Target::SharedPtr & msg, const Eigen::Vector3d & center_position,
  const Eigen::Vector3d & center_velocity, double cur_yaw, double & hit_yaw, double & hit_pitch)
{
  double min_yaw = DBL_MAX, min_dis = DBL_MAX;
  bool is_current_pair = true;
  double r = 0., target_dz = 0., fly_time = 0.;
  double target_pitch, target_yaw;
  Eigen::Vector3d target_position, target_predict_position;

  for (int i = 0; i < msg->armors_num; ++i) {
    double tmp_yaw = msg->yaw + i * (2 * M_PI / msg->armors_num);

    if (msg->armors_num == 4) {
      r = is_current_pair ? msg->radius_1 : msg->radius_2;
      is_current_pair = !is_current_pair;
      target_dz = is_current_pair ? 0. : msg->dz;
    } else {
      r = msg->radius_1;
      target_dz = 0.;
    }

    target_position =
      center_position + Eigen::Vector3d(-r * std::cos(tmp_yaw), -r * std::sin(tmp_yaw), target_dz);
    fly_time = target_position.head(2).norm() / shoot_speed_ + offset_time_;
    tmp_yaw += msg->v_yaw * fly_time;
    target_predict_position =
      center_position + center_velocity * fly_time +
      Eigen::Vector3d(-r * std::cos(tmp_yaw), -r * std::sin(tmp_yaw), target_dz);

    solver_->solve(
      target_predict_position.head(2).norm(), target_predict_position.z(), target_pitch);
    target_pitch = -target_pitch;
    target_yaw = std::atan2(target_predict_position.y(), target_predict_position.x());

    if (
      std::abs(std::fmod(tmp_yaw, M_PI) - cur_yaw) < min_yaw &&
      target_predict_position.head(2).norm() < min_dis) {
      min_yaw = std::abs(std::fmod(tmp_yaw, M_PI) - cur_yaw);
      min_dis = target_predict_position.head(2).norm();
      hit_yaw = target_yaw;
      hit_pitch = target_pitch;
    }
  }
}

void ProjectileMotionNode::publishGimbalCommand(double hit_pitch, double hit_yaw, uint8_t shoot)
{
  sensor_msgs::msg::JointState gimbal_cmd;
  gimbal_cmd.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
  gimbal_cmd.position = {hit_pitch + offset_pitch_, hit_yaw + offset_yaw_};
  gimbal_cmd_publisher_->publish(gimbal_cmd);
  if (abs(hit_pitch - cur_pitch_) < 0.1 && abs(hit_yaw - cur_yaw_) < 0.1) {
    example_interfaces::msg::UInt8 shoot_cmd;
    shoot_cmd.data = shoot;
    shoot_cmd_publisher_->publish(shoot_cmd);
  }
}

}  // namespace projectile_motion

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(projectile_motion::ProjectileMotionNode)
