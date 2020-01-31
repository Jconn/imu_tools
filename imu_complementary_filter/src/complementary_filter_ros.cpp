/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

	@section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "imu_complementary_filter/complementary_filter_ros.h"

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/convert.h>
#include "include/imu_transformer/tf2_sensor_msgs.h"
namespace imu_tools {

ComplementaryFilterROS::ComplementaryFilterROS(const std::string & node_name, bool intra_process_comms)
    : rclcpp_lifecycle::LifecycleNode(node_name,
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)), 
    initialized_filter_(false)
{
  RCLCPP_INFO(get_logger(), "Starting ComplementaryFilterROS");
  onInit();

}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ComplementaryFilterROS::on_cleanup(const rclcpp_lifecycle::State &)
{

  RCLCPP_INFO(get_logger(), "cleanup complete");
  imu_pub_.reset();
  rpy_pub_.reset();
  state_pub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ComplementaryFilterROS::on_shutdown(const rclcpp_lifecycle::State &)
{

  RCLCPP_INFO(get_logger(), "shutdown complete");
  imu_pub_.reset();
  rpy_pub_.reset();
  state_pub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ComplementaryFilterROS::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "activation complete");
  imu_pub_->on_activate();
  rpy_pub_->on_activate();
  state_pub_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  ComplementaryFilterROS::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "deactivation complete");
  imu_pub_->on_deactivate();
  rpy_pub_->on_deactivate();
  state_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ComplementaryFilterROS::on_configure(const rclcpp_lifecycle::State &)
{


  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // tfBuffer = new tf2_ros::Buffer(node->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  RCLCPP_INFO(get_logger(), "configure complete");
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  int queue_size = 25;

  // Register publishers:
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS() );

  rpy_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "imu/rpy/filtered", rclcpp::SensorDataQoS() );

  state_pub_ = this->create_publisher<std_msgs::msg::Bool>("imu/steady_state", rclcpp::SensorDataQoS() );

  // Register IMU raw data subscriber.
  //
  imu_sub_ =  this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&ComplementaryFilterROS::imuCallback, this, std::placeholders::_1)
      );

  // Register magnetic data subscriber.
  if (use_mag_)
  {

    mag_sub_ =  this->create_subscription<sensor_msgs::msg::MagneticField>(
        "imu/mag",
        rclcpp::SensorDataQoS(),
        std::bind(&ComplementaryFilterROS::magCallback, this, std::placeholders::_1)
        );

    //sync_->registerCallback(std::bind(&ComplementaryFilterROS::imuMagCallback, this->shared_from_this(), std::placeholders::_1, std::placeholders::_2) );

  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

ComplementaryFilterROS::~ComplementaryFilterROS()
{
  RCLCPP_INFO(get_logger(), "Destroying ComplementaryFilterROS");
}
void ComplementaryFilterROS::onInit()
{

  initializeParams();

}
void ComplementaryFilterROS::initializeParams()
{
  double gain_acc;
  double gain_mag;
  bool do_bias_estimation;
  double bias_alpha;
  bool do_adaptive_gain;
  double orientation_stddev;
  imu_frame_ = "";
  this->declare_parameter("fixed_frame", "odom");
  this->declare_parameter("use_mag", false);
  this->declare_parameter("publish_tf", false);
  this->declare_parameter("reverse_tf", false);
  this->declare_parameter("constant_dt", 0.0);
  this->declare_parameter("publish_debug_topics", false);
  this->declare_parameter("gain_acc", 0.01);
  this->declare_parameter("gain_mag", 0.01);
  this->declare_parameter("do_bias_estimation", true);
  this->declare_parameter("bias_alpha", 0.01);
  this->declare_parameter("do_adaptive_gain", true);
  this->declare_parameter("orientation_stddev", 0.0);

  this->get_parameter("fixed_frame", fixed_frame_);
  this->get_parameter("use_mag", use_mag_);
  this->get_parameter("publish_tf", publish_tf_);
  this->get_parameter("reverse_tf", reverse_tf_);
  this->get_parameter("constant_dt", constant_dt_);
  this->get_parameter("publish_debug_topics", publish_debug_topics_);
  this->get_parameter("gain_acc", gain_acc);
  this->get_parameter("gain_mag", gain_mag);
  this->get_parameter("do_bias_estimation", do_bias_estimation);
  this->get_parameter("bias_alpha", bias_alpha);
  this->get_parameter("do_adaptive_gain", do_adaptive_gain);
  this->get_parameter("orientation_stddev", orientation_stddev);


  orientation_variance_ = orientation_stddev * orientation_stddev;

  filter_.setDoBiasEstimation(do_bias_estimation);
  filter_.setDoAdaptiveGain(do_adaptive_gain);

  if(!filter_.setGainAcc(gain_acc))
    RCLCPP_WARN(get_logger(), "Invalid gain_acc passed to ComplementaryFilter.");
  if (use_mag_)
  {
    if(!filter_.setGainMag(gain_mag))
      RCLCPP_WARN(get_logger(), "Invalid gain_mag passed to ComplementaryFilter.");
  }
  if (do_bias_estimation)
  {
    if(!filter_.setBiasAlpha(bias_alpha))
      RCLCPP_WARN(get_logger(), "Invalid bias_alpha passed to ComplementaryFilter.");
  }

  // check for illegal constant_dt values
  if (constant_dt_ < 0.0)
  {
    // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
    // otherwise, it will be constant
    RCLCPP_WARN(get_logger(), "constant_dt parameter is %f, must be >= 0.0. Setting to 0.0", constant_dt_);
    constant_dt_ = 0.0;
  }
}

void ComplementaryFilterROS::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw)
{
  imu_frame_ = imu_msg_raw->header.frame_id;
  if(mag_msg_ != nullptr)
  {
    imuMagCallback(imu_msg_raw, mag_msg_);
    mag_msg_ = nullptr;
    return;
  }
  const geometry_msgs::msg::Vector3& a = imu_msg_raw->linear_acceleration;
  const geometry_msgs::msg::Vector3& w = imu_msg_raw->angular_velocity;
  const rclcpp::Time& time = imu_msg_raw->header.stamp;

  // Initialize.
  if (!initialized_filter_)
  {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // determine dt: either constant, or from IMU timestamp
  double dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - time_prev_).seconds();

  time_prev_ = time;

  // Update the filter.
  filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);

  // Publish state.
  publish(imu_msg_raw);
}

void ComplementaryFilterROS::magCallback(const sensor_msgs::msg::MagneticField::SharedPtr mag_msg)
{

  if (imu_frame_ == "") return;

  try
  {
      sensor_msgs::msg::MagneticField mag_out;
      tfBuffer_->transform(*mag_msg, mag_out, imu_frame_);
      mag_msg_ = std::make_shared<sensor_msgs::msg::MagneticField>(mag_out);
  }
  catch (tf2::TransformException ex)
  {

  }
}

void ComplementaryFilterROS::imuMagCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw,
                                            const sensor_msgs::msg::MagneticField::SharedPtr mag_msg)
{
  const geometry_msgs::msg::Vector3& a = imu_msg_raw->linear_acceleration;
  const geometry_msgs::msg::Vector3& w = imu_msg_raw->angular_velocity;
  const geometry_msgs::msg::Vector3& m = mag_msg->magnetic_field;
  const rclcpp::Time& time = imu_msg_raw->header.stamp;

  // Initialize.
  if (!initialized_filter_)
  {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // Calculate dt.
  double dt = (time - time_prev_).seconds();
  time_prev_ = time;
   //ros::Time t_in, t_out;
  //t_in = ros::Time::now();
  // Update the filter.
  if (std::isnan(m.x) || std::isnan(m.y) || std::isnan(m.z))
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);
  else
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, m.x, m.y, m.z, dt);

  //t_out = ros::Time::now();
  //float dt_tot = (t_out - t_in).toSec() * 1000.0; // In msec.
  //printf("%.6f\n", dt_tot);
  // Publish state.
  publish(imu_msg_raw);
}

tf2::Quaternion ComplementaryFilterROS::hamiltonToTFQuaternion(
    double q0, double q1, double q2, double q3) const
{
  // ROS uses the Hamilton quaternion convention (q0 is the scalar). However,
  // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
  return tf2::Quaternion(q1, q2, q3, q0);
}

void ComplementaryFilterROS::publish(
    const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw)
{
  // Get the orientation:
  double q0, q1, q2, q3;
  filter_.getOrientation(q0, q1, q2, q3);
  tf2::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

  // Create and publish fitlered IMU message.
  imu_msg_ = *imu_msg_raw;
  tf2::convert(q, imu_msg_.orientation);

  imu_msg_.orientation_covariance[0] = orientation_variance_;
  imu_msg_.orientation_covariance[1] = 0.0;
  imu_msg_.orientation_covariance[2] = 0.0;
  imu_msg_.orientation_covariance[3] = 0.0;
  imu_msg_.orientation_covariance[4] = orientation_variance_;
  imu_msg_.orientation_covariance[5] = 0.0;
  imu_msg_.orientation_covariance[6] = 0.0;
  imu_msg_.orientation_covariance[7] = 0.0;
  imu_msg_.orientation_covariance[8] = orientation_variance_;

  // Account for biases.
  if (filter_.getDoBiasEstimation())
  {
    imu_msg_.angular_velocity.x -= filter_.getAngularVelocityBiasX();
    imu_msg_.angular_velocity.y -= filter_.getAngularVelocityBiasY();
    imu_msg_.angular_velocity.z -= filter_.getAngularVelocityBiasZ();
  }

  imu_pub_->publish(imu_msg_);

  if (publish_debug_topics_)
  {
      // Create and publish roll, pitch, yaw angles
      geometry_msgs::msg::Vector3Stamped rpy;
      rpy.header = imu_msg_raw->header;

      tf2::Matrix3x3 M;
      M.setRotation(q);
      double x= filter_.getAngularVelocityBiasX();
      double y = filter_.getAngularVelocityBiasY();
      double z = filter_.getAngularVelocityBiasZ();

      M.getRPY(x, y, z);
      rpy_pub_->publish(rpy);

      // Publish whether we are in the steady state, when doing bias estimation
      if (filter_.getDoBiasEstimation())
      {
        std_msgs::msg::Bool state_msg;
        state_msg.data = filter_.getSteadyState();
        state_pub_->publish(state_msg);
      }
  }

  if (publish_tf_)
  {
      // Create and publish the ROS tf.
      tf2::Transform transform;
      geometry_msgs::msg::TransformStamped transform_msg;

      transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      transform.setRotation(q);

      transform_msg.header.stamp = imu_msg_raw->header.stamp;
      transform_msg.header.frame_id = fixed_frame_;
      transform_msg.child_frame_id = imu_msg_raw->header.frame_id;

      if (reverse_tf_)
      {
        transform = transform.inverse();
      }

      transform_msg.transform = tf2::toMsg(transform);
      //transform_msg.transform = transform;
      //transform_msg.transform.rotation.x = transform.x();
      tf_broadcaster_->sendTransform(transform_msg);
  }
}

}  // namespace imu_tools
