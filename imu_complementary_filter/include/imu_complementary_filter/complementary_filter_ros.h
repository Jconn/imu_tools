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

#ifndef IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
#define IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "tf2_ros/message_filter.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer_interface.h"

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "imu_complementary_filter/complementary_filter.h"

namespace imu_tools {

class ComplementaryFilterROS : public rclcpp_lifecycle::LifecycleNode 
{
  public:
    explicit ComplementaryFilterROS(const std::string & node_name, bool intra_process_comms = false);
    virtual ~ComplementaryFilterROS();

  private:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
      on_configure(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
      on_activate(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
      on_deactivate(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
      on_cleanup(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
      on_shutdown(const rclcpp_lifecycle::State &) override;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu> > imu_sub_;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::MagneticField> > mag_sub_;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imu_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>> rpy_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> state_pub_;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
         
    // Parameters:
    bool use_mag_;
    bool publish_tf_;
    bool reverse_tf_;
    double constant_dt_;
    bool publish_debug_topics_;
    std::string fixed_frame_;
    std::string imu_frame_;
    double orientation_variance_;

    // State variables:
    ComplementaryFilter filter_;
    rclcpp::Time time_prev_;
    bool initialized_filter_;
    sensor_msgs::msg::Imu imu_msg_;
    std::shared_ptr<sensor_msgs::msg::MagneticField> mag_msg_ = nullptr;

    void onInit();
    void initializeParams();
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw);
    void imuMagCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw,
                        const sensor_msgs::msg::MagneticField::SharedPtr mag_msg);

    void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr mag_msg);

    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::MagneticField, sensor_msgs::msg::Imu>> sync_;

    void publish(const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw);

    tf2::Quaternion hamiltonToTFQuaternion(
        double q0, double q1, double q2, double q3) const;
};

}  // namespace imu_tools

#endif // IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
