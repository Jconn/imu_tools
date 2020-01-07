IMU tools for ROS
===================================

Overview
-----------------------------------
Initial ROS2 port. Currently only supports: 

 * `imu_complementary_filter`: a filter which fuses angular velocities,
accelerations, and (optionally) magnetic readings from a generic IMU 
device into an orientation quaternion using a novel approach based on a complementary fusion. Based on the work of [2].

TODO:
 * `imu_filter_madgwick`

 * `rviz_imu_plugin`

Installing
-----------------------------------

More info
-----------------------------------

http://wiki.ros.org/imu_tools

License
-----------------------------------

 * `imu_filter_madgwick`: currently licensed as GPL, following the original implementation
 
 * `imu_complementary_filter`: BSD

 * `rviz_imu_plugin`: BSD

References
-----------------------------------
 [1] http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

 [2] http://www.mdpi.com/1424-8220/15/8/19302
