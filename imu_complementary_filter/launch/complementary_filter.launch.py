import os
import launch
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch_ros.actions import LifecycleNode

from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg

def generate_launch_description():
    imu_node = LifecycleNode(
            package = "imu_complementary_filter",
            node_executable = "complementary_filter_node",
            node_name = "complementary_filter",
            output = "screen",
            parameters=[{
                'do_bias_estimation': True,
                'do_adaptive_gain': True,
                'use_mag': True,
                'gain_acc': 0.01,
                'gain_mag': 0.01,
                'publish_tf': False,
                'orientation_stddev': 0.01
                }]
            )
    # Make the imu node take the 'configure' transition
    imu_configure_trans_event = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher = launch.events.matches_action(imu_node),
                transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )
            )

    # Make the ZED node take the 'activate' transition
    imu_activate_trans_event = EmitEvent(
            event = ChangeState(
                lifecycle_node_matcher = launch.events.matches_action(imu_node),
                transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )
            )

    # When the IMU node reaches the 'inactive' state, make it take the 'activate' transition and start the Robot State Publisher
    imu_inactive_state_handler = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node = imu_node,
                goal_state = 'inactive',
                entities = [
                    # Log
                    LogInfo( msg = "'IMU' reached the 'INACTIVE' state'. Now 'activating'." ),
                    # Change State event ( inactive -> active )
                    imu_activate_trans_event,
                    ],
                )
            )

    # When the ZED node reaches the 'active' state, log a message.
    imu_active_state_handler = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node = imu_node,
                goal_state = 'active',
                entities = [
                    # Log
                    LogInfo( msg = "'IMU' reached the 'ACTIVE' state" ),
                    ],
                )
            )
    # Launch Description
    ld = launch.LaunchDescription()
    ld.add_action(imu_inactive_state_handler)
    ld.add_action(imu_active_state_handler)
    ld.add_action(imu_node)
    ld.add_action(imu_configure_trans_event)

    return ld 
