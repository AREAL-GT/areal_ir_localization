
"""
This launch file starts the ir localization algorithm onboard a mobile robot. It
also starts the pubsub node to publish the pose data to the px4 topics
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Create LaunchDescription
    ld = LaunchDescription()

    # Create parameter file paths
    cam_config = os.path.join(
        get_package_share_directory('areal_ir_localization'), 'config',
        'cam_msg_params.yaml'
        )
    algorithm_config = os.path.join(
        get_package_share_directory('areal_ir_localization'), 'config',
        'algorithm_params.yaml'
        )
    gimbal_config = os.path.join(
        get_package_share_directory('areal_ir_localization'), 'config',
        'gimbal_params.yaml'
        )
    debug_config = os.path.join(
        get_package_share_directory('areal_ir_localization'), 'config',
        'debug_params.yaml'
        )
    msg_lookup_table = os.path.join(
        get_package_share_directory('areal_ir_localization'),
        'config',
        'beacon_msg_12_lookup.yaml'
        )
    pos_lookup_table = os.path.join(
        get_package_share_directory('areal_ir_localization'),
        'config',
        'beacon_pos_lookup.yaml'
        )
        
    # Create ir localization node
    ir_node = Node(
        package = 'areal_ir_localization',
        executable = 'ir_localization',
        
        # Compile all parameter paths together for list
        parameters = [cam_config, algorithm_config, gimbal_config, 
                      debug_config, msg_lookup_table, pos_lookup_table]  
    )

    # Create px4 pubsub node
    pub_topic = '/x500_v2_IR/fmu/in/vehicle_visual_odometry'
    sub_topic = 'ir_cam_pose'
    px4_pubsub_node = Node(
        package='areal_px4_communication',
        executable='irpnp_px4_pubsub',
        parameters=[{'pub_topic': pub_topic, 'sub_topic': sub_topic}])

    # Add node to launch description
    ld.add_action(ir_node)
    ld.add_action(px4_pubsub_node)
    return ld

