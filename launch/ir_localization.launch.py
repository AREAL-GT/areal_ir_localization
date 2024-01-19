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
    node=Node(

        package = 'areal_ir_localization',
        executable = 'ir_localization',
        
        # Compile all parameter paths together for list
        parameters = [cam_config, algorithm_config, gimbal_config, 
                      debug_config, msg_lookup_table, pos_lookup_table]
        
    )

    # Add node to launch description
    ld.add_action(node)
    return ld