from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import yaml


# def augment_moveit_config(moveit_config):
#     """Add cuMotion and its config to the planning_pipelines dict of a MoveItConfigs object."""
#     config_file_path = os.path.join(
#         get_package_share_directory('isaac_ros_cumotion_moveit'),
#         'config',
#         'isaac_ros_cumotion_planning.yaml'
#     )
    
#     # if can't find the config file, use local config
#     if not os.path.exists(config_file_path):
#         config_file_path = os.path.join(
#             get_package_share_directory('wheeled_humanoid_moveit'),
#             'config',
#             'isaac_ros_cumotion_planning.yaml'
#         )
    
#     with open(config_file_path) as config_file:
#         config = yaml.safe_load(config_file)
    
#     moveit_config.planning_pipelines['planning_pipelines'].append('isaac_ros_cumotion')
#     moveit_config.planning_pipelines['isaac_ros_cumotion'] = config
#     moveit_config.planning_pipelines['default_planning_pipeline'] = 'isaac_ros_cumotion'


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_model", package_name="wheeled_humanoid_moveit").to_moveit_configs()
    # augment_moveit_config(moveit_config)
    return generate_demo_launch(moveit_config)
