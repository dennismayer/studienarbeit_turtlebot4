#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path
from dotenv import load_dotenv

def generate_launch_description():
    # === LOAD .env FILE BEFORE ANYTHING ELSE ===
    workspace_root = Path(__file__).resolve().parents[3]  # Go up to workspace root
    env_file = workspace_root / '.env'
    
    if env_file.exists():
        print(f"✓ Loading .env from: {env_file}")
        load_dotenv(env_file)
    else:
        print(f"⚠ WARNING: .env file not found at {env_file}")
    
    # Get package directory
    pkg_dir = get_package_share_directory('car_museum_guide')
    
    # Path to params file
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Declare launch argument for custom params file (optional override)
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file'
    )
    
    enable_visitor_detection_arg = DeclareLaunchArgument(
        'enable_visitor_detection',
        default_value='true',
        description='Enable visitor detection using LiDAR'
    )
    
    # === SET ENVIRONMENT VARIABLES FOR ALL NODES ===
    set_azure_endpoint = SetEnvironmentVariable(
        'AZURE_OPENAI_ENDPOINT',
        os.getenv('AZURE_OPENAI_ENDPOINT', '')
    )
    
    set_azure_key = SetEnvironmentVariable(
        'AZURE_OPENAI_API_KEY',
        os.getenv('AZURE_OPENAI_API_KEY', '')
    )
    
    set_azure_deployment = SetEnvironmentVariable(
        'AZURE_OPENAI_DEPLOYMENT_NAME',
        os.getenv('AZURE_OPENAI_DEPLOYMENT_NAME', '')
    )
    
    set_api_version = SetEnvironmentVariable(
        'API_VERSION',
        os.getenv('API_VERSION', '2024-08-01-preview')
    )
    
    set_porcupine_key = SetEnvironmentVariable(
        'PORCUPINE_ACCESS_KEY',
        os.getenv('PORCUPINE_ACCESS_KEY', '')
    )
    
    # Visitor detection node
    visitor_detection_node = Node(
        package='car_museum_guide',
        executable='visitor_detection_node',
        name='visitor_detection',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(LaunchConfiguration('enable_visitor_detection'))
    )
    
    # Speech recognition node with wake word
    speech_recognition_node = Node(
        package='car_museum_guide',
        executable='speech_recognition_node',
        name='speech_recognition',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    # Car museum guide controller node
    museum_guide_node = Node(
        package='car_museum_guide',
        executable='car_museum_guide_node',
        name='car_museum_guide',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    return LaunchDescription([
        # Set environment variables FIRST
        set_azure_endpoint,
        set_azure_key,
        set_azure_deployment,
        set_api_version,
        set_porcupine_key,
        
        # Then launch arguments
        params_file_arg,
        enable_visitor_detection_arg,
        
        # Finally nodes
        visitor_detection_node,
        speech_recognition_node,
        museum_guide_node
    ])