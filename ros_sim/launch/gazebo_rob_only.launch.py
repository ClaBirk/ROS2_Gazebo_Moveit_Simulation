import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


	xacro_file = get_package_share_directory('ros_sim')+ "/urdf/kr10_gazebo_only.urdf.xacro"

	
	# Robot State Publisher 
	robot_state_publisher = Node(package    ='robot_state_publisher',
								 executable ='robot_state_publisher',
								 name       ='robot_state_publisher',
								 output     ='both',
								 parameters =[{'robot_description': Command(['xacro', ' ', xacro_file])           
								}])


	# Spawn the robot in Gazebo
	spawn_entity_robot = Node(package     ='gazebo_ros', 
							  executable  ='spawn_entity.py', 
							  arguments   = ['-entity', 'my_KR_10', '-topic', 'robot_description'],
							  output      ='screen')

	# Gazebo   
	world_file_name = 'empty.world'
	world = os.path.join(get_package_share_directory('ros_sim'), 'worlds', world_file_name)
	gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world,'-s', 'libgazebo_ros_factory.so'], output='screen')


	# load and START the controllers in launch file
	
	load_joint_state_broadcaster = ExecuteProcess(
										cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','joint_state_broadcaster'],
										output='screen')

	
	load_joint_trajectory_controller = ExecuteProcess( 
										cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'], 
										output='screen')


	return LaunchDescription([robot_state_publisher, spawn_entity_robot, gazebo_node, load_joint_state_broadcaster, load_joint_trajectory_controller  ])