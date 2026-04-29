from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    pkg_path = get_package_share_path('robot_description')
    default_model_path  = pkg_path / 'urdf/robot.urdf'
    default_rviz_config = pkg_path / 'rviz/urdf.rviz'

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Ruta absoluta al URDF')

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config),
        description='Ruta absoluta a la config de RViz2')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}])

    # El nodo IK está en el paquete visual_pubsub
    inverse_kinematics_node = Node(
        package='visual_pubsub',
        executable='inverse_kinematics',
        name='inverse_kinematics',
        output='screen')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')])

    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        inverse_kinematics_node,
        rviz_node,
    ])