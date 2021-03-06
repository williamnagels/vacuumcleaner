from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


# https://github.com/bponsler/xacro
def xacro_to_urdf(xacro_name, urdf_name):
    from xacro import process_file, open_output
    doc = process_file(get_package_share_directory("vacuumcleaner")+'/'+xacro_name)
    out = open_output(get_package_share_directory("vacuumcleaner")+'/'+urdf_name)
    out.write(doc.toprettyxml(indent='  '))


# https://stackoverflow.com/questions/1432126/how-to-get-content-of-a-small-ascii-file-in-python
def file_get_contents(filename):
    with open(filename) as f:
        return f.read()


def generate_launch_description():
    xacro_to_urdf("robot.xacro", "robot.urdf")

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    nav2_launchfile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("nav2_bringup"), '/launch/navigation_launch.py']),
    )

    model_path = file_get_contents(get_package_share_directory("vacuumcleaner")+'/'+"robot.urdf")

    return LaunchDescription([
        DeclareLaunchArgument('simulation',
                              default_value="false",
                              description='Use gazebo to simulate robot i.s.o. physical robot. Requires "gazebo_ros" '
                                          'ros package to be installed. Supply world file using "world:=<file>"'),
        nav2_launchfile,
        joint_state_publisher_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("gazebo_ros"), '/launch/gazebo.launch.py']),
            condition=IfCondition(LaunchConfiguration('simulation')),
            launch_arguments=[("world",[get_package_share_directory("vacuumcleaner"), '/', LaunchConfiguration('world')])] #"/home/william/ros_ws/src/vacuumcleaner/models/worlds/square_2m"
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_tf_static': False,
                         'use_sim_time': True,
                         'robot_description': model_path}],
            remappings=[('/joint_states', '/vacuumcleaner/joint_states')]),

        Node(executable='rviz2', package='rviz2', arguments=[' -d rviz']),
        Node(package="vacuumcleaner", executable="spawn_entity", arguments=["0.0", "0.0", "0.0"])
    ])
