from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rec_file = LaunchConfiguration('rec_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'rec_file',
            default_value='recording.vsc',
            description='file to record into'
        ),

        Node(
            package='romur_control',
            executable='control_gui',
            name='control_gui_joy',
            parameters=[{
                'control_mode': 0,
                'rec_path': rec_file
            }]
        ),
    ])