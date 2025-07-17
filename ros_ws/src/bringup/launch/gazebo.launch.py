from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # ——— configurable params (exposed by LaunchConfiguration so sim.launch.py can override) ———
    world   = LaunchConfiguration('world',   default='simple_world.sdf')
    verbose = LaunchConfiguration('verbose', default='false')
    gui     = LaunchConfiguration('gui',     default='true')

    world_path = PathJoinSubstitution([
        FindPackageShare('bringup'), 'worlds', world
    ])

    gzserver = ExecuteProcess(
        cmd=['gz', 'sim', '--server',
             world_path,
             '--render-engine', 'ogre2',
             ('--verbose' if verbose == 'true' else '')
        ],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=["gz", "sim", "--client"],
        output="screen",
        condition=IfCondition(gui),
    )


    return LaunchDescription([
        # set_plugin_path,
        gzserver,
        gzclient,
    ])
