import os
import yaml
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # moveit.yaml is passed by filename for now since it's node specific
    moveit_yaml_file_name = (
        get_package_share_directory("uav_inspections_ros2") + "/config/moveit.yaml"
    )

    # Component yaml files are grouped in separate namespaces
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("uav_inspections_ros2"),
            "urdf",
            "custom_iris.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "uav_inspections_ros2", "config/model.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "uav_inspections_ros2", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    moveit_simple_controllers_yaml = load_yaml(
        "uav_inspections_ros2", "config/uav_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 999.0,
            "default_workspace_bounds": 9999.0,
            "longest_valid_segment_fraction": 0.001
        },
    }
    ompl_planning_yaml = load_yaml(
        "uav_inspections_ros2", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "uav_inspections_ros2", "config/uav_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "robot_description_planning.shape_transform_cache_lookup_wait_time": 1000.0,
    }
    
    sensors_3d_yaml = load_yaml(
        "uav_inspections_ros2", "config/sensors_3d.yaml"
    )

    use_sim_time = {"use_sim_time": True}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        #  output={"stdout": "log"},
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            sensors_3d_yaml,
            use_sim_time
        ],
    )

    uav_moveit = Node(
        #  name="uav_moveit",
        package="uav_inspections_ros2",
        executable="uav_moveit",
        output="screen",
        parameters=[
            #  moveit_yaml_file_name,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            #  ompl_planning_pipeline_config,
            #  trajectory_execution,
            #  moveit_controllers,
            #  planning_scene_monitor_parameters,
            #  sensors_3d_yaml,
            use_sim_time
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("uav_inspections_ros2") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        #  name="rviz2",
        output={"stdout": "log"},
        #  namespace="rviz",
        arguments=["-d", rviz_config_file],
        parameters=[use_sim_time, robot_description, robot_description_semantic, ompl_planning_pipeline_config],
        #  remappings=[("move_group_interface_node", "rviz_move_group_interface_node")],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_odom",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "odom", "base_link"],
        parameters=[use_sim_time]
    )

    # NED Static TF
    ned_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world",
        output="log",
        arguments=["0.0", "0.0", "0.0", "1.57079632679", "0.0", "3.14159265359", "world", "world_ned"],
        parameters=[use_sim_time]
    )

    simple_goal_publisher = Node(
        package="uav_inspections_ros2",
        executable="simple_goal_publisher",
        name="simple_goal_publisher",
        output="both",
        parameters=[use_sim_time]
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("uav_inspections_ros2"),
        "config",
        "uav_ros_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path, use_sim_time],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "uav_controller",
        "joint_state_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]


    return LaunchDescription(
        [
            simple_goal_publisher,
            ros2_control_node,
            move_group_node,
            ned_static_tf,
            uav_moveit,
            #  static_tf,
            rviz_node,
        ]
        + load_controllers
    )
