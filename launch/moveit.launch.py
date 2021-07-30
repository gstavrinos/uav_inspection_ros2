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
            "start_state_max_bounds_error": 0.1,
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
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    #  uav_moveit = Node(
        #  name="uav_moveit",
        #  package="uav_inspections_ros2",
        #  executable="uav_moveit",
        #  output="screen",
        #  parameters=[
            #  moveit_yaml_file_name,
            #  robot_description,
            #  robot_description_semantic,
            #  kinematics_yaml,
            #  ompl_planning_pipeline_config,
            #  moveit_controllers,
        #  ],
    #  )

    # RViz
    rviz_config_file = (
        get_package_share_directory("uav_inspections_ros2") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description, robot_description_semantic],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    #  robot_state_publisher = Node(
        #  package="robot_state_publisher",
        #  executable="robot_state_publisher",
        #  name="robot_state_publisher",
        #  output="both",
        #  parameters=[robot_description],
    #  )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("uav_inspections_ros2"),
        "config",
        "uav_ros_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
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
            static_tf,
            #  robot_state_publisher,
            #  uav_moveit,
            move_group_node,
            ros2_control_node,
            rviz_node,
        ]
        + load_controllers
    )
