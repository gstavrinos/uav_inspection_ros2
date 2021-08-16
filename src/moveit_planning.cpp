#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("uav_moveit");

class MoveItPlanning {
public:
    MoveItPlanning(const rclcpp::Node::SharedPtr& node) : node_(node) {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
        move_group_->setPlanningTime(PLANNING_TIME_S);
        move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
        ref_link_ = move_group_->getPoseReferenceFrame();
    }

void run() {
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->setStartStateToCurrentState();

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = ref_link_;
    goal_pose.pose.position.x = 0.5;
    goal_pose.pose.position.y = -0.25;
    goal_pose.pose.position.z = 1.0;
    goal_pose.pose.orientation.x = 0.707;
    goal_pose.pose.orientation.w = 0.707;

    move_group_->setPoseTarget(goal_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool plan_success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan %s", plan_success ? "SUCCEEDED" : "FAILED");
}

private:
    std::string ref_link_;
    const double PLANNING_TIME_S = 30.0;
    const double PLANNING_ATTEMPTS = 5.0;
    const std::string PLANNING_GROUP = "uav";
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

int main(int argc, char** argv) {
    RCLCPP_INFO(LOGGER, "Initializing node");
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("uav_moveit", "", node_options);

    MoveItPlanning uav_moveit(node);
    std::thread moveit_thread([&uav_moveit]() {
        rclcpp::sleep_for(std::chrono::seconds(10));
        uav_moveit.run();
    });

    rclcpp::spin(node);
    moveit_thread.join();

    rclcpp::shutdown();

    return 0;
}

