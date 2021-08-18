#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("uav_moveit");

class MoveItPlanning : public rclcpp::Node {
public:
    MoveItPlanning(rclcpp::NodeOptions node_options) : Node("moveit_planning", "", node_options) {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
        move_group_->setPlanningTime(PLANNING_TIME_S);
        move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
        ref_link_ = move_group_->getPoseReferenceFrame();
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("simple_goal_publisher/goal", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom/perfect", 10, std::bind(&MoveItPlanning::odomCallback, this, _1));
        posestamped_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("moveit_planning/goal", 10, std::bind(&MoveItPlanning::poseStampedCallback, this, _1));
    }

    ~MoveItPlanning() {}

private:
    std::string ref_link_;
    rclcpp::Node::SharedPtr node_;
    double min_dist_threshold = 0.01;
    const double PLANNING_TIME_S = 30.0;
    const double PLANNING_ATTEMPTS = 5.0;
    geometry_msgs::msg::PoseStamped goal;
    const std::string PLANNING_GROUP = "uav";
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr posestamped_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr);
    void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};
    
// TODO instead of odom I should use a state monitor
void MoveItPlanning::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (goal.header.frame_id != "") {
        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();
        move_group_->setStartStateToCurrentState();
        moveit::core::RobotStatePtr goal_state = move_group_->getCurrentState(10);
        std::vector<double> joint_group_positions;
        const moveit::core::JointModelGroup *joint_model_group_ = move_group_->getRobotModel()->getJointModelGroup(PLANNING_GROUP);
        goal_state->copyJointGroupPositions(joint_model_group_, joint_group_positions);
        joint_group_positions[0] = goal.pose.position.x;
        joint_group_positions[1] = goal.pose.position.y;
        joint_group_positions[2] = goal.pose.position.z;
        joint_group_positions[3] = goal.pose.orientation.x;
        joint_group_positions[4] = goal.pose.orientation.y;
        joint_group_positions[5] = goal.pose.orientation.z;
        joint_group_positions[6] = goal.pose.orientation.w;
        move_group_->setJointValueTarget(joint_group_positions);
        const bool plan_success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "=================================================================");
        RCLCPP_INFO(LOGGER, "Plan %s", plan_success ? "SUCCEEDED" : "FAILED");
        RCLCPP_INFO(LOGGER, "=================================================================");
        if (plan_success) {
            geometry_msgs::msg::PoseStamped g;
            g.header.frame_id = goal.header.frame_id;
            g.header.stamp  = this->now();
            g.pose.position.x = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].translation.x;
            g.pose.position.y = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].translation.y;
            g.pose.position.z = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].translation.z;
            g.pose.orientation.x = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].rotation.x;
            g.pose.orientation.y = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].rotation.y;
            g.pose.orientation.z = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].rotation.z;
            g.pose.orientation.w = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].rotation.w;
            goal_pub_->publish(g);
        }
    }
}

void MoveItPlanning::poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    goal.header.frame_id = msg->header.frame_id;
    goal.pose.position.x= msg->pose.position.x;
    goal.pose.position.y= msg->pose.position.y;
    goal.pose.position.z= msg->pose.position.z;
    goal.pose.orientation.x= msg->pose.orientation.x;
    goal.pose.orientation.y= msg->pose.orientation.y;
    goal.pose.orientation.z= msg->pose.orientation.z;
    goal.pose.orientation.w= msg->pose.orientation.w;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    node_options.automatically_declare_parameters_from_overrides(true);
    // rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("uav_moveit", "", node_options);
    rclcpp::spin(std::make_shared<MoveItPlanning>(node_options));

    // MoveItPlanning uav_moveit(node);
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

