#include <memory>
#include <thread>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("uav_moveit");

class MoveItPlanning : public rclcpp::Node {
public:
    MoveItPlanning(std::string name, rclcpp::NodeOptions node_options, std::shared_ptr<rclcpp::Node> move_group_node) : Node(name, node_options) {
        node_ = move_group_node;
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setPlanningTime(PLANNING_TIME_S);
        move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);

        ref_link_ = move_group_->getPoseReferenceFrame();
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("simple_goal_publisher/goal", 10);
        posestamped_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("uav_moveit/goal", 10, std::bind(&MoveItPlanning::poseStampedCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(0.1s, [this]() {
                planning();
            });
    }

    ~MoveItPlanning() {}

private:
    std::string ref_link_;
    bool currently_planning;
    rclcpp::Node::SharedPtr node_;
    double min_dist_threshold = 0.01;
    // TODO
    double min_quat_threshold = 0.01;
    rclcpp::TimerBase::SharedPtr timer_;
    const double PLANNING_TIME_S = 25.0;
    const double PLANNING_ATTEMPTS = 21.0;
    const std::string PLANNING_GROUP = "uav";
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped goal_tfs;
    geometry_msgs::msg::PoseStamped goal, moveit_goal;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr posestamped_sub_;

    void planning();
    void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
};

void MoveItPlanning::planning() {
    if (goal.header.frame_id != "" and !currently_planning) {
        currently_planning = true;
        geometry_msgs::msg::TransformStamped tfs = tf_buffer_->lookupTransform(ref_link_, goal.header.frame_id, tf2::TimePointZero);

        tf2::doTransform(goal, moveit_goal, tfs);
        // move_group_->clearPoseTargets();
        // move_group_->clearPathConstraints();
        std::vector<double> joint_group_positions(7);
        joint_group_positions[0] = moveit_goal.pose.position.x;
        joint_group_positions[1] = moveit_goal.pose.position.y;
        joint_group_positions[2] = moveit_goal.pose.position.z;
        joint_group_positions[3] = moveit_goal.pose.orientation.x;
        joint_group_positions[4] = moveit_goal.pose.orientation.y;
        joint_group_positions[5] = moveit_goal.pose.orientation.z;
        joint_group_positions[6] = moveit_goal.pose.orientation.w;
        RCLCPP_INFO(LOGGER, "Planning....");
        move_group_->setJointValueTarget(joint_group_positions);
        const bool plan_success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        currently_planning = false;
        RCLCPP_INFO(LOGGER, "=================================================================");
        RCLCPP_INFO(LOGGER, "Plan %s", plan_success ? "SUCCEEDED" : "FAILED");
        RCLCPP_INFO(LOGGER, "=================================================================");
        if (plan_success) {

            geometry_msgs::msg::PoseStamped g;
            g.header.stamp  = plan.trajectory_.multi_dof_joint_trajectory.header.stamp;
            g.header.frame_id = plan.trajectory_.multi_dof_joint_trajectory.header.frame_id;
            g.pose.position.x = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].translation.x;
            g.pose.position.y = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].translation.y;
            g.pose.position.z = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].translation.z;
            g.pose.orientation.x = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].rotation.x;
            g.pose.orientation.y = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].rotation.y;
            g.pose.orientation.z = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].rotation.z;
            g.pose.orientation.w = plan.trajectory_.multi_dof_joint_trajectory.points[0].transforms[0].rotation.w;

            tf2::doTransform(g, g, goal_tfs);

            goal_pub_->publish(g);
        }
    }
    // TODO handle goal reached with something like this
    // if (sqrt(pow(moveit_goal.pose.position.x,2) + pow(moveit_goal.pose.position.y,2) + pow(moveit_goal.pose.position.z,2)) <= min_dist_threshold) {
        // // Disable the goal
        // goal.header.frame_id = "";
        // moveit_goal.header.frame_id = "";
    // }
}

void MoveItPlanning::poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal.header.frame_id = msg->header.frame_id;
    goal.pose.position.x = msg->pose.position.x;
    goal.pose.position.y = msg->pose.position.y;
    goal.pose.position.z = msg->pose.position.z;
    goal.pose.orientation.x = msg->pose.orientation.x;
    goal.pose.orientation.y = msg->pose.orientation.y;
    goal.pose.orientation.z = msg->pose.orientation.z;
    goal.pose.orientation.w = msg->pose.orientation.w;

    goal_tfs = tf_buffer_->lookupTransform("world", goal.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(goal, goal, goal_tfs);
    goal_tfs.transform.translation.x = goal.pose.position.x;
    goal_tfs.transform.translation.y = goal.pose.position.y;
    goal_tfs.transform.translation.z = goal.pose.position.z;
    goal_tfs.transform.rotation.x = goal.pose.orientation.x;
    goal_tfs.transform.rotation.y = goal.pose.orientation.y;
    goal_tfs.transform.rotation.z = goal.pose.orientation.z;
    goal_tfs.transform.rotation.w = goal.pose.orientation.w;

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr move_group_node = rclcpp::Node::make_shared("uav_moveit_move_group_helper_node", node_options);
    rclcpp::executors::MultiThreadedExecutor executor;
    // rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    rclcpp::spin(std::make_shared<MoveItPlanning>("uav_moveit", node_options, move_group_node));

    rclcpp::shutdown();
    return 0;
}

