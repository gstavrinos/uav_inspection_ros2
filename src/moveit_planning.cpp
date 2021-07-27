#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("uav_moveit");

class MoveItPlanning {
public:
  MoveItPlanning(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
  {
  }

  void run() {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit_cpp::PlanningComponent uav("uav", moveit_cpp_);

    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goal");
    uav.setGoal("extended");

    // Run actual plan
    RCLCPP_INFO(LOGGER, "Plan to goal");
    const auto plan_solution = uav.plan();
    // if (plan_solution)
    // {
      // RCLCPP_INFO(LOGGER, "uav.execute()");
      // uav.execute();
    // }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
};

int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("uav_moveit", "", node_options);

  MoveItPlanning uav_moveit(node);
  std::thread moveit_thread([&uav_moveit]() {
    // Let RViz initialize before running demo
    // TODO(henningkayser): use lifecycle events to launch node
    rclcpp::sleep_for(std::chrono::seconds(5));
    uav_moveit.run();
  });

  rclcpp::spin(node);
  moveit_thread.join();

  return 0;
}
