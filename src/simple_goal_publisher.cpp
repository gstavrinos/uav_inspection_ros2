#include <chrono>
#include <iostream>
#include <stdint.h>

#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/timesync.hpp>
#include <tf2_ros/transform_listener.h>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <px4_msgs/msg/offboard_control_mode.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleGoalPublisher: public rclcpp::Node {
    public:
        SimpleGoalPublisher() : Node("simple_goal_publisher") {
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
            trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
            vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("VehicleCommand_PubSubTopic", 10);
            timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10, 
                            [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
                                timestamp_.store(msg->timestamp);
                            });

            posestamped_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("simple_goal_publisher/goal", 10, std::bind(&SimpleGoalPublisher::poseStampedCallback, this, _1));

            robot_trajectory_sub_ = this->create_subscription<moveit_msgs::msg::RobotTrajectory>("move_group/plan", 10, std::bind(&SimpleGoalPublisher::robotTrajectoryCallback, this, _1));

            // Start hovering at 1 meter
            goal.z = -1.0;

            offboard_setpoint_counter_ = 0;

            auto timer_callback = [this]() -> void {

            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
            publishOffboardControlMode();
            publishTrajectorySetpoint();

            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
            };
            timer_ = this->create_wall_timer(100ms, timer_callback);
        }

        void arm() const;
        void disarm() const;

    private:
        rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
        rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr posestamped_sub_;
        rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr robot_trajectory_sub_;

        std::atomic<uint64_t> timestamp_;

        uint64_t offboard_setpoint_counter_;
        px4_msgs::msg::TrajectorySetpoint goal;

        void publishOffboardControlMode() const;
        void publishTrajectorySetpoint() const;
        void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
        void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
        void robotTrajectoryCallback(const moveit_msgs::msg::RobotTrajectory::SharedPtr);
};

void SimpleGoalPublisher::poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Got a new goal position!");
    if (msg->header.frame_id == "") {
        RCLCPP_WARN(this->get_logger(), "Goal has no frame_id. Dismissing...");
        return;
    }
    geometry_msgs::msg::PoseStamped g;
    geometry_msgs::msg::TransformStamped tfs = tf_buffer_->lookupTransform("world_ned", msg->header.frame_id, tf2::TimePointZero);

    tf2::doTransform(*msg, g, tfs);
    goal.timestamp = timestamp_.load();
    goal.x = g.pose.position.x;
    goal.y = g.pose.position.y;
    goal.z = g.pose.position.z;
    double siny_cosp = 2 * (g.pose.orientation.w * g.pose.orientation.z + g.pose.orientation.x * g.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (g.pose.orientation.y * g.pose.orientation.y + g.pose.orientation.z * g.pose.orientation.z);
    goal.yaw = std::atan2(siny_cosp, cosy_cosp);
}

void SimpleGoalPublisher::robotTrajectoryCallback(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Got a new plan!");
    // TODO delete this callback
}

void SimpleGoalPublisher::arm() const {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void SimpleGoalPublisher::disarm() const {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void SimpleGoalPublisher::publishOffboardControlMode() const {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = timestamp_.load();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_publisher_->publish(msg);
}


void SimpleGoalPublisher::publishTrajectorySetpoint() const {
    // std::cout << "GOAL:[" << goal.x << "," << goal.y << "," << goal.z << "]" << std::endl;
    trajectory_setpoint_publisher_->publish(goal);
}

void SimpleGoalPublisher::publishVehicleCommand(uint16_t command, float param1, float param2) const {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = timestamp_.load();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleGoalPublisher>());

    rclcpp::shutdown();
    return 0;
}
