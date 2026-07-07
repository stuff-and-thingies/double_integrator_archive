#include <chrono>
#include <memory>
#include <optional>

#include "autonomy_control/trajectory_tracker.hpp"
#include "autonomy_core/trajectory.hpp"
#include "autonomy_msgs/msg/acceleration_command.hpp"
#include "autonomy_msgs/msg/fake_drone_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class TrackerNode : public rclcpp::Node
{
   public:
    TrackerNode() : rclcpp::Node("tracker_node")
    {
        drone_state_sub_ =
            this->create_subscription<autonomy_msgs::msg::FakeDroneState>(
                "/fake_drone/state", rclcpp::SensorDataQoS(),
                [this](const autonomy_msgs::msg::FakeDroneState::SharedPtr msg)
                { update_latest_drone_state(msg); });

        trajectory_odom_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/trajectory/reference", rclcpp::SensorDataQoS(),
                [this](nav_msgs::msg::Odometry::SharedPtr msg)
                { update_latest_reference_point(msg); });

        accel_cmd_raw_pub_ =
            this->create_publisher<autonomy_msgs::msg::AccelerationCommand>(
                "/tracker/command_raw", rclcpp::SensorDataQoS());

        error_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/tracker/error", rclcpp::SensorDataQoS());
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                         [this]()
                                         {
                                             calculate_and_publish_accel_cmd();
                                             update_and_publish_error();
                                         });
    }

   private:
    void update_latest_drone_state(
        const autonomy_msgs::msg::FakeDroneState::SharedPtr msg)
    {
        if (!latest_drone_state_)
        {
            latest_drone_state_ = autonomy_sim::FakeDroneState{};
        }

        latest_drone_state_->position.x = msg->position.x;
        latest_drone_state_->position.y = msg->position.y;
        latest_drone_state_->position.z = msg->position.z;

        latest_drone_state_->velocity.x = msg->velocity.x;
        latest_drone_state_->velocity.y = msg->velocity.y;
        latest_drone_state_->velocity.z = msg->velocity.z;
    }

    void update_latest_reference_point(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!latest_reference_point_)
        {
            latest_reference_point_ = autonomy_core::TrajectoryPoint{};
        }

        latest_reference_point_->time.seconds =
            static_cast<double>(msg->header.stamp.sec) +
            static_cast<double>(msg->header.stamp.nanosec) * 1.0e-9;

        latest_reference_point_->pose.position.x = msg->pose.pose.position.x;
        latest_reference_point_->pose.position.y = msg->pose.pose.position.y;
        latest_reference_point_->pose.position.z = msg->pose.pose.position.z;

        latest_reference_point_->twist.linear.x = msg->twist.twist.linear.x;
        latest_reference_point_->twist.linear.y = msg->twist.twist.linear.y;
        latest_reference_point_->twist.linear.z = msg->twist.twist.linear.z;
    }

    void calculate_and_publish_accel_cmd()
    {
        if (!latest_reference_point_ || !latest_drone_state_)
        {
            return;
        }

        const auto state = latest_drone_state_.value();
        const auto reference = latest_reference_point_.value();

        autonomy_sim::AccelerationCommand accel_cmd =
            trajectory_tracker_.compute_command(state, reference);

        autonomy_msgs::msg::AccelerationCommand msg;

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";

        msg.acceleration.x = accel_cmd.acceleration.x;
        msg.acceleration.y = accel_cmd.acceleration.y;
        msg.acceleration.z = accel_cmd.acceleration.z;

        accel_cmd_raw_pub_->publish(msg);
    }

    void update_and_publish_error()
    {
        if (!latest_reference_point_ || !latest_drone_state_)
        {
            return;
        }

        latest_error_msg_.header.stamp = this->get_clock()->now();
        latest_error_msg_.header.frame_id = "map";

        latest_error_msg_.pose.pose.position.x =
            latest_reference_point_->pose.position.x -
            latest_drone_state_->position.x;
        latest_error_msg_.pose.pose.position.y =
            latest_reference_point_->pose.position.y -
            latest_drone_state_->position.y;
        latest_error_msg_.pose.pose.position.z =
            latest_reference_point_->pose.position.z -
            latest_drone_state_->position.z;

        latest_error_msg_.twist.twist.linear.x =
            latest_reference_point_->twist.linear.x -
            latest_drone_state_->velocity.x;
        latest_error_msg_.twist.twist.linear.y =
            latest_reference_point_->twist.linear.y -
            latest_drone_state_->velocity.y;
        latest_error_msg_.twist.twist.linear.z =
            latest_reference_point_->twist.linear.z -
            latest_drone_state_->velocity.z;

        latest_error_msg_.pose.pose.orientation.w = 1.0;

        error_pub_->publish(latest_error_msg_);
    }

   private:
    autonomy_control::TrajectoryTracker trajectory_tracker_{
        autonomy_control::TrajectoryTrackerGains{1.0, 8.0, 5.0}};

    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<autonomy_sim::FakeDroneState> latest_drone_state_;
    std::optional<autonomy_core::TrajectoryPoint> latest_reference_point_;

    nav_msgs::msg::Odometry latest_error_msg_;

    rclcpp::Subscription<autonomy_msgs::msg::FakeDroneState>::SharedPtr
        drone_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        trajectory_odom_sub_;
    rclcpp::Publisher<autonomy_msgs::msg::AccelerationCommand>::SharedPtr
        accel_cmd_raw_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr error_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrackerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
