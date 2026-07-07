#include <chrono>
#include <cmath>
#include <memory>
#include <optional>

#include "autonomy_msgs/msg/acceleration_command.hpp"
#include "autonomy_msgs/msg/fake_drone_state.hpp"
#include "autonomy_msgs/msg/safety_status.hpp"
#include "autonomy_sim/fake_drone_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class SafetyMonitorNode : public rclcpp::Node
{
   public:
    SafetyMonitorNode() : rclcpp::Node("safety_monitor_node")
    {
        safety_status_msg_.active_faults.reserve(10);

        drone_state_sub_ =
            this->create_subscription<autonomy_msgs::msg::FakeDroneState>(
                "/fake_drone/state", rclcpp::SensorDataQoS(),
                [this](const autonomy_msgs::msg::FakeDroneState::SharedPtr msg)
                { update_last_drone_state(msg); });

        drone_command_sub_ =
            this->create_subscription<autonomy_msgs::msg::AccelerationCommand>(
                "/tracker/command_raw", rclcpp::SensorDataQoS(),
                [this](const autonomy_msgs::msg::AccelerationCommand::SharedPtr
                           msg) { update_last_accel_command(msg); });

        tracker_error_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/tracker/error", rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            { update_last_tracking_error(msg); });

        safety_status_pub_ =
            this->create_publisher<autonomy_msgs::msg::SafetyStatus>(
                "/safety/status", rclcpp::SensorDataQoS());

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]()
            {
                rclcpp::Time now = this->get_clock()->now();

                check_drone_state_staleness(now);
                check_accel_command_staleness(now);
                check_error_staleness(now);
                check_speed();
                check_acceleration_command_magnitude();
                check_tracking_error_magnitude();
                check_geofence_violation();
                check_finite();
                publish_safety_status_msg(now);
            });
    }

   private:
    void update_last_drone_state(
        const autonomy_msgs::msg::FakeDroneState::SharedPtr msg)
    {
        last_drone_state_time_ = msg->header.stamp;

        if (!last_drone_state_)
        {
            last_drone_state_ = autonomy_sim::FakeDroneState{};
        }

        last_drone_state_->position.x = msg->position.x;
        last_drone_state_->position.y = msg->position.y;
        last_drone_state_->position.z = msg->position.z;

        last_drone_state_->velocity.x = msg->velocity.x;
        last_drone_state_->velocity.y = msg->velocity.y;
        last_drone_state_->velocity.z = msg->velocity.z;
    }

    void update_last_accel_command(
        const autonomy_msgs::msg::AccelerationCommand::SharedPtr msg)
    {
        last_accel_command_time_ = msg->header.stamp;

        if (!last_accel_command_)
        {
            last_accel_command_ = autonomy_sim::AccelerationCommand{};
        }

        last_accel_command_->acceleration.x = msg->acceleration.x;
        last_accel_command_->acceleration.y = msg->acceleration.y;
        last_accel_command_->acceleration.z = msg->acceleration.z;
    }

    void update_last_tracking_error(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        last_tracking_error_time_ = msg->header.stamp;
        last_tracking_error_ = *msg;
    }

    void check_drone_state_staleness(const rclcpp::Time& now)
    {
        if (!last_drone_state_time_)
        {
            safety_status_msg_.state_stale = true;
            safety_status_msg_.active_faults.emplace_back("state_stale");
            return;
        }

        if ((now - last_drone_state_time_.value()).nanoseconds() > time_out_ns_)
        {
            safety_status_msg_.state_stale = true;
            safety_status_msg_.active_faults.emplace_back("state_stale");
            return;
        }
        safety_status_msg_.state_stale = false;
    }

    void check_accel_command_staleness(const rclcpp::Time& now)
    {
        if (!last_accel_command_time_)
        {
            safety_status_msg_.command_stale = true;
            safety_status_msg_.active_faults.emplace_back("command_stale");
            return;
        }

        if ((now - last_accel_command_time_.value()).nanoseconds() >
            time_out_ns_)
        {
            safety_status_msg_.command_stale = true;
            safety_status_msg_.active_faults.emplace_back("command_stale");
            return;
        }

        safety_status_msg_.command_stale = false;
    }

    void check_error_staleness(const rclcpp::Time& now)
    {
        if (!last_tracking_error_time_)
        {
            safety_status_msg_.error_stale = true;
            safety_status_msg_.active_faults.emplace_back("error_stale");
            return;
        }

        if ((now - last_tracking_error_time_.value()).nanoseconds() >
            time_out_ns_)
        {
            safety_status_msg_.error_stale = true;
            safety_status_msg_.active_faults.emplace_back("error_stale");
            return;
        }

        safety_status_msg_.error_stale = false;
    }

    void check_speed()
    {
        if (!last_drone_state_)
        {
            safety_status_msg_.speed_limit_exceeded = true;
            safety_status_msg_.active_faults.emplace_back(
                "speed_limit_exceeded");
            return;
        }

        if (last_drone_state_->velocity.norm() > max_velocity_mps)
        {
            safety_status_msg_.speed_limit_exceeded = true;
            safety_status_msg_.active_faults.emplace_back(
                "speed_limit_exceeded");
            return;
        }

        safety_status_msg_.speed_limit_exceeded = false;
    }

    void check_acceleration_command_magnitude()
    {
        if (!last_accel_command_)
        {
            safety_status_msg_.accel_limit_exceeded = true;
            safety_status_msg_.active_faults.emplace_back(
                "accel_limit_exceeded");
            return;
        }

        if (last_accel_command_->acceleration.norm() > max_acceleration_mps2)
        {
            safety_status_msg_.accel_limit_exceeded = true;
            safety_status_msg_.active_faults.emplace_back(
                "accel_limit_exceeded");
            return;
        }

        safety_status_msg_.accel_limit_exceeded = false;
    }

    void check_tracking_error_magnitude()
    {
        if (!last_tracking_error_)
        {
            safety_status_msg_.tracking_error_exceeded = true;
            safety_status_msg_.active_faults.emplace_back(
                "tracking_error_exceeded");
            return;
        }

        const double pos_error_norm_m =
            std::sqrt(last_tracking_error_->pose.pose.position.x *
                          last_tracking_error_->pose.pose.position.x +
                      last_tracking_error_->pose.pose.position.y *
                          last_tracking_error_->pose.pose.position.y +
                      last_tracking_error_->pose.pose.position.z *
                          last_tracking_error_->pose.pose.position.z);

        const double vel_error_norm_mps =
            std::sqrt(last_tracking_error_->twist.twist.linear.x *
                          last_tracking_error_->twist.twist.linear.x +
                      last_tracking_error_->twist.twist.linear.y *
                          last_tracking_error_->twist.twist.linear.y +
                      last_tracking_error_->twist.twist.linear.z *
                          last_tracking_error_->twist.twist.linear.z);

        if (pos_error_norm_m > pos_error_fault_threshold_m ||
            vel_error_norm_mps > vel_error_fault_threshold_mps)
        {
            safety_status_msg_.tracking_error_exceeded = true;
            safety_status_msg_.active_faults.emplace_back(
                "tracking_error_exceeded");
            return;
        }

        safety_status_msg_.tracking_error_exceeded = false;
    }

    void check_geofence_violation()
    {
        if (!last_drone_state_)
        {
            safety_status_msg_.geofence_violation = true;
            safety_status_msg_.active_faults.emplace_back("geofence_violation");
            return;
        }

        if (last_drone_state_->position.x > map_boundary_x_max_m ||
            last_drone_state_->position.y > map_boundary_y_max_m ||
            last_drone_state_->position.x < map_boundary_x_min_m ||
            last_drone_state_->position.y < map_boundary_y_min_m)
        {
            safety_status_msg_.geofence_violation = true;
            safety_status_msg_.active_faults.emplace_back("geofence_violation");
            return;
        }

        safety_status_msg_.geofence_violation = false;
    }

    void check_finite()
    {
        if (!last_drone_state_ || !last_accel_command_ || !last_tracking_error_)
        {
            safety_status_msg_.nonfinite_value = false;
            return;
        }

        if (!std::isfinite(last_drone_state_->position.x) ||
            !std::isfinite(last_drone_state_->position.y) ||
            !std::isfinite(last_drone_state_->position.z) ||
            !std::isfinite(last_drone_state_->velocity.x) ||
            !std::isfinite(last_drone_state_->velocity.y) ||
            !std::isfinite(last_drone_state_->velocity.z) ||
            !std::isfinite(last_accel_command_->acceleration.x) ||
            !std::isfinite(last_accel_command_->acceleration.y) ||
            !std::isfinite(last_accel_command_->acceleration.z) ||
            !std::isfinite(last_tracking_error_->pose.pose.position.x) ||
            !std::isfinite(last_tracking_error_->pose.pose.position.y) ||
            !std::isfinite(last_tracking_error_->pose.pose.position.z) ||
            !std::isfinite(last_tracking_error_->twist.twist.linear.x) ||
            !std::isfinite(last_tracking_error_->twist.twist.linear.y) ||
            !std::isfinite(last_tracking_error_->twist.twist.linear.z))
        {
            safety_status_msg_.nonfinite_value = true;
            safety_status_msg_.active_faults.emplace_back("nonfinite_value");
            return;
        }

        safety_status_msg_.nonfinite_value = false;
    }

    void publish_safety_status_msg(const rclcpp::Time& now)

    {
        safety_status_msg_.header.stamp = now;
        safety_status_msg_.header.frame_id = "drone";

        safety_status_msg_.all_ok = false;

        if (!safety_status_msg_.state_stale &&
            !safety_status_msg_.command_stale &&
            !safety_status_msg_.error_stale &&
            !safety_status_msg_.speed_limit_exceeded &&
            !safety_status_msg_.accel_limit_exceeded &&
            !safety_status_msg_.tracking_error_exceeded &&
            !safety_status_msg_.geofence_violation &&
            !safety_status_msg_.nonfinite_value)
        {
            safety_status_msg_.all_ok = true;
        }

        safety_status_pub_->publish(safety_status_msg_);

        safety_status_msg_.active_faults.clear();
    }

   private:
    const std::uint32_t time_out_ns_ = 250000000;

    double max_velocity_mps = 11.0;
    double max_acceleration_mps2 = 6.0;

    double pos_error_fault_threshold_m = 2.0;
    double vel_error_fault_threshold_mps = 2.0;

    double map_boundary_x_min_m = -0.5;
    double map_boundary_y_min_m = -0.5;
    double map_boundary_x_max_m = 10.0;
    double map_boundary_y_max_m = 10.0;

    std::optional<rclcpp::Time> last_drone_state_time_;
    std::optional<rclcpp::Time> last_accel_command_time_;
    std::optional<rclcpp::Time> last_tracking_error_time_;

    std::optional<autonomy_sim::FakeDroneState> last_drone_state_;
    std::optional<autonomy_sim::AccelerationCommand> last_accel_command_;
    std::optional<nav_msgs::msg::Odometry> last_tracking_error_;

    autonomy_msgs::msg::SafetyStatus safety_status_msg_;

    rclcpp::Subscription<autonomy_msgs::msg::FakeDroneState>::SharedPtr
        drone_state_sub_;
    rclcpp::Subscription<autonomy_msgs::msg::AccelerationCommand>::SharedPtr
        drone_command_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tracker_error_sub_;

    rclcpp::Publisher<autonomy_msgs::msg::SafetyStatus>::SharedPtr
        safety_status_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SafetyMonitorNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
