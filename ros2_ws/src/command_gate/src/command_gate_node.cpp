#include <chrono>
#include <memory>
#include <optional>

#include "autonomy_msgs/msg/acceleration_command.hpp"
#include "autonomy_msgs/msg/mission_state.hpp"
#include "autonomy_msgs/msg/safety_status.hpp"
#include "rclcpp/rclcpp.hpp"

class CommandGateNode : public rclcpp::Node
{
   public:
    CommandGateNode() : rclcpp::Node("command_gate_node")
    {
        accel_cmd_raw_sub_ =
            this->create_subscription<autonomy_msgs::msg::AccelerationCommand>(
                "/tracker/command_raw", rclcpp::SensorDataQoS(),
                [this](const autonomy_msgs::msg::AccelerationCommand::SharedPtr
                           msg) { update_latest_accel_command_raw(msg); });

        safety_status_sub_ =
            this->create_subscription<autonomy_msgs::msg::SafetyStatus>(
                "/safety/status", rclcpp::SensorDataQoS(),
                [this](const autonomy_msgs::msg::SafetyStatus::SharedPtr msg)
                { update_latest_safety_status(msg); });

        mission_state_sub_ =
            this->create_subscription<autonomy_msgs::msg::MissionState>(
                "/mission/state", rclcpp::SensorDataQoS(),
                [this](const autonomy_msgs::msg::MissionState::SharedPtr msg)
                { update_latest_mission_state(msg); });

        accel_cmd_gated_pub_ =
            this->create_publisher<autonomy_msgs::msg::AccelerationCommand>(
                "/fake_drone/command", rclcpp::SensorDataQoS());

        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(20), [this]()
                                    { check_and_publish_gated_accel_cmd(); });
    }

   private:
    void update_latest_accel_command_raw(
        const autonomy_msgs::msg::AccelerationCommand::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        latest_accel_command_raw_msg_ = *msg;
        latest_raw_command_receive_time_ = this->get_clock()->now();
    }

    void update_latest_safety_status(
        const autonomy_msgs::msg::SafetyStatus::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        latest_safety_status_msg_ = *msg;
        latest_safety_status_receive_time_ = this->get_clock()->now();
    }

    void update_latest_mission_state(
        const autonomy_msgs::msg::MissionState::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        latest_mission_state_msg_ = *msg;
        latest_mission_state_receive_time_ = this->get_clock()->now();
    }

    void check_and_publish_gated_accel_cmd()
    {
        autonomy_msgs::msg::AccelerationCommand accel_gated_cmd;

        const rclcpp::Time now = this->get_clock()->now();

        accel_gated_cmd.header.stamp = now;
        accel_gated_cmd.header.frame_id = "map";

        accel_gated_cmd.acceleration.x = 0.0;
        accel_gated_cmd.acceleration.y = 0.0;
        accel_gated_cmd.acceleration.z = 0.0;

        if (!latest_accel_command_raw_msg_ || !latest_safety_status_msg_ ||
            !latest_mission_state_msg_)
        {
            accel_cmd_gated_pub_->publish(accel_gated_cmd);
            return;
        }

        if ((now - latest_raw_command_receive_time_.value()).nanoseconds() >
            time_out_ns_)
        {
            accel_cmd_gated_pub_->publish(accel_gated_cmd);
            return;
        }

        if ((now - latest_safety_status_receive_time_.value()).nanoseconds() >
            time_out_ns_)
        {
            accel_cmd_gated_pub_->publish(accel_gated_cmd);
            return;
        }

        if ((now - latest_mission_state_receive_time_.value()).nanoseconds() >
            time_out_ns_)
        {
            accel_cmd_gated_pub_->publish(accel_gated_cmd);
            return;
        }

        if (latest_mission_state_msg_->state != "TRACK")
        {
            accel_cmd_gated_pub_->publish(accel_gated_cmd);
            return;
        }

        if (!latest_safety_status_msg_->all_ok)
        {
            accel_cmd_gated_pub_->publish(accel_gated_cmd);
            return;
        }

        accel_gated_cmd.acceleration.x =
            latest_accel_command_raw_msg_->acceleration.x;
        accel_gated_cmd.acceleration.y =
            latest_accel_command_raw_msg_->acceleration.y;
        accel_gated_cmd.acceleration.z =
            latest_accel_command_raw_msg_->acceleration.z;

        accel_cmd_gated_pub_->publish(accel_gated_cmd);
    }

   private:
    const std::uint32_t time_out_ns_ = 250000000;

    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<rclcpp::Time> latest_raw_command_receive_time_;
    std::optional<rclcpp::Time> latest_safety_status_receive_time_;
    std::optional<rclcpp::Time> latest_mission_state_receive_time_;

    std::optional<autonomy_msgs::msg::AccelerationCommand>
        latest_accel_command_raw_msg_;
    std::optional<autonomy_msgs::msg::SafetyStatus> latest_safety_status_msg_;
    std::optional<autonomy_msgs::msg::MissionState> latest_mission_state_msg_;

    rclcpp::Subscription<autonomy_msgs::msg::AccelerationCommand>::SharedPtr
        accel_cmd_raw_sub_;
    rclcpp::Subscription<autonomy_msgs::msg::SafetyStatus>::SharedPtr
        safety_status_sub_;
    rclcpp::Subscription<autonomy_msgs::msg::MissionState>::SharedPtr
        mission_state_sub_;

    rclcpp::Publisher<autonomy_msgs::msg::AccelerationCommand>::SharedPtr
        accel_cmd_gated_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CommandGateNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
