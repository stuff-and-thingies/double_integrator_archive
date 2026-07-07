#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

#include "autonomy_msgs/msg/acceleration_command.hpp"
#include "autonomy_msgs/msg/fake_drone_state.hpp"
#include "autonomy_sim/double_integrator.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class FakeDroneSimNode : public rclcpp::Node
{
   public:
    FakeDroneSimNode() : Node("fake_drone_sim_node")
    {
        drone_state_pub_ =
            this->create_publisher<autonomy_msgs::msg::FakeDroneState>(
                "/fake_drone/state", rclcpp::SensorDataQoS());

        drone_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/fake_drone/pose", rclcpp::SensorDataQoS());

        accel_command_sub_ =
            this->create_subscription<autonomy_msgs::msg::AccelerationCommand>(
                "/fake_drone/command", rclcpp::SensorDataQoS(),
                [this](const autonomy_msgs::msg::AccelerationCommand::SharedPtr
                           accel_command_received)
                { update_latest_accel_command(accel_command_received); });

        timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(model_update_timestep_ns_),
            [this]() { model_step(); });

        model_ = autonomy_sim::DoubleIntegrator{
            autonomy_sim::DoubleIntegratorLimits{5.0, 10.0}};
        drone_state_ = autonomy_sim::FakeDroneState{};
        latest_accel_command_ = autonomy_sim::AccelerationCommand{};
    }

   private:
    void update_latest_accel_command(
        const autonomy_msgs::msg::AccelerationCommand::SharedPtr
            accel_command_received)
    {
        latest_accel_command_.acceleration.x =
            accel_command_received->acceleration.x;
        latest_accel_command_.acceleration.y =
            accel_command_received->acceleration.y;
        latest_accel_command_.acceleration.z =
            accel_command_received->acceleration.z;

        last_accel_command_time_ = this->get_clock()->now();
    }

    void zero_latest_accel_command()
    {
        latest_accel_command_.acceleration.x = 0.0;
        latest_accel_command_.acceleration.y = 0.0;
        latest_accel_command_.acceleration.z = 0.0;
    }

    void check_accel_command_staleness(const rclcpp::Time& now)
    {
        if (!last_accel_command_time_)
        {
            zero_latest_accel_command();
            return;
        }

        const rclcpp::Duration command_age =
            now - last_accel_command_time_.value();

        // RCLCPP_INFO(this->get_logger(), "dt = %f", dt.seconds());

        if (command_age.nanoseconds() > accel_cmd_time_out_ns_)
        {
            zero_latest_accel_command();
        }
    }

    void model_step()
    {
        const rclcpp::Time now = this->get_clock()->now();

        if (!last_model_step_time_)
        {
            last_model_step_time_ = now;
            return;
        }

        const rclcpp::Duration ros_dt = now - last_model_step_time_.value();
        last_model_step_time_ = now;

        const double dt_s = ros_dt.seconds();

        if (dt_s <= 0.0)
        {
            return;
        }

        autonomy_core::Duration model_dt{dt_s};

        check_accel_command_staleness(now);

        model_.update(drone_state_, latest_accel_command_, model_dt);

        autonomy_msgs::msg::FakeDroneState msg;

        msg.position.x = drone_state_.position.x;
        msg.position.y = drone_state_.position.y;
        msg.position.z = drone_state_.position.z;

        msg.velocity.x = drone_state_.velocity.x;
        msg.velocity.y = drone_state_.velocity.y;
        msg.velocity.z = drone_state_.velocity.z;

        msg.header.stamp = now;
        msg.header.frame_id = "map";

        drone_state_pub_->publish(msg);

        geometry_msgs::msg::PoseStamped pose_msg;

        pose_msg.header = msg.header;

        pose_msg.pose.position.x = drone_state_.position.x;
        pose_msg.pose.position.y = drone_state_.position.y;
        pose_msg.pose.position.z = drone_state_.position.z;

        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 1.0;

        drone_pose_pub_->publish(pose_msg);
    }

    autonomy_sim::DoubleIntegrator model_;
    autonomy_sim::FakeDroneState drone_state_;
    autonomy_sim::AccelerationCommand latest_accel_command_;

    std::optional<rclcpp::Time> last_accel_command_time_;
    std::optional<rclcpp::Time> last_model_step_time_;

    const std::uint32_t model_update_timestep_ns_ = 4000000;  // 250 Hz
    const std::uint32_t accel_cmd_time_out_ns_ = 250000000;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<autonomy_msgs::msg::AccelerationCommand>::SharedPtr
        accel_command_sub_;
    rclcpp::Publisher<autonomy_msgs::msg::FakeDroneState>::SharedPtr
        drone_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        drone_pose_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FakeDroneSimNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
