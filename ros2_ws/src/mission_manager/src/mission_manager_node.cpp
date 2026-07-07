#include <chrono>
#include <memory>
#include <optional>

#include "autonomy_msgs/msg/mission_state.hpp"
#include "autonomy_msgs/msg/safety_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

enum class MissionState
{
    STARTING,
    TRACK,
    COMPLETE,
    ABORT
};

class MissionManagerNode : public rclcpp::Node
{
   public:
    MissionManagerNode() : rclcpp::Node("mission_manager_node")
    {
        safety_status_sub_ =
            this->create_subscription<autonomy_msgs::msg::SafetyStatus>(
                "/safety/status", rclcpp::SensorDataQoS(),
                [this](const autonomy_msgs::msg::SafetyStatus::SharedPtr msg)
                { update_latest_safety_status(msg); });

        trajectory_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/trajectory/path", rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Path::SharedPtr msg)
            { update_trajectory_status(msg); });

        mission_state_pub_ =
            this->create_publisher<autonomy_msgs::msg::MissionState>(
                "/mission/state", rclcpp::SensorDataQoS());

        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(20), [this]()
                                    { check_and_publish_mission_state(); });
    }

   private:
    void update_trajectory_status(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        if (!msg->poses.empty())
        {
            trajectory_loaded_ = true;
        }
    }

    void update_latest_safety_status(
        const autonomy_msgs::msg::SafetyStatus::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }
        latest_safety_status_msg_ = *msg;
    }

    void check_and_publish_mission_state()
    {
        const rclcpp::Time now = this->get_clock()->now();

        if (!latest_safety_status_msg_)
        {
            state_ = MissionState::STARTING;
            startup_ready_since_.reset();
            publish_state(now);
            return;
        }

        switch (state_)
        {
            case MissionState::STARTING:
            {
                if (startup_ready())
                {
                    if (!startup_ready_since_)
                    {
                        startup_ready_since_ = now;
                    }

                    if ((now - startup_ready_since_.value()).seconds() >
                        startup_ready_duration_s_)
                    {
                        state_ = MissionState::TRACK;
                        track_enter_time_ = now;
                    }
                }
                else
                {
                    startup_ready_since_.reset();
                }

                break;
            }

            case MissionState::TRACK:
            {
                const bool grace_period_active =
                    track_enter_time_ &&
                    (now - track_enter_time_.value()).seconds() <
                        tracking_fault_grace_s_;

                if (critical_fault())
                {
                    state_ = MissionState::ABORT;
                }
                else if (!grace_period_active && tracking_fault())
                {
                    state_ = MissionState::ABORT;
                }

                break;
            }

            case MissionState::ABORT:
            {
                break;
            }

            case MissionState::COMPLETE:
            {
                break;
            }
        }

        publish_state(now);
    }

    void publish_state(const rclcpp::Time& now)
    {
        autonomy_msgs::msg::MissionState msg;

        msg.header.stamp = now;
        msg.header.frame_id = "drone";

        switch (state_)
        {
            case MissionState::STARTING:
                msg.state = "STARTING";
                break;

            case MissionState::TRACK:
                msg.state = "TRACK";
                break;

            case MissionState::ABORT:
                msg.state = "ABORT";
                break;

            case MissionState::COMPLETE:
                msg.state = "COMPLETE";
                break;
        }

        mission_state_pub_->publish(msg);
    }

    bool startup_ready() const
    {
        if (!latest_safety_status_msg_)
        {
            return false;
        }

        const auto& s = latest_safety_status_msg_.value();

        return trajectory_loaded_ && !s.state_stale &&
               !s.speed_limit_exceeded && !s.geofence_violation &&
               !s.nonfinite_value;
    }

    bool tracking_fault() const
    {
        if (!latest_safety_status_msg_)
        {
            return true;
        }

        const auto& s = latest_safety_status_msg_.value();

        return s.state_stale || s.command_stale || s.error_stale ||
               s.speed_limit_exceeded || s.accel_limit_exceeded ||
               s.tracking_error_exceeded || s.geofence_violation ||
               s.nonfinite_value;
    }

    bool critical_fault() const
    {
        if (!latest_safety_status_msg_)
        {
            return true;
        }

        const auto& s = latest_safety_status_msg_.value();

        return s.state_stale || s.speed_limit_exceeded ||
               s.geofence_violation || s.nonfinite_value;
    }

   private:
    MissionState state_{MissionState::STARTING};

    std::optional<rclcpp::Time> startup_ready_since_;
    std::optional<rclcpp::Time> track_enter_time_;
    double startup_ready_duration_s_{0.5};

    double tracking_fault_grace_s_{0.75};
    bool trajectory_loaded_{false};

    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<autonomy_msgs::msg::SafetyStatus> latest_safety_status_msg_;

    rclcpp::Subscription<autonomy_msgs::msg::SafetyStatus>::SharedPtr
        safety_status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_path_sub_;

    rclcpp::Publisher<autonomy_msgs::msg::MissionState>::SharedPtr
        mission_state_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MissionManagerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
