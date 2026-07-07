#include <chrono>
#include <memory>
#include <optional>

#include "autonomy_core/path.hpp"
#include "autonomy_core/pose3.hpp"
#include "autonomy_core/trajectory.hpp"
#include "autonomy_core/vector3.hpp"
#include "autonomy_msgs/msg/mission_state.hpp"
#include "autonomy_trajectory/path_to_trajectory.hpp"
#include "autonomy_trajectory/trajectory_sampler.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class TrajectoryNode : public rclcpp::Node
{
   public:
    TrajectoryNode() : Node{"trajectory_node"}
    {
        planner_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planner/smoothed_path", rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Path::SharedPtr msg)
            { planner_path_sub_callback(msg); });

        mission_state_sub_ =
            this->create_subscription<autonomy_msgs::msg::MissionState>(
                "/mission/state", rclcpp::SensorDataQoS(),
                [this](const autonomy_msgs::msg::MissionState::SharedPtr msg)
                { update_latest_mission_state(msg); });

        trajectory_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/trajectory/path", rclcpp::SensorDataQoS());

        trajectory_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/trajectory/reference", rclcpp::SensorDataQoS());

        reference_trajectory_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            [this]() { sample_and_publish_reference_trajectory_point(); });

        trajectory_path_timer_ =
            this->create_wall_timer(std::chrono::milliseconds(500),
                                    [this]() { publish_trajectory_path(); });
    }

   private:
    void update_latest_mission_state(
        const autonomy_msgs::msg::MissionState::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        latest_mission_state_msg_ = *msg;

        if (msg->state == "TRACK" && trajectory_ && !trajectory_started_)
        {
            trajectory_start_time_ = this->now();
            trajectory_started_ = true;
        }
    }

    void planner_path_sub_callback(
        const nav_msgs::msg::Path::SharedPtr path_msg)
    {
        if (!path_msg)
        {
            return;
        }

        trajectory_path_msg_ = *path_msg;

        if (trajectory_)
        {
            return;
        }

        autonomy_core::Path autonomy_path;

        for (const geometry_msgs::msg::PoseStamped& pose_stamped :
             path_msg->poses)
        {
            const geometry_msgs::msg::Pose& pose = pose_stamped.pose;

            autonomy_core::Pose3 autonomy_pose{
                autonomy_core::Vector3{pose.position.x, pose.position.y,
                                       pose.position.z},
                autonomy_core::Vector3{}};

            autonomy_path.poses.emplace_back(autonomy_pose);
        }

        autonomy_core::Trajectory candidate_trajectory =
            autonomy_trajectory::path_to_constant_speed_trajectory(
                autonomy_path, 1.0);

        if (candidate_trajectory.empty())
        {
            return;
        }

        trajectory_ = candidate_trajectory;

        if (!sent_debug_msg_)
        {
            if (trajectory_->empty())
            {
                RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
                return;
            }

            RCLCPP_INFO(this->get_logger(),
                        "Trajectory has %zu points, duration %.2f s",
                        trajectory_.value().points.size(),
                        trajectory_.value().points.back().time.seconds);

            sent_debug_msg_ = true;
        }

        if (path_msg)
        {
            trajectory_path_pub_->publish(*path_msg);
        }
    }

    void sample_and_publish_reference_trajectory_point()
    {
        if (!trajectory_ || !trajectory_start_time_ || !trajectory_started_)
        {
            return;
        }

        rclcpp::Time current_time = this->now();

        rclcpp::Duration time_since_start =
            current_time - trajectory_start_time_.value();

        std::optional<autonomy_core::TrajectoryPoint>
            reference_trajectory_point = autonomy_trajectory::sample_trajectory(
                trajectory_.value(), time_since_start.seconds());

        if (reference_trajectory_point.has_value())
        {
            publish_reference_trajectory_point(
                reference_trajectory_point.value(), current_time);
        }
    }

    void publish_trajectory_path()
    {
        if (trajectory_path_msg_)
        {
            trajectory_path_pub_->publish(trajectory_path_msg_.value());
        }
    }

    void publish_reference_trajectory_point(
        const autonomy_core::TrajectoryPoint& trajectory_point,
        const rclcpp::Time stamp)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "map";
        odom.child_frame_id = "trajectory_reference";

        odom.pose.pose.position.x = trajectory_point.pose.position.x;
        odom.pose.pose.position.y = trajectory_point.pose.position.y;
        odom.pose.pose.position.z = trajectory_point.pose.position.z;

        odom.twist.twist.linear.x = trajectory_point.twist.linear.x;
        odom.twist.twist.linear.y = trajectory_point.twist.linear.y;
        odom.twist.twist.linear.z = trajectory_point.twist.linear.z;

        odom.twist.twist.angular.x = trajectory_point.twist.angular.x;
        odom.twist.twist.angular.y = trajectory_point.twist.angular.y;
        odom.twist.twist.angular.z = trajectory_point.twist.angular.z;

        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = 0.0;
        odom.pose.pose.orientation.w = 1.0;

        trajectory_odom_pub_->publish(odom);
    }

   private:
    bool sent_debug_msg_ = false;

    std::optional<rclcpp::Time> trajectory_start_time_;

    rclcpp::TimerBase::SharedPtr reference_trajectory_timer_;
    rclcpp::TimerBase::SharedPtr trajectory_path_timer_;
    bool trajectory_started_{false};

    std::optional<autonomy_core::Trajectory> trajectory_;
    std::optional<autonomy_msgs::msg::MissionState> latest_mission_state_msg_;

    std::optional<nav_msgs::msg::Path> trajectory_path_msg_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planner_path_sub_;
    rclcpp::Subscription<autonomy_msgs::msg::MissionState>::SharedPtr
        mission_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr trajectory_odom_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrajectoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
