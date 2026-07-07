#include <optional>

#include "autonomy_planning/astar_planner.hpp"
#include "autonomy_planning/grid_map.hpp"
#include "autonomy_planning/obstacle_inflation.hpp"
#include "autonomy_planning/path_conversion.hpp"
#include "autonomy_planning/path_smoother.hpp"
#include "autonomy_planning/planner_result.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class PlannerNode : public rclcpp::Node
{
   public:
    PlannerNode() : Node{"planner_node"}
    {
        grid_map_ = autonomy_planning::GridMap{10, 10, 1.0};
        grid_map_.set_occupied(autonomy_planning::GridIndex{5, 5}, true);
        grid_map_.set_occupied(autonomy_planning::GridIndex{3, 3}, true);
        grid_map_.set_occupied(autonomy_planning::GridIndex{0, 4}, true);

        inflated_map_ = autonomy_planning::inflate_obstacles(grid_map_, 1.0);

        start_.col = 0;
        start_.row = 0;

        goal_.col = 9;
        goal_.row = 9;

        const std::optional<autonomy_planning::PlannerResult> astar_result =
            astar_planner_.plan(inflated_map_, start_, goal_);

        if (astar_result.has_value())
        {
            RCLCPP_INFO(this->get_logger(), "PATH FOUND");
            raw_world_path_ = astar_result.value().world_path;

            RCLCPP_INFO(this->get_logger(),
                        "A* cost: %.2f, grid path nodes: %zu",
                        astar_result.value().cost_m,
                        astar_result.value().grid_path.size());

            const std::vector<autonomy_planning::GridIndex> smoothed_grid_path =
                autonomy_planning::smooth_grid_path_line_of_sight(
                    inflated_map_, astar_result.value().grid_path);

            RCLCPP_INFO(this->get_logger(), "Smoothed path nodes: %zu",
                        smoothed_grid_path.size());

            smoothed_world_path_ = autonomy_planning::grid_path_to_world_path(
                inflated_map_, smoothed_grid_path);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "PATH NOT FOUND");
        }

        raw_astar_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planner/raw_path", rclcpp::SensorDataQoS());
        smoothed_astar_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planner/smoothed_path", rclcpp::SensorDataQoS());
        obstacle_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planner/obstacle_cells", rclcpp::SensorDataQoS());

        timer_ = this->create_wall_timer(std::chrono::nanoseconds(1000000000),
                                         [this]() { publish_nav_msgs(); });
    }

   private:
    void publish_nav_msgs()
    {
        nav_msgs::msg::Path raw_path_msg;
        nav_msgs::msg::Path smoothed_path_msg;
        nav_msgs::msg::Path obstacle_msg;

        const rclcpp::Time now = this->get_clock()->now();

        raw_path_msg.header.stamp = now;
        raw_path_msg.header.frame_id = "map";

        smoothed_path_msg.header.stamp = now;
        smoothed_path_msg.header.frame_id = "map";

        obstacle_msg.header.stamp = now;
        obstacle_msg.header.frame_id = "map";

        for (const autonomy_core::Pose3& autonomy_pose : raw_world_path_.poses)
        {
            geometry_msgs::msg::PoseStamped ros_pose;

            ros_pose.header.stamp = now;
            ros_pose.header.frame_id = "map";

            ros_pose.pose.position.x = autonomy_pose.position.x;
            ros_pose.pose.position.y = autonomy_pose.position.y;
            ros_pose.pose.position.z = autonomy_pose.position.z;

            ros_pose.pose.orientation.x = 0.0;
            ros_pose.pose.orientation.y = 0.0;
            ros_pose.pose.orientation.z = 0.0;
            ros_pose.pose.orientation.w = 1.0;

            raw_path_msg.poses.push_back(ros_pose);
        }

        for (const autonomy_core::Pose3& autonomy_pose :
             smoothed_world_path_.poses)
        {
            geometry_msgs::msg::PoseStamped ros_pose;

            ros_pose.header.stamp = now;
            ros_pose.header.frame_id = "map";

            ros_pose.pose.position.x = autonomy_pose.position.x;
            ros_pose.pose.position.y = autonomy_pose.position.y;
            ros_pose.pose.position.z = autonomy_pose.position.z;

            ros_pose.pose.orientation.x = 0.0;
            ros_pose.pose.orientation.y = 0.0;
            ros_pose.pose.orientation.z = 0.0;
            ros_pose.pose.orientation.w = 1.0;

            smoothed_path_msg.poses.push_back(ros_pose);
        }

        for (size_t occupied_index : inflated_map_.get_occupied_indices())
        {
            const autonomy_planning::GridIndex occupied_grid_index =
                inflated_map_.flat_index_to_grid_index(occupied_index);

            const autonomy_core::Vector2 occupied_world_position =
                inflated_map_.grid_to_world(occupied_grid_index);

            geometry_msgs::msg::PoseStamped ros_pose;

            ros_pose.header.stamp = now;
            ros_pose.header.frame_id = "map";

            ros_pose.pose.position.x = occupied_world_position.x;
            ros_pose.pose.position.y = occupied_world_position.y;
            ros_pose.pose.position.z = 0.0;

            ros_pose.pose.orientation.x = 0.0;
            ros_pose.pose.orientation.y = 0.0;
            ros_pose.pose.orientation.z = 0.0;
            ros_pose.pose.orientation.w = 1.0;

            obstacle_msg.poses.push_back(ros_pose);
        }

        raw_astar_path_pub_->publish(raw_path_msg);
        smoothed_astar_path_pub_->publish(smoothed_path_msg);
        obstacle_path_pub_->publish(obstacle_msg);
    }

   private:
    autonomy_planning::GridIndex start_;
    autonomy_planning::GridIndex goal_;

    autonomy_planning::GridMap grid_map_;
    autonomy_planning::GridMap inflated_map_;

    autonomy_planning::AstarPlanner astar_planner_;

    autonomy_core::Path raw_world_path_;
    autonomy_core::Path smoothed_world_path_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_astar_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_astar_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr obstacle_path_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PlannerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
