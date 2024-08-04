#ifndef FILTER_HPP
#define FILTER_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <unordered_set>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
// #include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <nav2_util/geometry_utils.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

class Filter : public rclcpp::Node
{
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    struct FrontierCluster
    {
        // TODO: remove vector storage
        std::vector<geometry_msgs::msg::Point> frontiers;
        geometry_msgs::msg::Point center;
        double cost;
        double info_gain;

        public:
            bool operator() (FrontierCluster& f1, FrontierCluster& f2)
            {
                return f1.cost > f2.cost;
            }
    };

    struct PointHashFunction
    {
        size_t operator()(const geometry_msgs::msg::Point& point) const
        {
            size_t xHash = std::hash<double>()(point.x);
            size_t yHash = std::hash<double>()(point.y) << 1;
            return xHash ^ yHash;
        }
    };

    // TODO
    // class FrontierVisualizer
    public:
        explicit Filter(const rclcpp::NodeOptions & = rclcpp::NodeOptions(), const std::string& = "filter");

    private:
        // methods
        void filter_frontiers();
        void cluster_frontiers(std::vector<geometry_msgs::msg::Point>);
        void calculate_cluster_center();
        void calculate_cluster_cost();
        void publish_frontier();

        // callbacks
        void local_frontier_callback(const geometry_msgs::msg::Point&);
        void global_frontier_callback(const geometry_msgs::msg::Point&);
        void map_callback(const nav_msgs::msg::OccupancyGrid&);
        // TODO: Move
        void goal_response_callback(const GoalHandleNavigate::WrappedResult&);

        // helper methods
        void visualize_frontiers(std::vector<FrontierCluster>);
        bool is_unexplored(const geometry_msgs::msg::Point&);
        double revenue_function(double, double);
        void calculate_cluster_center(FrontierCluster&);
        double calculate_info_gain(const std::vector<geometry_msgs::msg::Point>&);
        void calculate_cluster_cost(FrontierCluster&, geometry_msgs::msg::Point&);
        // TODO: move
        void send_goal(const geometry_msgs::msg::Point&);

        // constants
        // specifies how many frontiers need to be queued before clustering
        // TODO: make a parameter
        const size_t CLUSTERING_THRESHOLD = 60;

        // variables
        std::unordered_set<geometry_msgs::msg::Point, PointHashFunction> frontier_queue_;
        std::priority_queue<FrontierCluster, std::vector<FrontierCluster>, FrontierCluster> cluster_queue_;
        nav_msgs::msg::OccupancyGrid::SharedPtr map_;
        size_t last_markers_count_ = 0;

        // TODO: remove this
        bool goal_ongoing;
        // last valid transform - ensuring the robustness of navigation cost calculation
        // geometry_msgs::msg::TransformStamped::SharedPtr robot2map_last_;

        // flags
        bool clear_queue_;
        bool valid_map_;

        // coordinates transform
        // tf2::BufferCore tf_buffer_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // timers, publishers, subscribers, services
        rclcpp::TimerBase::SharedPtr filter_timer_;
        rclcpp::TimerBase::SharedPtr frontier_pub_timer_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr local_frontier_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr global_frontier_sub_;
        // Frontier point Publisher
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr frontier_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

        rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
};

#endif