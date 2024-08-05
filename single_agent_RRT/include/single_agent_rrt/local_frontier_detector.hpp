#ifndef LOCAL_FRONTIER_DETECTOR_HPP
#define LOCAL_FRONTIER_DETECTOR_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
// #include <rclcpp_components/register_node_macro.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_util/geometry_utils.hpp>
// #include <nav2_util/simple_action_server.hpp>
// #include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// nanoflann lib
#include "nanoflann.hpp"
#include "nanoflann_utils.hpp"

#define MAX_LEAF 10
#define POINT_CLOUD_SIZE 1000 // 100

class LocalFrontierDetector: public rclcpp::Node
{
    // TODO: move to robot task allocator
    // using NavigateToPose = nav2_msgs::action::NavigateToPose;
    // using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    using KDTree_t = nanoflann::KDTreeSingleIndexDynamicAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloud<double>>,
        PointCloud<double>, 2 /* dim */
    >;

    class RRTVisualizer {
        public:
            RRTVisualizer(rclcpp::Node* node) {
                this->rrt_node = node;
                this->marker_pub_ = rrt_node->create_publisher<visualization_msgs::msg::Marker>("/rrt_local_tree", 20);
                // marker_pub_ = rrt_node->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt_tree", 10);
                // marker_array_.markers.resize(1);
                init_marker(this->marker_);
                // initializeMarker(marker_array_.markers[0]);
            }

            void add_edge(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
                // std::vector<visualization_msgs::msg::Marker> &markers = marker_array_.markers;
                // visualization_msgs::msg::Marker marker;
                // initializeMarker(marker);

                // // p1
                // marker_.id = marker_id_++;
                // marker.pose.position = p1;
                // markers.push_back(marker);
                // // p2
                // marker.id = marker_id_++;
                // marker.pose.position = p2;
                // markers.push_back(marker);
                // change marker
                marker_.points.clear();
                marker_.points.push_back(p1);
                marker_.points.push_back(p2);
                
                marker_pub_->publish(marker_);
                marker_pub_->publish(marker_);

                marker_.id++;
            }

            void clear_tree()
            {
                visualization_msgs::msg::Marker marker;
                marker.action = visualization_msgs::msg::Marker::DELETE;
                for (int id = 0; id <= this->marker_.id; id++)
                {
                    marker.id = id;
                    marker_pub_->publish(marker);
                }
                this->marker_.id = 0;
                this->marker_.color.r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                this->marker_.color.g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                this->marker_.color.b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            }
        
        private:
            void init_marker(visualization_msgs::msg::Marker &marker) {
                marker.header.frame_id = "map";  // Set the frame id to match your setup
                marker.header.stamp = rrt_node->now();
                marker.ns = "rrt_local_tree";
                marker.id = 0;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.scale.x = 0.06;  // Width of the lines
                marker.color.r = 0.0;
                marker.color.g = 0;
                marker.color.b = 1.0;
                marker.color.a = 1.0;  // Alpha value
                marker.frame_locked = true;
                marker.lifetime = rclcpp::Duration(std::chrono::microseconds(0)); // lives forever
            }

            rclcpp::Node* rrt_node;
            visualization_msgs::msg::Marker marker_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    };

    enum MAP_STATUS {
        FREE = 0,
        OBSTACLE = 100,
        UNKNOWN = -1,
        UNDEFINED = 2,
    };

    public:
        explicit LocalFrontierDetector(const rclcpp::NodeOptions & = rclcpp::NodeOptions(), const std::string& = "local_frontier_detector");

    private:
        // callbacks
        void map_callback(const nav_msgs::msg::OccupancyGrid&);

        // methods
        void detect_frontiers();
        void publish_frontier(const geometry_msgs::msg::Point&);

        // dbg methods
        void dbg_map_print();

        // TODO: move to robot task allocator
        // void goal_response_callback(const GoalHandleNavigate::WrappedResult&);

        // RRT methods
        bool RRT_reset();
        void RRT_add_point(const geometry_msgs::msg::Point&);
        // void RRT_visualize_edge(const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&)
        // void RRT_visualize();

        geometry_msgs::msg::Point RRT_find_nearest_neighbor(const geometry_msgs::msg::Point&);
        std::pair<geometry_msgs::msg::Point::SharedPtr, MAP_STATUS> RRT_steer(const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&);

        // helper methods
        double euclidian_dist(const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&);
        bool comparable(double, double);
        bool within_expansion_dist(const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&);
        MAP_STATUS grid_check(const geometry_msgs::msg::Point&);

        // constants
        // TODO: make as a parameter
        const double RRT_EXPANSION_RATE = 1.6;//1.5;

        // variables
        // TODO?: hash table for explored frontiers
        // RRT as KD-Tree
        KDTree_t RRT_;
        RRTVisualizer RRT_viz_;
        PointCloud<double> pcloud_;
        // used when inserting new points in RRT
        size_t pcloud_index_;
        nav_msgs::msg::OccupancyGrid::SharedPtr map_;

        // flags
        bool valid_map_;
        // needed for RRT initialization at each detection step
        bool reset_RRT_;

        // coordinates transform
        tf2::BufferCore tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // timers, publishers, subscribers, services
        rclcpp::TimerBase::SharedPtr detector_timer_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr frontier_pub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        // TODO?
        // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr explored_frontier_sub_;
        // rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
};

#endif