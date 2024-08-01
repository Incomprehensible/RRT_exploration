#include <functional>
#include <sstream>
#include <random>

// TODO: common math utils

#include "single_agent_rrt/filter.hpp"
#include "single_agent_rrt/utils.hpp"

Filter::Filter(const rclcpp::NodeOptions &options, const std::string& name)
    : Node(name, options),
    tf_buffer_(std::make_shared<rclcpp::Clock>()), tf_listener_(tf_buffer_)
{
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // this->valid_pose_ = false;

    this->local_frontier_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/local_frontier_detector", 10, std::bind(&Filter::local_frontier_callback, this, std::placeholders::_1));
    this->global_frontier_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/global_frontier_detector", 10, std::bind(&Filter::global_frontier_callback, this, std::placeholders::_1)) ;
    this->frontier_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/frontier_detector", 10);

    this->filter_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Filter::filter_frontiers, this));
    this->frontier_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&Filter::publish_frontier, this));

    // TODO: move to robot task allocator
    // Nav2 navigation action
    this->nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
}

void Filter::local_frontier_callback(const geometry_msgs::msg::Point &frontier)
{
    RCLCPP_INFO(this->get_logger(), "NAD: got the local frontier!");
    // check frontiers for being explored before inserting here
    this->frontier_queue_.insert(frontier);
}

void Filter::global_frontier_callback(const geometry_msgs::msg::Point &frontier)
{
    // check frontiers for being explored before inserting here
    this->frontier_queue_.insert(frontier);
}

// TODO: include info gain information? or dont and just recalculate nav cost in task allocator?
// TODO: try sending all stored clusters vs sending one by one
void Filter::publish_frontier()
{
    // TODO: remove this code
    if (goal_ongoing) {
        RCLCPP_INFO(this->get_logger(), "NAD: Goal still in completion...");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "NAD: Publishing clustered frontier...");

    // while (!this->cluster_queue_.empty()) {
    //     auto frontier = this->cluster_queue_.pop();
    //     this->frontier_pub_->publish(frontier);
    // }
    if (this->cluster_queue_.empty())
        return;
    auto frontier = this->cluster_queue_.top().center;
    this->cluster_queue_.pop();
    this->frontier_pub_->publish(frontier);

    RCLCPP_INFO(this->get_logger(), "NAD: Sending goal!");
    // TODO: remove this code
    this->cluster_queue_ = {};
    send_goal(frontier);
    goal_ongoing = true; // remove this
}


void Filter::filter_frontiers()
{
    // if (!this->get_clock()->ros_time_is_active())
    //     return;
    if (!this->tf_buffer_.canTransform("map", "odom", tf2::TimePointZero, tf2::durationFromSec(1.0)))
        return;
    
    RCLCPP_INFO(this->get_logger(), "NAD: filtering frontiers...");
    RCLCPP_INFO(this->get_logger(), "NAD: frontier queue size: %ld", this->frontier_queue_.size());

    if (this->frontier_queue_.size() < CLUSTERING_THRESHOLD)
        return;
    
    cluster_frontiers();
    // TODO: implement radius-based explored frontiers checking and storage of cluster centers
}

void Filter::calculate_cluster_center(FrontierCluster& cluster)
{
    for (const auto& frontier: cluster.frontiers) {
        cluster.center.x += frontier.x;
        cluster.center.y += frontier.y;
    }
    cluster.center.x /= cluster.frontiers.size();
    cluster.center.y /= cluster.frontiers.size();
}

// TODO: move to task allocator
inline double Filter::revenue_function(double N, double I)
{
    // TODO: make parameters
    const double h_gain = 1.2;
    const double h_r = 3;
    const double lambda = 2.0;
    // hysteresis gain
    double h = (N > h_r)? 1 : h_gain;

    return lambda*h*I - N;
}

// TODO: implement map subscriber and calculate proper info gain
// now it's only based on size
double Filter::calculate_info_gain(std::vector<geometry_msgs::msg::Point>& p)
{
    return p.size();
}

// navigation cost is outdated with each robot movement
// therefore it'll be reconsidered in task allocator node after frontiers are received
// and here it's considered locally for priority queue reordering
// the only information about the cost being stored is Information Gain cost
void Filter::calculate_cluster_cost(FrontierCluster& frontier, geometry_msgs::msg::Point& r)
{
    double nav_cost = utils::euclidian_dist(frontier.center, r);
    double info_gain = calculate_info_gain(frontier.frontiers);

    frontier.cost = revenue_function(nav_cost, info_gain);
    frontier.info_gain = info_gain;
}

void Filter::cluster_frontiers()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

    size_t n = this->frontier_queue_.size();
    std::unordered_set<geometry_msgs::msg::Point>::iterator it = this->frontier_queue_.begin();

    pcl::PointXYZ pc;
    while (n--) {
        pc.x = (*it).x;
        pc.y = (*it).y;
        pc.z = (*it).z;
        cloud->push_back(pc);
        if (n)
            ++it;
    }
    // this->frontier_queue_.erase(this->frontier_queue_.begin(), it);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // TODO: make parameters
    ec.setClusterTolerance (0.05); // 8cm
    ec.setMinClusterSize (8);
    ec.setMaxClusterSize (500);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    // std::vector<FrontierCluster> clusters;
    // clusters.reserve(cluster_indices.size());

    FrontierCluster frontier;
    geometry_msgs::msg::Point p;
    // size_t j = 0;
    for (const auto& cluster : cluster_indices)
    {
        for (const auto& idx : cluster.indices) {
            p.x = (*cloud)[idx].x;
            p.y = (*cloud)[idx].y;
            p.z = 0;
            frontier.frontiers.push_back(p);
            // cluster[j].frontiers->push_back(p);
        }
        calculate_cluster_center(frontier);
        geometry_msgs::msg::TransformStamped::SharedPtr robot2map_tf = utils::get_robot_position(&tf_buffer_);
        // if (robot2map_tf == nullptr)
        //     return false;
        auto robot2map = *(robot2map_tf.get());
        geometry_msgs::msg::Point pose;
        pose.x = robot2map.transform.translation.x;
        pose.y = robot2map.transform.translation.y;
        pose.z = 0;
        calculate_cluster_cost(frontier, pose);

        // calculate_cluster_center(cluster[j]);
        // calculate_cluster_cost(cluster[j]);
        // j++;
        this->cluster_queue_.push(frontier);
    }
    if (this->cluster_queue_.size() > 0)
        this->frontier_queue_.erase(this->frontier_queue_.begin(), it);
    RCLCPP_INFO(this->get_logger(), "NAD: cluster queue size: %ld", this->cluster_queue_.size());
}

void Filter::goal_response_callback(const GoalHandleNavigate::WrappedResult &result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Goal reached.");
    }    
    else {
        RCLCPP_INFO(this->get_logger(), "Goal failed or was canceled.");
    }
    // TODO: remove this
    goal_ongoing = false;
}

// TODO: move to task allocator
void Filter::send_goal(const geometry_msgs::msg::Point& frontier)
{
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.pose.position.x = frontier.x;
    goal_pose.pose.position.y = frontier.y; 
    goal_pose.pose.position.z = 0; 
    goal_pose.pose.orientation.w = 1.0;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->get_clock()->now();

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&Filter::goal_response_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Wanted position x=%f, y=%f.", goal_pose.pose.position.x, goal_pose.pose.position.y);
    nav2_client_->async_send_goal(goal_msg, send_goal_options);
}

RCLCPP_COMPONENTS_REGISTER_NODE(Filter)