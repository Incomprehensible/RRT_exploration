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

    this->clear_queue_ = false;
    this->valid_map_ = false;
    this->goal_ongoing = false;

    this->local_frontier_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/local_frontier_detector", 100, std::bind(&Filter::local_frontier_callback, this, std::placeholders::_1));
    this->global_frontier_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/global_frontier_detector", 200, std::bind(&Filter::global_frontier_callback, this, std::placeholders::_1)) ;
    this->frontier_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/frontier_detector", 10);

    this->marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/clusters", 100);

    this->filter_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2000),
        std::bind(&Filter::filter_frontiers, this));
    // this->frontier_pub_timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(200),
    //     std::bind(&Filter::publish_frontier, this));

    // TODO: move to robot task allocator
    // Nav2 navigation action
    this->nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
}

void Filter::map_callback(const nav_msgs::msg::OccupancyGrid & map)
{
    this->map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(map);

    if (!this->valid_map_)
        this->valid_map_ = true;
}

void Filter::local_frontier_callback(const geometry_msgs::msg::Point &frontier)
{
    if (this->clear_queue_) {
        this->frontier_queue_.erase(this->frontier_queue_.begin(), this->frontier_queue_.end());
        this->clear_queue_ = false;
    }
    // check frontiers for being explored before inserting here
    this->frontier_queue_.insert(frontier);
}

void Filter::global_frontier_callback(const geometry_msgs::msg::Point &frontier)
{
    if (this->clear_queue_) {
        this->frontier_queue_.erase(this->frontier_queue_.begin(), this->frontier_queue_.end());
        this->clear_queue_ = false;
    }
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

    // RCLCPP_INFO(this->get_logger(), "NAD: Sending goal!");
    // // TODO: remove this code
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

    if (this->frontier_queue_.size() < CLUSTERING_THRESHOLD ||goal_ongoing)
        return;
    
    std::vector<geometry_msgs::msg::Point> v;
    copy(this->frontier_queue_.begin(), this->frontier_queue_.end(), back_inserter(v));  
    cluster_frontiers(std::move(v));
    // TODO: implement radius-based explored frontiers checking and storage of cluster centers
    publish_frontier();
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
    const double h_gain = 1.1;
    const double h_r = 1.2;
    const double lambda = 3.0;
    // hysteresis gain
    double h = (N > h_r)? 1 : h_gain;

    return lambda*h*I - N;
}

bool Filter::is_unexplored(const geometry_msgs::msg::Point& p)
{
    size_t i = std::max<double>(0, (p.x - this->map_->info.origin.position.x)) / this->map_->info.resolution;
    size_t j = std::max<double>(0, (p.y - this->map_->info.origin.position.y)) / this->map_->info.resolution;

    // map is outdated
    if (i >= this->map_->info.width || j >= this->map_->info.height)
        return false;
    
    size_t index = j*this->map_->info.width + i;
    // RCLCPP_INFO(this->get_logger(), "NAD: grid status at p_cand: %d", this->map_->data[j*this->map_->info.width + i]);

    return (this->map_->data[index] == -1);
}

// TODO: implement map subscriber and calculate proper info gain
// now it's only based on size
double Filter::calculate_info_gain(const std::vector<geometry_msgs::msg::Point>& points)
{
    if (!this->valid_map_)
        return points.size();
    
    size_t frontiers = 0;
    // TODO: check only within lidar range value
    for (const auto &p : points)
    {
        if (is_unexplored(p))
            ++frontiers;
    }
    return (double)frontiers*this->map_->info.resolution;
}

// navigation cost is outdated with each robot movement
// therefore it'll be reconsidered in task allocator node after frontiers are received
// and here it's considered locally for priority queue reordering
// the only information about the cost being stored is Information Gain cost
void Filter::calculate_cluster_cost(FrontierCluster& frontier, geometry_msgs::msg::Point& r)
{
    double nav_cost = utils::euclidian_dist(frontier.center, r);
    double info_gain = calculate_info_gain(frontier.frontiers);

    // frontier.cost = revenue_function(nav_cost, info_gain);
    frontier.info_gain = info_gain;
    frontier.cost = info_gain;
}

void Filter::cluster_frontiers(std::vector<geometry_msgs::msg::Point> points)
{
    // this->filter_timer_->cancel();
    // this->local_frontier_sub_.reset();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

    // size_t n = this->frontier_queue_.size();
    // std::unordered_set<geometry_msgs::msg::Point>::iterator it = this->frontier_queue_.begin();

    // pcl::PointXYZ pc;
    // while (n--) {
    //     pc.x = (*it).x;
    //     pc.y = (*it).y;
    //     pc.z = (*it).z;
    //     cloud->push_back(pc);
    //     if (n)
    //         ++it;
    // }

    pcl::PointXYZ pc;
    for (const auto& p: points) {
        pc.x = p.x;
        pc.y = p.y;
        pc.z = p.z;
        cloud->push_back(pc);
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // TODO: make parameters
    ec.setClusterTolerance (0.05); // 8cm
    ec.setMinClusterSize (8);
    ec.setMaxClusterSize (20);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    FrontierCluster frontier;
    geometry_msgs::msg::Point p;
    // TODO: change visualization code and remove this code
    std::vector<FrontierCluster> clusters;

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
        clusters.push_back(frontier); // for visualization
        RCLCPP_INFO(this->get_logger(), "NAD: cluster size: %ld", frontier.frontiers.size());
    }
    if (this->cluster_queue_.size() > 0) {
        this->clear_queue_ = true;
        // this->frontier_queue_.erase(this->frontier_queue_.begin(), this->frontier_queue_.end());
    }
    RCLCPP_INFO(this->get_logger(), "NAD: cluster queue size: %ld", this->cluster_queue_.size());
    if (clusters.size() > 0)
        visualize_frontiers(clusters);
    // this->local_frontier_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    //     "/local_frontier_detector", 10, std::bind(&Filter::local_frontier_callback, this, std::placeholders::_1));
    // this->filter_timer_->reset();
}

void Filter::visualize_frontiers(std::vector<FrontierCluster> frontiers)
{
    visualization_msgs::msg::MarkerArray marker_vec;
    std::vector<visualization_msgs::msg::Marker> &markers = marker_vec.markers;

    visualization_msgs::msg::Marker m;

    m.header.frame_id = "map";
    m.header.stamp = this->get_clock()->now();
    m.ns = "clusters";
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 255;
    m.color.a = 255;
    m.lifetime = rclcpp::Duration(std::chrono::microseconds(0)); // lives forever
    m.frame_locked = true;
    m.type = visualization_msgs::msg::Marker::SPHERE;

    int id = 0;

    double max_cost = frontiers[frontiers.size() - 1].cost;
    double min_cost = frontiers[0].cost;
    for (auto frontier : frontiers)
    {
        m.id = id++;
        m.pose.position = frontier.center;
        m.scale.x = std::max((1 - (frontier.cost - min_cost) / (max_cost - min_cost)) * 0.6, 0.2);
        m.scale.y = std::max((1 - (frontier.cost - min_cost) / (max_cost - min_cost)) * 0.6, 0.2);
        m.scale.z = std::max((1 - (frontier.cost - min_cost) / (max_cost - min_cost)) * 0.6, 0.2);
        markers.push_back(m);
    }
    size_t current_markers_count = markers.size();

    // delete previous markers, which are now unused
    m.action = visualization_msgs::msg::Marker::DELETE;
    for (; id < last_markers_count_; id++)
    {
        m.id = id;
        markers.push_back(m);
    }
    last_markers_count_ = current_markers_count;
    marker_array_publisher_->publish(marker_vec);
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

// RCLCPP_COMPONENTS_REGISTER_NODE(Filter)
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Filter>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return(0);
}