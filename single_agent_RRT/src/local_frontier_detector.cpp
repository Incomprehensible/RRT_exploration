#include <functional>
#include <sstream>
#include <random>
// #include <cmath>
// #include <float.h>

#include "single_agent_rrt/local_frontier_detector.hpp"
#include "single_agent_rrt/utils.hpp"

// we don't have a trim_RRT() method here because it resets itself at each detection step

LocalFrontierDetector::LocalFrontierDetector(const rclcpp::NodeOptions &options, const std::string& name)
    : Node(name, options),
    RRT_(2 /*dim*/, this->pcloud_, {MAX_LEAF /* max leaf */}), RRT_viz_(this),
    tf_buffer_(), tf_listener_(tf_buffer_)
{
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    this->pcloud_.pts.resize(POINT_CLOUD_SIZE); 

    this->valid_map_ = false;
    this->reset_RRT_ = true;
    this->pcloud_index_ = 0;

    this->detector_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&LocalFrontierDetector::detect_frontiers, this));
    this->detector_timer_->cancel();
    
    this->frontier_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/local_frontier_detector", 10);
    this->map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&LocalFrontierDetector::map_callback, this, std::placeholders::_1));
    // this->explored_frontier_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        // "active_frontier", 10, std::bind(&LocalFrontierDetector::active_frontier_callback, this, std::placeholders::_1)) ;

    // TODO: move to robot task allocator
    // Nav2 navigation action
    // this->nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    // this->marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt", 10);
}

void LocalFrontierDetector::map_callback(const nav_msgs::msg::OccupancyGrid & map)
{
    this->map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(map);

    if (!this->valid_map_)
    {
        this->valid_map_ = true;
        // RCLCPP_INFO(this->get_logger(), "NAD: Got valid map!");
        this->detector_timer_->reset();
    }
    // this->map_.origin_ = map.info.origin;
    // this->map_.width_ = map.info.width;
    // this->map_.height_ = map.info.height;
    // this->map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(map);
    //copy(map.data.begin(), map.data.end(), std::back_inserter(this->map_.grid_));  
}

// void LocalFrontierDetector::active_frontier_callback(const geometry_msgs::msg::Point &frontier)
// {

// }

// geometry_msgs::msg::TransformStamped::SharedPtr LocalFrontierDetector::get_robot_position()
// {
//     geometry_msgs::msg::TransformStamped robot2map;

//     try {
//         robot2map = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
//     } catch (tf2::TransformException& e) {
//         RCLCPP_ERROR(this->get_logger(), "Map to robot transform not found: %s", e.what());
//         return nullptr;
//     }
//     return std::make_shared<geometry_msgs::msg::TransformStamped>(std::move(robot2map));
// }

// initializes RRT with current robot's pose p_init
bool LocalFrontierDetector::RRT_reset()
{
    // RCLCPP_INFO(this->get_logger(), "NAD: resetting RRT!");
    // TODO: do I need to clear the tree?
    if (this->pcloud_index_ != 0) {
        for (size_t i = 0; i <= this->pcloud_index_; ++i)
            this->RRT_.removePoint(i);
    }
    this->pcloud_index_ = 0;
    // needed for RRT initialization at each detection step
    geometry_msgs::msg::TransformStamped::SharedPtr robot2map_tf = utils::get_robot_position(&tf_buffer_);
    if (robot2map_tf == nullptr)
        return false;
    auto robot2map = *(robot2map_tf.get());

    geometry_msgs::msg::PoseStamped robot_pose = geometry_msgs::msg::PoseStamped();
    robot_pose.pose.position.x = robot2map.transform.translation.x;
    robot_pose.pose.position.y = robot2map.transform.translation.y;
    robot_pose.header.stamp = robot2map.header.stamp;
    
    // RCLCPP_INFO(this->get_logger(), "NAD: got robot pose: x:%f, y:%f", robot_pose.pose.position.x, robot_pose.pose.position.y);
    
    // this->pcloud_.pts.clear();
    this->pcloud_.pts[0].x = robot_pose.pose.position.x;
    this->pcloud_.pts[0].y = robot_pose.pose.position.y;
    // TODO: remove this line
    this->pcloud_.pts[0].z = robot_pose.pose.position.z;

    RRT_.addPoints(0, 0);
    // this->pcloud_index_++;

    // visualization reset
    // this->RRT_viz_.clear_tree();

    return true;
}

void LocalFrontierDetector::RRT_add_point(const geometry_msgs::msg::Point& p)
{
    this->pcloud_index_++;

    if (this->pcloud_.pts.size() == POINT_CLOUD_SIZE)
        this->pcloud_.pts.resize(POINT_CLOUD_SIZE*2);
    
    this->pcloud_.pts[this->pcloud_index_].x = p.x;
    this->pcloud_.pts[this->pcloud_index_].y = p.y;
    // TODO: remove this line
    this->pcloud_.pts[this->pcloud_index_].z = p.z;

    RRT_.addPoints(this->pcloud_index_, this->pcloud_index_);
    // this->pcloud_index_++;
}

geometry_msgs::msg::Point LocalFrontierDetector::RRT_find_nearest_neighbor(const geometry_msgs::msg::Point& p_rand)
{
    // std::cout << "Searching for 1 element..." << std::endl;

    // do a knn search
    const size_t num_results = 1;
    size_t       ret_index;
    double      out_dist_sqr;

    double query_pt[3] = {p_rand.x, p_rand.y, p_rand.z};

    nanoflann::KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_index, &out_dist_sqr);

    // TODO: why 10?
    RRT_.findNeighbors(resultSet, query_pt, {10});
    // std::cout << "knnSearch(nn=" << num_results << "): \n";
    // std::cout << "ret_index=" << ret_index
    //           << " out_dist_sqr=" << out_dist_sqr << std::endl;
    // std::cout << "point: ("
    //           << "point: (" << this->pcloud_.pts[ret_index].x << ", "
    //           << this->pcloud_.pts[ret_index].y << ", " << this->pcloud_.pts[ret_index].z
    //           << ")" << std::endl;
    // std::cout << std::endl;

    geometry_msgs::msg::Point p_nearest;
    p_nearest.x = this->pcloud_.pts[ret_index].x;
    p_nearest.y = this->pcloud_.pts[ret_index].y;
    // TODO: remove this line
    p_nearest.z = this->pcloud_.pts[ret_index].z;

    return p_nearest;
}

// TODO: remove
bool LocalFrontierDetector::RRT_is_outdated_frontier(const geometry_msgs::msg::Point& frontier)
{
    return false;
}

// inline double LocalFrontierDetector::euclidian_dist(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
// {
//     return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
// }

// inline bool LocalFrontierDetector::comparable(double a, double b)
// {
//     return std::fabs(a - b) <= DBL_EPSILON;
// }

inline bool LocalFrontierDetector::within_expansion_dist(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
{
    // RCLCPP_INFO(this->get_logger(), "NAD: dist between p_nearest and p_cand: %f", utils::euclidian_dist(p1, p2));
    return (utils::euclidian_dist(p1, p2)) <= RRT_EXPANSION_RATE;
    // return (comparable(RRT_EXPANSION_RATE, dist))? true : false;
}

LocalFrontierDetector::MAP_STATUS LocalFrontierDetector::grid_check(const geometry_msgs::msg::Point& p)
{
    int i = (p.x - this->map_->info.origin.position.x) / this->map_->info.resolution;
    int j = (p.y - this->map_->info.origin.position.y) / this->map_->info.resolution;

    // RCLCPP_INFO(this->get_logger(), "NAD: grid status at p_cand: %d", this->map_->data[j*this->map_->info.width + i]);

    return static_cast<MAP_STATUS>(this->map_->data[j*this->map_->info.width + i]);
}

// TODO: WTF obstacle is 100?
void LocalFrontierDetector::dbg_map_print()
{
    RCLCPP_INFO(this->get_logger(), "NAD: map:");
    for (size_t j=0; j<this->map_->info.height; ++j) {
        for (size_t i=0; i<this->map_->info.width; ++i) {
            RCLCPP_INFO(this->get_logger(), "map[j][i]: %d", this->map_->data[j*this->map_->info.width + i]);
        }
    }
}

// Is a function that takes two points x, y, and returns
// a point z, where ‖z − y‖ is minimized, while ‖z − x‖ ≤ η,
// for an η > 0, η is the tree growth rate. Large value of η
// corresponds to a faster tree growth (i.e tree expands faster).
std::pair<geometry_msgs::msg::Point::SharedPtr, LocalFrontierDetector::MAP_STATUS> 
    LocalFrontierDetector::RRT_steer(const geometry_msgs::msg::Point& p_nearest, const geometry_msgs::msg::Point& p_rand)
{
    // for simplicity choose randomly sample point if it's already close enough
    // we know p_rand was sampled from free space
    // if (within_expansion_dist(p_nearest, p_rand))
    //     return std::make_pair(std::make_shared<geometry_msgs::msg::Point>(p_rand), MAP_STATUS::FREE);
    
    geometry_msgs::msg::Point::SharedPtr p_new_ptr = nullptr;
    MAP_STATUS p_new_status = MAP_STATUS::UNDEFINED;
    auto p_new = std::make_pair(p_new_ptr, p_new_status);

    geometry_msgs::msg::Point p_cand;
    // construct a line between x and y
    const double step = this->map_->info.resolution;//1 / this->map_->info.resolution;
    // RCLCPP_INFO(this->get_logger(), "NAD: step: %f", this->map_->info.resolution);

    double lambda = step;
    p_cand.x = (1-lambda)*p_nearest.x + lambda*p_rand.x;
    p_cand.y = (1-lambda)*p_nearest.y + lambda*p_rand.y;
    // RCLCPP_INFO(this->get_logger(), "NAD: 1st p_cand: x:%f, y:%f", p_cand.x, p_cand.y);

    while (!utils::comparable(lambda, 1) && within_expansion_dist(p_nearest, p_cand)) //&& grid_check(p_cand) != MAP_STATUS::OBSTACLE)
    {
        if ((p_new_status = grid_check(p_cand)) == MAP_STATUS::OBSTACLE)
            break;
        
        // RCLCPP_INFO(this->get_logger(), "NAD: Doing one step!");
        lambda += step;
        p_new.first = std::make_shared<geometry_msgs::msg::Point>(p_cand);
        // if we had an unknown point on the line we consider new point a frontier by deafult
        p_new.second = (p_new.second == MAP_STATUS::UNKNOWN)? p_new.second : p_new_status;
        // p_new_ptr = std::make_shared<geometry_msgs::msg::Point>(p_cand);
        // RCLCPP_INFO(this->get_logger(), "NAD: Last chosen cand: x:%f, y:%f!", p_cand.x, p_cand.y);

        p_cand.x = (1-lambda)*p_nearest.x + lambda*p_rand.x;
        p_cand.y = (1-lambda)*p_nearest.y + lambda*p_rand.y;
    }

    return p_new;
}

// searches for frontier points and builds local RRT tree
// keeps a memory buffer of explored frontier points (centroids of frontier clusters which map to one frontier point found)
// discards a found frontier point if it is contained in a buffer (hash table)
void LocalFrontierDetector::detect_frontiers()
{
    // RCLCPP_INFO(this->get_logger(), "NAD: Starting to detect frontiers!");

    if (this->reset_RRT_) {
        if (RRT_reset())
            this->reset_RRT_ = false;
        else
            return;
    }

    // TODO: check difference in initialization
    // auto seed = chrono::steady_clock::now().time_since_epoch().count();
    std::random_device seed; // obtain a random number from hardware

    // RCLCPP_INFO(this->get_logger(), "NAD: map width: %d", this->map_->info.width);
    // RCLCPP_INFO(this->get_logger(), "NAD: map height: %d", this->map_->info.height);

    // distribution over closed interval
    std::uniform_int_distribution d_row(0, (int)(this->map_->info.width)-1);
    std::uniform_int_distribution d_col(0, (int)(this->map_->info.height)-1);
    auto gen_row = std::bind(d_row, std::mt19937(seed()));
    auto gen_col = std::bind(d_col, std::mt19937(seed()));

    // randomly sample points and build RRT tree until frontier is found
    int32_t i, j;

    geometry_msgs::msg::Point p_rand;
    geometry_msgs::msg::Point p_nearest;
    geometry_msgs::msg::Point p_new;
    while (true)
    {
        i = gen_row();
        j = gen_col();

        // RCLCPP_INFO(this->get_logger(), "NAD: map value: i:%d, j:%d, val: %d", i, j, this->map_->data[j*this->map_->info.width + i]);
        // sample point p from free space
        if (this->map_->data[j*this->map_->info.width + i] == MAP_STATUS::FREE)
        {
            // The origin of the map [m, m, rad]. This is the real-world pose of the cell (0,0) in the map.
            p_rand.x = this->map_->info.origin.position.x + i*this->map_->info.resolution;
            p_rand.y = this->map_->info.origin.position.y + j*this->map_->info.resolution;

            // RCLCPP_INFO(this->get_logger(), "NAD: random point: x:%f, y:%f", p_rand.x, p_rand.y);

            p_nearest = RRT_find_nearest_neighbor(p_rand);
            // RCLCPP_INFO(this->get_logger(), "NAD: nearest point: x:%f, y:%f", p_nearest.x, p_nearest.y);

            auto [p_new_ptr, p_new_status] = RRT_steer(p_nearest, p_rand);
            // no point found or we hit an obstacle in that direction
            if (p_new_ptr == nullptr)
                continue;
            p_new = *(p_new_ptr.get());
            // auto p_new_ptr = RRT_steer(p_nearest, p_rand);
            // if (p_new_ptr == nullptr)
            //     continue;
            // p_new = *(p_new_ptr.get());
            // RCLCPP_INFO(this->get_logger(), "NAD: new point: x:%f, y:%f", p_new.x, p_new.y);
            // RCLCPP_INFO(this->get_logger(), "NAD: new point status: %d", p_new_status);
            // here check in memory if found frontier was explored
            // add point to RRT or publish and set rrt reset flag
            if (p_new_status == MAP_STATUS::UNKNOWN && !RRT_is_outdated_frontier(p_new)) {
                publish_frontier(p_new);
                this->reset_RRT_ = true;
                break;
            }
            else // FREE SPACE 
                RRT_add_point(p_new);

            // this->RRT_viz_.add_edge(p_nearest, p_new);
        }
    }
}

void LocalFrontierDetector::publish_frontier(const geometry_msgs::msg::Point& frontier)
{
    RCLCPP_INFO(this->get_logger(), "NAD: PUBLISH FRONTIER!");
    this->frontier_pub_->publish(frontier);

    // TODO: remove this code
    // this->detector_timer_->cancel();

    // TODO: move this code to robot task allocator
    // geometry_msgs::msg::PoseStamped goal_pose;
    // goal_pose.pose.position.x = frontier.x;
    // goal_pose.pose.position.y = frontier.y; 
    // goal_pose.pose.orientation.w = 1.0;
    // goal_pose.header.frame_id = "map";
    // goal_pose.header.stamp = this->get_clock()->now();

    // auto goal_msg = NavigateToPose::Goal();
    // goal_msg.pose = goal_pose;

    // auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // send_goal_options.result_callback = std::bind(&LocalFrontierDetector::goal_response_callback, this, std::placeholders::_1);

    // RCLCPP_INFO(this->get_logger(), "Wanted position x=%f, y=%f.", goal_pose.pose.position.x, goal_pose.pose.position.y);
    // nav2_client_->async_send_goal(goal_msg, send_goal_options);
}

// TODO: move to robot task allocator
// void LocalFrontierDetector::goal_response_callback(const GoalHandleNavigate::WrappedResult &result)
// {
//     if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//         RCLCPP_INFO(this->get_logger(), "Goal reached.");
//     }    
//     else {
//         RCLCPP_INFO(this->get_logger(), "Goal failed or was canceled.");
//     }
//     // TODO: remove this code
//     this->detector_timer_->reset();
// }

RCLCPP_COMPONENTS_REGISTER_NODE(LocalFrontierDetector)