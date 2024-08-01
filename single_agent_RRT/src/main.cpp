#include <memory>

#include "single_agent_rrt/local_frontier_detector.hpp"
#include "single_agent_rrt/filter.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // rclcpp::executors::SingleThreadedExecutor executor;
        rclcpp::executors::MultiThreadedExecutor executor;
    // TODO: switch to static
    // rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto filter = std::make_shared<Filter>(options);
    executor.add_node(filter);

    auto local_frontier_detector = std::make_shared<LocalFrontierDetector>(options);
    executor.add_node(local_frontier_detector);

    executor.spin();

    rclcpp::shutdown();
  
    return 0;
}