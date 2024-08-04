
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using nav2_costmap_2d::FREE_SPACE;

namespace nav2_custom_costmap_plugin
{

class CustomLayer : public nav2_costmap_2d::Layer
{
    public:
        CustomLayer() : 
        tf_buffer_(std::make_shared<rclcpp::Clock>()), tf_listener_(tf_buffer_)
        {
        }

        // This method is called at the end of plugin initialization.
        // It contains ROS parameter(s) declaration and initialization
        // of need_recalculation_ variable.
        void onInitialize()
        {
            delta_ = R_CLEARANCE_FACTOR*layered_costmap_->getCircumscribedRadius();

            auto node = node_.lock(); 
            declareParameter("enabled", rclcpp::ParameterValue(true));
            node->get_parameter(name_ + "." + "enabled", enabled_);

            need_recalculation_ = false;
            current_ = true;
        }

        void updateBounds(double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
            double * min_y, double * max_x, double * max_y)
        {
            static std::shared_ptr<geometry_msgs::msg::PoseStamped> start_pose_ptr = nullptr;
            if (!enabled_)
                return;

            geometry_msgs::msg::PoseStamped map_pose;
            geometry_msgs::msg::PoseStamped odom_pose;
            map_pose.pose.position.x = robot_x;
            map_pose.pose.position.y = robot_y;
            map_pose.header.frame_id = "map";
            map_pose.header.stamp = rclcpp::Time(0);

            try {
                tf_buffer_.transform(map_pose, odom_pose, "odom", tf2::durationFromSec(1.0));
            }
            catch (tf2::TransformException &ex) {
                RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "CustomLayer: Transform failed: %s", ex.what());
                return;
            }

            if (!start_pose_ptr)
            {
                start_pose_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>(odom_pose);
                RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "CustomLayer: START: min_y: %f, max_y: %f, min_x: %f, max_x: %f", *min_y, *max_y, *min_x, *max_x);
            }
            
            if (sqrt(pow(odom_pose.pose.position.x-(*start_pose_ptr).pose.position.x,2)+pow(odom_pose.pose.position.y-(*start_pose_ptr).pose.position.y,2)) <= delta_) {
                need_recalculation_ = true;
                RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "CustomLayer: robot is within starting pose!");
            }
            else {
                RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "CustomLayer: robot is no longer within starting position!: (%f, %f), r: %f",
                odom_pose.pose.position.x, odom_pose.pose.position.y, delta_);
                need_recalculation_ = false;
            }
    
            if (need_recalculation_) {
                *min_x = robot_x-delta_;
                *min_y = robot_y-delta_;
                *max_x = robot_x+delta_;
                *max_y = robot_y+delta_;
            }
        }

        void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
        {
            if (!enabled_ || !need_recalculation_)
                return;

            // master_array - is a direct pointer to the resulting master_grid.
            // master_grid - is a resulting costmap combined from all layers.
            // By using this pointer all layers will be overwritten!
            // To work with costmap layer and merge it with other costmap layers,
            // please use costmap_ pointer instead (this is pointer to current
            // costmap layer grid) and then call one of updates methods:
            // - updateWithAddition()
            // - updateWithMax()
            // - updateWithOverwrite()
            // - updateWithTrueOverwrite()
            // In this case using master_array pointer is equal to modifying local costmap_
            // pointer and then calling updateWithTrueOverwrite():
            unsigned char * master_array = master_grid.getCharMap();
            unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

            // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
            // These variables are used to update the costmap only within this window
            // avoiding the updates of whole area.

            // Fixing window coordinates with map size if necessary.
            min_i = std::max(0, min_i);
            min_j = std::max(0, min_j);
            max_i = std::min(static_cast<int>(size_x), max_i);
            max_j = std::min(static_cast<int>(size_y), max_j);

            // Simply setting one-by-one cost per each cell            
            for (int j = min_j; j < max_j; j++) {
              for (int i = min_i; i < max_i; i++) {
                int index = master_grid.getIndex(i, j);
                master_array[index] = FREE_SPACE;
              }
            }
        }

        virtual void reset()
        {
          return;
        }

        // The method is called when footprint was changed.
        // Here it updates the active region boundary.
        void onFootprintChanged()
        {
            delta_ = R_CLEARANCE_FACTOR*layered_costmap_->getCircumscribedRadius();
            
            RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "CustomLayer::onFootprintChanged(): num footprint points: %lu", 
                layered_costmap_->getFootprint().size());
        }

        virtual bool isClearable() {return false;}

    private:
        // Indicates that the costmap region should be recalculated next time.
        bool need_recalculation_;

        // Robot footprint-dependent increment for defining the active region around the robot
        double delta_;
        // Factor applied to clearance radius around the robot footprint
        float R_CLEARANCE_FACTOR = 1.5;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
};

}  // namespace nav2_custom_costmap_plugin

// This is the macro allowing a nav2_custom_costmap_plugin::CustomLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_custom_costmap_plugin::CustomLayer, nav2_costmap_2d::Layer)