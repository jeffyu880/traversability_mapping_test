#include <pluginlib/class_list_macros.hpp>
#include "planner/tm_planner.h"
#include <unistd.h>

//register this planner as a Nav2 GlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(tm_planner::TMPlanner, nav2_core::GlobalPlanner)
 
namespace tm_planner {

    TMPlanner::TMPlanner() : configured_(false) {}

    void TMPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, 
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent;
        auto node = node_.lock();
        name_ = name;
        tf_buffer_ = tf;
        costmap_ros_ = costmap_ros;
        global_frame_ = costmap_ros_->getGlobalFrameID();

        // Initialize subscriptions and publishers
        subPath = node->create_subscription<nav_msgs::msg::Path>(
            "/global_path", 5, 
            std::bind(&TMPlanner::pathHandler, this, std::placeholders::_1));
        
        pubGoal = node->create_publisher<geometry_msgs::msg::PoseStamped>("/prm_goal", 5);
        
        // Visualize twist command
        subTwistCommand1 = node->create_subscription<nav_msgs::msg::Path>(
            "/local_plan", 5, 
            std::bind(&TMPlanner::twistCommandHandler, this, std::placeholders::_1));
        
        subTwistCommand2 = node->create_subscription<nav_msgs::msg::Path>(
            "/transformed_global_plan", 5, 
            std::bind(&TMPlanner::twistCommandHandler, this, std::placeholders::_1));
        
        // Publisher
        pubTwistCommand = node->create_publisher<nav_msgs::msg::Path>("/twist_command", 5);

        configured_ = true;
    }

    void TMPlanner::cleanup()
    {
        configured_ = false;
    }

    void TMPlanner::activate()
    {
        // Activation logic if needed
    }

    void TMPlanner::deactivate()
    {
        // Deactivation logic if needed
    }

    // visualize twist command
    void TMPlanner::twistCommandHandler(const nav_msgs::msg::Path::SharedPtr pathMsg){

        try{ 
            auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            
            nav_msgs::msg::Path outTwist = *pathMsg;

            for (size_t i = 0; i < outTwist.poses.size(); ++i)
                outTwist.poses[i].pose.position.z = transform.transform.translation.z + 1.0;

            pubTwistCommand->publish(outTwist);
        } 
        catch (tf2::TransformException& ex){ 
            return; 
        }
    }

    // receive path from prm global planner
    void TMPlanner::pathHandler(const nav_msgs::msg::Path::SharedPtr pathMsg){
        // std::lock_guard<std::mutex> lock(mtx);
        // if the planner couldn't find a feasible path, pose size should be 0
        globalPath = *pathMsg;
    }

    nav_msgs::msg::Path TMPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal)
    {
        nav_msgs::msg::Path plan;
        auto node = node_.lock();
        plan.header.stamp = node->now();
        plan.header.frame_id = global_frame_;

        if (!configured_) {
            RCLCPP_ERROR(node->get_logger(), "Planner has not been configured");
            return plan;
        }

        // 1. Publish Goal to PRM Planner
        RCLCPP_INFO(node->get_logger(), "Goal Received at: [%lf, %lf, %lf]", 
                    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
        pubGoal->publish(goal);

        // 2. Wait for path from PRM planner (simple approach - in practice might want callback)
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        // 3. if the planner couldn't find a feasible path, pose size should be 0
        if (globalPath.poses.size() == 0){
            pubGoal->publish(goal);
            RCLCPP_WARN(node->get_logger(), "No valid path found");
            return plan;
        }

        RCLCPP_INFO(node->get_logger(), "A Valid Path Received!");

        // 4. Extract Path
        for (size_t i = 0; i < globalPath.poses.size(); ++i){
            plan.poses.push_back(globalPath.poses[i]);
        }

        // Make sure the goal orientation is preserved
        if (!plan.poses.empty()) {
            plan.poses.back().pose.orientation = goal.pose.orientation;
        }

        globalPath.poses.clear();

        return plan; 
    }

}