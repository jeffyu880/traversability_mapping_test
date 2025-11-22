#ifndef TM_PLANNER_H_
#define TM_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace tm_planner {

    class TMPlanner : public nav2_core::GlobalPlanner {

    public:
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubGoal;

        // visualize twist command
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subTwistCommand1; // twist command from nav2
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subTwistCommand2; // twist command from nav2
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubTwistCommand; // adjust twist command height to show above the robot

        nav_msgs::msg::Path globalPath;

        std::mutex mtx;

        TMPlanner(); 
        ~TMPlanner() = default;

        // overridden classes from interface nav2_core::GlobalPlanner
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name, 
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped & start,
            const geometry_msgs::msg::PoseStamped & goal) override;

        void pathHandler(const nav_msgs::msg::Path::SharedPtr pathMsg);

        // visualize twist command
        void twistCommandHandler(const nav_msgs::msg::Path::SharedPtr pathMsg);

    private:
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        std::string global_frame_, name_;
        bool configured_;
    };

}

#endif