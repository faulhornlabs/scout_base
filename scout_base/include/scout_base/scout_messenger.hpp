/* 
 * scout_messenger.hpp
 * 
 * Created on: Jun 14, 2019 10:24
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_MESSENGER_HPP
#define SCOUT_MESSENGER_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <scout_msgs/msg/scout_light_cmd.hpp>
#include <scout_msgs/msg/scout_status.hpp>
#include <ugv_sdk/scout/scout_base.hpp>
#ifdef NO_ERROR
#undef NO_ERROR
#endif

namespace westonrobot
{
class ScoutROSMessenger
{
public:
    explicit ScoutROSMessenger(rclcpp::Node::SharedPtr nh);
    ScoutROSMessenger(std::shared_ptr<ScoutBase> scout, rclcpp::Node::SharedPtr nh);

    std::string odom_frame_;
    std::string base_frame_;
    std::string odom_topic_name_;

    bool simulated_robot_ = false;
    int sim_control_rate_ = 50;

    void SetupSubscription();

    void PublishStateToROS();
    void PublishSimStateToROS(double linear, double angular);

    void GetCurrentMotionCmdForSim(double &linear, double &angular);

private:
    std::shared_ptr<ScoutBase> scout_;
    rclcpp::Node::SharedPtr nh_;

    std::mutex twist_mutex_;
    geometry_msgs::msg::Twist current_twist_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<scout_msgs::msg::ScoutStatus>::SharedPtr status_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_subscriber_;
    rclcpp::Subscription<scout_msgs::msg::ScoutLightCmd>::SharedPtr light_cmd_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // speed variables
    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    rclcpp::Time last_time_;
    rclcpp::Time current_time_;

    void TwistCmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void LightCmdCallback(const scout_msgs::msg::ScoutLightCmd::ConstSharedPtr msg);
    void PublishOdometryToROS(double linear, double angular, double dt);
};
} // namespace westonrobot

#endif /* SCOUT_MESSENGER_HPP */
