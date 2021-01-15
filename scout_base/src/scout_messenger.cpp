/*
 * scout_messenger.cpp
 *
 * Created on: Apr 26, 2019 22:14
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_base/scout_messenger.hpp"
#include <scout_msgs/msg/scout_status.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace westonrobot
{
  ScoutROSMessenger::ScoutROSMessenger(rclcpp::Node::SharedPtr nh)
      : scout_(nullptr), nh_(nh) {}

  ScoutROSMessenger::ScoutROSMessenger(std::shared_ptr<ScoutBase> scout, rclcpp::Node::SharedPtr nh)
      : scout_(scout), nh_(nh) {}

  void ScoutROSMessenger::SetupSubscription()
  {
    // odometry publisher
      if (nh_->get_parameter("publish_odometry").as_bool())
      {
          RCLCPP_INFO(nh_->get_logger(), "Createing odom publisher on topic: %s", odom_topic_name_.c_str());
          odom_publisher_ = nh_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, rclcpp::QoS(1));
      }
    
    status_publisher_ = nh_->create_publisher<scout_msgs::msg::ScoutStatus>("/scout_status", rclcpp::QoS(1));

    // cmd subscriber
    motion_cmd_subscriber_ = nh_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 5,
      std::bind(&ScoutROSMessenger::TwistCmdCallback, this, std::placeholders::_1));
    light_cmd_subscriber_ = nh_->create_subscription<scout_msgs::msg::ScoutLightCmd>("/scout_light_control", 5,
      std::bind(&ScoutROSMessenger::LightCmdCallback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_.get());
    current_time_ = nh_->now();
    last_time_ = nh_->now();
  }

  void ScoutROSMessenger::TwistCmdCallback(
      const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    if (!simulated_robot_)
    {
      scout_->SetMotionCommand(msg->linear.x, msg->angular.z);
    }
    else
    {
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }

  void ScoutROSMessenger::GetCurrentMotionCmdForSim(double &linear,
                                                    double &angular)
  {
    std::lock_guard<std::mutex> guard(twist_mutex_);
    linear = current_twist_.linear.x;
    angular = current_twist_.angular.z;
  }

  void ScoutROSMessenger::LightCmdCallback(
      const scout_msgs::msg::ScoutLightCmd::ConstSharedPtr msg)
  {
    if (!simulated_robot_)
    {
      if (msg->enable_cmd_light_control)
      {
        ScoutLightCmd cmd;

        switch (msg->front_mode)
        {
        case scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_OFF:
        {
          cmd.front_mode = ScoutLightCmd::LightMode::CONST_OFF;
          break;
        }
        case scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_ON:
        {
          cmd.front_mode = ScoutLightCmd::LightMode::CONST_ON;
          break;
        }
        case scout_msgs::msg::ScoutLightCmd::LIGHT_BREATH:
        {
          cmd.front_mode = ScoutLightCmd::LightMode::BREATH;
          break;
        }
        case scout_msgs::msg::ScoutLightCmd::LIGHT_CUSTOM:
        {
          cmd.front_mode = ScoutLightCmd::LightMode::CUSTOM;
          cmd.front_custom_value = msg->front_custom_value;
          break;
        }
        }

        switch (msg->rear_mode)
        {
        case scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_OFF:
        {
          cmd.rear_mode = ScoutLightCmd::LightMode::CONST_OFF;
          break;
        }
        case scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_ON:
        {
          cmd.rear_mode = ScoutLightCmd::LightMode::CONST_ON;
          break;
        }
        case scout_msgs::msg::ScoutLightCmd::LIGHT_BREATH:
        {
          cmd.rear_mode = ScoutLightCmd::LightMode::BREATH;
          break;
        }
        case scout_msgs::msg::ScoutLightCmd::LIGHT_CUSTOM:
        {
          cmd.rear_mode = ScoutLightCmd::LightMode::CUSTOM;
          cmd.rear_custom_value = msg->rear_custom_value;
          break;
        }
        }

        scout_->SetLightCommand(cmd);
      }
      else
      {
        scout_->DisableLightCmdControl();
      }
    }
    else
    {
      std::cout << "simulated robot received light control cmd" << std::endl;
    }
  }

  void ScoutROSMessenger::PublishStateToROS()
  {
    current_time_ = nh_->now();
    double dt = std::chrono::duration<double>((current_time_ - last_time_).nanoseconds()).count();

    static bool init_run = true;
    if (init_run)
    {
      last_time_ = current_time_;
      init_run = false;
      return;
    }

    auto state = scout_->GetScoutState();

    // publish scout state message
    scout_msgs::msg::ScoutStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = state.linear_velocity;
    status_msg.angular_velocity = state.angular_velocity;

    status_msg.base_state = state.base_state;
    status_msg.control_mode = state.control_mode;
    status_msg.fault_code = state.fault_code;
    status_msg.battery_voltage = state.battery_voltage;

    for (int i = 0; i < 4; ++i)
    {
      status_msg.motor_states[i].current = state.motor_states[i].current;
      status_msg.motor_states[i].rpm = state.motor_states[i].rpm;
      status_msg.motor_states[i].temperature = state.motor_states[i].temperature;
    }

    status_msg.light_control_enabled = state.light_control_enabled;
    status_msg.front_light_state.mode = state.front_light_state.mode;
    status_msg.front_light_state.custom_value =
        state.front_light_state.custom_value;
    status_msg.rear_light_state.mode = state.rear_light_state.mode;
    status_msg.rear_light_state.custom_value =
        state.front_light_state.custom_value;
    status_publisher_->publish(status_msg);

    // publish odometry and tf
    if (odom_publisher_)
    {
        PublishOdometryToROS(state.linear_velocity, state.angular_velocity, dt);
    }

    // record time for next integration
    last_time_ = current_time_;
  }

  void ScoutROSMessenger::PublishSimStateToROS(double linear, double angular)
  {
    current_time_ = nh_->now();
    double dt = std::chrono::duration<double>((current_time_ - last_time_).nanoseconds()).count();

    static bool init_run = true;
    if (init_run)
    {
      last_time_ = current_time_;
      init_run = false;
      return;
    }

    // publish scout state message
    scout_msgs::msg::ScoutStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = linear;
    status_msg.angular_velocity = angular;

    status_msg.base_state = 0x00;
    status_msg.control_mode = 0x01;
    status_msg.fault_code = 0x00;
    status_msg.battery_voltage = 29.5;

    // for (int i = 0; i < 4; ++i)
    // {
    //     status_msg.motor_states[i].current = state.motor_states[i].current;
    //     status_msg.motor_states[i].rpm = state.motor_states[i].rpm;
    //     status_msg.motor_states[i].temperature =
    //     state.motor_states[i].temperature;
    // }

    status_msg.light_control_enabled = false;
    // status_msg.front_light_state.mode = state.front_light_state.mode;
    // status_msg.front_light_state.custom_value =
    // state.front_light_state.custom_value; status_msg.rear_light_state.mode =
    // state.rear_light_state.mode; status_msg.rear_light_state.custom_value =
    // state.front_light_state.custom_value;

    status_publisher_->publish(status_msg);

    // publish odometry and tf
    if (odom_publisher_)
    {
        PublishOdometryToROS(linear, angular, dt);
    }

    // record time for next integration
    last_time_ = current_time_;
  }

  void ScoutROSMessenger::PublishOdometryToROS(double linear, double angular,
                                               double dt)
  {
    // perform numerical integration to get an estimation of pose
    linear_speed_ = linear;
    angular_speed_ = angular;

    double d_x = linear_speed_ * std::cos(theta_) * dt;
    double d_y = linear_speed_ * std::sin(theta_) * dt;
    double d_theta = angular_speed_ * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    // publish tf transformation
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;
    RCLCPP_INFO(nh_->get_logger(), "Sending transform from odom_frame %s to base_frame %s ", odom_frame_.c_str(), base_frame_.c_str());
    tf_broadcaster_->sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed_;

    odom_publisher_->publish(std::move(odom_msg));

    // TODO - hack do not know how to do this
    ////map and odometry are identical frames
    //geometry_msgs::msg::TransformStamped T_map_odom;
    //T_map_odom.header.frame_id = "map";
    //T_map_odom.header.stamp = current_time_;
    //T_map_odom.child_frame_id = odom_frame_;
    //tf2::Transform odom_to_map;
    //odom_to_map.setIdentity();
    //T_map_odom.transform = tf2::toMsg(odom_to_map);
    //tf_broadcaster_->sendTransform(T_map_odom);
  }
} // namespace westonrobot