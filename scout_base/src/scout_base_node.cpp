#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ugv_sdk/scout/scout_base.hpp>
#include <scout_base/scout_messenger.hpp>


void DetachRobot(std::shared_ptr<westonrobot::ScoutBase> robot)
{
  robot->Disconnect();
  robot->Terminate();
}

int main(int argc, char **argv) 
{
  // setup ROS node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("scout_base_node");

  rclcpp::Logger logger = node->get_logger();

  // check wether controlling scout mini
  bool is_scout_mini = node->declare_parameter("is_scout_mini", true);
  RCLCPP_INFO(logger, "Working as scout mini: %d",  int(is_scout_mini));

  // instantiate a robot object
  std::shared_ptr<westonrobot::ScoutBase> robot =
    std::make_shared<westonrobot::ScoutBase>(is_scout_mini);
  std::shared_ptr<westonrobot::ScoutROSMessenger> messenger =
    std::make_shared<westonrobot::ScoutROSMessenger>(robot, node);

  // fetch parameters before connecting to robot
  std::string port_name = node->declare_parameter("port_name", std::string("can0"));
  messenger->odom_frame_ = node->declare_parameter("odom_frame", std::string("odom"));
  messenger->base_frame_ = node->declare_parameter("base_frame", std::string("base_link"));
  messenger->simulated_robot_ = node->declare_parameter("simulated_robot", false);
  messenger->sim_control_rate_ = node->declare_parameter("control_rate", 50);
  messenger->odom_topic_name_ = node->declare_parameter("odom_topic_name", std::string("odom"));
  node->declare_parameter("publish_odometry", false);

  if (!messenger->simulated_robot_) {
    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos) {
      robot->Connect(port_name);
      RCLCPP_INFO(logger, "Using CAN bus to talk with the robot, interface name: %s", port_name.c_str());
    } else {
      robot->Connect(port_name, 115200);
      RCLCPP_INFO(logger, "Using UART to talk with the robot");
    }
  }
  messenger->SetupSubscription();

  // publish robot state at 50Hz while listening to twist commands
  rclcpp::Rate loop_rate(50);
  while (rclcpp::ok()) 
  {
    if (!messenger->simulated_robot_)
    {
      messenger->PublishStateToROS();
    }
    else
    {
      double linear, angular;
      messenger->GetCurrentMotionCmdForSim(linear, angular);
      messenger->PublishSimStateToROS(linear, angular);
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  RCLCPP_INFO(logger, "Detach robot");
  DetachRobot(robot);
  RCLCPP_INFO(logger, "Reset messenger!");
  messenger.reset();
  RCLCPP_INFO(logger, "Reset robot");
  robot.reset();
  RCLCPP_INFO(logger, "Reset node");
  node.reset();
  RCLCPP_INFO(logger, "Shutdown ros");
  rclcpp::shutdown();
  return 0;
}