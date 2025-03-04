#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PositionPublisher : public rclcpp::Node, public std::enable_shared_from_this<PositionPublisher>
{
public:
  PositionPublisher()
  : Node("position_publisher")
  {
    topic_based_joint_commands_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("topic_based_joint_commands", 10);
    
    // moveit::planning_interface::MoveGroupInterface move_group_interface(std::make_shared<rclcpp::Node>(this->get_name()), "arm");

    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        std::shared_ptr<rclcpp::Node>(this), "arm");
    // auto move_group_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    // std::shared_ptr<rclcpp::Node>(this), "gripper");
    move_group_interface_->setMaxVelocityScalingFactor(1.0);
    move_group_interface_->setMaxAccelerationScalingFactor(1.0);
    move_group_interface_->setPlanningTime(20.0); // seconds


    // std::vector<double> joint_values = {0, 0, 0, 0, 0, 0};
    // move_group_interface->setJointValueTarget(joint_values);
    dupa();
    // setTargetPose(0.0, 0.0, -0.15);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_states_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_commands_publisher_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

  void dupa()
  {
    // At the start of setTargetPose
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_interface_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_interface_->getEndEffectorLink().c_str());

    // List known poses
    std::vector<std::string> named_targets = move_group_interface_->getNamedTargets();
    for (const auto& target : named_targets) {
      RCLCPP_INFO(this->get_logger(), "Named target: %s", target.c_str());
      // Try a named target if available
    if (!named_targets.empty()) {
      move_group_interface_->setNamedTarget(named_targets[1]);
    }
}
  }

  

  void setTargetPose(double x, double y, double z)
  {
    RCLCPP_INFO(this->get_logger(), "Setting target pose: x=%f, y=%f, z=%f", x, y, z);
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.pose.orientation.x = 0;
    target_pose.pose.orientation.y = 0;
    target_pose.pose.orientation.z = 0;
    target_pose.pose.orientation.w = 1.0;
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
    target_pose.header.frame_id = "tcp_connector";
    target_pose.header.stamp = this->now();

    // geometry_msgs::msg::PoseStamped current_pose = move_group_interface->getCurrentPose();
    // RCLCPP_INFO(rclcpp::get_logger("position_publisher"), "Current pose: x=%f, y=%f, z=%f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);


    move_group_interface_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded, sending joint commands");
      move_group_interface_->execute(plan);
      for (const auto& trajectory_point : plan.trajectory_.joint_trajectory.points)
      {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.name = plan.trajectory_.joint_trajectory.joint_names;
        joint_state_msg.position = trajectory_point.positions;
        joint_state_msg.velocity = trajectory_point.velocities;
        joint_state_msg.effort = {}; 

        topic_based_joint_commands_publisher_->publish(joint_state_msg);
        
        RCLCPP_INFO(this->get_logger(), "Publishing joint command");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
  }

  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
  // std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<PositionPublisher>());
  auto node = std::make_shared<PositionPublisher>();

    // Uruchomienie węzła
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

