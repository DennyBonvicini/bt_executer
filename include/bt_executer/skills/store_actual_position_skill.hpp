#include <bt_executer/utils.hpp>
#include <std_msgs/msg/string.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using DynamicJointState = control_msgs::msg::DynamicJointState;
using InterfaceValue = control_msgs::msg::InterfaceValue;

class StoreActualPositionNode : public BT::RosTopicSubNode<DynamicJointState>
{
public:
  StoreActualPositionNode(const std::string& name, const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
          {
            BT::InputPort<std::string>("topic_name")
          }
          );
  }

  BT::NodeStatus onTick(const std::shared_ptr<DynamicJointState>& last_msg) override;

private:
  std::string ns_;
  std::shared_ptr<rclcpp::Publisher<InterfaceValue>> joint_position_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Pose>> cartesian_position_publisher_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
