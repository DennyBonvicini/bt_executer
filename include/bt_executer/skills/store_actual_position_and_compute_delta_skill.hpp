#include <bt_executer/utils.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <atomic>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using DynamicJointState = control_msgs::msg::DynamicJointState;
using InterfaceValue = control_msgs::msg::InterfaceValue;

class StoreActualPositionNodeAndComputeDelta : public BT::RosTopicSubNode<DynamicJointState>
{
public:
  StoreActualPositionNodeAndComputeDelta(const std::string& name, const BT::NodeConfig& conf,
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
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> delta_joint_position_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> delta_cartesian_position_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cartesian_position_sub_;
  rclcpp::Subscription<InterfaceValue>::SharedPtr joint_position_sub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Callback per il subscriber al topic /previous_position
  void previousPositionCallback(const InterfaceValue::SharedPtr msg);
  double previous_x, previous_y, previous_z;

  // Callback per il subscriber al topic /previous_position
  void cartesianPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

  //variabile per memorizzare posizioni precedenti
  std::vector<float> previous_positions_;
  
  // Flag per verificare se i dati sono stati ricevuti
  std::atomic<bool> previous_positions_received_{false};
};
