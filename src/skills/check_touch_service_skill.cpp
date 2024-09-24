#include <bt_executer/skills/check_touch_service_skill.hpp>
#include <chrono>
#include <behaviortree_ros2/plugins.hpp>

CheckingTouch::CheckingTouch(const std::string& name,
                                         const BT::NodeConfig& conf,
                                         const BT::RosNodeParams& params)
  : BT::RosServiceNode<force_sensor::srv::CheckTouch>(name, conf, params)
{
  auto param_ns = getInput<std::string>("param_ns");
  ns_ = "/bt_executer/" + param_ns.value();
}

bool CheckingTouch::setRequest(Request::SharedPtr& request)
{  
	std::string topic_name;
  	std::float_t soglia;
  	
	// Get required parameters
  	std::string w;
  	bt_executer::utils::get_param(node_.lock().get(), ns_, "/topic_name", topic_name, w);
  	bt_executer::utils::get_param(node_.lock().get(), ns_, "/soglia", soglia, w);

  	// send request to service
  	request->topic_name = topic_name;
  	request->soglia = soglia;
  	return true;
}

BT::NodeStatus CheckingTouch::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResponseReceived. Done = %s", name().c_str(),
              response->contact ? "true" : "false");
  if (response->contact)
  {
    RCLCPP_INFO_STREAM(node_.lock()->get_logger(), "OK: " << response->message);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_.lock()->get_logger(), "Error: " << response->message);
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus CheckingTouch::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
} 

// Plugin registration.
// The class CheckingTouch will self register with name  "CheckTouchSkill".
CreateRosNodePlugin(CheckingTouch, "CheckTouchSkill");

