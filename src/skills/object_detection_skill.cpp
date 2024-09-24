#include <bt_executer/skills/object_detection_skill.hpp>
#include <chrono>
#include <behaviortree_ros2/plugins.hpp>

using ObjectDetection = yolov8::srv::ObjectDetection;

Detection::Detection(const std::string& name,
                                         const BT::NodeConfig& conf,
                                         const BT::RosNodeParams& params)
  : BT::RosServiceNode<ObjectDetection>(name, conf, params)
{
  auto param_ns = getInput<std::string>("param_ns");
  ns_ = "/bt_executer/" + param_ns.value();
}

bool Detection::setRequest(Request::SharedPtr& request)
{  
	std::string script_path;
  	
	// Get required parameters
  	std::string w;
  	bt_executer::utils::get_param(node_.lock().get(), ns_, "/script_path", script_path, w);
  	

  	// send request to service
  	request->script_path = script_path;
  	return true;
}

BT::NodeStatus Detection::onResponseReceived(const Response::SharedPtr& response)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResponseReceived. Done = %s", name().c_str(),
              response->detected ? "true" : "false");
  if(response->detected)
  {
    RCLCPP_ERROR_STREAM(node_.lock()->get_logger(), "OK: " << response->message);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_.lock()->get_logger(), "Error: " << response->message);
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus Detection::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
} 

// Plugin registration.
// The class Detection will self register with name  "ObjectDetectionSkill".
CreateRosNodePlugin(Detection, "ObjectDetectionSkill");

