#include <bt_executer/skills/compute_and_publish_center_skill.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

using ComputeCenter = tf::srv::ComputeCenter;

ComputeAndPublishCenter::ComputeAndPublishCenter(const std::string& name,
                  const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params)
      : BT::RosServiceNode<ComputeCenter>(name, conf, params)
    {
        //auto param_ns = getInput<std::string>("param_ns");
        //ns_ = "/bt_executer/" + param_ns.value();
    }

bool ComputeAndPublishCenter::setRequest(Request::SharedPtr& request)
{
    return true;
}

BT::NodeStatus ComputeAndPublishCenter::onResponseReceived(const Response::SharedPtr& response)
{
        RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResponseReceived. Success = %s",
                    name().c_str(), response->success ? "true" : "false");
        if (response->success)
        {
            RCLCPP_INFO_STREAM(node_.lock()->get_logger(), "TFs successfully published.");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR_STREAM(node_.lock()->get_logger(), "Failed to publish Centered Screw TF.");
            return BT::NodeStatus::FAILURE;
        }
}

BT::NodeStatus ComputeAndPublishCenter::onFailure(BT::ServiceNodeErrorCode error)
{
        RCLCPP_ERROR(node_.lock()->get_logger(), "%s: onFailure with error: %s",
                     name().c_str(), toStr(error));
        return BT::NodeStatus::FAILURE;
}

// Plugin registration.
// The class ComputeAndPublishCenter will self register with name  "ComputeAndPublishCenterSkill".
CreateRosNodePlugin(ComputeAndPublishCenter, "ComputeAndPublishCenterSkill");

