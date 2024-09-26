#include <bt_executer/skills/tf_publish_service_skill.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

using TfPublisher = tf::srv::TfPublish;

TfPublishSkill::TfPublishSkill(const std::string& name,
                  const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params)
      : BT::RosServiceNode<TfPublisher>(name, conf, params)
    {
        //auto param_ns = getInput<std::string>("param_ns");
        //ns_ = "/bt_executer/" + param_ns.value();
    }

bool TfPublishSkill::setRequest(Request::SharedPtr& request) 
{

        // Create TFs based on some logic (this is just an example)
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        //Creo TF per approccio vite
        geometry_msgs::msg::TransformStamped approach_transform;
        approach_transform.header.stamp = this->now();
        approach_transform.header.frame_id = "screw_frame";
        approach_transform.child_frame_id = "screw_approach_frame";
        approach_transform.transform.translation.x = 0.0;
        approach_transform.transform.translation.y = 0.0;
        approach_transform.transform.translation.z = -0.3; //ricordarsi di cambiare: valore che serve oer non andare in collisione su Rviz
        approach_transform.transform.rotation.x = 0.0;
        approach_transform.transform.rotation.y = 0.0;
        approach_transform.transform.rotation.z = 0.0;
        approach_transform.transform.rotation.w = 1.0;
        //aggiungo TF alla lista
        transforms.push_back(approach_transform);
        
        //Creo TF per movimento a destra
        geometry_msgs::msg::TransformStamped right_transform;
        right_transform.header.stamp = this->now();
        right_transform.header.frame_id = "screw_frame";
        right_transform.child_frame_id = "right_frame";
        right_transform.transform.translation.x = 0.4;
        right_transform.transform.translation.y = 0.0;
        right_transform.transform.translation.z = 0.0; //ricordarsi di cambiare: valore che serve oer non andare in collisione su Rviz
        right_transform.transform.rotation.x = 0.0;
        right_transform.transform.rotation.y = 0.0;
        right_transform.transform.rotation.z = 0.0;
        right_transform.transform.rotation.w = 1.0;
        //aggiungo TF alla lista
        transforms.push_back(right_transform);
        
        //Creo TF per movimento a sinistra
        geometry_msgs::msg::TransformStamped left_transform;
        left_transform.header.stamp = this->now();
        left_transform.header.frame_id = "screw_frame";
        left_transform.child_frame_id = "left_frame";
        left_transform.transform.translation.x = -0.4;
        left_transform.transform.translation.y = 0.0;
        left_transform.transform.translation.z = 0.0; //ricordarsi di cambiare: valore che serve oer non andare in collisione su Rviz
        left_transform.transform.rotation.x = 0.0;
        left_transform.transform.rotation.y = 0.0;
        left_transform.transform.rotation.z = 0.0;
        left_transform.transform.rotation.w = 1.0;
        //aggiungo TF alla lista
        transforms.push_back(left_transform);
        
        //Creo TF per movimento in su 
        geometry_msgs::msg::TransformStamped up_transform;
        up_transform.header.stamp = this->now();
        up_transform.header.frame_id = "screw_frame";
        up_transform.child_frame_id = "up_frame";
        up_transform.transform.translation.x = 0.0;
        up_transform.transform.translation.y = -0.4;
        up_transform.transform.translation.z = 0.0; //ricordarsi di cambiare: valore che serve oer non andare in collisione su Rviz
        up_transform.transform.rotation.x = 0.0;
        up_transform.transform.rotation.y = 0.0;
        up_transform.transform.rotation.z = 0.0;
        up_transform.transform.rotation.w = 1.0;
        //aggiungo TF alla lista
        transforms.push_back(up_transform);
        
        //Creo TF per movimento in giu
        geometry_msgs::msg::TransformStamped down_transform;
        down_transform.header.stamp = this->now();
        down_transform.header.frame_id = "screw_frame";
        down_transform.child_frame_id = "down_frame";
        down_transform.transform.translation.x = 0.0;
        down_transform.transform.translation.y = 0.4;
        down_transform.transform.translation.z = 0.0; //ricordarsi di cambiare: valore che serve oer non andare in collisione su Rviz
        down_transform.transform.rotation.x = 0.0;
        down_transform.transform.rotation.y = 0.0;
        down_transform.transform.rotation.z = 0.0;
        down_transform.transform.rotation.w = 1.0;
        //aggiungo TF alla lista
        transforms.push_back(down_transform);
        
        // Set request
        request->tf_list = transforms;
        return true;
}

BT::NodeStatus TfPublishSkill::onResponseReceived(const Response::SharedPtr& response) 
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
            RCLCPP_ERROR_STREAM(node_.lock()->get_logger(), "Failed to publish TFs.");
            return BT::NodeStatus::FAILURE;
        }
}

BT::NodeStatus TfPublishSkill::onFailure(BT::ServiceNodeErrorCode error) 
{
        RCLCPP_ERROR(node_.lock()->get_logger(), "%s: onFailure with error: %s",
                     name().c_str(), toStr(error));
        return BT::NodeStatus::FAILURE;
}

// Plugin registration.
// The class TfPublishSkill will self register with name  "TfPublishSkill".
CreateRosNodePlugin(TfPublishSkill, "TfPublishSkill");

