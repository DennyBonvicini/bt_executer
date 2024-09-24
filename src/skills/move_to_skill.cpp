#include <bt_executer/skills/move_to_skill.hpp>

MoveToSkill::MoveToSkill(const std::string& name,
                         const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params)
  : RosActionNode<trajectory_loader::action::MoveToAction>(name, conf, params)
{
  auto param_ns = getInput<std::string>("param_ns");
  ns_ = "/bt_executer/" + param_ns.value();

  std::string w;
  if(not bt_executer::utils::get_param(node_.lock().get(), ns_, "/world_name", world_, w))
    world_ = "world";

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.lock()->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

bool MoveToSkill::setGoal(RosActionNode::Goal &goal)
{
  int scaling;
  float acceleration_scaling_factor, velocity_scaling_factor;
  bool simulation;
  std::string fjt_action_name, action_name, group_name, ik_service_name, location_name, speed_scaling_topic, pipeline_id, planner_id;

  // Get required parameters
  std::string w;
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/group_name", group_name, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/ik_service_name", ik_service_name, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/simulation", simulation, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/location_name", location_name, w);  //pose to reach
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/fjt_action_name", fjt_action_name, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/speed_scaling_topic", speed_scaling_topic, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/scaling", scaling, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/pipeline_id", pipeline_id, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/planner_id", planner_id, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/acceleration_scaling_factor", acceleration_scaling_factor, w);
  bt_executer::utils::get_param(node_.lock().get(), ns_, "/velocity_scaling_factor", velocity_scaling_factor, w);

  // Get queried location
  int iter = 0;
  geometry_msgs::msg::TransformStamped transform;
  while(not tf_buffer_->canTransform(world_,location_name,tf2::TimePointZero,tf2::durationFromSec(1.0)))
  {
    if(iter>=100)
    {
      RCLCPP_INFO(node_.lock()->get_logger(), "Transform from %s to %s not available",location_name.c_str(), world_.c_str());
      return false;
    }
    iter++;
  }

  try
  {
    transform = tf_buffer_->lookupTransform(world_,location_name,tf2::TimePointZero);
  }catch (const tf2::TransformException & ex)
  {
    RCLCPP_INFO(node_.lock()->get_logger(), "Could not transform %s to %s: %s",location_name.c_str(), world_.c_str(),ex.what());
    return false;
  }

  // Set goal fields
  geometry_msgs::msg::PoseStamped pose;
  tf2::convert(transform,pose);
  RCLCPP_INFO_STREAM(node_.lock()->get_logger(),"Transform:\n"<<geometry_msgs::msg::to_yaml(transform));
  RCLCPP_INFO_STREAM(node_.lock()->get_logger(),"Pose:\n"<<geometry_msgs::msg::to_yaml(pose));

//  pose.header.frame_id="bla";
//  pose.pose.position.x=0.2;

  goal.group_name = group_name;
  goal.ik_service_name = ik_service_name;
  goal.simulation = simulation;
  goal.fjt_action_name = fjt_action_name;
  goal.speed_scaling_topic = speed_scaling_topic;
  goal.scaling = scaling;
  goal.pose = pose;
  goal.pipeline_id = pipeline_id;  // IMPORT FROM PARAM
  goal.planner_id = planner_id; // IMPORT FROM PARAM  (BiTRRT)
  goal.acceleration_scaling_factor = acceleration_scaling_factor; // IMPORT FROM PARAM
  goal.velocity_scaling_factor = velocity_scaling_factor; // IMPORT FROM PARAM



  RCLCPP_INFO_STREAM(node_.lock()->get_logger(),"Goal:\n"<<
                     " group_name = " << goal.group_name <<
                     " ik_service_name = " << goal.ik_service_name <<
                     " simulation = " << goal.simulation <<
                     " fjt_action_name = " << goal.fjt_action_name <<
                     " speed_scaling_topic = " << goal.speed_scaling_topic <<
                     " scaling = " << goal.scaling <<
                     " pipeline_id = " << goal.pipeline_id <<
                     " planner_id = " << goal.planner_id <<
                     " acceleration_scaling_factor = " << goal.acceleration_scaling_factor <<
                     " velocity_scaling_factor = " << goal.velocity_scaling_factor);
  return true;
}

BT::NodeStatus MoveToSkill::onResultReceived(const RosActionNode::WrappedResult &wr)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              wr.result->ok ? "true" : "false");
  if (wr.result->ok)
    return BT::NodeStatus::SUCCESS;
  else
  {
    RCLCPP_ERROR_STREAM(node_.lock()->get_logger(), "Error: " << wr.result->error);
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveToSkill::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToSkill::onFeedback(const std::shared_ptr<const trajectory_loader::action::MoveToAction::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class MoveToSkill will self register with name  "MoveToSkill".
CreateRosNodePlugin(MoveToSkill, "MoveToSkill");
