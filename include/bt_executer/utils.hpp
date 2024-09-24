#ifndef BT_EXECUTER_INCLUDE_BT_EXECUTER_UTILS_GET_PARAM
#define BT_EXECUTER_INCLUDE_BT_EXECUTER_UTILS_GET_PARAM

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include <cnr_param/cnr_param.h>
#include <rclcpp/node.hpp>

namespace bt_executer
{
namespace utils
{

/**
 * @brief Tokenize a string, using a/multiple delimiters
 *
 * @param node The node where the log will be printed
 * @param ns The namespace of the parameter
 * @param param_name The name of the parameter
 * @param param The output parameter
 * @param what The description of the output of the parameter loader
 * @return bool True if the parameter is loaded correctly, false otherwise
 */
template<typename T>
inline bool get_param(rclcpp::Node *node, std::string ns, std::string param_name, T& param, std::string what)
{
  if(cnr::param::has(ns + param_name, what))
  {
    if(not cnr::param::get(ns + param_name, param, what))
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot load " << ns + param_name + " parameter.");
      RCLCPP_ERROR_STREAM(node->get_logger(), "what:" << what);
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), ns + param_name + " parameter not available.");
    RCLCPP_ERROR_STREAM(node->get_logger(), "what: " << what);
    return false;
  }
  return true;
}

} // namespace utils

} // namespace bt_executer

#endif /* BT_EXECUTER_INCLUDE_BT_EXECUTER_UTILS_GET_PARAM */
