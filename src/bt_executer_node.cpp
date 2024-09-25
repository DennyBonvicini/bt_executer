#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

#include <cnr_param/cnr_param.h>

void searchForPlugins(std::vector<std::string>& plugin_paths)
{
  std::string ament_prefix_path = std::getenv("AMENT_PREFIX_PATH");
  std::istringstream iss(ament_prefix_path);

  std::string path;
  while(std::getline(iss, path, ':'))
  {
    std::string fs_path = std::filesystem::path(path);
    if(!std::filesystem::exists(fs_path) || !std::filesystem::is_directory(fs_path))
      continue;

    for (const auto& entry : std::filesystem::recursive_directory_iterator(fs_path))
    {
      if (entry.is_regular_file())
      {
        std::string filename = entry.path().filename().string();
        if (filename.find("plugin.so") != std::string::npos)
          plugin_paths.push_back(entry.path().string());
      }
    }
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("bt_executer_node", options);

  std::vector<std::string> available_plugins;
  searchForPlugins(available_plugins);

  // Display found plugin paths
  std::string plugins_found = "\nPlugins found:";
  for (const auto& plugin_path : available_plugins)
    plugins_found = plugins_found+"\n -"+plugin_path;
  RCLCPP_INFO_STREAM(node->get_logger(),plugins_found);

  std::string ns= "/bt_executer";

  std::string w, bt_package;
  if(cnr::param::has(ns+"/bt_package",w))
  {
    if(not cnr::param::get(ns+"/bt_package",bt_package,w))
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),"cannot load "<<ns+"/bt_package");
      RCLCPP_ERROR_STREAM(node->get_logger(),"what:\n"<<w);

      return 1;
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),ns+"/bt_package is not an available parameter");
    RCLCPP_ERROR_STREAM(node->get_logger(),"what:\n"<<w);

    return 1;
  }

  std::string bt_name;
  if(cnr::param::has(ns+"/bt_name",w))
  {
    if(not cnr::param::get(ns+"/bt_name",bt_name,w))
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),"cannot load "<<ns+"/bt_name");
      RCLCPP_ERROR_STREAM(node->get_logger(),"what:\n"<<w);

      return 1;
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),ns+"/bt_name is not an available parameter");
    RCLCPP_ERROR_STREAM(node->get_logger(),"what:\n"<<w);

    return 1;
  }

  std::vector<std::string> plugins_to_load;
  if(cnr::param::has(ns+"/plugins",w))
  {
    if(not cnr::param::get(ns+"/plugins",plugins_to_load,w))
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),"cannot load "<<ns+"/plugins");
      RCLCPP_ERROR_STREAM(node->get_logger(),"what:\n"<<w);

      return 1;
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),ns+"/plugins is not an available parameter");
    RCLCPP_ERROR_STREAM(node->get_logger(),"what:\n"<<w);

    return 1;
  }

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;
  rclcpp::executors::MultiThreadedExecutor executor;

  std::vector<rclcpp::Node::SharedPtr> node_handles;
  for(const std::string& plugin_name:plugins_to_load)
  {
    std::string path_to_plugin;
    for(const std::string& this_plugin:available_plugins)
    {
      if(this_plugin.find(plugin_name) != std::string::npos)
      {
        path_to_plugin = this_plugin;
        break;
      }
    }

    auto nh = rclcpp::Node::make_shared("bt_executer_node_"+plugin_name, options);
    executor.add_node(nh);
    node_handles.push_back(nh);

    BT::RosNodeParams params;
    params.nh = nh;
    params.server_timeout = std::chrono::milliseconds(1000000);
    params.wait_for_server_timeout = std::chrono::milliseconds(10000);

    RCLCPP_INFO_STREAM(node->get_logger(),"Path to plugin loaded "<<path_to_plugin);
    RegisterRosNode(factory,path_to_plugin,params);
  }

  std::string path_to_bt = ament_index_cpp::get_package_share_directory(bt_package)+"/trees/"+bt_name;
  RCLCPP_INFO_STREAM(node->get_logger(),"Loading bt: "<<path_to_bt);

  factory.registerBehaviorTreeFromFile(path_to_bt);
  RCLCPP_INFO_STREAM(node->get_logger(),"Behavior tree registered! ");

  BT::Tree tree = factory.createTree("MainTree");
  RCLCPP_INFO_STREAM(node->get_logger(),"Factory successfully created!");


  RCLCPP_INFO_STREAM(node->get_logger(),"Starting the execution of the behavior tree..");

  bool finish = false;
  while (!finish && rclcpp::ok())
  {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}
