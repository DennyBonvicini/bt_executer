#include <bt_executer/utils.hpp>
//#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf/srv/tf_publish.hpp>

using TfPublisher = tf::srv::TfPublish;

class TfPublishSkill: public BT::RosServiceNode<TfPublisher>
{
public:
  TfPublishSkill(const std::string& name,
                      const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
          {
            BT::InputPort<std::string>("param_ns")
          }
          );
  }

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
  std::string ns_;
};
