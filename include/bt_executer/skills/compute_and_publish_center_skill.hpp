#include <bt_executer/utils.hpp>
#include <tf/srv/compute_center.hpp>

using ComputeCenter = tf::srv::ComputeCenter;

class ComputeAndPublishCenter: public BT::RosServiceNode<ComputeCenter>
{
public:
  ComputeAndPublishCenter(const std::string& name,
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
