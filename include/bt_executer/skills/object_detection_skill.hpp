#include <bt_executer/utils.hpp>
#include <yolov8/srv/object_detection.hpp>

using ObjectDetection = yolov8::srv::ObjectDetection;

class Detection : public BT::RosServiceNode<ObjectDetection>
{
public:
  Detection(const std::string& name,
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

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override;

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

protected:
  std::string ns_;
};

