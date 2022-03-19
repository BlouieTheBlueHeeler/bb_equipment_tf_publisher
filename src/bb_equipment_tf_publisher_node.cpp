#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/xmlrpc_manager.h>
#include "bb_equipment_tf_publisher/bb_equipment_tf_publisher_node.h"

#include <signal.h>
#include <boost/thread.hpp>

sig_atomic_t g_shutdown_request = 0;

boost::shared_ptr<BBEquipmentTFPublisher> bb_tf_publisher;
boost::shared_ptr<boost::thread> bb_tf_publisher_thread;

void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();

  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN_STREAM_NAMED("bb_equipment_tf_publisher", "Shutdown request received: " << reason);
    g_shutdown_request = 1;
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

void signalHandler(int sig)
{
  ROS_WARN_STREAM_NAMED("bb_equipment_tf_publisher", "Signal received: " << sig);
  g_shutdown_request = 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bb_equipment_tf_publisher", ros::init_options::NoSigintHandler);

  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);

  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  ROS_INFO_STREAM_NAMED("bb_equipment_tf_publisher", "bb_equipment_tf_publisher starting.");

  ros::NodeHandle nh;
  bb_tf_publisher.reset(new BBEquipmentTFPublisher(nh));

  bb_tf_publisher_thread.reset(new boost::thread(boost::bind(&BBEquipmentTFPublisher::rosLoop, bb_tf_publisher.get())));
  bb_tf_publisher->retrieveEquipmentTransformsList();
  bb_tf_publisher->retrieveMapOdomBaseLinkConfig();

  ros::Rate main_rate(5);
  while (ros::ok())
  {
    if (g_shutdown_request == 1)
    {
      bb_tf_publisher->setExitFlag(true);
      ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Waiting for bb_tf_publisher_thread to join()...");
      if (!bb_tf_publisher_thread->try_join_for(boost::chrono::milliseconds(100)))
      {
        ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "bb_tf_publisher_thread has not joined, interrupting it!");
        bb_tf_publisher_thread->interrupt();
      }
      break;
    }
    main_rate.sleep();
  }

  ROS_INFO_STREAM_NAMED("bb_equipment_tf_publisher", "bb_equipment_tf_publisher exiting.");
  bb_tf_publisher.reset();

  return 0;
}
