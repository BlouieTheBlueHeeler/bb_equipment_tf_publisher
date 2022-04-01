#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/xmlrpc_manager.h>
#include "bb_equipment_tf_publisher/bb_equipment_tf_marker_server.h"
#include <tf2/utils.h>

#include <signal.h>
#include <boost/thread.hpp>

BBEquipmentTFMarkerServer::BBEquipmentTFMarkerServer(ros::NodeHandle& nh): initial_frame_list_queried_(false)
{

}

BBEquipmentTFMarkerServer::~BBEquipmentTFMarkerServer()
{

}

bool BBEquipmentTFMarkerServer::setup()
{
  tf_list_query_timer_ = this->nh_.createTimer(ros::Duration(5.0), &BBEquipmentTFMarkerServer::updateTfList, this, false, true);
  marker_server_.reset(new interactive_markers::InteractiveMarkerServer("bb_equipment_tf_publisher", "bb_equipment_tf_publisher", true));

  ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "InteractiveMarkers for transforms created.");
  return true;
}

void BBEquipmentTFMarkerServer::shutdown()
{
  marker_server_.reset();
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "processFeedback() called.");
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }
}


void BBEquipmentTFMarkerServer::updateTfList(const ros::TimerEvent& ev)
{
  ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "updateTfList event timer triggered.");
  ros::ServiceClient tf_list_client = nh_.serviceClient<bb_equipment_tf_publisher::StaticTfs>("static_tf_list");

  bb_equipment_tf_publisher::StaticTfs::Request tf_req;
  bb_equipment_tf_publisher::StaticTfs::Response tf_resp;
  if (tf_list_client.call(tf_req, tf_resp))
  {
    if (tf_resp.frame_ids.size() != tf_resp.static_transforms.size())
    {
      // Bail out if number of entries in frame ID and transform data list does not match
      ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Received static transform list with mismatched numbers of entries in frame ID and transform data lists, can not continue!");
      return;
    }

    ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Received list of static frame IDs from equipment publisher node: " << tf_resp.frame_ids.size() << " frames.");
    if (!initial_frame_list_queried_)
    {
      // marker_menu_handler_.insert("TEST", &processFeedback); // boost::bind(&BBEquipmentTFMarkerServer::processMarkerFeedback, this, _1));

      ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Recording initial list of static transforms.");
      for (size_t k = 0; k < tf_resp.frame_ids.size(); k++)
      {
        ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Adding static transform: " << tf_resp.frame_ids[k]);
        known_frame_ids_.emplace_back(tf_resp.frame_ids[k]);
        known_tfs_.emplace_back(tf_resp.static_transforms[k]);

        make6DofMarker(tf_resp.frame_ids[k], false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
                       tf2::Vector3(tf_resp.static_transforms[k].position.x, tf_resp.static_transforms[k].position.y, tf_resp.static_transforms[k].position.z), true);

        // marker_menu_handler_.apply(*marker_server_, int_marker.name);
      }

      marker_server_->applyChanges();

      initial_frame_list_queried_ = true;
    }
    else
    {
      if (known_frame_ids_.size() != tf_resp.frame_ids.size())
      {
        ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Updating list of known static transforms.");

        std::vector<std::string> frame_ids;
        std::vector<geometry_msgs::Pose> static_tfs;
        std::vector<geometry_msgs::Pose> added_tfs_data, updated_tfs_data;
        std::vector<std::string> frame_ids_to_add, frame_ids_to_update, frame_ids_to_remove;

        // Look for frame IDs that have been added
        for (size_t k = 0; k < tf_resp.frame_ids.size(); k++)
        {
          if (std::find(known_frame_ids_.begin(), known_frame_ids_.end(), tf_resp.frame_ids[k]) == known_frame_ids_.end())
          {
            ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Adding previously unknown frame ID " << tf_resp.frame_ids[k]);
            frame_ids.emplace_back(tf_resp.frame_ids[k]);
            static_tfs.emplace_back(tf_resp.static_transforms[k]);
            frame_ids_to_add.emplace_back(tf_resp.frame_ids[k]);
            added_tfs_data.emplace_back(tf_resp.static_transforms[k]);
          }
        }
        // Then for frame IDs that have been removed, keep known entries otherwise
        for (size_t k = 0; k < known_frame_ids_.size(); k++)
        {
          if (std::find(tf_resp.frame_ids.begin(), tf_resp.frame_ids.end(), known_frame_ids_[k]) == tf_resp.frame_ids.end())
          {
            ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Removing frame ID " << known_frame_ids_[k] << " as it is no longer published by the equipment transform node.");
            frame_ids_to_remove.emplace_back(known_frame_ids_[k]);
          }
          else
          {
            frame_ids.emplace_back(known_frame_ids_[k]);
            static_tfs.emplace_back(known_tfs_[k]);
            // Check if position or orientation have been updated
            if (known_tfs_[k].position.x != tf_resp.static_transforms[k].position.x ||
                known_tfs_[k].position.y != tf_resp.static_transforms[k].position.y ||
                known_tfs_[k].position.z != tf_resp.static_transforms[k].position.z ||
                known_tfs_[k].orientation.x != tf_resp.static_transforms[k].orientation.x ||
                known_tfs_[k].orientation.y != tf_resp.static_transforms[k].orientation.y ||
                known_tfs_[k].orientation.z != tf_resp.static_transforms[k].orientation.z ||
                known_tfs_[k].orientation.w != tf_resp.static_transforms[k].orientation.w)
            {
              frame_ids_to_update.emplace_back(tf_resp.frame_ids[k]);
              updated_tfs_data.emplace_back(tf_resp.static_transforms[k]);
            }
          }
        }

        ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Now updating interactive marker server.");
        updateMarkerServer(frame_ids_to_add, added_tfs_data, frame_ids_to_remove, frame_ids_to_update, updated_tfs_data);

        known_frame_ids_ = frame_ids;
        known_tfs_ = static_tfs;
      }
    }
  }
}

void BBEquipmentTFMarkerServer::updateMarkerServer(const std::vector<std::string>& frames_to_add, const std::vector<geometry_msgs::Pose> &frame_data_to_add, const std::vector<std::string> &frames_to_remove, const std::vector<std::string> &frames_to_update, const std::vector<geometry_msgs::Pose> &frame_data_to_update)
{
  ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "updateMarkerServer() called: " << frames_to_add.size() << " frames to add, " << frame_data_to_add.size() << " frames to remove.");
  for (size_t k = 0; k < frames_to_remove.size(); k++)
  {
    ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Removing marker for frame ID " << frames_to_remove[k] << " from interactive marker server.");
    marker_server_->erase(frames_to_remove[k]);
  }

  for (size_t k = 0; k < frames_to_add.size(); k++)
  {
    ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Adding new marker for frame ID " << frames_to_add[k] << " to interactive marker server.");
    make6DofMarker(frames_to_add[k], false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
                   tf2::Vector3(frame_data_to_add[k].position.x, frame_data_to_add[k].position.y, frame_data_to_add[k].position.z), true);
  }

  for (size_t k = 0; k < frames_to_update.size(); k++)
  {
    ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Updating frame ID " << frames_to_update[k] << ", transform data has changed.");
    marker_server_->setPose(frames_to_update[k], frame_data_to_update[k]);
  }

  marker_server_->applyChanges();
}

geometry_msgs::Quaternion BBEquipmentTFMarkerServer::quaternionToGeometryMsg(tf2::Quaternion quat)
{
  geometry_msgs::Quaternion quat_msg;

  //tf2::convert(quat_msg , quat);
  // or
  //tf2::fromMsg(quat_msg, quat_tf);
  // or for the other conversion direction
  quat_msg = tf2::toMsg(quat);

  return quat_msg;
}

visualization_msgs::Marker BBEquipmentTFMarkerServer::makeBox(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::InteractiveMarkerControl& BBEquipmentTFMarkerServer::makeBoxControl(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void BBEquipmentTFMarkerServer::make6DofMarker(const std::string& marker_name, bool fixed, unsigned int interaction_mode, const tf2::Vector3& position, bool show_6dof)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();

  int_marker.scale = 1;

  int_marker.name = marker_name;
  int_marker.description = marker_name + " 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  visualization_msgs::InteractiveMarkerControl control;

  if (fixed)
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
    std::string mode_text;
    if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
    if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
    if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
    int_marker.name = marker_name + "_" + mode_text;
    int_marker.description = marker_name + " " + std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = this->quaternionToGeometryMsg(orien);

    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = this->quaternionToGeometryMsg(orien);

    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    control.orientation = this->quaternionToGeometryMsg(orien);

    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  marker_server_->insert(int_marker);
  marker_server_->setCallback(int_marker.name, &processFeedback); // boost::bind(&BBEquipmentTFMarkerServer::processMarkerFeedback, this, _1));
  // if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
    ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "Adding menu handler to marker for frame ID " << marker_name);
    marker_menu_handler_.apply(*marker_server_, int_marker.name);
  }
}

void BBEquipmentTFMarkerServer::processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_INFO_STREAM_NAMED("bb_tf_marker_server", "processMarkerFeedback() called.");
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if (feedback->mouse_point_valid)
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type)
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM_NAMED("bb_tf_marker_server", s.str() << ": button click" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM_NAMED("bb_tf_marker_server", s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM_NAMED("bb_tf_marker_server", s.str() << ": pose changed"
                            << "\nposition = "
                            << feedback->pose.position.x
                            << ", " << feedback->pose.position.y
                            << ", " << feedback->pose.position.z
                            << "\norientation = "
                            << feedback->pose.orientation.w
                            << ", " << feedback->pose.orientation.x
                            << ", " << feedback->pose.orientation.y
                            << ", " << feedback->pose.orientation.z
                            << "\nframe: " << feedback->header.frame_id
                            << " time: " << feedback->header.stamp.sec << "sec, "
                            << feedback->header.stamp.nsec << " nsec");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM_NAMED("bb_tf_marker_server", s.str() << ": mouse down" << mouse_point_ss.str() << ".");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM_NAMED("bb_tf_marker_server", s.str() << ": mouse up" << mouse_point_ss.str() << ".");
      break;
  }

  marker_server_->applyChanges();
}


sig_atomic_t g_shutdown_request = 0;

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

std::shared_ptr<BBEquipmentTFMarkerServer> bb_tf_marker_server;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bb_equipment_tf_publisher", ros::init_options::NoSigintHandler);

  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);

  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  ros::NodeHandle nh;

  bb_tf_marker_server.reset(new BBEquipmentTFMarkerServer(nh));
  if (bb_tf_marker_server->setup())
  {
    ros::Rate main_rate(10);
    while (ros::ok())
    {
      if (g_shutdown_request == 1)
      {
        bb_tf_marker_server->shutdown();
        bb_tf_marker_server.reset();
        break;
      }
      ros::spinOnce();
    }
  }

  ros::shutdown();
  ros::waitForShutdown();

  return 0;
}
