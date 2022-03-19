#include "bb_equipment_tf_publisher/bb_equipment_tf_publisher.h"

#include <tf2/utils.h>

BBEquipmentTFPublisher::BBEquipmentTFPublisher(ros::NodeHandle& nh): nh_(nh), exit_flag_(false),
  equipment_tfs_retrieved_(false), map_odom_base_link_params_retrieved_(false),
  publish_map_odom_base_link_tfs_(false), publish_map_odom_tf_(false), publish_odom_base_link_tf_(false)

{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "bb_tf_equipment_publisher: Constructor.");
  tf_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster());

  map_odom_base_link_tf_srv_ = nh_.advertiseService("map_odom_base_link_tf_control", &BBEquipmentTFPublisher::mapOdomBaseLinkTfControl, this);
  static_tf_update_srv_ = nh_.advertiseService("static_tf_updates", &BBEquipmentTFPublisher::staticTfUpdate, this);
}

BBEquipmentTFPublisher::~BBEquipmentTFPublisher()
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "bb_tf_equipment_publisher: Destructor.");
  map_odom_base_link_tf_srv_.shutdown();
}

void BBEquipmentTFPublisher::setExitFlag(bool value)
{
  exit_flag_ = value;
}

bool BBEquipmentTFPublisher::mapOdomBaseLinkTfControl(bb_equipment_tf_publisher::MapOdomBaseLinkTfsRequest& req,
                                                      bb_equipment_tf_publisher::MapOdomBaseLinkTfsResponse& resp)
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "mapOdomBaseLinkTfControl() called.");
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "publish_map_odom_tf = " << (int) req.publish_map_odom_tf << ", publish_odom_base_link_tf = " << req.publish_odom_base_link_tf);

  if (req.publish_map_odom_tf != publish_map_odom_tf_)
  {
    publish_map_odom_tf_ = req.publish_map_odom_tf;
    publish_map_odom_base_link_tfs_ = (publish_map_odom_tf_ || publish_odom_base_link_tf_);
  }

  if (req.publish_odom_base_link_tf != publish_odom_base_link_tf_)
  {
    publish_odom_base_link_tf_ = req.publish_odom_base_link_tf;
    publish_map_odom_base_link_tfs_ = (publish_map_odom_tf_ || publish_odom_base_link_tf_);
  }

  resp.map_odom_tf_is_published = publish_map_odom_tf_;
  resp.odom_base_link_tf_is_published = publish_odom_base_link_tf_;

  return true;
}

bool BBEquipmentTFPublisher::staticTfUpdate(bb_equipment_tf_publisher::StaticTfUpdate::Request& req,
                                            bb_equipment_tf_publisher::StaticTfUpdate::Response& resp)
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "staticTfUpdate() called.");
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Static transforms to update: " << req.static_transforms.size());

  if (req.frame_ids.empty() || req.static_transforms.empty() ||
      req.frame_ids.size() != req.static_transforms.size())
  {
    ROS_ERROR_STREAM_NAMED("bb_tf_equipment_publisher", "Invalid or empty static transform data has been provided, can not proceed with update operation!");
    return false;
  }

  for (size_t k = 0; k < req.frame_ids.size(); k++)
  {
    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Updating static transform " << req.frame_ids[k] << " to: translation ("
                          << req.static_transforms[k].position.x << "," << req.static_transforms[k].position.y << "," << req.static_transforms[k].position.z << "), rotation = ("
                          << req.static_transforms[k].orientation.x << "," << req.static_transforms[k].orientation.y << "," << req.static_transforms[k].orientation.z << "," << req.static_transforms[k].orientation.w << ")");

    for (std::vector<geometry_msgs::TransformStamped>::iterator it = equipment_tfs_.begin(); it != equipment_tfs_.end(); it++)
    {
      if (it->child_frame_id == req.frame_ids[k])
      {
        it->transform.translation.x = req.static_transforms[k].position.x;
        it->transform.translation.y = req.static_transforms[k].position.y;
        it->transform.translation.z = req.static_transforms[k].position.z;

        it->transform.rotation.x = req.static_transforms[k].orientation.x;
        it->transform.rotation.y = req.static_transforms[k].orientation.y;
        it->transform.rotation.z = req.static_transforms[k].orientation.z;
        it->transform.rotation.w = req.static_transforms[k].orientation.w;

        break;
      }
    }
  }

  return true;
}

void BBEquipmentTFPublisher::retrieveMapOdomBaseLinkConfig()
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Retrieving map/odom/base_link options from ROS parameter server.");
  if (nh_.hasParam("/bb_map_odom_base_link_transforms"))
  {
    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "/bb_map_odom_base_link_transforms parameter found.");
    XmlRpc::XmlRpcValue tf_param_value;
    if (nh_.getParam("/bb_map_odom_base_link_transforms", tf_param_value))
    {
      ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "/bb_equipment_transforms parameter retrieved.");
      if (tf_param_value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Parameter type is XmlRpc::XmlRpcValue::TypeStruct with " << tf_param_value.size() << " entries.");
        map_odom_base_link_params_retrieved_ = true;

        for (XmlRpc::XmlRpcValue::ValueStruct::iterator it = tf_param_value.begin(); it != tf_param_value.end(); it++)
        {
          ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  Struct member: " << it->first << " of type " << it->second.getType());
          if (it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
          {
            ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  Struct entry of size: " << it->second.size());
            BBEquipmentTransform equipment_tf;
            bool tf_def_valid = false;
            bool frame_id_found = false;
            bool child_frame_id_found = false;
            bool tr_found = false;
            bool rot_found = false;

            for (XmlRpc::XmlRpcValue::ValueStruct::iterator it_tf = it->second.begin(); it_tf != it->second.end(); it_tf++)
            {
              if (it_tf->first == "frame_id")
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   frame_id value found: " << it_tf->second);
                equipment_tf.frame_id = (std::string) it_tf->second;
                frame_id_found = true;
              }
              if (it_tf->first == "child_frame_id")
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   child_frame_id value found: " << it_tf->second);
                equipment_tf.child_frame_id = (std::string) it_tf->second;
                child_frame_id_found = true;
              }

              if (it_tf->first == "translation")
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  translation section found, type " << it_tf->second.getType());
                if (it_tf->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                  if (it_tf->second.hasMember("x") &&
                      it_tf->second.hasMember("y") &&
                      it_tf->second.hasMember("z"))
                  {
                    equipment_tf.translation_x = (double) it_tf->second["x"];
                    equipment_tf.translation_y = (double) it_tf->second["y"];
                    equipment_tf.translation_z = (double) it_tf->second["z"];
                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   translation values found: (" << equipment_tf.translation_x << "," << equipment_tf.translation_y << "," << equipment_tf.translation_z << ")");
                    tr_found = true;
                  }
                  else
                  {
                    ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "One or more translation values are missing, TF definition is invalid!");
                  }
                }
              }

              if (it_tf->first == "rotation")
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  rotation section found, type " << it_tf->second.getType());
                if (it_tf->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                  if (it_tf->second.hasMember("r") &&
                      it_tf->second.hasMember("p") &&
                      it_tf->second.hasMember("y"))
                  {
                    equipment_tf.roll = (double) it_tf->second["r"];
                    equipment_tf.pitch = (double) it_tf->second["p"];
                    equipment_tf.yaw = (double) it_tf->second["y"];
                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   rotation values found: (" << equipment_tf.roll << "," << equipment_tf.pitch << "," << equipment_tf.yaw << ")");
                    rot_found = true;
                  }
                  else
                  {
                    ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "One or more rotation values are missing, TF definition is invalid!");
                  }
                }
              }

              ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  validation flags: frame_id_found = " << (int) frame_id_found << ", child_frame_id_found = " << (int) child_frame_id_found <<
                                    ", tr_found = " << (int) tr_found << ", rot_found = " << (int) rot_found);
              if (frame_id_found && child_frame_id_found && tr_found && rot_found)
              {
                tf_def_valid = true;
              }

              if (tf_def_valid)
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Adding new TF definition for: " << equipment_tf.frame_id << " -> " << equipment_tf.child_frame_id);
                if (it->first == "map_odom_transform")
                {
                  map_odom_transform_.header.frame_id = equipment_tf.frame_id;
                  map_odom_transform_.child_frame_id = equipment_tf.child_frame_id;

                  map_odom_transform_.transform.translation.x = equipment_tf.translation_x;
                  map_odom_transform_.transform.translation.y = equipment_tf.translation_y;
                  map_odom_transform_.transform.translation.z = equipment_tf.translation_z;

                  tf2::Quaternion quat;
                  quat.setRPY(equipment_tf.roll, equipment_tf.pitch, equipment_tf.yaw);

                  map_odom_transform_.transform.rotation.x = quat.x();
                  map_odom_transform_.transform.rotation.y = quat.y();
                  map_odom_transform_.transform.rotation.z = quat.z();
                  map_odom_transform_.transform.rotation.w = quat.w();

                  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "map -> odom transform : " << map_odom_transform_);
                }
                else if (it->first == "odom_base_link_transform")
                {
                  odom_base_link_transform_.header.frame_id = equipment_tf.frame_id;
                  odom_base_link_transform_.child_frame_id = equipment_tf.child_frame_id;

                  odom_base_link_transform_.transform.translation.x = equipment_tf.translation_x;
                  odom_base_link_transform_.transform.translation.y = equipment_tf.translation_y;
                  odom_base_link_transform_.transform.translation.z = equipment_tf.translation_z;

                  tf2::Quaternion quat;
                  quat.setRPY(equipment_tf.roll, equipment_tf.pitch, equipment_tf.yaw);

                  odom_base_link_transform_.transform.rotation.x = quat.x();
                  odom_base_link_transform_.transform.rotation.y = quat.y();
                  odom_base_link_transform_.transform.rotation.z = quat.z();
                  odom_base_link_transform_.transform.rotation.w = quat.w();

                  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "odom -> base_link transform : " << odom_base_link_transform_);
                }
              }
            }
          }
          else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean && it->first == "publish_transforms_from_start")
          {
            publish_map_odom_base_link_tfs_ = it->second.operator bool &();
            ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Parameter publish_transforms_from_start set to: " << (int) publish_map_odom_base_link_tfs_);
            if (publish_map_odom_base_link_tfs_)
            {
              publish_map_odom_tf_ = true;
              publish_odom_base_link_tf_ = true;
            }
          }
        }
      }
    }
  }
}

void BBEquipmentTFPublisher::retrieveEquipmentTransformsList()
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Retrieving transform definitions from ROS parameter server.");
  if (nh_.hasParam("/bb_equipment_transforms"))
  {
    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "/bb_equipment_transforms parameter found.");
    XmlRpc::XmlRpcValue tf_param_value;
    if (nh_.getParam("/bb_equipment_transforms", tf_param_value))
    {
      ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "/bb_equipment_transforms parameter retrieved.");
      if (tf_param_value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Parameter type is XmlRpc::XmlRpcValue::TypeStruct as expected with " << tf_param_value.size() << " entries.");
        equipment_tfs_retrieved_ = true;
        equipment_tfs_.clear();

        for (XmlRpc::XmlRpcValue::ValueStruct::iterator it = tf_param_value.begin(); it != tf_param_value.end(); it++)
        {
          ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  Struct member: " << it->first << " of type " << it->second.getType());

          if (it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
          {
            ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  Struct entry of size: " << it->second.size());
            BBEquipmentTransform equipment_tf;
            bool tf_def_valid = false;
            bool frame_id_found = false;
            bool child_frame_id_found = false;
            bool tr_found = false;
            bool rot_found = false;

            for (XmlRpc::XmlRpcValue::ValueStruct::iterator it_tf = it->second.begin(); it_tf != it->second.end(); it_tf++)
            {
              if (it_tf->first == "frame_id")
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   frame_id value found: " << it_tf->second);
                equipment_tf.frame_id = (std::string) it_tf->second;
                frame_id_found = true;
              }
              if (it_tf->first == "child_frame_id")
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   child_frame_id value found: " << it_tf->second);
                equipment_tf.child_frame_id = (std::string) it_tf->second;
                child_frame_id_found = true;
              }

              if (it_tf->first == "translation")
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  translation section found, type " << it_tf->second.getType());
                if (it_tf->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                  if (it_tf->second.hasMember("x") &&
                      it_tf->second.hasMember("y") &&
                      it_tf->second.hasMember("z"))
                  {
                    equipment_tf.translation_x = (double) it_tf->second["x"];
                    equipment_tf.translation_y = (double) it_tf->second["y"];
                    equipment_tf.translation_z = (double) it_tf->second["z"];
                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   translation values found: (" << equipment_tf.translation_x << "," << equipment_tf.translation_y << "," << equipment_tf.translation_z << ")");
                    tr_found = true;
                  }
                  else
                  {
                    ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "One or more translation values are missing, TF definition is invalid!");
                  }
                }
              }

              if (it_tf->first == "rotation")
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  rotation section found, type " << it_tf->second.getType());
                if (it_tf->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                  if (it_tf->second.hasMember("r") &&
                      it_tf->second.hasMember("p") &&
                      it_tf->second.hasMember("y"))
                  {
                    equipment_tf.roll = (double) it_tf->second["r"];
                    equipment_tf.pitch = (double) it_tf->second["p"];
                    equipment_tf.yaw = (double) it_tf->second["y"];
                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   rotation values found: (" << equipment_tf.roll << "," << equipment_tf.pitch << "," << equipment_tf.yaw << ")");
                    rot_found = true;
                  }
                  else
                  {
                    ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "One or more rotation values are missing, TF definition is invalid!");
                  }
                }
              }

              ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  validation flags: frame_id_found = " << (int) frame_id_found << ", child_frame_id_found = " << (int) child_frame_id_found <<
                                    ", tr_found = " << (int) tr_found << ", rot_found = " << (int) rot_found);
              if (frame_id_found && child_frame_id_found && tr_found && rot_found)
              {
                tf_def_valid = true;
              }

              if (tf_def_valid)
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Adding new TF definition for: " << equipment_tf.frame_id << " -> " << equipment_tf.child_frame_id);
                equipment_tf_values_.push_back(equipment_tf);
              }
            }
          }
        }

        if (equipment_tf_values_.size() > 0)
        {
          ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Static transforms to publish: " << equipment_tf_values_.size());
          for (size_t k = 0; k < equipment_tf_values_.size(); k++)
          {
            geometry_msgs::TransformStamped equipment_tf;

            equipment_tf.header.frame_id = equipment_tf_values_[k].frame_id;
            equipment_tf.child_frame_id = equipment_tf_values_[k].child_frame_id;

            equipment_tf.transform.translation.x = equipment_tf_values_[k].translation_x;
            equipment_tf.transform.translation.y = equipment_tf_values_[k].translation_y;
            equipment_tf.transform.translation.z = equipment_tf_values_[k].translation_z;

            tf2::Quaternion quat;
            quat.setRPY(equipment_tf_values_[k].roll, equipment_tf_values_[k].pitch, equipment_tf_values_[k].yaw);

            equipment_tf.transform.rotation.x = quat.x();
            equipment_tf.transform.rotation.y = quat.y();
            equipment_tf.transform.rotation.z = quat.z();
            equipment_tf.transform.rotation.w = quat.w();

            ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Transform " << k << ": " << equipment_tf);

            equipment_tfs_.push_back(equipment_tf);
          }
        }
      }
      else
      {
        ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Parameter type is " << tf_param_value.getType() << ", but expected type " << XmlRpc::XmlRpcValue::TypeStruct << ".");
      }
    }
  }
}

geometry_msgs::Quaternion BBEquipmentTFPublisher::quaternionToGeometryMsg(tf2::Quaternion quat)
{
  geometry_msgs::Quaternion quat_msg;

  //tf2::convert(quat_msg , quat);
  // or
  //tf2::fromMsg(quat_msg, quat_tf);
  // or for the other conversion direction
  quat_msg = tf2::toMsg(quat);

  return quat_msg;
}

visualization_msgs::Marker BBEquipmentTFPublisher::makeBox(visualization_msgs::InteractiveMarker &msg)
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

visualization_msgs::InteractiveMarkerControl& BBEquipmentTFPublisher::makeBoxControl(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void BBEquipmentTFPublisher::make6DofMarker(bool fixed, unsigned int interaction_mode, const tf2::Vector3& position, bool show_6dof)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf2: ::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

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
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    tf2::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  marker_server_->insert(int_marker);
  marker_server_->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    marker_menu_handler_.apply(*marker_server_, int_marker.name);
}

void BBEquipmentTFPublisher::rosLoop()
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Entering rosLoop().");
  ros::Rate tf_pub_rate(1);
  while (ros::ok())
  {
    ROS_DEBUG_STREAM_NAMED("bb_tf_equipment_publisher", "rosLoop() iterating.");
    tf_pub_rate.sleep();

    if (publish_map_odom_base_link_tfs_)
    {
      map_odom_transform_.header.stamp = ros::Time::now();
      tf_broadcaster_->sendTransform(map_odom_transform_);

      odom_base_link_transform_.header.stamp = ros::Time::now();
      tf_broadcaster_->sendTransform(odom_base_link_transform_);
    }

    for (size_t k = 0; k < equipment_tfs_.size(); k++)
    {
      equipment_tfs_[k].header.seq += 1;
      equipment_tfs_[k].header.stamp = ros::Time::now();
      tf_broadcaster_->sendTransform(equipment_tfs_[k]);
    }
  }
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Exiting rosLoop().");
}