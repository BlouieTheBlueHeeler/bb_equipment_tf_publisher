#include "bb_equipment_tf_publisher/bb_equipment_tf_publisher.h"

#include <boost/algorithm/string.hpp>

BBEquipmentTFPublisher::BBEquipmentTFPublisher(ros::NodeHandle& nh): nh_(nh), exit_flag_(false),
  equipment_tfs_retrieved_(false), map_odom_base_link_params_retrieved_(false),
  publish_equipment_tfs_(false),
  default_parking_position_set_(false),
  publish_map_odom_base_link_tfs_(false), publish_map_odom_tf_(false), publish_odom_base_link_tf_(false),
  publish_map_odom_base_link_without_prefix_(false), publish_world_tf_(false), publish_world_tf_without_prefix_(false)
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "bb_tf_equipment_publisher: Constructor.");
  static_tf_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster());
  tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());

  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "tf_broadcaster_ instantiated.");
}

BBEquipmentTFPublisher::~BBEquipmentTFPublisher()
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "bb_tf_equipment_publisher: Destructor.");
}

bool BBEquipmentTFPublisher::setup()
{
  char host_name[HOST_NAME_MAX];
  int status = gethostname(host_name, HOST_NAME_MAX);
  if (status < 0)
  {
    ROS_ERROR_STREAM_NAMED("bb_tf_equipment_publisher", "Failed to retrieve this node's host name!");
    return false;
  }
  host_name[HOST_NAME_MAX - 1] = '\0';
  node_name_ = host_name;
  std::replace(node_name_.begin(), node_name_.end(), '-', '_');

  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Using node name: " << node_name_);

  std::string tf_prefix_param_uri("/" + node_name_ + "/transform_prefix");
  if (nh_.hasParam(tf_prefix_param_uri))
  {
    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "\"" + tf_prefix_param_uri + "\" parameter found.");
    if (nh_.getParam(tf_prefix_param_uri, tf_prefix_))
    {
      ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Transform prefix: " << tf_prefix_);
    }
  }
  else
  {
    ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "No transform prefix parameter is set. Will potentially publish conflicting TF data!");
  }

  if (!retrieveMapOdomBaseLinkConfig())
  {
    ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "Failed to retrieve settings for map->odom->base_link transform chain, will not publish any of these transforms.");
    publish_map_odom_base_link_tfs_ = false;
    publish_map_odom_tf_ = false;
    publish_odom_base_link_tf_ = false;
  }

  if (!retrieveEquipmentTransformsList())
  {
    ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "No list of transforms to publish specified, will only handle map->odom->base_link chain.");
  }

  map_odom_base_link_tf_srv_.reset(new ros::ServiceServer(nh_.advertiseService("/" + node_name_ + "/map_odom_base_link_tf_control", &BBEquipmentTFPublisher::mapOdomBaseLinkTfControl, this)));
  static_tf_update_srv_.reset(new ros::ServiceServer(nh_.advertiseService("/" + node_name_ + "/static_tf_updates", &BBEquipmentTFPublisher::staticTfUpdate, this)));
  static_tf_list_srv_.reset(new ros::ServiceServer(nh_.advertiseService("/" + node_name_ + "/static_tf_list", &BBEquipmentTFPublisher::staticTfList, this)));
  slam_tf_update_srv_.reset(new ros::ServiceServer(nh_.advertiseService("/" + node_name_ + "/slam_tf_update", &BBEquipmentTFPublisher::slamTfUpdate, this)));

  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "ROS services instantiated.");

  return true;
}

void BBEquipmentTFPublisher::shutdown()
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "shutdown() called.");
  map_odom_base_link_tf_srv_->shutdown();
  static_tf_update_srv_->shutdown();
  static_tf_list_srv_->shutdown();
  slam_tf_update_srv_->shutdown();

  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "ROS services de-instantiated");
}

void BBEquipmentTFPublisher::setExitFlag(bool value)
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "exit_flag_ set to: " << static_cast<int>(value));
  exit_flag_ = value;
}

bool BBEquipmentTFPublisher::mapOdomBaseLinkTfControl(bb_equipment_tf_msgs::MapOdomBaseLinkTfsRequest &req,
                                                      bb_equipment_tf_msgs::MapOdomBaseLinkTfsResponse &resp)
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "mapOdomBaseLinkTfControl() called.");
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "publish_map_odom_tf = " << (int) req.publish_map_odom_tf
                        << ", publish_odom_base_link_tf = " << (int) req.publish_odom_base_link_tf
                        << ", publish_map_odom_base_link_without_prefix = " << (int) req.publish_map_odom_base_link_without_prefix);

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

  if (req.publish_map_odom_base_link_without_prefix != publish_map_odom_base_link_without_prefix_)
  {
    publish_map_odom_base_link_without_prefix_ = req.publish_map_odom_base_link_without_prefix;
  }

  if (req.publish_world_tf != publish_world_tf_)
  {
    publish_world_tf_ = req.publish_world_tf;
  }

  if (req.publish_world_tf_without_prefix != publish_world_tf_without_prefix_)
  {
    publish_world_tf_without_prefix_ = req.publish_map_odom_base_link_without_prefix;
  }

  if (publish_map_odom_base_link_without_prefix_)
  {
    map_odom_transform_.header.frame_id = map_odom_tf_frame_id_;
    map_odom_transform_.child_frame_id = map_odom_tf_child_frame_id_;
    odom_base_link_transform_.header.frame_id = odom_base_link_tf_frame_id_;
    odom_base_link_transform_.child_frame_id = odom_base_link_tf_child_frame_id_;
  }
  else
  {
    map_odom_transform_.header.frame_id = tf_prefix_ + map_odom_tf_frame_id_;
    map_odom_transform_.child_frame_id = tf_prefix_ + map_odom_tf_child_frame_id_;
    odom_base_link_transform_.header.frame_id = tf_prefix_ + odom_base_link_tf_frame_id_;
    odom_base_link_transform_.child_frame_id = tf_prefix_ + odom_base_link_tf_child_frame_id_;
  }

  if (publish_world_tf_without_prefix_)
  {
    world_transform_.child_frame_id = world_tf_child_frame_id_;
  }
  else
  {
    world_transform_.child_frame_id = tf_prefix_ + world_tf_child_frame_id_;
  }

  /*std::string map_odom_frame_id = map_odom_transform_.header.frame_id;
  std::string map_odom_child_frame_id = map_odom_transform_.child_frame_id;
  std::string odom_base_link_frame_id = odom_base_link_transform_.header.frame_id;
  std::string odom_base_link_child_frame_id = odom_base_link_transform_.child_frame_id;

  map_odom_transform_.header.frame_id = map_odom_frame_id;
  map_odom_transform_.child_frame_id = map_odom_child_frame_id;
  odom_base_link_transform_.header.frame_id = odom_base_link_frame_id;
  odom_base_link_transform_.child_frame_id = odom_base_link_child_frame_id;*/

  resp.map_odom_tf_is_published = publish_map_odom_tf_;
  resp.odom_base_link_tf_is_published = publish_odom_base_link_tf_;

  return true;
}

bool BBEquipmentTFPublisher::staticTfList(bb_equipment_tf_msgs::StaticTfs::Request &req, bb_equipment_tf_msgs::StaticTfs::Response &resp)
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "staticTfList() called.");
  if (equipment_tfs_.size() == 0)
  {
    ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "No static transforms have been provided, will return an empty TF list! Continuing anyway.");
  }

  for (size_t k = 0; k < equipment_tf_values_.size(); k++)
  {
    resp.frame_ids.emplace_back(equipment_tf_values_[k].frame_id);
    resp.child_frame_ids.emplace_back(equipment_tf_values_[k].child_frame_id);

    geometry_msgs::Pose tf_pose;
    tf_pose.position.x = equipment_tf_values_[k].translation_x;
    tf_pose.position.y = equipment_tf_values_[k].translation_y;
    tf_pose.position.z = equipment_tf_values_[k].translation_z;

    tf2::Quaternion quat;
    quat.setRPY(equipment_tf_values_[k].roll, equipment_tf_values_[k].pitch, equipment_tf_values_[k].yaw);
    tf_pose.orientation.x = quat.x();
    tf_pose.orientation.y = quat.y();
    tf_pose.orientation.z = quat.z();
    tf_pose.orientation.w = quat.w();

    resp.static_transforms.emplace_back(tf_pose);
  }

  return true;
}

bool BBEquipmentTFPublisher::staticTfUpdate(bb_equipment_tf_msgs::StaticTfUpdate::Request &req,
                                            bb_equipment_tf_msgs::StaticTfUpdate::Response &resp)
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "staticTfUpdate() called.");
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Static transforms to update: " << req.static_transforms.size());

  if (req.frame_ids.empty() || req.static_transforms.empty() ||
      req.frame_ids.size() != req.static_transforms.size())
  {
    ROS_ERROR_STREAM_NAMED("bb_tf_equipment_publisher", "Invalid or empty static transform data has been provided, can not proceed with update operation!");

    resp.success = false;
    resp.status_message = "Invalid or empty static transform data has been provided, can not proceed with update operation!";

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

  resp.success = true;
  resp.status_message = "Static transform for " + std::to_string(req.frame_ids.size()) + " updated successfully.";

  return true;
}

bool BBEquipmentTFPublisher::slamTfUpdate(carecules_slam_msgs::TFPublishControl::Request& req,
                                          carecules_slam_msgs::TFPublishControl::Response& resp)
{

  return true;
}

bool BBEquipmentTFPublisher::slamInstanceControl(carecules_slam_msgs::SlamInstanceControl::Request& req,
                                                 carecules_slam_msgs::SlamInstanceControl::Response& resp)
{
  if (req.slam_instance_id.empty())
  {
    ROS_ERROR_STREAM_NAMED("bb_tf_equipment_publisher", "Empty SLAM algorithm instance ID provided, can not process control request!");
    resp.success = false;
    resp.status_message = "Empty SLAM algorithm instance ID provided, can not process control request!";

    return false;
  }

  slam_instances_active_[req.slam_instance_id] = req.online;
  slam_instances_last_pose_estimates_[req.slam_instance_id] = req.last_pose_estimate;

  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "SLAM algorithm instance '" << req.slam_instance_id << " switched to: " << (req.online ? "online" : "offline"));

  resp.success = true;
  resp.status_message = "Instance control request for SLAM algorithm " + req.slam_instance_id + " processed succesfully.";

  return true;
}

bool BBEquipmentTFPublisher::parkingPositionControl(bb_equipment_tf_msgs::ParkingPosition::Request &req,
                                                    bb_equipment_tf_msgs::ParkingPosition::Response &resp)
{
  if (req.set_parking_position)
  {
    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Setting new default parking position to: " << req.new_parking_position);
    default_parking_position_ = req.new_parking_position;
  }

  resp.current_parking_position = default_parking_position_;

  return true;
}

bool BBEquipmentTFPublisher::retrieveMapOdomBaseLinkConfig()
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Retrieving map/odom/base_link options from ROS parameter server.");

  std::string map_odom_param_uri("/" + node_name_ + "/bb_map_odom_base_link_transforms");
  if (nh_.hasParam(map_odom_param_uri))
  {
    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "\"" + map_odom_param_uri + "\" parameter found.");
    XmlRpc::XmlRpcValue tf_param_value;
    if (nh_.getParam(map_odom_param_uri, tf_param_value))
    {
      ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "\"" + map_odom_param_uri + "\" parameter retrieved.");
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
                if (it->first == "world_transform")
                {
                  // This should always be 'world'?
                  world_tf_frame_id_ = "world"; // equipment_tf.frame_id;
                  world_tf_child_frame_id_ = equipment_tf.child_frame_id;

                  world_transform_.header.frame_id = world_tf_frame_id_;
                  if (publish_world_tf_without_prefix_)
                  {
                    world_transform_.child_frame_id = world_tf_child_frame_id_;
                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Using non-prefix version for world child frame: " <<world_transform_.child_frame_id);
                  }
                  else
                  {
                    world_transform_.child_frame_id = tf_prefix_ + world_tf_child_frame_id_;
                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Using prefix version for world child frame: " << world_transform_.child_frame_id);
                  }

                  world_transform_.transform.translation.x = equipment_tf.translation_x;
                  world_transform_.transform.translation.y = equipment_tf.translation_y;
                  world_transform_.transform.translation.z = equipment_tf.translation_z;

                  tf2::Quaternion quat;
                  quat.setRPY(equipment_tf.roll, equipment_tf.pitch, equipment_tf.yaw);

                  world_transform_.transform.rotation.x = quat.x();
                  world_transform_.transform.rotation.y = quat.y();
                  world_transform_.transform.rotation.z = quat.z();
                  world_transform_.transform.rotation.w = quat.w();

                  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "world -> map transform frame IDs: " << world_transform_.header.frame_id << " -> " << world_transform_.child_frame_id);
                  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "world transform: " << world_transform_);
                }
                else if (it->first == "map_odom_transform")
                {
                  map_odom_tf_frame_id_ = equipment_tf.frame_id;
                  map_odom_tf_child_frame_id_ = equipment_tf.child_frame_id;

                  if (publish_map_odom_base_link_without_prefix_)
                  {
                    map_odom_transform_.header.frame_id = map_odom_tf_frame_id_;
                    map_odom_transform_.child_frame_id = map_odom_tf_child_frame_id_;
                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Publishing map -> odom transforms WITHOUT prefix as: " << map_odom_transform_.header.frame_id << " -> " << map_odom_transform_.child_frame_id);
                  }
                  else
                  {
                    map_odom_transform_.header.frame_id = tf_prefix_ + map_odom_tf_frame_id_;
                    map_odom_transform_.child_frame_id = tf_prefix_ + map_odom_tf_child_frame_id_;
                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Publishing map -> odom transforms WITH    prefix \"" << tf_prefix_ << "\"  as: " << map_odom_transform_.header.frame_id << " -> " << map_odom_transform_.child_frame_id);
                  }

                  map_odom_transform_.transform.translation.x = equipment_tf.translation_x;
                  map_odom_transform_.transform.translation.y = equipment_tf.translation_y;
                  map_odom_transform_.transform.translation.z = equipment_tf.translation_z;

                  tf2::Quaternion quat;
                  quat.setRPY(equipment_tf.roll, equipment_tf.pitch, equipment_tf.yaw);

                  map_odom_transform_.transform.rotation.x = quat.x();
                  map_odom_transform_.transform.rotation.y = quat.y();
                  map_odom_transform_.transform.rotation.z = quat.z();
                  map_odom_transform_.transform.rotation.w = quat.w();

                  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "map -> odom transform: " << map_odom_transform_);
                }
                else if (it->first == "odom_base_link_transform")
                {
                  odom_base_link_tf_frame_id_ = equipment_tf.frame_id;
                  odom_base_link_tf_child_frame_id_ = equipment_tf.child_frame_id;

                  if (publish_map_odom_base_link_without_prefix_)
                  {
                    odom_base_link_transform_.header.frame_id = odom_base_link_tf_frame_id_;
                    odom_base_link_transform_.child_frame_id = odom_base_link_tf_child_frame_id_;

                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Publishing odom -> base_link transforms WITHOUT prefix as: " << odom_base_link_transform_.header.frame_id << " -> " << odom_base_link_transform_.child_frame_id);
                  }
                  else
                  {
                    odom_base_link_transform_.header.frame_id = tf_prefix_ + odom_base_link_tf_frame_id_;
                    odom_base_link_transform_.child_frame_id = tf_prefix_ + odom_base_link_tf_child_frame_id_;

                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Publishing odom -> base_link transforms WITH   prefix \"" << tf_prefix_ << "\" as: " << odom_base_link_transform_.header.frame_id << " -> " << odom_base_link_transform_.child_frame_id);
                  }

                  odom_base_link_transform_.transform.translation.x = equipment_tf.translation_x;
                  odom_base_link_transform_.transform.translation.y = equipment_tf.translation_y;
                  odom_base_link_transform_.transform.translation.z = equipment_tf.translation_z;

                  tf2::Quaternion quat;
                  quat.setRPY(equipment_tf.roll, equipment_tf.pitch, equipment_tf.yaw);

                  odom_base_link_transform_.transform.rotation.x = quat.x();
                  odom_base_link_transform_.transform.rotation.y = quat.y();
                  odom_base_link_transform_.transform.rotation.z = quat.z();
                  odom_base_link_transform_.transform.rotation.w = quat.w();

                  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "odom -> base_link transform: " << odom_base_link_transform_);
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
          else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean && it->first == "publish_map_odom_base_link_without_prefix")
          {
            publish_map_odom_base_link_without_prefix_ = it->second.operator bool &();
            ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Parameter publish_map_odom_base_link_without_prefix set to: " << (int) publish_map_odom_base_link_without_prefix_);
          }
          else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean && it->first == "publish_world_tf")
          {
            publish_world_tf_ = it->second.operator bool &();
            ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Parameter publish_world_tf set to: " << (int) publish_world_tf_);
          }
          else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean && it->first == "publish_world_tf_without_prefix")
          {
            publish_world_tf_without_prefix_ = it->second.operator bool &();
            ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Parameter publish_world_tf_without_prefix set to: " << (int) publish_world_tf_without_prefix_);
          }
        }
      }
    }
    else
    {
      ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Could not retrieve \"" + map_odom_param_uri + "\" parameter, can not read settings for map->odom->base_link transform chain!");
      return false;
    }
  }
  else
  {
    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "\"" + map_odom_param_uri + "\" parameter NOT found, can not read settings for map->odom->base_link transform chain!");
    return false;
  }

  return true;
}

bool BBEquipmentTFPublisher::retrieveEquipmentTransformsList()
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Retrieving transform definitions from ROS parameter server.");

  std::string transforms_param_uri("/" + node_name_ + "/bb_equipment_transforms");
  if (nh_.hasParam(transforms_param_uri))
  {
    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "\"" + transforms_param_uri + "\" parameter found.");
    XmlRpc::XmlRpcValue tf_param_value;
    if (nh_.getParam(transforms_param_uri, tf_param_value))
    {
      ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", transforms_param_uri + " parameter retrieved.");
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

            // Equipment transforms expected to always use the given TF prefix
            // Except when this node publishes the "world" frame without TF prefix
            // Then all TFs who are attached to the "world" need to refer to it as such without TF prefix
            ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", " * Static TF " << k << ": " << equipment_tf_values_[k].frame_id << " -> " << equipment_tf_values_[k].child_frame_id);
            ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   publish_world_tf_without_prefix_ = " << (int) publish_world_tf_without_prefix_ << "; equipment_tf_values_[k].frame_id == \"world\" == " << (int) (equipment_tf_values_[k].frame_id == "world"));
            if (publish_world_tf_without_prefix_ && equipment_tf_values_[k].frame_id == "world")
            {
              equipment_tf.header.frame_id = equipment_tf_values_[k].frame_id;
            }
            else
            {
              equipment_tf.header.frame_id = tf_prefix_ + equipment_tf_values_[k].frame_id;
            }
            equipment_tf.child_frame_id = tf_prefix_ +equipment_tf_values_[k].child_frame_id;

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
        ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "Parameter type is " << tf_param_value.getType() << ", but expected type " << XmlRpc::XmlRpcValue::TypeStruct << ".");
        return false;
      }
    }
    else
    {
      ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "Failed to retrieve \"" + transforms_param_uri + "\" parameter NOT found, can not read list of transform definitions to publish!");
      return false;
    }
  }
  else
  {
    ROS_WARN_STREAM_NAMED("bb_tf_equipment_publisher", "\"" + transforms_param_uri + "\" parameter NOT found, can not read list of transform definitions to publish!");
    return false;
  }

  std::string parking_position_param_uri("/" + node_name_ + "/parking_position");
  if (nh_.hasParam(parking_position_param_uri))
  {
    XmlRpc::XmlRpcValue parking_pos_param_value;
    if (nh_.getParam(parking_position_param_uri, parking_pos_param_value))
    {
      ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", parking_position_param_uri + " parameter retrieved.");
      if (parking_pos_param_value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        for (XmlRpc::XmlRpcValue::ValueStruct::iterator it = parking_pos_param_value.begin(); it != parking_pos_param_value.end(); it++)
        {
          ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  Struct member: " << it->first << " of type " << it->second.getType());

          if (it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
          {
            ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  Struct entry of size: " << it->second.size());

            bool parking_pos_valid = false;
            bool frame_id_found = false;
            bool child_frame_id_found = false;
            bool tr_found = false;
            bool rot_found = false;

            std::string parking_pos_frame_id, parking_pos_child_frame_id;

            for (XmlRpc::XmlRpcValue::ValueStruct::iterator it_tf = it->second.begin(); it_tf != it->second.end(); it_tf++)
            {
              if (it_tf->first == "frame_id")
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   Parking position frame_id value found: " << it_tf->second);
                parking_pos_frame_id = (std::string) it_tf->second;
                frame_id_found = true;
              }
              if (it_tf->first == "child_frame_id")
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   Parking position child_frame_id value found: " << it_tf->second);
                parking_pos_child_frame_id = (std::string) it_tf->second;
                child_frame_id_found = true;
              }

              if (it_tf->first == "translation")
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  Parking position translation section found, type " << it_tf->second.getType());
                if (it_tf->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                  if (it_tf->second.hasMember("x") &&
                      it_tf->second.hasMember("y") &&
                      it_tf->second.hasMember("z"))
                  {
                    default_parking_position_.translation.x = (double) it_tf->second["x"];
                    default_parking_position_.translation.y = (double) it_tf->second["y"];
                    default_parking_position_.translation.z = (double) it_tf->second["z"];
                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   Parking position translation values found: (" << default_parking_position_.translation.x << "," << default_parking_position_.translation.y << "," << default_parking_position_.translation.z << ")");
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
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "  Parking position rotation section found, type " << it_tf->second.getType());
                if (it_tf->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                  if (it_tf->second.hasMember("r") &&
                      it_tf->second.hasMember("p") &&
                      it_tf->second.hasMember("y"))
                  {
                    tf2::Quaternion quat;
                    quat.setEuler((double) it_tf->second["y"], (double) it_tf->second["p"], (double) it_tf->second["r"]);

                    default_parking_position_.rotation.w = quat.w();
                    default_parking_position_.rotation.x = quat.x();
                    default_parking_position_.rotation.y = quat.y();
                    default_parking_position_.rotation.z = quat.z();

                    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "   Parking position rotation values found: (" << (double) it_tf->second["r"] << "," << (double) it_tf->second["p"] << "," << (double) it_tf->second["y"] << ")");
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
                parking_pos_valid = true;
              }

              if (parking_pos_valid)
              {
                ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Parking position frame IDs: " << parking_pos_frame_id << " -> " << parking_pos_child_frame_id);
                default_parking_position_set_ = true;
                default_parking_position_frame_id_ = parking_pos_frame_id;
                default_parking_position_child_frame_id_ = parking_pos_child_frame_id;
              }
            }
          }
        }
      }
    }
  }

  return true;
}

void BBEquipmentTFPublisher::rosLoop()
{
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Entering rosLoop().");
  ros::Rate tf_pub_rate(1); // TODO: Make configurable?
  while (ros::ok())
  {
    ROS_DEBUG_STREAM_NAMED("bb_tf_equipment_publisher", "rosLoop() iterating.");
    tf_pub_rate.sleep();

    if (publish_world_tf_)
    {
      world_transform_.header.stamp = ros::Time::now();

      ROS_DEBUG_STREAM_NAMED("bb_tf_equipment_publisher", "Publishing world TF: " << world_transform_.header.frame_id << " -> " << world_transform_.child_frame_id);
      static_tf_broadcaster_->sendTransform(world_transform_);
    }

    if (publish_map_odom_base_link_tfs_)
    {
      if (publish_map_odom_tf_)
      {
        map_odom_transform_.header.stamp = ros::Time::now();
        ROS_DEBUG_STREAM_NAMED("bb_tf_equipment_publisher", "Publishing map -> odom TF: " << map_odom_transform_.header.frame_id << " -> " << map_odom_transform_.child_frame_id << " -- publish_map_odom_base_link_without_prefix_ = " << (int) publish_map_odom_base_link_without_prefix_);

        static_tf_broadcaster_->sendTransform(map_odom_transform_);
      }

      if (publish_odom_base_link_tf_)
      {
        odom_base_link_transform_.header.stamp = ros::Time::now();
        ROS_DEBUG_STREAM_NAMED("bb_tf_equipment_publisher", "Publishing odom -> base_link TF: " << odom_base_link_transform_.header.frame_id << " -> " << odom_base_link_transform_.child_frame_id << " -- publish_map_odom_base_link_without_prefix_ = " << (int) publish_map_odom_base_link_without_prefix_);

        static_tf_broadcaster_->sendTransform(odom_base_link_transform_);
      }
    }

    ROS_DEBUG_STREAM_NAMED("bb_tf_equipment_publisher", "Publishing equipment TFs: " << equipment_tfs_.size());
    for (size_t k = 0; k < equipment_tfs_.size(); k++)
    {
      ROS_DEBUG_STREAM_NAMED("bb_tf_equipment_publisher", " * TF " << k << ": " << equipment_tfs_[k].header.frame_id << " -> " << equipment_tfs_[k].child_frame_id);
      equipment_tfs_[k].header.seq += 1;
      equipment_tfs_[k].header.stamp = ros::Time::now();
      tf_broadcaster_->sendTransform(equipment_tfs_[k]);
    }
  }
  ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "Exiting rosLoop().");
}
