#ifndef BB_EQUIPMENT_TF_PUBLISHER_H
#define BB_EQUIPMENT_TF_PUBLISHER_H

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <bb_equipment_tf_msgs/MapOdomBaseLinkTfs.h>
#include <bb_equipment_tf_msgs/StaticTfUpdate.h>
#include <bb_equipment_tf_msgs/StaticTfs.h>

#include <carecules_slam_msgs/TFPublishControl.h>
#include <carecules_slam_msgs/SlamInstanceControl.h>

#include <bb_equipment_tf_msgs/ParkingPosition.h>
#include <bb_equipment_tf_msgs/Waypoints.h>

#include <carecules_slam_msgs/TFPublishControl.h>
#include <carecules_slam_msgs/SlamInstanceControl.h>

#include <bb_equipment_tf_publisher/ParkingPosition.h>
#include <bb_equipment_tf_publisher/Waypoints.h>

struct BBEquipmentTransform
{
public:
  std::string frame_id;
  std::string child_frame_id;

  double translation_x, translation_y, translation_z;
  double roll, pitch, yaw;

  BBEquipmentTransform():
    translation_x(0.0), translation_y(0.0), translation_z(0.0),
    roll(0.0), pitch(0.0), yaw(0.0)
  {

  }

  BBEquipmentTransform(const BBEquipmentTransform& other)
  {
    if (this != &other)
    {
      frame_id = other.frame_id;
      child_frame_id = other.child_frame_id;
      translation_x = other.translation_x;
      translation_y = other.translation_y;
      translation_z = other.translation_z;
      roll = other.roll;
      pitch = other.pitch;
      yaw = other.yaw;
    }
  }

  BBEquipmentTransform& operator=(const BBEquipmentTransform& other)
  {
    if (this != &other)
    {
      frame_id = other.frame_id;
      child_frame_id = other.child_frame_id;
      translation_x = other.translation_x;
      translation_y = other.translation_y;
      translation_z = other.translation_z;
      roll = other.roll;
      pitch = other.pitch;
      yaw = other.yaw;
    }
    return *this;
  }
};

class BBEquipmentTFPublisher
{
  public:
    BBEquipmentTFPublisher(ros::NodeHandle&);
    virtual ~BBEquipmentTFPublisher();

    void rosLoop();
    void setExitFlag(bool);

    bool setup();
    void shutdown();

  private:
    bool retrieveEquipmentTransformsList();
    bool retrieveMapOdomBaseLinkConfig();

    bool saveEquipmentTransformsList(const std::string&);
    bool saveMapOdomTransformsList(const std::string&);
    bool saveWaypointLists(const std::string&);
    bool saveParkingPositions(const std::string&);

    bool mapOdomBaseLinkTfControl(bb_equipment_tf_msgs::MapOdomBaseLinkTfsRequest& req,
                                  bb_equipment_tf_msgs::MapOdomBaseLinkTfsResponse& resp);

    bool staticTfUpdate(bb_equipment_tf_msgs::StaticTfUpdate::Request& req,
                        bb_equipment_tf_msgs::StaticTfUpdate::Response& resp);

    bool staticTfList(bb_equipment_tf_msgs::StaticTfs::Request& req,
                      bb_equipment_tf_msgs::StaticTfs::Response& resp);

    bool slamTfUpdate(carecules_slam_msgs::TFPublishControl::Request& req,
                      carecules_slam_msgs::TFPublishControl::Response& resp);

    bool slamInstanceControl(carecules_slam_msgs::SlamInstanceControl::Request& req,
                              carecules_slam_msgs::SlamInstanceControl::Response& resp);

    bool parkingPositionControl(bb_equipment_tf_msgs::ParkingPosition::Request& req,
                                bb_equipment_tf_msgs::ParkingPosition::Response& resp);

    bool slamTfUpdate(carecules_slam_msgs::TFPublishControl::Request& req,
                      carecules_slam_msgs::TFPublishControl::Response& resp);

    bool slamInstanceControl(carecules_slam_msgs::SlamInstanceControl::Request& req,
                              carecules_slam_msgs::SlamInstanceControl::Response& resp);

    bool parkingPositionControl(bb_equipment_tf_publisher::ParkingPosition::Request& req,
                                bb_equipment_tf_publisher::ParkingPosition::Response& resp);

    ros::NodeHandle nh_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string node_name_;

    // Transform prefix used for transforms publishd from this node
    std::string tf_prefix_;
    // frame_ids for map->odom->base_link TF chain
    std::string world_tf_frame_id_, world_tf_child_frame_id_;
    std::string map_odom_tf_frame_id_, map_odom_tf_child_frame_id_;
    std::string odom_base_link_tf_frame_id_, odom_base_link_tf_child_frame_id_;

    bool equipment_tfs_retrieved_;
    bool map_odom_base_link_params_retrieved_;

    bool publish_equipment_tfs_;
    bool publish_world_tf_;
    bool publish_world_tf_without_prefix_;

    bool publish_map_odom_base_link_tfs_;
    bool publish_map_odom_tf_;
    bool publish_odom_base_link_tf_;
    bool publish_map_odom_base_link_without_prefix_;

    std::map<std::string, bool> slam_instances_active_;
    std::map<std::string, geometry_msgs::TransformStamped> slam_instances_last_pose_estimates_;

    std::map<std::string, bool> slam_instances_active_;
    std::map<std::string, geometry_msgs::TransformStamped> slam_instances_last_pose_estimates_;

    geometry_msgs::TransformStamped world_transform_;
    geometry_msgs::TransformStamped map_odom_transform_;
    geometry_msgs::TransformStamped odom_base_link_transform_;
    std::vector<BBEquipmentTransform> equipment_tf_values_;
    std::vector<geometry_msgs::TransformStamped> equipment_tfs_;

    std::map<std::string, geometry_msgs::Transform> parking_positions_;
    bool default_parking_position_set_;
    geometry_msgs::Transform default_parking_position_;
    std::string default_parking_position_frame_id_, default_parking_position_child_frame_id_;

    std::map<std::string, std::vector<geometry_msgs::Transform>> waypoints_;

    std::shared_ptr<ros::ServiceServer> map_odom_base_link_tf_srv_;
    std::shared_ptr<ros::ServiceServer> static_tf_update_srv_;
    std::shared_ptr<ros::ServiceServer> static_tf_list_srv_;
    std::shared_ptr<ros::ServiceServer> slam_tf_update_srv_;

    bool exit_flag_;
};

#endif //BB_EQUIPMENT_TF_PUBLISHER_H
