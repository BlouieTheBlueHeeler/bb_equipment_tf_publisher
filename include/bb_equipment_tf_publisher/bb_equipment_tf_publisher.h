#ifndef BB_EQUIPMENT_TF_PUBLISHER_NODE_H
#define BB_EQUIPMENT_TF_PUBLISHER_NODE_H

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <bb_equipment_tf_publisher/MapOdomBaseLinkTfs.h>
#include <bb_equipment_tf_publisher/StaticTfUpdate.h>

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

    void retrieveEquipmentTransformsList();
    void retrieveMapOdomBaseLinkConfig();

  private:
    bool mapOdomBaseLinkTfControl(bb_equipment_tf_publisher::MapOdomBaseLinkTfsRequest& req,
                                  bb_equipment_tf_publisher::MapOdomBaseLinkTfsResponse& resp);

    bool staticTfUpdate(bb_equipment_tf_publisher::StaticTfUpdate::Request& req,
                        bb_equipment_tf_publisher::StaticTfUpdate::Response& resp);

    visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);
    visualization_msgs::InteractiveMarkerControl& makeBoxControl(visualization_msgs::InteractiveMarker &msg);
    void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf2::Vector3& position, bool show_6dof);

    geometry_msgs::Quaternion quaternionToGeometryMsg(tf2::Quaternion qzat);

    ros::NodeHandle nh_;
    boost::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    bool equipment_tfs_retrieved_;
    bool map_odom_base_link_params_retrieved_;

    bool publish_map_odom_base_link_tfs_;
    bool publish_map_odom_tf_;
    bool publish_odom_base_link_tf_;

    geometry_msgs::TransformStamped map_odom_transform_;
    geometry_msgs::TransformStamped odom_base_link_transform_;
    std::vector<BBEquipmentTransform> equipment_tf_values_;
    std::vector<geometry_msgs::TransformStamped> equipment_tfs_;

    ros::ServiceServer map_odom_base_link_tf_srv_;
    ros::ServiceServer static_tf_update_srv_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
    interactive_markers::MenuHandler marker_menu_handler_;

    bool exit_flag_;
};

#endif //BB_EQUIPMENT_TF_PUBLISHER_NODE_H