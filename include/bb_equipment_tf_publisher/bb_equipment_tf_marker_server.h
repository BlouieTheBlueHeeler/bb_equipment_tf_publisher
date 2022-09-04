#ifndef BB_EQUIPMENT_TF_MARKER_SERVER_H
#define BB_EQUIPMENT_TF_MARKER_SERVER_H

#include <ros/ros.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <bb_equipment_tf_publisher/StaticTfUpdate.h>
#include <bb_equipment_tf_publisher/StaticTfs.h>

class BBEquipmentTFMarkerServer
{
  public:
    BBEquipmentTFMarkerServer(ros::NodeHandle& nh);
    virtual ~BBEquipmentTFMarkerServer();

    bool setup();
    void shutdown();

  private:
    visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);
    visualization_msgs::InteractiveMarkerControl& makeBoxControl(visualization_msgs::InteractiveMarker &msg);
    void make6DofMarker(const std::string &frame_id, const std::string& child_frame_id, bool fixed, unsigned int interaction_mode, const tf2::Vector3& position, bool show_6dof);

    void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    geometry_msgs::Quaternion quaternionToGeometryMsg(tf2::Quaternion quat);

    void updateTfList(const ros::TimerEvent&);
    void updateMarkerServer(const std::vector<std::string>& frames_to_add,
                            const std::vector<std::string>& child_frames_to_add,
                            const std::vector<geometry_msgs::Pose>& frame_data_to_add,
                            const std::vector<std::string>& frames_to_remove,
                            const std::vector<std::string>& frames_to_update,
                            const std::vector<geometry_msgs::Pose>& frame_data_to_update);

    ros::NodeHandle nh_;

    ros::Timer tf_list_query_timer_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
    interactive_markers::MenuHandler marker_menu_handler_;

    std::vector<std::string> known_frame_ids_;
    std::vector<geometry_msgs::Pose> known_tfs_;
    bool initial_frame_list_queried_;
};

#endif // BB_EQUIPMENT_TF_MARKER_SERVER_H
