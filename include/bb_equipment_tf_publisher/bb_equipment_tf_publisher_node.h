#ifndef BB_EQUIPMENT_TF_PUBLISHER_NODE_H
#define BB_EQUIPMENT_TF_PUBLISHER_NODE_H

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

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

  private:
    ros::NodeHandle nh_;
    boost::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    bool equipment_tfs_retrieved_;
    std::vector<BBEquipmentTransform> equipment_tf_values_;
    std::vector<geometry_msgs::TransformStamped> equipment_tfs_;

    bool exit_flag_;
};

#endif //BB_EQUIPMENT_TF_PUBLISHER_NODE_H
