#include "bb_equipment_tf_publisher/bb_equipment_tf_publisher_node.h"

#include <tf2/utils.h>

BBEquipmentTFPublisher::BBEquipmentTFPublisher(ros::NodeHandle& nh): nh_(nh), exit_flag_(false), equipment_tfs_retrieved_(false)
{
    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "bb_tf_equipment_publisher: Constructor.");
    tf_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster());
}

BBEquipmentTFPublisher::~BBEquipmentTFPublisher()
{
    ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "bb_tf_equipment_publisher: Destructor.");
}

void BBEquipmentTFPublisher::setExitFlag(bool value)
{
    exit_flag_ = value;
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

void BBEquipmentTFPublisher::rosLoop()
{
    ros::Rate tf_pub_rate(1);
    while (ros::ok())
    {
        ROS_INFO_STREAM_NAMED("bb_tf_equipment_publisher", "rosLoop() iterating.");
        tf_pub_rate.sleep();

        for (size_t k = 0; k < equipment_tfs_.size(); k++)
        {
          equipment_tfs_[k].header.seq += 1;
          equipment_tfs_[k].header.stamp = ros::Time::now();
          tf_broadcaster_->sendTransform(equipment_tfs_[k]);
        }
    }
}
