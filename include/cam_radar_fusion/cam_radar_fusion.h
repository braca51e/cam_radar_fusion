 /*
 ********************
 *  v1.0: amc-nu (braca51e@gmail.com)
 *
 * cam_radar_fusion.h
 *
 *  
 */

#ifndef PROJECT_CAM_RADAR_FUSION_H
#define PROJECT_CAM_RADAR_FUSION_H

#define __APP_NAME__ "cam_radar_fusion"

#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>

#include <ros/ros.h>
#include <tf/tf.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_geometry/pinhole_camera_model.h>

#include <image_transport/image_transport.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Eigen>

namespace std {
	template <>
	class hash< cv::Point >{
	public :
		size_t operator()(const cv::Point &cam_radar ) const
		{
			return hash<std::string>()( std::to_string(cam_radar.x) + "|" + std::to_string(cam_radar.y) );
		}
	};
};

class CamRadarFusionApp
{
	ros::NodeHandle                     node_handle_;
	image_transport::Publisher          publisher_fused_image_;
	ros::Subscriber                     intrinsics_subscriber_;
	image_geometry::PinholeCameraModel  cam_model_;

	tf::TransformListener*              transform_listener_;
	tf::StampedTransform                cam_radar_tf_;

	cv::Size                            image_size_;


	std::string 						image_frame_id_;

	bool                                camera_info_ok_;
	bool                                cam_radar_tf_ok_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> SyncPolicyT;
	typedef message_filters::Synchronizer<SyncPolicyT> Cloud_Sync;
	boost::shared_ptr<Cloud_Sync>        cloud_synchronizer_;

	message_filters::Subscriber<sensor_msgs::PointCloud2>       cloud_subscriber_;
	message_filters::Subscriber<sensor_msgs::CompressedImage>   image_subscriber_;
	
	pcl::PointXYZ TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform);

	void FusionCallback(const sensor_msgs::CompressedImageConstPtr& in_image_msg, const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg);

	/*!
	 * Obtains Transformation between two transforms registered in the TF Tree
	 * @param in_target_frame
	 * @param in_source_frame
	 * @return the found transformation in the tree
	 */
	tf::StampedTransform
	FindTransform(const std::string &in_target_frame, const std::string &in_source_frame);

	void IntrinsicsCallback(const sensor_msgs::CameraInfo& in_message);

	/*!
	 * Reads the config params from the command line
	 * @param in_private_handle
	 */
	void InitializeRosIo(ros::NodeHandle &in_private_handle);

public:
	void Run();
	CamRadarFusionApp();
};

#endif //PROJECT_CAM_RADAR_FUSION_H
