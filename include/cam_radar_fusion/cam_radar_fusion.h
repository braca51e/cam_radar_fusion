/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * cam_radar_fusion.h
 *
 *  Created on: May, 19th, 2018
 */

#ifndef PROJECT_PIXEL_CLOUD_FUSION_H
#define PROJECT_PIXEL_CLOUD_FUSION_H

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
	ros::Publisher                      publisher_fused_cloud_;
	ros::Subscriber                     intrinsics_subscriber_;
	image_geometry::PinholeCameraModel  cam_model_;

	tf::TransformListener*              transform_listener_;
	tf::StampedTransform                cam_radar_tf_;

	cv::Size                            image_size_;
	//cv::Mat                             camera_instrinsics_;
	//cv::Mat                             distortion_coefficients_;
	//cv::Mat                             current_frame_;

	std::string 						image_frame_id_;

	//bool                                processing_;
	bool                                camera_info_ok_;
	bool                                cam_radar_tf_ok_;

	//float                               fx_, fy_, cx_, cy_;
	//pcl::PointCloud<pcl::PointXYZRGB>   colored_cloud_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> SyncPolicyT;
	//typedef message_filters::Synchronizer<SyncPolicyT>* cloud_synchronizer_;
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

#endif //PROJECT_PIXEL_CLOUD_FUSION_H
