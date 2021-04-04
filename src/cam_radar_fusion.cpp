/*
 ********************
 *  v1.0: amc-nu (braca51e@gmail.com)
 *
 * cam_radar_fusion.cpp
 *
 *  
 */

#include "cam_radar_fusion/cam_radar_fusion.h"

pcl::PointXYZ
CamRadarFusionApp::TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
{
	tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
	tf::Vector3 tf_point_t = in_transform * tf_point;
	return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

void CamRadarFusionApp::FusionCallback(const sensor_msgs::CompressedImageConstPtr &in_image_msg , const sensor_msgs::PointCloud2ConstPtr &in_cloud_msg)
{
	if (!cam_radar_tf_ok_)
	{
		cam_radar_tf_ = FindTransform(in_image_msg->header.frame_id,
		                                 in_cloud_msg->header.frame_id);
	}
	if (!camera_info_ok_ || !cam_radar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Radar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}
    //ROS_INFO("[%s] Transforming point starts!", __APP_NAME__);
	// Get image
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
	image_size_.height = cv_image->image.rows;
	image_size_.width = cv_image->image.cols;

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*in_cloud_msg, *in_cloud);

	//std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());
	pcl::PointXYZ transformed_point;
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		//ROS_INFO("[%s] x: %.4f y: %.4f z: %.4f", __APP_NAME__, in_cloud->points[i].x, in_cloud->points[i].y, in_cloud->points[i].z);
		if (in_cloud->points[i].x > 0)
		{
			transformed_point = TransformPoint(in_cloud->points[i], cam_radar_tf_);
			cv::Point2d xy_point = cam_model_.project3dToPixel(cv::Point3d(transformed_point.x, transformed_point.y, transformed_point.z));
			if ((xy_point.x >= 0) && (xy_point.x < image_size_.width)
			    && (xy_point.y >= 0) && (xy_point.y < image_size_.height)
				//&& transformed_point.z > 0
					)
			{
				// draw circle 
				cv::circle(cv_image->image, cv::Point(xy_point.x, xy_point.y), 3, cv::Scalar(0, 255, 0), 5);
				std::ostringstream ss;
				//sprintf(buffer, "%d plus %d is %d", a, b, a+b);
                ss << std::setprecision(2) << transformed_point.z << "m";
				cv::putText(cv_image->image, ss.str(), cv::Point(xy_point.x, xy_point.y-2), cv::FONT_HERSHEY_DUPLEX, 0.9, cv::Scalar(0,0,255), 1, false);
			}
		}
	}

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image->image).toImageMsg();
	publisher_fused_image_.publish(msg);
}

void CamRadarFusionApp::IntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
{
	image_size_.height = in_message.height;
	image_size_.width = in_message.width;

	cam_model_.fromCameraInfo(in_message);

	intrinsics_subscriber_.shutdown();
	camera_info_ok_ = true;
	ROS_INFO("[%s] CameraIntrinsics obtained.", __APP_NAME__);
}

tf::StampedTransform
CamRadarFusionApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
	tf::StampedTransform transform;

	cam_radar_tf_ok_ = false;
	try
	{
		transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
		cam_radar_tf_ok_ = true;
		ROS_INFO("[%s] Camera-Radar TF obtained", __APP_NAME__);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
	}

	return transform;
}

void CamRadarFusionApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
	//get params
	std::string points_src, image_src, camera_info_src, fused_topic_str = "/points_fused";
	std::string name_space_str = ros::this_node::getNamespace();

	ROS_INFO("[%s] This node requires: Registered TF(Camera-Radar), CameraInfo, Image, and PointCloud.", __APP_NAME__);
	in_private_handle.param<std::string>("points_src", points_src, "/points_raw");
	ROS_INFO("[%s] points_src: %s", __APP_NAME__, points_src.c_str());

	in_private_handle.param<std::string>("image_src", image_src, "/image_rectified");
	ROS_INFO("[%s] image_src: %s", __APP_NAME__, image_src.c_str());
	// Keep frame id to register to center clouds
	image_frame_id_ = image_src;

	in_private_handle.param<std::string>("camera_info_src", camera_info_src, "/camera_info");
	ROS_INFO("[%s] camera_info_src: %s", __APP_NAME__, camera_info_src.c_str());

	if (name_space_str != "/")
	{
		if (name_space_str.substr(0, 2) == "//")
		{
			name_space_str.erase(name_space_str.begin());
		}
		image_src = name_space_str + image_src;
		fused_topic_str = name_space_str + fused_topic_str;
		camera_info_src = name_space_str + camera_info_src;
	}

	//generate subscribers and sychronizers
	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, camera_info_src.c_str());
	intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src,
	                                                     1,
	                                                     &CamRadarFusionApp::IntrinsicsCallback, this);

	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, points_src.c_str());
	cloud_subscriber_.subscribe(in_private_handle,
		                        points_src.c_str(),
	                            1);
	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, image_src.c_str());
	image_subscriber_.subscribe(in_private_handle,
		                        image_src.c_str(),
	                            1); 

	ROS_INFO("[%s] Subscribing to %s and %s to approximate time synchronizer.", __APP_NAME__, image_src.c_str(), points_src.c_str());
	cloud_synchronizer_.reset(new Cloud_Sync(SyncPolicyT(10), image_subscriber_, cloud_subscriber_));
	cloud_synchronizer_->registerCallback(boost::bind(&CamRadarFusionApp::FusionCallback, this, _1, _2));

	image_transport::ImageTransport image_transport_(in_private_handle);
	publisher_fused_image_ = image_transport_.advertise(fused_topic_str, 1);
	ROS_INFO("[%s] Publishing fused pointcloud in %s", __APP_NAME__, fused_topic_str.c_str());
}

void CamRadarFusionApp::Run()
{
	ros::NodeHandle private_node_handle("~");
	tf::TransformListener transform_listener;

	transform_listener_ = &transform_listener;

	InitializeRosIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

	ros::spin();

	ROS_INFO("[%s] END", __APP_NAME__);
}

CamRadarFusionApp::CamRadarFusionApp()
{
	cam_radar_tf_ok_ = false;
	camera_info_ok_ = false;
	image_frame_id_ = "";
}