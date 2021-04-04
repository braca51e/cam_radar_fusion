# Camera-Radar Fusion

## Camera-Radar fusion node

This node projects PointCloud to Image plane and publishes a Image with points and distance.

### Requirements

1. Camera intrinsics
1. Compressed Image rectified.

### How to launch

* From a sourced terminal:

`roslaunch cam_radar_fusion cam_radar_fusion.launch`


### Parameters

Launch file available parameters:

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`points_src`|*String* |Name of the PointCloud topic to subscribe.|Default `points_raw`|
|`image_src`|*String*|Name of the Image topic to subscribe|
|`camera_info_src`|*String*|Name of the CameraInfo topic that contains the intrinsic matrix for the Image.|`camera_info`|

### Subscriptions/Publications


```
Node [/cam_radar_fusion]
Publications: 
 * /points_fused [sensor_msgs/CompressedImage]

Subscriptions: 
 * /image_raw [sensor_msgs/CompressedImage]
 * /points_raw [sensor_msgs/PointCloud2]
 * /camera_info [sensor_msgs/CameraInfo]
 * /tf [tf2_msgs/TFMessage]
```
