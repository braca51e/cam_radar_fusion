 /*
 ********************
 *  v1.0: amc-nu (braca51e@gmail.com)
 *
 * cam_radar_fusion_node.cpp
 *
 *  
 */

#include "cam_radar_fusion/cam_radar_fusion.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, __APP_NAME__);

	CamRadarFusionApp app;

	app.Run();

	return 0;
}
