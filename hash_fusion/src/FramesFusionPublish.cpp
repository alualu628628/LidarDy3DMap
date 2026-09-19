#include "FramesFusion.h"

void InitMeshMsg(visualization_msgs::Marker& oMeshMsgs, string frame_id, int id, float r, float g, float b) {
	
	oMeshMsgs.header.frame_id = frame_id;
	oMeshMsgs.header.stamp = ros::Time::now();
	oMeshMsgs.type = visualization_msgs::Marker::TRIANGLE_LIST;
	oMeshMsgs.action = visualization_msgs::Marker::MODIFY;
	oMeshMsgs.id = id; 

	oMeshMsgs.scale.x = 1.0;
	oMeshMsgs.scale.y = 1.0;
	oMeshMsgs.scale.z = 1.0;

	oMeshMsgs.pose.position.x = 0.0;
	oMeshMsgs.pose.position.y = 0.0;
	oMeshMsgs.pose.position.z = 0.0;

	oMeshMsgs.pose.orientation.x = 0.0;
	oMeshMsgs.pose.orientation.y = 0.0;
	oMeshMsgs.pose.orientation.z = 0.0;
	oMeshMsgs.pose.orientation.w = 1.0;

	oMeshMsgs.color.a = 1.0;
	oMeshMsgs.color.r = r;
	oMeshMsgs.color.g = g;
	oMeshMsgs.color.b = b;
}


void FramesFusion::PublishMeshs(const pcl::PolygonMesh & oMeshModel){
  	
	pcl::PointCloud<pcl::PointXYZ> vPublishCloud;
	pcl::fromPCLPointCloud2(oMeshModel.cloud, vPublishCloud);

  	//new a visual message
	visualization_msgs::MarkerArray oMeshMsgList;
	visualization_msgs::Marker oMeshMsgs, oMeshDynamic, oMeshAdded, oMeshFused;
	
	//define header of message
	InitMeshMsg(oMeshMsgs, 		odomFrameId, 0, 0.9, 0.9 , 0.9);
	InitMeshMsg(oMeshDynamic, 	odomFrameId, 1, 0.9, 0.9 , 0.9);
	// add
	InitMeshMsg(oMeshAdded, 	odomFrameId, 2, 0.9, 0.9 , 0.9);

	// fused
	InitMeshMsg(oMeshFused,		odomFrameId, 3, 0.9, 0.9 , 0.9);


	//for each face
	for (int i = 0; i != oMeshModel.polygons.size(); ++i){

		uint32_t mesh_type = oMeshModel.polygons[i].vertices.back();
		
		//for each face vertex id
		for (int j = 0; j != 3; ++j){

			//vertex id in each sector
			int iVertexIdx =  oMeshModel.polygons[i].vertices[j];

			//temp point
    		geometry_msgs::Point oPTemp;
        	oPTemp.x = vPublishCloud.points[iVertexIdx].x;
        	oPTemp.y = vPublishCloud.points[iVertexIdx].y;
        	oPTemp.z = vPublishCloud.points[iVertexIdx].z;

			if(mesh_type >= __INT_MAX__) {
				oMeshAdded.points.push_back(oPTemp);
				// oMeshMsgs.points.push_back(oPTemp);
			}
			else if(mesh_type >= 0x3fffffff) {
				oMeshFused.points.push_back(oPTemp);
			}
			else if(mesh_type < 1) {
			// else if(mesh_type >= 1) {
       			oMeshMsgs.points.push_back(oPTemp);
			}
			else {
				oMeshDynamic.points.push_back(oPTemp);
			}

		}//end k

	}//end j

	oMeshMsgList.markers.push_back(oMeshMsgs);
	oMeshMsgList.markers.push_back(oMeshDynamic);
	oMeshMsgList.markers.push_back(oMeshAdded);
	oMeshMsgList.markers.push_back(oMeshFused);

	MeshPublisher.publish(oMeshMsgList);
}

void FramesFusion::PublishPointCloud(const pcl::PointCloud<pcl::PointNormal> & vCloudNormal){

    //convert to pc2 message
	sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(vCloudNormal, vCloudData);

	//other informations
	vCloudData.header.frame_id = odomFrameId;

	vCloudData.header.stamp = ros::Time::now();

	//publish
	CloudPublisher.publish(vCloudData);
}

void FramesFusion::PublishDynamicCloud(const pcl::PointCloud<pcl::PointNormal> & vDynamicPoints){

	//convert to pc2 message
	sensor_msgs::PointCloud2 vdynamicCloudData;
	pcl::toROSMsg(vDynamicPoints, vdynamicCloudData);

	//other informations
	vdynamicCloudData.header.frame_id = odomFrameId;
	vdynamicCloudData.header.stamp = ros::Time::now();

	//publish
	DynamicCloudPublisher.publish(vdynamicCloudData);
}

void FramesFusion::PublishStaticCloud(const pcl::PointCloud<pcl::PointNormal> & vStaticPoints){

	//convert to pc2 message
	sensor_msgs::PointCloud2 vCloudData;
	pcl::toROSMsg(vStaticPoints, vCloudData);

	//other informations
	vCloudData.header.frame_id = odomFrameId;
	vCloudData.header.stamp = ros::Time::now();

	//publish
	StaticCloudPublisher.publish(vCloudData);
}

void FramesFusion::PublishFreeSpace(const pcl::PointCloud<pcl::DistanceIoVoxel> & locale_free_space, const pcl::PointCloud<pcl::DistanceIoVoxel> & global_free_space, bool publishGlobalFreeSpace){

    //convert to pc2 message
	sensor_msgs::PointCloud2 localeCloudData;
	sensor_msgs::PointCloud2 golbalCloudData;
	
	pcl::PointCloud<pcl::_PointDistanceIo> locale_points;
	pcl::PointCloud<pcl::_PointDistanceIo> global_points;

	for(int i=0;i<locale_free_space.size();i++)
	{
		if(locale_free_space.points[i].io < 0.5) continue; // 剔除靠近mesh的空间体素
		pcl::_PointDistanceIo point;
		point.x = locale_free_space.points[i].x;
		point.y = locale_free_space.points[i].y;
		point.z = locale_free_space.points[i].z;
		point.distance = locale_free_space.points[i].distance;
		point.io = locale_free_space.points[i].io;
		locale_points.push_back(point);
	}

	for(int i=0; publishGlobalFreeSpace && i<global_free_space.size();i++)
	{
		if(global_free_space.points[i].io < 0.5) continue; // 剔除靠近mesh的空间体素
		pcl::_PointDistanceIo point;
		point.x = global_free_space.points[i].x;
		point.y = global_free_space.points[i].y;
		point.z = global_free_space.points[i].z;
		// point.intensity = global_free_space.points[i].distance;
		point.distance = global_free_space.points[i].distance;
		point.io = global_free_space.points[i].io;
		global_points.push_back(point);
	}

	pcl::toROSMsg(locale_points, localeCloudData);
	pcl::toROSMsg(global_points, golbalCloudData);

	//other informations
	localeCloudData.header.frame_id = odomFrameId;
	golbalCloudData.header.frame_id = odomFrameId;

	localeCloudData.header.stamp = ros::Time::now();
	golbalCloudData.header.stamp = ros::Time::now();

	//publish
	localeFreeSpacePublisher.publish(localeCloudData);
	if(publishGlobalFreeSpace)
		globalFreeSpacePublisher.publish(golbalCloudData);
		if(outputFiles) {
		std::stringstream localOutputPath, golbalOutputPath;
		localOutputPath << outputPathList["locale_free_space"] << "dy_" << std::setw(4) << std::setfill('0') << pcFrameCount << "_fs_locale.ply";
		golbalOutputPath << outputPathList["global_free_space"] << "dy_" << std::setw(4) << std::setfill('0') << pcFrameCount << "_fs_global.ply";
		pcl::io::savePLYFileBinary(localOutputPath.str(), locale_points);
		if(publishGlobalFreeSpace)
			pcl::io::savePLYFileBinary(golbalOutputPath.str(), global_points);
	}
}
