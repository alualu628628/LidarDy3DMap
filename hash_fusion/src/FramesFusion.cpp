#include "FramesFusion.h"

#include <sstream>
#include <random>
#include <algorithm>
#include <cmath>
#include <exception>
#include <unordered_set>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "tools/ThreadPool.h"

/*************************************************
Function: FramesFusion
Description: constrcution function for FramesFusion class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: node - a ros node class
     nodeHandle - a private ros node class
*************************************************/
FramesFusion::FramesFusion(ros::NodeHandle & node,
                       ros::NodeHandle & nodeHandle): 
					   m_oGlobalNode(node), m_oNodeHandle(nodeHandle), odomCount(0), 
					   averageReconstructTime(0), maxReconstructTime(0),  reconstructFrameNum(0),
					   averageFusionTime(0), maxFusionTime(0), fusionFrameNum(0), odomLoopRate(1),
					   projectUpdater(ProjectUpdater::GetInstance()),
					   rayUpdater(RayUpdater::GetInstance()),
					   meshUpdater(Updater::MeshUpdater::GetInstance()),
					   rosPubManager(RosPublishManager::GetInstance()),
					   fusionThreadPool(new Tools::BufferLimitThreadPool(1, 1)) {

	//read parameters
	ReadLaunchParams(nodeHandle);

	//***subscriber related*** 
	//subscribe (hear) the point cloud topic
	cloudSuber = nodeHandle.subscribe(subCloudTopic, 5, &FramesFusion::HandleCloud, this);

	// subscribe mesh topic
	meshSuber = nodeHandle.subscribe(subMeshTopic, 5, &FramesFusion::HandleMesh, this);

	//subscribe (hear) the odometry information (trajectory)
	odomSuber = nodeHandle.subscribe(
		subOdomTopic, 
		1,
		asyncReconstruction ? &FramesFusion::HandleTrajectoryThread : &FramesFusion::HandleTrajectory, 
		this
	);

	//***publisher related*** 
	//publish point cloud after processing
	CloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(pubCloudTopic, 1, true);

	MeshPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("/hash_fusion/fusion_recon", 1, true);
	//publish hotmap for navigation
	HotMapPublisher = nodeHandle.advertise<grid_map_msgs::GridMap>(pubHotMapTopic, 1, true);
	DynamicCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("/hash_fusion/dynamic_clouds", 1, true);
	StaticCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("/hash_fusion/static_clouds", 1, true);

	localeFreeSpacePublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("/hash_fusion/locale_free_space", 1, true);
	globalFreeSpacePublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("/hash_fusion/global_free_space", 1, true);
}


void FramesFusion::LazyLoading() {
	
    std::cout << "Load Fixed Resolution Fusion..." << std::endl;
}


FramesFusion::~FramesFusion() {

	std::cout << output::format_green
	<< "Static Extract Frame numbers: " << pcFrameCount << std::endl
	<< "Average static extract per frame: " << averageExtractTimel / pcFrameCount << "ms;\t"
	<< "Max extract time: " << maxExtractTime << "ms"
	<< output::format_white << std::endl;

	std::cout << output::format_cyan
	<< "Fusion frame numbers: " << fusionFrameNum << std::endl
	<< "Average fusion per frame: " << averageFusionTime / fusionFrameNum << "ms;\t"
	<< "Max fusion time: " << maxFusionTime << "ms"
	<< output::format_white << std::endl;

	std::cout << output::format_cyan
	<< "Sdf extract frame numbers: " << reconstructFrameNum << std::endl
	<< "Average sdf extract per frame: " << averageReconstructTime / reconstructFrameNum << "ms;\t"
	<< "Max extract time: " << maxReconstructTime << "ms" << std::endl
	// << "Final point nums: " << mapPCN.size() + mapPCNAdded.size() + mapPCNTrueAdded.size()
	<< output::format_white << std::endl;

    //output to the screen
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "********************************************************************************" << std::endl;

	//output point clouds with computed normals to the files when the node logs out
	if(outputFiles) {
		
		std::cout << "Please do not force killing the programe, the process is writing output PLY file." << std::endl;
		std::cout << "It may take times (Writing 500M file takes about 20 seconds in usual)." << std::endl;

		//output times
		std::stringstream sFuseEvalFileName;
		sFuseEvalFileName << fileHead << "FuseTime.csv";
		std::cout << "The output eval file is " << sFuseEvalFileName.str() << std::endl;
		fuse_timer.OutputDebug(sFuseEvalFileName.str());

		std::stringstream sReconEvalFileName;
		sReconEvalFileName << fileHead << "ReconstructTime.csv";
		std::cout << "The output eval file is " << sReconEvalFileName.str() << std::endl;
		reconstruct_timer.OutputDebug(sReconEvalFileName.str());

		std::stringstream sDynaEvalFileName;
		sDynaEvalFileName << fileHead << "DynamicExtractTime.csv";
		std::cout << "The output eval file is " << sDynaEvalFileName.str() << std::endl;
		dynamic_timer.OutputDebug(sDynaEvalFileName.str());

		// SaveFinalMeshAndPointCloud();
		SaveVoxels();

		std::stringstream sFreeSpaceFileName;
		sFreeSpaceFileName << fileHead << "FreeSpaceTime.csv";
		std::cout << "The output eval file is " << sFreeSpaceFileName.str() << std::endl;
		meshUpdater.OutputTimeCost(sFreeSpaceFileName.str());

		/** 
			if the point cloud has too many points, ros may not wait it to save.
			to solve this problem, change the file: /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py
				DEFAULT_TIMEOUT_SIGINT = 15.0  ->  DEFAULT_TIMEOUT_SIGINT = 60.0 
		**/
		std::cout << "Output is complete! The process will be automatically terminated. Thank you for waiting. " << std::endl;
	}
}


bool FramesFusion::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

	// distanceIoVolume type
	int iVolumeType;
	// nodeHandle.param("distanceIoVolume_type", iVolumeType, static_cast<int>(VolumeBase::VolumeType::HASH_VOXELER));
	distanceIoVolume.reset(VolumeBase::CreateVolume(VolumeBase::VolumeType::DISTANCE_IO));
	// LOG(INFO) << "Volume type: " << iVolumeType;
	distanceIoVolume->InitLog();
	
	// hashVoxeler.reset(VolumeBase::CreateVolume(VolumeBase::VolumeType::DISTANCE_IO));
	// hashVoxeler->InitLog();

 	//output file name
 	nodeHandle.param("file_output_path", fileHead, std::string());
	outputFiles = !fileHead.empty();
	// 输出静态点云 动态点云 局部自由空间点云 全局自由空间点云 HotMap
	if (outputFiles) {
		outputPathList["static_clouds"] = CheckOutputPath(fileHead, "static_clouds");
		outputPathList["dynamic_clouds"] = CheckOutputPath(fileHead, "dynamic_clouds");
		outputPathList["locale_free_space"] = CheckOutputPath(fileHead, "locale_free_space");
		outputPathList["global_free_space"] = CheckOutputPath(fileHead, "global_free_space");
		outputPathList["hotmap"] = CheckOutputPath(fileHead, "hotmap");
    }


 	//sub point cloud topic
	nodeHandle.param("sub_cloud_topic", subCloudTopic, std::string("/frame_cloudnormals"));
	nodeHandle.param("sub_mesh_topic", subMeshTopic, std::string("/frame_mesh_algo"));
  	nodeHandle.param("sub_odom_topic", subOdomTopic, std::string("/odometry/filtered"));

	//input odom topic
	nodeHandle.param("pub_cloud_topic", pubCloudTopic, std::string("/hash_fusion/static_clouds"));


	rosPubManager.m_sFrameId = odomFrameId;
	rosPubManager.m_pNodeHandle = &nodeHandle;

	//input odom topic
	// nodeHandle.param("polygon_out_topic", m_sOutMeshTopic, std::string("/surrounding_meshes"));
	nodeHandle.param("pub_hot_map_out_topic", pubHotMapTopic, std::string("hotmap3d"));

	//nearbt lengths
	nodeHandle.param("voxel_total_size", nearLengths, 20.0f);
	nearLengths = nearLengths / 2.0f;	
	meshUpdater.SetUpdateRange(nearLengths);

	//point cloud sampling number
	nodeHandle.param("sample_pcframe_num", frameSmplingNum, 1);

	//point cloud sampling number
	nodeHandle.param("sample_inputpoints_num", inputCloudSamlingNum, 1);

	//nearby mesh update period in second
	nodeHandle.param("mesh_update_period", nearMeshPeriod, 2.0f);

	//side length of cube (voxel)
  	float fCubeSize;
  	nodeHandle.param("voxel_cube_size", fCubeSize, 0.5f);
 	voxelResolution = pcl::PointXYZ(fCubeSize, fCubeSize, fCubeSize);
	distanceIoVolume->SetResolution(voxelResolution);
 	voxelResolution = pcl::PointXYZ(fCubeSize*2, fCubeSize*2, fCubeSize*2);
	// hashVoxeler.SetResolution(voxelResolution);

	int eStrategyType;
  	nodeHandle.param("strategy_type", eStrategyType, static_cast<int>(eEmptyStrategy));
	distanceIoVolume->SetStrategy(static_cast<vus>(eStrategyType));
	// hashVoxeler.SetStrategy(static_cast<vus>(eStrategyType));

	//use surfel fusion?
	nodeHandle.param("use_surfel_fusion", surfelFusion, true);
	// use additional points for reconstruction?
	nodeHandle.param("additional_points", useAdditionalPoints, true);

	nodeHandle.param("async_reconstruct", asyncReconstruction, true);
	nodeHandle.param("use_union_set", useUnionSetConnection, true);
	nodeHandle.param("only_max_union_set", onlyMaxUnionSet, true);
	nodeHandle.param("recon_range", reconstructRange, 30.0f);
	nodeHandle.param("strict_dot_ref", strictDotRef, 0.95f);
	nodeHandle.param("soft_dot_ref", softDotRef, 0.3f);
	nodeHandle.param("remove_size_ref", removeSizeRef, 200);
	nodeHandle.param("remove_time_ref", removeTimeRef, 10.0f);
	nodeHandle.param("confidence_level_length", confidenceLevelLength, 8.0f);
	nodeHandle.param("center_based_recon", centerBasedRecon, false);

	//count processed point cloud frame
	pcFrameCount = 0;

	//true indicates the file has not been generated
	outPCFileFlag = true;

	odomLoopRate = ros::Rate(1 / nearMeshPeriod);

	// sdf meshing params
	nodeHandle.param("keep_time", keepTime, 30);
	nodeHandle.param("conv_dim", convDim, 3);
	nodeHandle.param("conv_add_point_ref", convAddPointNumRef, 5); 
	nodeHandle.param("conv_distance_ref", convFusionDistanceRef1, 0.95f);
	nodeHandle.param("dynamic_debug", dynamicDebug, false);
	nodeHandle.param("keep_voxel", keepVoxel, false);
	nodeHandle.param("publish_global_free_space", publishGlobalFreeSpace, false);
	// m_pSdf = new SignedDistance(keepTime, convDim, convAddPointNumRef, convFusionDistanceRef1);
	// distanceIoVolume->m_iMaxRecentKeep = max(500u, (uint32_t)keepTime);
	// hashVoxeler.m_iMaxRecentKeep = max(500u, (uint32_t)keepTime);

	// octree params
	int octree_level;
	nodeHandle.param("octree_level", octree_level, 0);
	meshUpdater.SetOctreeLevel(octree_level);

	return true;

}


void FramesFusion::HandleCloud(const sensor_msgs::PointCloud2 & vCloud) {

	dynamic_timer.NewLine();

	++pcFrameCount;

	pcl::PointCloud<pcl::PointNormal> oCloud;
	pcl::fromROSMsg(vCloud, oCloud);

	DistanceIoVolume* pDistanceIoVolume = dynamic_cast<DistanceIoVolume*>(distanceIoVolume.get());
	if(pDistanceIoVolume == nullptr) {
		// ROS_INFO_CYAN("%s", "[FramesFusion] Volume type is not DistanceIoVolume!");
		LOG(WARNING)<< "[FramesFusion] Volume type is not DistanceIoVolume!";
		return;
	}

	pcl::PointNormal viewPoint;

    pcl::PointCloud<pcl::PointNormal> staticPoints, dynamicPoints;
    for(auto& oPoint : oCloud) {
		if(oPoint.curvature == -1) { // viewpoint marker is always retained
			viewPoint = oPoint;
			continue;
		}
        const float sdf = pDistanceIoVolume->SearchSdf(oPoint.getVector3fMap());
        // Sparse/unobserved map cells are unknown, not static evidence.
        if(!std::isfinite(sdf)) continue;
        if(sdf <= pDistanceIoVolume->GetStaticExpandDistance())
			staticPoints.push_back(oPoint);
		else dynamicPoints.push_back(oPoint);
    }

	double process_time = dynamic_timer.DebugTime("all");

	averageExtractTimel += process_time;
	maxExtractTime = process_time > maxExtractTime ? process_time : maxExtractTime;

	PublishStaticCloud(staticPoints);
	PublishDynamicCloud(dynamicPoints);

	// SurfelFusionQuick(viewPoint, staticPoints);

	//merge one frame data
	UpdateOneFrame(viewPoint, staticPoints);

	
	if(outputFiles) {
		//TODO fix 多线程保存
		std::stringstream staticOutputPath, dynamicOutputPath;
		staticOutputPath << outputPathList["static_clouds"] << "dy_" << std::setw(4) << std::setfill('0') << pcFrameCount <<  "_pc_static.ply";
		dynamicOutputPath << outputPathList["dynamic_clouds"] << "dy_" << std::setw(4) << std::setfill('0') << pcFrameCount << "_pc_dyamic.ply";

		if(staticPoints.size() > 0)
			pcl::io::savePLYFileBinary(staticOutputPath.str(), staticPoints);
		if(dynamicPoints.size() > 0)
			pcl::io::savePLYFileBinary(dynamicOutputPath.str(), dynamicPoints);
	}
}

void FramesFusion::HandleMesh(const fusion_msgs::MeshArray & vMeshRosData)
{
	fusionThreadPool->AddTask([&, vMeshRosData](){ // 注意函数参数不能为引用

		++fusionFrameNum;

		fuse_timer.NewLine();
		pcl::PointCloud<pcl::DistanceIoVoxel> locale_free_space;
		pcl::PointCloud<pcl::DistanceIoVoxel> global_free_space;

		// 转换网格格式
		std::vector<pcl::PolygonMesh> vSectorMeshList;
		for(auto&& mesh : vMeshRosData.data) {
			
			pcl::PolygonMesh::Ptr pFrameMesh(new pcl::PolygonMesh);
			pcl::PointCloud<pcl::PointXYZ> oCloud;
			for(auto&& point : mesh.vertices) {
				oCloud.push_back(pcl::PointXYZ(point.x, point.y, point.z));
			}
			pcl::toPCLPointCloud2(oCloud, pFrameMesh->cloud);
			for(auto&& triangle : mesh.triangles) {
				pcl::Vertices oVertices;
				oVertices.vertices.push_back(triangle.vertex_indices[0]);
				oVertices.vertices.push_back(triangle.vertex_indices[1]);
				oVertices.vertices.push_back(triangle.vertex_indices[2]);
				pFrameMesh->polygons.push_back(oVertices);
			}

			vSectorMeshList.push_back(*pFrameMesh);
		}

		// 添加面片置信度
		std::vector<std::vector<float>> vMeshConfidence;
		for(auto&& confidenceList : vMeshRosData.pseudo_tokens) {
			vMeshConfidence.push_back(confidenceList.tokens);
		}

		fuse_timer.DebugTime("1_transfer_mesh");

		meshUpdater.MeshFusion( pcl::PointNormal(), 
								vSectorMeshList, 
								vMeshConfidence, 
								*distanceIoVolume, 
								dynamicDebug || keepVoxel,
								locale_free_space,
								global_free_space,
                                publishGlobalFreeSpace);

		double frames_fusion_time = fuse_timer.DebugTime("2_main_fusion");
		
		PublishFreeSpace(locale_free_space, global_free_space, publishGlobalFreeSpace);

		averageFusionTime += frames_fusion_time;
		maxFusionTime = frames_fusion_time > maxFusionTime ? frames_fusion_time : maxFusionTime;

		fuse_timer.GetCurrentLineTime();
	});
}

void FramesFusion::UpdateOneFrame(const pcl::PointNormal& oViewPoint, pcl::PointCloud<pcl::PointNormal>& vFilteredMeasurementCloud) {

	///* limit the distance
	int n = vFilteredMeasurementCloud.size();
	for(int i = 0; i < n; ++i) {
		auto & oPoint = vFilteredMeasurementCloud[i];
		Eigen::Vector3f vDistance(oViewPoint.x - oPoint.x, oViewPoint.y - oPoint.y, oViewPoint.z - oPoint.z);
		if(vDistance.norm() > reconstructRange) {
			swap(vFilteredMeasurementCloud[i--], vFilteredMeasurementCloud[--n]);
		}
	}
	vFilteredMeasurementCloud.erase(vFilteredMeasurementCloud.begin()+n, vFilteredMeasurementCloud.end());
	//*/

	distanceIoVolume->VoxelizePointsAndFusion(vFilteredMeasurementCloud);
	// hashVoxeler.VoxelizePointsAndFusion(vFilteredMeasurementCloud);
	mapPCN += vFilteredMeasurementCloud;
}

void FramesFusion::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{

	//a flag indicates whether to calculate this odom
	bool bComputeFlag = false;

	//count input frames
	if(!odomCount){

		//initialize the time of last calculation as beginning
		lastModelingTime = oTrajectory.header.stamp;
		//need to compute
		bComputeFlag = true;

	}else{

		//compute the time difference
		ros::Duration oModelduration = oTrajectory.header.stamp - lastModelingTime;

		//
		if(oModelduration.toSec() > nearMeshPeriod)
			//
			bComputeFlag = true;

	}

	odomCount++;

	//if it is in the calculation period
	//it would end this function and wait for the next input	
	if(!bComputeFlag)
		return;
	
	//if need to be updated
	//get the newest information
	lastModelingTime = oTrajectory.header.stamp;

	//save the position of trajectory
	Eigen::Vector3f oLidarPos;
	oLidarPos.x() = oTrajectory.pose.pose.position.x;
	oLidarPos.y() = oTrajectory.pose.pose.position.y;
	oLidarPos.z() = oTrajectory.pose.pose.position.z;
	distanceIoVolume->UpdateLidarCenter(oLidarPos);
	// hashVoxeler.UpdateLidarCenter(oLidarPos);

	//get the reconstructed surfaces
	// pcl::PolygonMesh oNearbyMeshes;

	// Mesh Generate
	clock_t start_time = clock();

	// if(!surfelFusion && useAdditionalPoints) 
		// SurroundModelingWithPointProcessing(oOdomPoint.oLocation, oNearbyMeshes, reconstructFrameNum);
	// else 
	// SlideModeling(oNearbyMeshes, oLidarPos, reconstructFrameNum);
	// GenerateHotMap(oLidarPos, reconstructFrameNum, oTrajectory.header.stamp);
	ExtractSdfMaps(oLidarPos, reconstructFrameNum, oTrajectory.header.stamp);
	clock_t frames_fusion_time = 1000.0 * (clock() - start_time) / CLOCKS_PER_SEC;

	++reconstructFrameNum;
	// std::cout << output::format_cyan 
	// 	<< "The No. " << reconstructFrameNum 
	// 	<< ";\tframes_fusion_time: " << frames_fusion_time << "ms" 
	// 	<< output::format_white << std::endl;
	averageReconstructTime += frames_fusion_time;
	maxReconstructTime = frames_fusion_time > maxReconstructTime ? frames_fusion_time : maxReconstructTime;

	//output the nearby surfaces
	// PublishMeshs(oNearbyMeshes);

	// auto vAllCloud = AllCloud(mapPCN);
	// std::cout << "### cloud_size: " << vAllCloud->size() << " ###" << std::endl; 
	// PublishPointCloud(*vAllCloud);
}

void FramesFusion::HandleTrajectoryThread(const nav_msgs::Odometry & oTrajectory) {

	// get odom
	bool bComputeFlag = false;
	if(!odomCount){
		lastModelingTime = oTrajectory.header.stamp;
		bComputeFlag = true;
	}else{
		//compute the time difference
		ros::Duration oModelduration = oTrajectory.header.stamp - lastModelingTime;
		if(oModelduration.toSec() > nearMeshPeriod)
			bComputeFlag = true;
	}
	odomCount++;
	if(!bComputeFlag) return;
	lastModelingTime = oTrajectory.header.stamp;
	
	Eigen::Vector3f oLidarPos;
	oLidarPos.x() = oTrajectory.pose.pose.position.x;
	oLidarPos.y() = oTrajectory.pose.pose.position.y;
	oLidarPos.z() = oTrajectory.pose.pose.position.z;
	distanceIoVolume->UpdateLidarCenter(oLidarPos);
	// hashVoxeler.UpdateLidarCenter(oLidarPos);

	//if need to be updated
	//get the newest information
	int now_frame_num = reconstructFrameNum++;

	auto ModelingFunction = [&, now_frame_num, oLidarPos]() {

		// std::cout << output::format_cyan << "No. " << now_frame_num << " reconstruct start" << output::format_white << std::endl;

		//get the reconstructed surfaces
		pcl::PolygonMesh oResultMeshes;

		struct timeval start;
		gettimeofday(&start, NULL);

		// if(!surfelFusion && useAdditionalPoints) 
		// 	SurroundModelingWithPointProcessing(oOdomPoint.oLocation, oResultMeshes, now_frame_num);
		// else 
		// SlideModeling(oResultMeshes, oLidarPos, now_frame_num);
		// GenerateHotMap(oLidarPos, now_frame_num, oTrajectory.header.stamp);
		ExtractSdfMaps(oLidarPos, now_frame_num, oTrajectory.header.stamp);

		struct timeval end;
		gettimeofday(&end,NULL);
		double frame_reconstruct_time = (end.tv_sec - start.tv_sec) * 1000.0 +(end.tv_usec - start.tv_usec) * 0.001;

		averageReconstructTime += frame_reconstruct_time;
		maxReconstructTime = frame_reconstruct_time > maxReconstructTime ? frame_reconstruct_time : maxReconstructTime;

		//output the nearby surfaces
		// PublishMeshs(oResultMeshes);
	};

	std::thread Modeling(ModelingFunction);

	Modeling.detach();
}

