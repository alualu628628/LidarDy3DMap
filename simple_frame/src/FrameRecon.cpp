#include "FrameRecon.h"
#include <cstddef>

FrameRecon::FrameRecon(ros::NodeHandle &node, ros::NodeHandle &nodeHandle)
    : trajFrameNum(0), averageReconstructTime(0), reconstructFrameNum(0), maxReconstructTime(0), node(node),
      nodeHandle(nodeHandle) {}

void FrameRecon::LazyLoading() {
    std::cout << "Load GHPR Reconstruction..." << std::endl;

    // read parameters
    ReadLaunchParams(nodeHandle);
    //***subscriber related***
    // 记录运动信息到 odomHistory 的循环数组之中，并且 trajCount++
    odomSuber = nodeHandle.subscribe(subOdomTopic, 2, &FrameRecon::HandleTrajectory, this);
    // 在mapPCN记录法向点集，发布Mesh主题
    cloudSuber = nodeHandle.subscribe(subCloudTopic, 1, &FrameRecon::HandlePointClouds, this);

    //***publisher related***
    // publish point cloud after processing
    FramePointsNormalPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(pubFramePointsNormalTopic, 1, true);
    AdditionalPointsPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(pubAdditionalPointsTopic, 1, true);
    // publish polygon constructed from one frame point cloud
    // 在接受到点云重建完之后， 被 PublishMeshs() 函数调用
    FrameMeshPublisher = nodeHandle.advertise<visualization_msgs::Marker>(pubFrameMeshTopic, 1, true);
    FrameMeshAlgoPublisher = nodeHandle.advertise<fusion_msgs::MeshArray>(pubFrameMeshAlgoTopic, 1, true);
    // 用于后续处理的网格可视化消息
    FrameMeshAlgoViewPublisher = nodeHandle.advertise<visualization_msgs::Marker>(pubFrameMeshAlgoViewTopic, 1, true);
    viewPointPublisher = nodeHandle.advertise<geometry_msgs::PointStamped>("/frame_recon/viewpoint", 1, true);
    odomPublisher = nodeHandle.advertise<nav_msgs::Odometry>("/frame_recon/robotOdom", 1, true);
    // debug
	FrameDebugPointsPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("/frame_recon/debug_points", 1, true);
}

FrameRecon::~FrameRecon() {
    std::cout << std::format_yellow << "Reconstructed frame numbers: " << reconstructFrameNum
              << ";\tTotal frame numbers : " << totalFrameNum << std::endl
              << "Average recontime per frame: " << averageReconstructTime / reconstructFrameNum << "ms"
              << ";\t Max frame time: " << maxReconstructTime << "ms" << std::format_white << std::endl;

    // /* output times
    if (outputFiles) {
        std::stringstream sEvalFileName;
        if (sfOutFileHead.back() != '/')
            sfOutFileHead += "/";
        sEvalFileName << sfOutFileHead << "AlgoTime.csv";
        std::cout << "The output eval file is " << sEvalFileName.str() << std::endl;
        timer.OutputDebug(sEvalFileName.str());
    }
    //*/
}

bool FrameRecon::ReadLaunchParams(ros::NodeHandle &nodeHandle) {
    // output file name
    nodeHandle.param("sf_output_path", sfOutFileHead, std::string(""));
    outputFiles = !sfOutFileHead.empty();

    // 要存的文件： 单帧带法向点云，单帧网格（原始网格、剔除伪面的网格），原始点云
    if(outputFiles){
        outputPathList["sf_pc"] = CheckOutputPath(sfOutFileHead, "sf_pc"); 
        outputPathList["sf_pc_normal"] = CheckOutputPath(sfOutFileHead, "sf_pc_normal");
        outputPathList["sf_mesh"] = CheckOutputPath(sfOutFileHead, "sf_mesh");
        outputPathList["sf_mesh_algo"] = CheckOutputPath(sfOutFileHead, "sf_mesh_algo"); 
    }

    // frame id
    nodeHandle.param("lidar_frame_id", lidarFrameId, std::string("lidar_sensor_VLP16"));
    nodeHandle.param("base_link_frame_id", baselinkFrameId, std::string("base_link"));
    nodeHandle.param("odom_frame_id", odomFrameId, std::string("odom"));

    // Lidar Type
    nodeHandle.param("lidar_type", lidarType, std::string("VLP16"));
    if (lidarType == "VLP16") {

        lidarLineMin = 0;
        lidarLineMax = 15;
		LOG(INFO) << std::format_yellow << "LIDAR TYPR [VLP16] MinLine: 0 MaxLine: 15" << std::format_white;
    } else if (lidarType == "Pandar40") {
        lidarLineMin = 0;
        lidarLineMax = 39;
		LOG(INFO) << std::format_yellow << "LIDAR TYPR [Pandar40] MinLine: 0 MaxLine: 39" << std::format_white;
    } else {
        nodeHandle.param("lidar_line_min", lidarLineMin, 0);
        nodeHandle.param("lidar_line_max", lidarLineMax, 15);
    }

    // input odom topic
    nodeHandle.param("sub_odom_topic", subOdomTopic, std::string("/odometry/filtered"));
    nodeHandle.param("sub_cloud_topic", subCloudTopic, std::string("/cloud_points"));

    nodeHandle.param("pub_cloud_normal_topic", pubFramePointsNormalTopic,
                     std::string("/frame_recon/frame_cloudnormals"));
    nodeHandle.param("pub_additional_points_topic", pubAdditionalPointsTopic,
                     std::string("/frame_recon/additionalPoints"));
    nodeHandle.param("pub_frame_mesh_topic", pubFrameMeshTopic, std::string("/frame_recon/frame_meshs"));
    nodeHandle.param("pub_frame_mesh_algo_topic", pubFrameMeshAlgoTopic, std::string("/frame_recon/frame_mesh_algo"));
    nodeHandle.param("pub_frame_mesh_alog_view_topic", pubFrameMeshAlgoViewTopic,
                     std::string("/frame_recon/frame_mesh_algo_view"));

    // point cloud sampling number
    nodeHandle.param("sample_pcframe_num", frameSmpNum, 1);
    // point cloud sampling number
    nodeHandle.param("sample_inputpoints_num", sampleInPNum, 1);

    // explicit reconstruction related
    // number of sectors
    nodeHandle.param("sector_num", sectorNum, 1);
    meshAlgoBuilder.HorizontalSectorSize(sectorNum);
    // height of viewpoint
    nodeHandle.param("viewp_zoffset", viewZOffset, 0.0f);

	bool bMultiThread;
	nodeHandle.param("multi_thread", bMultiThread, true);
	meshAlgoBuilder.SetMultiThread(bMultiThread);
	bool useCgalBackend;
	nodeHandle.param("use_cgal_backend", useCgalBackend, true);
	meshAlgoBuilder.SetUseCgalBackend(useCgalBackend);
	int ghprWorkerThreads;
	nodeHandle.param("ghpr_worker_threads", ghprWorkerThreads, 0);
	meshAlgoBuilder.SetWorkerCount(ghprWorkerThreads);

    nodeHandle.param("is_debug", isDebug, false);

    // count processed point cloud frame
    pcFrameCount = 0;

    // count processed odom frame
    trajCount = 0;

    // true indicates the file has not been generated
    outPCFileFlag = true;

    return true;
}

void FrameRecon::HandlePointClouds(const sensor_msgs::PointCloud2 &vLaserData) {
    if (!(pcFrameCount % frameSmpNum)) { // 根据帧采样频率记录
        totalFrameNum = vLaserData.header.seq + 1;

        // struct timeval start;
        // gettimeofday(&start, NULL);
        timer.NewLine();
        // ring字段转移到intensity字段
        // 存在ring字段的情况下，intensity字段不会被初始化
        // 不存在ring字段的情况下，intensity字段会被初始化
        // 上节点算法将ring存在intensity字段中，因此需要将ring字段转移到intensity字段
        bool haveIntensity = false;
        bool haveRing = false;
        for (const auto &field : vLaserData.fields) {
            if (field.name == "intensity") { // 找到 intensity 字段
                haveIntensity = true;
            }
            if (field.name == "ring") { // 找到 ring 字段
                haveRing = true;
            }
        }
        // a point clouds in PCL type
        pcl::PointCloud<pcl::PointXYZI>::Ptr pRawCloud(new pcl::PointCloud<pcl::PointXYZI>);

        if (haveRing) {
            if (lidarType == "VLP16") {
                pcl::PointCloud<LiDarPointType::Velodyne16>::Ptr pXYZRCloud(new pcl::PointCloud<LiDarPointType::Velodyne16>);
                pcl::fromROSMsg(vLaserData, *pXYZRCloud);
                for (int i = 0; i < pXYZRCloud->points.size(); i++) {
                    pcl::PointXYZI point;
                    point.x = pXYZRCloud->points[i].x;
                    point.y = pXYZRCloud->points[i].y;
                    point.z = pXYZRCloud->points[i].z;
                    point.intensity = pXYZRCloud->points[i].ring;
                    pRawCloud->points.push_back(point);
                }
            } else if (lidarType == "Pandar40") {
                pcl::PointCloud<LiDarPointType::HesaiPander40>::Ptr pXYZRCloud(new pcl::PointCloud<LiDarPointType::HesaiPander40>);
                pcl::fromROSMsg(vLaserData, *pXYZRCloud);
                for (int i = 0; i < pXYZRCloud->points.size(); i++) {
                    pcl::PointXYZI point;
                    point.x = pXYZRCloud->points[i].x;
                    point.y = pXYZRCloud->points[i].y;
                    point.z = pXYZRCloud->points[i].z;
                    point.intensity = pXYZRCloud->points[i].ring;
                    pRawCloud->points.push_back(point);
                }
            } else {
                LOG_FIRST_N(WARNING,1) << "Unknown point cloud data type with ring field, failed to parse ring field, set to 2";
				pcl::PointCloud<pcl::PointXYZ>::Ptr pXYZCloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::fromROSMsg(vLaserData, *pXYZCloud);
				for (int i = 0; i < pXYZCloud->points.size(); i++) {
					pcl::PointXYZI point;
					point.x = pXYZCloud->points[i].x;
					point.y = pXYZCloud->points[i].y;
					point.z = pXYZCloud->points[i].z;
					point.intensity = 2;
					pRawCloud->points.push_back(point);
            	}	
            }
        } else {
			LOG_FIRST_N(WARNING,1) << "Unknown LIDAR type , ring field set to 2";
            pcl::PointCloud<pcl::PointXYZ>::Ptr pXYZCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(vLaserData, *pXYZCloud);
            for (int i = 0; i < pXYZCloud->points.size(); i++) {
                pcl::PointXYZI point;
                point.x = pXYZCloud->points[i].x;
                point.y = pXYZCloud->points[i].y;
                point.z = pXYZCloud->points[i].z;
                point.intensity = 2;
                pRawCloud->points.push_back(point);
            }
        }
        if(baselinkFrameId != lidarFrameId) {// base_link 和 lidar frame 不重合 转换到lidar坐标系下
            tf::StampedTransform base_link2lidar_transform;
            try {
                listener.lookupTransform(baselinkFrameId, lidarFrameId, ros::Time(0), base_link2lidar_transform);
            } catch (tf::TransformException &ex) {
                LOG(ERROR) << "[FrameRecon HandleTrajectory]Getting TF timestamp error for ["<<baselinkFrameId.c_str()<<"] to ["<<lidarFrameId.c_str()<<"}] (" << ex.what() << ")";
            }
                Eigen::Matrix4f eigen_transform;
                pcl_ros::transformAsMatrix(base_link2lidar_transform, eigen_transform);

                // 应用变换到点云
                pcl::transformPointCloud(*pRawCloud, *pRawCloud, eigen_transform);
        }


        if (vLaserData.header.frame_id != odomFrameId) {
            tf::StampedTransform transform;
            try {
                listener.lookupTransform(odomFrameId, vLaserData.header.frame_id, ros::Time(0), transform);
            } catch (tf::TransformException &ex) {
                // ROS_WARN("Getting TF timestamp error for [%s] to [%s] (%s)",
                //          vLaserData.header.frame_id.c_str(), ex.what());
                LOG(ERROR)<< "[FrameRecon HandlePointClouds]Getting TF timestamp error for ["<<vLaserData.header.frame_id.c_str()<<"] to ["<<odomFrameId.c_str()<<"] ("<< ex.what()<<")";
				return;
            }

            // 将 TF 变换转换为 Eigen 格式以便于 PCL 使用
            Eigen::Matrix4f eigen_transform;
            pcl_ros::transformAsMatrix(transform, eigen_transform);

            // 应用变换到点云
            pcl::transformPointCloud(*pRawCloud, *pRawCloud, eigen_transform);
        }

        timer.DebugTime("1_transfer_cloud");

        // if have corresponding trajectory point (viewpoint)
        pcl::PointXYZI oCurrentViewP;

        if (odomHistory.size() /*&& vLaserData.header.stamp <= odomHistory.last().oTimeStamp*/) {
            // 当前点云对应的观测位置（Odom与frame并非一一对应，因此需要计算插值）
            oCurrentViewP = ComputeQueryTraj(vLaserData.header.stamp);

            // else waiting for sync
        } else {
            std::cout << std::format_red << " Error: No odom matched!" << std::format_white << std::endl;
            return;
        }
		// 
		geometry_msgs::PointStamped oCurrGoalOdom;
		oCurrGoalOdom.header.stamp = ros::Time::now();
		oCurrGoalOdom.header.frame_id = "odom";

		// set the position
		oCurrGoalOdom.point.x = oCurrentViewP.x;
		oCurrGoalOdom.point.y = oCurrentViewP.y;
		oCurrGoalOdom.point.z = oCurrentViewP.z;


		viewPointPublisher.publish(oCurrGoalOdom);

        timer.DebugTime("2_get_viewpoint");

        // point sample
        pcl::PointCloud<pcl::PointXYZI>::Ptr pSceneCloud(new pcl::PointCloud<pcl::PointXYZI>);
		// if points size > 20000 ==> sample point
		if(pRawCloud->points.size() > 30000){
			SamplePoints(*pRawCloud, *pSceneCloud, sampleInPNum, true);
		}
		else{
			*pSceneCloud = *pRawCloud;
		}
        timer.DebugTime("3_sample_points");

        if (outputFiles) { // 输出采样点云
            std::stringstream filename;
            filename << outputPathList["sf_pc"]<< "sf_" << std::setw(4) << std::setfill('0') << reconstructFrameNum << "_pc.ply";

            std::thread tOutputThread([=](string path) { pcl::io::savePLYFileASCII(path, *pSceneCloud); },
                                      filename.str());
            tOutputThread.detach();
        }
        //*/
		// PublishPointCloud(*pSceneCloud,FrameDebugPointsPublisher);
        timer.DebugTime("4_save_sampled_pc");

        // frame reconstruct
        // struct timeval reconstruct_start;
        // gettimeofday(&reconstruct_start, NULL);
        pcl::PointCloud<pcl::PointNormal>::Ptr pFramePNormal(new pcl::PointCloud<pcl::PointNormal>);

        meshAlgoBuilder.setWorkingFrameCount(pcFrameCount);
        meshAlgoBuilder.SetViewPoint(oCurrentViewP, viewZOffset);
        meshAlgoBuilder.OriginalAndShowReconstruction(*pSceneCloud, *pFramePNormal, lidarLineMin, lidarLineMax); // 得到带权重的mesh,和可视化mesh

        // 添加中心视点，方便多帧进程识别
        pcl::PointNormal oViewPoint;
        oViewPoint.getVector3fMap() = oCurrentViewP.getVector3fMap();
        oViewPoint.curvature = -1; // 识别码
        pFramePNormal->push_back(oViewPoint);

        double parallel_time = timer.DebugTime("5_main_reconstruction");

        // publish
        PublishFramePointsNormal(*pFramePNormal);
        PublishFrameMesh();
        PublishFrameMeshAlgorithm();
        PublishFrameMeshAlgorithmView();

        timer.DebugTime("6_publish");

        if (outputFiles) {
            // output normaled pc
            std::stringstream sPcOutputPath;
            sPcOutputPath << outputPathList["sf_pc_normal"]<< "sf_" << std::setw(4) << std::setfill('0') << reconstructFrameNum << "_npc.ply";

            // output mesh
            pcl::PolygonMesh oSigMesh;
            pcl::PointCloud<pcl::PointXYZI> vSigCloud;
            for (int sector_index = 0; sector_index < meshAlgoBuilder.m_vAllShowClouds.size(); ++sector_index) {
                for (int face_index = 0; face_index < meshAlgoBuilder.m_vAllShowFaces[sector_index].size();
                     ++face_index) {
                    pcl::Vertices oCurrentFaceSig;
                    oCurrentFaceSig.vertices.push_back(
                        meshAlgoBuilder.m_vAllShowFaces[sector_index][face_index].vertices[0] + vSigCloud.size());
                    oCurrentFaceSig.vertices.push_back(
                        meshAlgoBuilder.m_vAllShowFaces[sector_index][face_index].vertices[1] + vSigCloud.size());
                    oCurrentFaceSig.vertices.push_back(
                        meshAlgoBuilder.m_vAllShowFaces[sector_index][face_index].vertices[2] + vSigCloud.size());
                    oSigMesh.polygons.push_back(oCurrentFaceSig);
                }
                vSigCloud += *meshAlgoBuilder.m_vAllShowClouds[sector_index];
            }

            // output the mesh
            pcl::toPCLPointCloud2(vSigCloud, oSigMesh.cloud);
            std::stringstream sSigOutputPath;
            sSigOutputPath << outputPathList["sf_mesh"] << "sf_" << std::setw(4) << std::setfill('0') << reconstructFrameNum << "_mesh.ply";

            pcl::PolygonMesh oFullMesh;
            pcl::PointCloud<pcl::PointXYZI> vFullCloud;
            for (int sector_index = 0; sector_index < meshAlgoBuilder.m_vAllSectorClouds.size(); ++sector_index) {
                for (int face_index = 0; face_index < meshAlgoBuilder.m_vAllSectorFaces[sector_index].size();
                     ++face_index) {
                    pcl::Vertices oCurrentFace;
                    oCurrentFace.vertices.push_back(
                        meshAlgoBuilder.m_vAllSectorFaces[sector_index][face_index].vertices[0] + vFullCloud.size());
                    oCurrentFace.vertices.push_back(
                        meshAlgoBuilder.m_vAllSectorFaces[sector_index][face_index].vertices[1] + vFullCloud.size());
                    oCurrentFace.vertices.push_back(
                        meshAlgoBuilder.m_vAllSectorFaces[sector_index][face_index].vertices[2] + vFullCloud.size());
                    oFullMesh.polygons.push_back(oCurrentFace);
                }
                vFullCloud += *meshAlgoBuilder.m_vAllSectorClouds[sector_index];
            }
            pcl::toPCLPointCloud2(vFullCloud, oFullMesh.cloud);
            std::stringstream sMeshOutputPath;
            sMeshOutputPath << outputPathList["sf_mesh_algo"]<< "sf_" << std::setw(4) << std::setfill('0') << reconstructFrameNum
                            << "_Algo_mesh.ply";

            std::thread tOutputThread( 
                [=](string pc_path, string mesh_path, string sig_path) {
                    pcl::io::savePLYFileASCII(pc_path, *pFramePNormal);
                    pcl::io::savePLYFileBinary(mesh_path, oFullMesh);
                    pcl::io::savePLYFileBinary(sig_path, oSigMesh);
                },
                sPcOutputPath.str(), sMeshOutputPath.str(), sSigOutputPath.str());

            tOutputThread.detach();
            timer.DebugTime("7_output_file");
        }
        // 结束算法计时并记录执行时间
        timer.DebugTime("8_clear_data");
        double frame_reconstruct_time = timer.GetCurrentLineTime();
        if (isDebug) {
            std::cout << "Now frame count is: " << pcFrameCount << ";\t"
                      << "header is: {" << vLaserData.header << "}";
            std::cout << ";\tsize: " << pRawCloud->size();
            std::cout << "\trecon_time:" << parallel_time << "ms";
            std::cout << ";\tframe_time:" << frame_reconstruct_time << "ms" << std::endl;
        }
        averageReconstructTime += frame_reconstruct_time;
        maxReconstructTime = frame_reconstruct_time > maxReconstructTime ? frame_reconstruct_time : maxReconstructTime;
        ++reconstructFrameNum;
    }

    // count
    pcFrameCount++;

    return;
}

void FrameRecon::HandleTrajectory(const nav_msgs::Odometry &oTrajectory) {
    if (isDebug) {
        std::cout << std::format_yellow << "Now Odome count is: " << trajCount << ";\t"
                  << "header is: {" << oTrajectory.header << "}" << "\tPose: (" << oTrajectory.pose.pose.position.x
                  << "," << oTrajectory.pose.pose.position.y << "," << oTrajectory.pose.pose.position.z << ")"
                  << std::format_white << std::endl;
    }

    // 无论位姿信息是否在odom坐标系下，考虑两种情况
    // 1、base_link 和 lidar frame 重合
    // 2、base_link 和 lidar frame 不重合
    // 重建视点一定是雷达的位置，无论在哪个坐标系下，都需要将雷达的位置转换到odom坐标系下

    // 从位姿中获取视点
    tf::Transform transform;
    // 初始化为零平移和零旋转
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));   // 平移 (x=0, y=0, z=0)
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0)); // 单位四元数表示零旋转
    if(baselinkFrameId != lidarFrameId) {// base_link 和 lidar frame 不重合 转换到lidar坐标系下
        tf::StampedTransform base_link2lidar_transform;
        try {
            listener.lookupTransform(baselinkFrameId, lidarFrameId, ros::Time(0), base_link2lidar_transform);
        } catch (tf::TransformException &ex) {
            LOG(ERROR) << "[FrameRecon HandleTrajectory]Getting TF timestamp error for ["<<baselinkFrameId.c_str()<<"] to ["<<lidarFrameId.c_str()<<"}] (" << ex.what() << ")";
        }
        transform = base_link2lidar_transform * tf::StampedTransform(transform, oTrajectory.header.stamp,baselinkFrameId, lidarFrameId);
    }

    // LOG(ERROR)<< "[FrameRecon HandleTrajectory] ["<<oTrajectory.header.frame_id.c_str()<<"] to ["<<odomFrameId.c_str()<<"] ";
    if(oTrajectory.header.frame_id != odomFrameId){ // 位姿信息不在odom坐标系下 转换到odom坐标系下
        tf::StampedTransform msg_frame2odom_transform;
        try {
            listener.lookupTransform( odomFrameId, oTrajectory.header.frame_id, ros::Time(0), msg_frame2odom_transform);
             
        } catch (tf::TransformException &ex) {
            LOG(ERROR)<< "[FrameRecon HandleTrajectory]Getting TF timestamp error for ["<<oTrajectory.header.frame_id.c_str()<<"] to ["<<odomFrameId.c_str()<<"] ("<< ex.what()<<")";
        }
        transform = msg_frame2odom_transform * tf::StampedTransform(transform, oTrajectory.header.stamp,oTrajectory.header.frame_id, odomFrameId);
    }

    // 将 TF 变换转换为 Eigen 格式以便于 PCL 使用
    // 从 tf::StampedTransform 获取旋转和位移信息
    tf::Quaternion tf_q = transform.getRotation();
    tf::Vector3 tf_t = transform.getOrigin();

    // 将 tf::Quaternion 转换为 Eigen::Quaternionf
    Eigen::Quaternionf eigen_q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());

    // 将 tf::Vector3 转换为 Eigen::Vector3f
    Eigen::Vector3f eigen_t(tf_t.x(), tf_t.y(), tf_t.z());

    // 创建 Eigen::Affine3f 变换，并设置平移和旋转
    Eigen::Affine3f eigen_transform = Eigen::Affine3f::Identity();
    eigen_transform.translate(eigen_t); // 应用平移
    // eigen_transform.rotate(eigen_q);    // 应用旋转

    // save the into the memory
    // save the position of trajectory
    RosTimePoint oOdomPoint;
    oOdomPoint.oLocation.x = oTrajectory.pose.pose.position.x;
    oOdomPoint.oLocation.y = oTrajectory.pose.pose.position.y;
    oOdomPoint.oLocation.z = oTrajectory.pose.pose.position.z;
    // 应用变换到点
    oOdomPoint.oLocation = pcl::transformPoint(oOdomPoint.oLocation, eigen_transform);
    // oTrajectory.twist.twist.angular/linear 表示角速度和线速度
    /**
     * 一. odom和imu传感器位姿信息输入
    odom传感器数据格式: odom( x, y, z, roll,pitch,yaw):
    其中,x,y作为智能车在平面地图的x和y坐标;z坐标忽略恒等于0;
            roll(车身纵向翻滚角)和pitch(车身俯仰角)也不做考虑恒等于0.
    yaw(偏航角)作为智能车在平面地图上左右转向方向角. 即: odom( x, y, 0,
    0,0,yaw),共6个测量值数据. 此外odom传感器还提供自身的噪音协方差矩阵,为一个
    6x6矩阵.

    imu惯性传感器数据格式: imu(roll,pitch,yaw):
            其中roll(车身纵向翻滚角)和pitch(车身俯仰角)也不做考虑恒等于0.
    yaw(偏航角)作为智能车在平面地图上左右转向方向角. 即: imu(
    0,0,yaw),共3个测量值数据. 此外imu传感器还提供自身的噪音协方差矩阵,为一个
    3x3矩阵.

            关于利用噪音协方差矩阵进行位姿的矫正，或许可以从“卡尔曼滤波EKF”入手进行研究
            这里不考虑该因素，aloam 的 /slam_odom 中也只有 pose 和 orientation,
    因此可以忽略其存在。
    */
    // 至于把点反投影到当前坐标系中，或许不需要雷达的旋转位姿，毕竟是360度全角度扫描，可以直接在世界坐标系的平移系下作投影。

    // save record time
    oOdomPoint.oTimeStamp = oTrajectory.header.stamp;

    // add to trajectory array
    odomHistory.push(oOdomPoint);

    trajCount++;
    
    //odom publish
    nav_msgs::Odometry robotOdom;
    
    robotOdom.header.stamp = ros::Time::now();
    robotOdom.header.frame_id = odomFrameId;

    robotOdom.pose.pose.position.x = oTrajectory.pose.pose.position.x;
    robotOdom.pose.pose.position.y = oTrajectory.pose.pose.position.y;
    robotOdom.pose.pose.position.z = oTrajectory.pose.pose.position.z;
    robotOdom.pose.pose.orientation = oTrajectory.pose.pose.orientation;

    odomPublisher.publish(robotOdom);
}

void FrameRecon::InterpolateTraj(const RosTimePoint &oCurrent, const RosTimePoint &oPast, const float &fRatio,
                                 pcl::PointXYZI &oInter) {
    // The ratio is from the interpolated value to oCurrent value
    // Complementary ratio
    float fCompRatio = 1 - fRatio;
    // p+(c-p)(1-r)
    oInter.x = oCurrent.oLocation.x * fCompRatio + oPast.oLocation.x * fRatio;
    oInter.y = oCurrent.oLocation.y * fCompRatio + oPast.oLocation.y * fRatio;
    oInter.z = oCurrent.oLocation.z * fCompRatio + oPast.oLocation.z * fRatio;
}

pcl::PointXYZI FrameRecon::ComputeQueryTraj(const ros::Time &oQueryTime) {
    pcl::PointXYZI oResTraj;
    // clear the output
    oResTraj.x = 0.0;
    oResTraj.y = 0.0;
    oResTraj.z = 0.0;
    // index
    int iTrajIdx = 0;
    // time different
    double timeDiff = (oQueryTime - odomHistory[iTrajIdx].oTimeStamp).toSec();
    // search the most recent time
    while (iTrajIdx < odomHistory.size() - 1 && timeDiff > 0) {
        // increase index
        iTrajIdx++;
        // time different
        timeDiff = (oQueryTime - odomHistory[iTrajIdx].oTimeStamp).toSec();
    }

    // if the querytime is out of the stored time section
    if (iTrajIdx == 0 || timeDiff > 0) {
        // turn back zero
        oResTraj.x = odomHistory[iTrajIdx].oLocation.x;
        oResTraj.y = odomHistory[iTrajIdx].oLocation.y;
        oResTraj.z = odomHistory[iTrajIdx].oLocation.z;

    } else { // if it is between two stored times
        // get the ratio
        // ROS_INFO("Trajtime between: %f and %f",
        // odomHistory[iTrajIdx].oTimeStamp.toSec(), odomHistory[iTrajIdx -
        // 1].oTimeStamp.toSec());

        float ratio = -timeDiff / (odomHistory[iTrajIdx].oTimeStamp - odomHistory[iTrajIdx - 1].oTimeStamp).toSec();
        // interpolate an accuracy value
        InterpolateTraj(odomHistory[iTrajIdx], odomHistory[iTrajIdx - 1], ratio, oResTraj);
    }

    return oResTraj;
}

void FrameRecon::SamplePoints(const pcl::PointCloud<pcl::PointXYZI> &vCloud, pcl::PointCloud<pcl::PointXYZI> &vNewCloud,
                              int iSampleNum, bool bIntervalSamp) {
    vNewCloud.clear();

	int ring[lidarLineMax + 1] = {0}; 
    // sample by interval number
    if (bIntervalSamp) { // 在点云环上采样
        // std::uniform_int_distribution <int> irandom(1, iSampleNum);
        // std::default_random_engine e(time(0));
        for (int i = 0; i < vCloud.points.size(); i++){
            // if(irandom(e) == 1)
			if(ring[int(vCloud.points[i].intensity)] >= iSampleNum){
            	vNewCloud.push_back(vCloud.points[i]);
				ring[int(vCloud.points[i].intensity)] = 0;
			}
			else{
				ring[int(vCloud.points[i].intensity)]++;
			}
		}
        // over the function and output
        return;

    } // end if

    // Sampling according to the given maximum total number
    // get the interval point number - how muny points should be skipped
    int iInterval = std::ceil(float(vCloud.points.size()) / float(iSampleNum));
    // sample
    for (int i = 0; i < vCloud.points.size(); i = i + iInterval)
        vNewCloud.push_back(vCloud.points[i]);

    // output
    return;
}
