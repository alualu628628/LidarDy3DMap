#ifndef FramesFusion_H
#define FramesFusion_H

#include <string>
#include <ctime>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <mutex>
#include <memory>
#include <filesystem>

#include <thread>

#include <glog/logging.h>

//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <shape_msgs/Mesh.h>
#include <fusion_msgs/MeshArray.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

//pcl related
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>         
#include <pcl/io/ply_io.h>         
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

//project related
#include "CircularVector.h"
#include "SignedDistance.h"
#include "CIsoSurface.h"
#include "MeshOperation.h"

#include "volume/VolumeBase.h"
#include "volume/HashBlock.h"
#include "volume/HashVoxeler.h"

#include "tools/CloudVector.h"
#include "tools/OutputUtils.h"
#include "tools/DebugManager.h"
#include "tools/RosPublishManager.h"
#include "tools/BufferLimitThreadPool.h"

#include "updater/ProjectUpdater.h"
#include "updater/RayUpdater.h"
#include "updater/MeshUpdater.h"

// Trajectory state data. 
struct RosTimePoint{

    // The time of the measurement leading to this state (in seconds). //
    ros::Time oTimeStamp;

    // The global trajectory position in 3D space. 
    pcl::PointXYZ oLocation;

};

pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(pcl::PointCloud<pcl::PointNormal>& cloud_vector);

class FramesFusion{

public:
    // in FramesFusion.cpp: FramesFusion Ros
    //Constructor
    FramesFusion(ros::NodeHandle & node, ros::NodeHandle & nodeHandle);
    virtual void LazyLoading();
    //Destructor
    virtual ~FramesFusion();
    //Reads and verifies the ROS parameters.
    bool ReadLaunchParams(ros::NodeHandle & nodeHandle);  
    void HandleMesh(const fusion_msgs::MeshArray & vMeshRosData);
    void HandleCloud(const sensor_msgs::PointCloud2 & vCloud);
    //handle the trajectory information
    void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);
    //multiple thread version
    void HandleTrajectoryThread(const nav_msgs::Odometry & oTrajectory);


    // in FramesFusionFusionRecon.cpp: FusionRecon
    float EuclideanDistance(const pcl::PointXYZ & oBasedP, const pcl::PointNormal & oTargetP);
    // Build surface models based on new points that received from ros
    virtual void SlideModeling(pcl::PolygonMesh & oResultMesh, const Eigen::Vector3f& vCenter, const int iFrameId);
    //get the nearby point clouds
    void NearbyClouds(const pcl::PointCloud<pcl::PointNormal> & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);
    void NearbyClouds(CloudVector & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);
    
    //get the nearby point clouds and delete the geted point in rawCloud
    void ExtractNearbyClouds(pcl::PointCloud<pcl::PointNormal> & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);
    void ExtractNearbyClouds(CloudVector & pRawCloud, const pcl::PointXYZ & oBasedP, pcl::PointCloud<pcl::PointNormal> & pNearCloud, float fLength);


    // in FramesFusionHotMap.cpp: hot map functions
    void GenerateHotMap(const Eigen::Vector3f& vCenter, const int iFrameId, const ros::Time oTimeStamp);
    void ExtractSdfMaps(const Eigen::Vector3f& vCenter, const int iFrameId, const ros::Time oTimeStamp);


    // in FramesFusionPublish.cpp: 发布点云、mesh
    void PublishMeshs(const pcl::PolygonMesh & oMeshModel);
    void PublishPointCloud(const pcl::PointCloud<pcl::PointNormal> & vCloudNormal);
    void PublishDynamicCloud(const pcl::PointCloud<pcl::PointNormal> & vDynamicPoints);
    void PublishStaticCloud(const pcl::PointCloud<pcl::PointNormal> & vStaticPoints);
    void PublishFreeSpace(const pcl::PointCloud<pcl::DistanceIoVoxel> & locale_free_space, const pcl::PointCloud<pcl::DistanceIoVoxel> & global_free_space, bool publishGlobalFreeSpace);

    // in FramesFusionOutFile.cpp: 输出点云、mesh、SDFmap 保存文件 
    void SaveFinalMeshAndPointCloud();
    void SaveVoxels();
    void OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, bool bAllRecord = false);
    void OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const std::vector<float> & vFeatures, bool bAllRecord = false);
    void OutputSdfMapImage(const Eigen::Vector3f& vCenter, grid_map::GridMap& oSdfMap, int iFrameIndex);

protected: 
    // in FramesFusion.cpp: FramesFusion Ros
    virtual void UpdateOneFrame(const pcl::PointNormal& oViewPoint, pcl::PointCloud<pcl::PointNormal>& vFilteredMeasurementCloud);

    // in FramesFusionFusionRecon.cpp: FusionRecon
    void SurfelFusionCore(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud, pcl::PointCloud<pcl::PointNormal>& vPointCloudBuffer);
    virtual void SurfelFusionQuick(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud);


    //***file related***
    std::string fileHead;
    //wether output evaluate files
    bool outputFiles;
    //full name of output txt that records the point clouds
    std::stringstream outPCFileName; 
    //whether the file is generated or not
    bool outPCFileFlag;
    //ouput file
    std::ofstream outPCFile;

    std::unordered_map<std::string, std::string> outputPathList;
    std::string CheckOutputPath(std::string & outFileHead, const std::string & fileName);



    std::string subCloudTopic;
    ros::Subscriber cloudSuber;
    //the MeshSuber subscirber is to hear input mesh topic
    std::string subMeshTopic; 
    ros::Subscriber meshSuber;
    //the odomSuber subscirber is to hearinput  odometry topic
    std::string subOdomTopic;
    ros::Subscriber odomSuber;

    //publish point cloud topic
    std::string pubCloudTopic;
    //point cloud publisher for test
    ros::Publisher CloudPublisher;

    // output dynamic point cloud
    // 发布内容：静态点云、动态点云  
    ros::Publisher DynamicCloudPublisher;
    ros::Publisher StaticCloudPublisher;

    ros::Publisher localeFreeSpacePublisher;
    ros::Publisher globalFreeSpacePublisher;

    //***for output mesh***
    //output point cloud topic
    std::string pubMeshTopic;
    ros::Publisher MeshPublisher;


    //***for output gridmap***
    std::string pubHotMapTopic;
    ros::Publisher HotMapPublisher;

    std::string odomFrameId = "odom";

    //nearby length
    float nearLengths;

    //voxel resolution
    pcl::PointXYZ voxelResolution;

    //frame sampling
    int frameSmplingNum;

    //sampling number of input point clouds
    int inputCloudSamlingNum;

    //nearby mesh update period
    float nearMeshPeriod;


    //circle vector of odom
    ros::Time lastModelingTime;

    //frame count
    unsigned int odomCount;

    //map point clouds with normals
    //accumulated processed point cloud
    // std::mutex m_mPCNMutex;
    pcl::PointCloud<pcl::PointNormal> mapPCN;
    pcl::PointCloud<pcl::PointNormal> mapPCNAdded;
    pcl::PointCloud<pcl::PointNormal> mapPCNTrueAdded;

    bool useAdditionalPoints;

    //features of map point clouds
    //Features can be specified
    std::vector<float> mapPCFeas;

    // Reconstruct time statics
    double averageReconstructTime;
    double maxReconstructTime;
    int reconstructFrameNum;

    // data associate time statics
    double averageFusionTime;
    double maxFusionTime;
    int fusionFrameNum;
    std::unique_ptr<Tools::ThreadPool> fusionThreadPool;

    // static extract time statics
    double averageExtractTimel;
    double maxExtractTime;
    int pcFrameCount;

    // data association switch
    bool surfelFusion;
    // viewpoint and current frame for surfel fusion
    // void SurfelFusion(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud);
    // more strict when filtering the points
    // void AddedSurfelFusion(pcl::PointNormal oLidarPos, pcl::PointCloud<pcl::PointNormal>& vDepthMeasurementCloud);
    // viewpoint and current frame for surfel fusion - multi-thread
    ProjectUpdater& projectUpdater;
    RayUpdater& rayUpdater;
    Updater::MeshUpdater& meshUpdater;
    RosPublishManager& rosPubManager;

    bool asyncReconstruction;

    // simple to publish in a new topic
    ros::NodeHandle& m_oGlobalNode;
    ros::NodeHandle& m_oNodeHandle;

    ros::Rate odomLoopRate;

    std::unique_ptr<VolumeBase> distanceIoVolume;
    HashVoxeler hashVoxeler;

    // meshing params
	int keepTime;
	int convDim;
	int convAddPointNumRef;
	float convFusionDistanceRef1;

    // use union set to judge connection
    bool useUnionSetConnection;
    bool onlyMaxUnionSet;
    float strictDotRef;
    float softDotRef;
    int removeSizeRef;
    float removeTimeRef;

    bool dynamicDebug;
    bool centerBasedRecon;
    bool keepVoxel;
    bool publishGlobalFreeSpace;
    float confidenceLevelLength;

    // // sdf
    // SignedDistance* m_pSdf;
    // std::mutex m_mSdfMutex;
    float reconstructRange;

    TimeDebugger fuse_timer;
    TimeDebugger dynamic_timer;
    TimeDebuggerThread reconstruct_timer;
};

#endif
