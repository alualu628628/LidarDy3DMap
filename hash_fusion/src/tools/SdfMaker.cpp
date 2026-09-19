#include "tools/SdfMaker.h"

#include <ros/ros.h>
#include "tools/OutputUtils.h"

SdfMaker::SdfMaker()
{
    m_pDevice = InitializeDevice(NULL);
}

SdfMaker::~SdfMaker()
{
    for(auto& [_,scene] : m_vpScene) {
        if(scene != nullptr){
            rtcReleaseScene(scene);
            scene = nullptr;
        }
    }
    rtcReleaseDevice(m_pDevice);
}


// first make scene
void SdfMaker::NewScene(const pcl::PointCloud<pcl::PointXYZI> & vClouds, const std::vector<pcl::Vertices> & vMeshVertices, const int iSectorId) 
{
    // get ref of the scene id 
    RTCScene& scene = m_vpScene[iSectorId];

    // release old scene
    if(scene != nullptr) {
        rtcReleaseScene(scene);
        scene = nullptr;
    }

    // create new scene
    scene = PushSingleMeshToScene(vClouds, vMeshVertices);
}

// second query
void SdfMaker::QuerySdf(const pcl::PointXYZ & oViewPoint, pcl::PointCloud<pcl::DistanceIoVoxel> & vQueryPoints, const int iSectorId)
{
    // get ref of the scene id 
    RTCScene& scene = m_vpScene[iSectorId];

    // check scene
    if(scene == nullptr) {
        ROS_ERROR("The Scene %d not find!", iSectorId);
        return;
    }

    // The bundled Embree runtime is not stable under concurrent packet queries.
    // Keep the critical section narrow: scene construction and the caller's
    // surrounding voxel work can still run in parallel.
    std::lock_guard<std::mutex> lock(m_oIntersectMutex);
    CastRay(scene, oViewPoint, vQueryPoints);
}

void SdfMaker::QueryLos(const pcl::PointXYZ & oViewPoint, pcl::PointCloud<pcl::DistanceIoVoxel> & vQueryPoints, const int iSectorId)
{
    RTCScene& scene = m_vpScene[iSectorId];
    if(scene == nullptr) {
        ROS_ERROR("The Scene %d not find!", iSectorId);
        return;
    }

    // The bundled Embree runtime must not be queried concurrently.  Packet
    // traversal still removes the old three scalar intersections per voxel.
    std::lock_guard<std::mutex> lock(m_oIntersectMutex);
    CastLos(scene, oViewPoint, vQueryPoints);
}

tools::BoundingBox SdfMaker::GetBoundingBox(const int iSectorId) {

    // get ref of the scene id 
    RTCScene& scene = m_vpScene[iSectorId];

    // check scene
    if(scene == nullptr) {
        ROS_ERROR("The Scene %d not find!", iSectorId);
        return tools::BoundingBox();
    }

    RTCBounds oBound;
    rtcGetSceneBounds(scene, &oBound);
    
    tools::BoundingBox oBoundingBox;
    oBoundingBox.GetMinBound() = Eigen::Map<Eigen::Vector3f>((float*)&oBound);
    oBoundingBox.GetMaxBound() = Eigen::Map<Eigen::Vector3f>((float*)(&oBound)+4);
    return oBoundingBox;
}

std::vector<float> SdfMaker::MakeSdf(
    const pcl::PointXYZ & oViewPoint, 
    const pcl::PointCloud<pcl::PointXYZ> & vQueryPoints, 
    const pcl::PointCloud<pcl::PointXYZI>& vClouds, 
    const std::vector<pcl::Vertices>& vMeshVertices) 
{
    m_oTimeDebugger.DebugTime("block");

    RTCScene scene = PushSingleMeshToScene(vClouds, vMeshVertices);
    m_oTimeDebugger.DebugTime("make_scene");

    // ray cast
    std::vector<float> vDis = CastRay(scene, oViewPoint, vQueryPoints);
    m_oTimeDebugger.DebugTime("cast_ray");

    rtcReleaseScene(scene);

    return vDis;
}

void SdfMaker::CoutDetailedTime() 
{
    m_oTimeDebugger.CoutCurrentLine();
}

void SdfMaker::NewTimeRecord() 
{
    m_oTimeDebugger.NewLine();
}

void SdfMaker::ErrorCallback(void* userPtr, enum RTCError error, const char* str) 
{
    ROS_ERROR("Embree Device Error %d: %s\n", error, str);
}

RTCDevice SdfMaker::InitializeDevice(const char* config) 
{
    // create device for embree
    m_pDevice = rtcNewDevice(config);

    // error process
    if (m_pDevice == nullptr)
        ROS_ERROR("error %d: cannot create device\n", rtcGetDeviceError(NULL));
    rtcSetDeviceErrorFunction(m_pDevice, SdfMaker::ErrorCallback, NULL);

    // get and print embree config infos
    ssize_t version = rtcGetDeviceProperty(m_pDevice, RTC_DEVICE_PROPERTY_VERSION);
    ssize_t order04 = rtcGetDeviceProperty(m_pDevice, RTC_DEVICE_PROPERTY_NATIVE_RAY4_SUPPORTED);
    ssize_t order08 = rtcGetDeviceProperty(m_pDevice, RTC_DEVICE_PROPERTY_NATIVE_RAY8_SUPPORTED);
    ssize_t order16 = rtcGetDeviceProperty(m_pDevice, RTC_DEVICE_PROPERTY_NATIVE_RAY16_SUPPORTED);

    ROS_INFO(
        "\nEmbree Version %d.\nIntersectInOrder4/8/16:%s/%s/%s.\n",
        version,
        order04?output::format_yes:output::format_no,
        order08?output::format_yes:output::format_no,
        order16?output::format_yes:output::format_no
    );

    return m_pDevice;
}

RTCScene SdfMaker::PushSingleMeshToScene(const pcl::PointCloud<pcl::PointXYZI> & vClouds, const std::vector<pcl::Vertices> & vMeshVertices)
{
    RTCScene scene = rtcNewScene(m_pDevice);
    
    
    //TODO： 可能的优化项-shared buffer-省去拷贝步骤
    // rtcSetSharedGeometryBuffer
    RTCGeometry geom = rtcNewGeometry(m_pDevice, RTC_GEOMETRY_TYPE_TRIANGLE);

    // rtcSetSceneBuildQuality / rtcSetSceneFlags 选择构建用到的数据结构

    // low quality 很有效的优化，时间从20ms变为10ms左右
    rtcSetSceneBuildQuality(scene, RTC_BUILD_QUALITY_LOW);
    rtcSetSceneFlags(scene, RTC_SCENE_FLAG_DYNAMIC);
    rtcSetGeometryBuildQuality(geom, RTC_BUILD_QUALITY_LOW);

    // high quality 似乎也没有多高质量
    // rtcSetSceneBuildQuality(scene, RTC_BUILD_QUALITY_HIGH);
    // rtcSetSceneFlags(scene, RTC_SCENE_FLAG_ROBUST);

    int iVerticesNum = vClouds.points.size();
    int iMeshNum = vMeshVertices.size();
    Vertex* vertices = (Vertex*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), iVerticesNum);
    Triangle* triangles = (Triangle*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), iMeshNum);

    //copy vertices value
    for (int i = 0; i < iVerticesNum; ++i) {
        vertices[i].x = vClouds.points[i].x;
        vertices[i].y = vClouds.points[i].y;
        vertices[i].z = vClouds.points[i].z;
    }

    //copy indices value
    for (int i = 0; i < iMeshNum; ++i) {
        triangles[i].v0 = vMeshVertices[i].vertices[0];
        triangles[i].v1 = vMeshVertices[i].vertices[1];
        triangles[i].v2 = vMeshVertices[i].vertices[2];
    }

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene);

    return scene;
}

// 8ray 打包，有用但不多
#define RAY_PACKAGE_8
std::vector<float> SdfMaker::CastRay(RTCScene& scene, const pcl::PointXYZ& oViewPoint, const pcl::PointCloud<pcl::PointXYZ>& vQueryPoints)
{
    // Embree receives a non-const arguments pointer.  Keep it local so parallel
    // sector queries never share mutable query state.
    RTCIntersectArguments intersectArguments;
    rtcInitIntersectArguments(&intersectArguments);
    intersectArguments.flags = RTC_RAY_QUERY_FLAG_COHERENT;

    // const params
    constexpr int bit_move = 3;
    constexpr int ray_group_size = 1 << bit_move;
    constexpr int valid_mask = -1;
    constexpr int invalid_mask = 0;
    
    // create result container
    std::vector<float> vHitDis(vQueryPoints.size());
    memset(vHitDis.data(), -1, vHitDis.size() * sizeof(float));

    #ifdef RAY_PACKAGE_8

    // make valid list, to fit the ray group size.
    int iListSize = vQueryPoints.size() >> bit_move << bit_move;
    if(iListSize < vQueryPoints.size()) iListSize += ray_group_size;
    std::vector<int> vValidVector(iListSize);
    int* vValidList = vValidVector.data();
    memset(vValidList, valid_mask, iListSize * sizeof(int));
    for(int i = vQueryPoints.size(); i < iListSize; ++i) vValidList[i] = invalid_mask;

    // make rayhit packages
    RTCRayHit8 rayhit{};
    for(int i = 0; i < vQueryPoints.size(); ++i) {
        
        const int offset = i % ray_group_size;
        pcl::PointXYZ oUnitVec;

        oUnitVec.getVector3fMap() = (vQueryPoints[i].getVector3fMap() - oViewPoint.getVector3fMap()).normalized();
        rayhit.ray.org_x  [offset] = vQueryPoints[i].x;
        rayhit.ray.org_y  [offset] = vQueryPoints[i].y;
        rayhit.ray.org_z  [offset] = vQueryPoints[i].z;
        rayhit.ray.dir_x  [offset] = oUnitVec.x;
        rayhit.ray.dir_y  [offset] = oUnitVec.y;
        rayhit.ray.dir_z  [offset] = oUnitVec.z;
        rayhit.ray.tnear  [offset] = 0.1f;
        rayhit.ray.tfar   [offset] = std::numeric_limits<float>::infinity();
        rayhit.ray.mask   [offset] = -1;
        rayhit.ray.flags  [offset] = 0;
        rayhit.hit.geomID [offset] = RTC_INVALID_GEOMETRY_ID;
        rayhit.hit.instID[0][offset] = RTC_INVALID_GEOMETRY_ID;

        // do ray intersect
        if(offset == ray_group_size - 1 || i == vQueryPoints.size() - 1) {

            rtcIntersect8(vValidList+i-offset, scene, &rayhit, &intersectArguments);

            for(int k = 0; k <= offset; ++k) {
                if(rayhit.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID)
                    vHitDis[i-offset+k] = rayhit.ray.tfar[k];
            }
        }
    }

    #endif
    
    #ifdef RAY_PACKAGE_1
    for (int i = 0; i < vQueryPoints.size(); ++i) {

        pcl::PointXYZ oUnitVec;
        oUnitVec.getVector3fMap() = (vQueryPoints[i].getVector3fMap() - oViewPoint.getVector3fMap()).normalized();

        struct RTCRayHit rayhit{};
        rayhit.ray.org_x = vQueryPoints[i].x;
        rayhit.ray.org_y = vQueryPoints[i].y;
        rayhit.ray.org_z = vQueryPoints[i].z;
        rayhit.ray.dir_x = oUnitVec.x;
        rayhit.ray.dir_y = oUnitVec.y;
        rayhit.ray.dir_z = oUnitVec.z;
        rayhit.ray.tnear = 0;
        rayhit.ray.tfar = std::numeric_limits<float>::infinity();
        rayhit.ray.mask = -1;
        rayhit.ray.flags = 0;
        rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

        rtcIntersect1(scene, &rayhit, &intersectArguments);

        if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) vHitDis[i] = rayhit.ray.tfar;

    }//end for i
    #endif

    return vHitDis;
}

void SdfMaker::CastRay(RTCScene & scene, const pcl::PointXYZ& oViewPoint, pcl::PointCloud<pcl::DistanceIoVoxel>& vQueryPoints)
{
    // One argument object per call is required because sector queries execute
    // concurrently in MeshUpdater's worker pool.
    RTCIntersectArguments intersectArguments;
    rtcInitIntersectArguments(&intersectArguments);
    intersectArguments.flags = RTC_RAY_QUERY_FLAG_COHERENT;

    for (auto& oQueryPoint : vQueryPoints) {
        Eigen::Vector3f vRayVec = oQueryPoint.getVector3fMap() - oViewPoint.getVector3fMap();
        oQueryPoint.distance = vRayVec.norm();
        if (oQueryPoint.distance <= std::numeric_limits<float>::epsilon()) {
            oQueryPoint.io = 0.0f;
            continue;
        }
        vRayVec /= oQueryPoint.distance;

        RTCRayHit rayhit{}, downhit{}, uphit{};
        auto initializeRay = [](RTCRayHit& hit, float ox, float oy, float oz, float dx, float dy, float dz) {
            hit.ray.org_x = ox;
            hit.ray.org_y = oy;
            hit.ray.org_z = oz;
            hit.ray.dir_x = dx;
            hit.ray.dir_y = dy;
            hit.ray.dir_z = dz;
            hit.ray.tnear = 0.1f;
            hit.ray.tfar = std::numeric_limits<float>::infinity();
            hit.ray.mask = -1;
            hit.ray.flags = 0;
            hit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
        };

        initializeRay(rayhit, oViewPoint.x, oViewPoint.y, oViewPoint.z, vRayVec.x(), vRayVec.y(), vRayVec.z());
        initializeRay(downhit, oQueryPoint.x, oQueryPoint.y, oQueryPoint.z, 0.0f, 0.0f, -1.0f);
        initializeRay(uphit, oQueryPoint.x, oQueryPoint.y, oQueryPoint.z, 0.0f, 0.0f, 1.0f);

        rtcIntersect1(scene, &rayhit, &intersectArguments);
        rtcIntersect1(scene, &downhit, &intersectArguments);
        rtcIntersect1(scene, &uphit, &intersectArguments);

        if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
            oQueryPoint.io = 0.0f;
            oQueryPoint.distance = std::numeric_limits<float>::infinity();
            continue;
        }

        float sdf = rayhit.ray.tfar - oQueryPoint.distance;
        oQueryPoint.io = sdf < 0 ? 0.0f : 1.0f;
        oQueryPoint.distance = std::abs(sdf);
        if (downhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
            oQueryPoint.distance = std::min(oQueryPoint.distance, std::abs(downhit.ray.tfar));
        if (uphit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
            oQueryPoint.distance = std::min(oQueryPoint.distance, std::abs(uphit.ray.tfar));
    }
}

void SdfMaker::CastLos(RTCScene& scene, const pcl::PointXYZ& oViewPoint,
                       pcl::PointCloud<pcl::DistanceIoVoxel>& vQueryPoints)
{
    RTCIntersectArguments intersectArguments;
    rtcInitIntersectArguments(&intersectArguments);
    intersectArguments.flags = RTC_RAY_QUERY_FLAG_COHERENT;

    constexpr size_t kPacketSize = 8;
    for(size_t base = 0; base < vQueryPoints.size(); base += kPacketSize) {
        RTCRayHit8 rayhit{};
        int valid[kPacketSize]{};
        float queryRange[kPacketSize]{};
        const size_t count = std::min(kPacketSize, vQueryPoints.size() - base);

        for(size_t lane = 0; lane < count; ++lane) {
            pcl::DistanceIoVoxel& query = vQueryPoints[base + lane];
            // A no-hit is not proof of free space: preserve it as unknown.
            query.io = 0.0f;
            query.distance = std::numeric_limits<float>::infinity();
            query.weight = 0.0f;

            Eigen::Vector3f ray = query.getVector3fMap() - oViewPoint.getVector3fMap();
            queryRange[lane] = ray.norm();
            if(queryRange[lane] <= std::numeric_limits<float>::epsilon())
                continue;

            ray /= queryRange[lane];
            valid[lane] = -1;
            rayhit.ray.org_x[lane] = oViewPoint.x;
            rayhit.ray.org_y[lane] = oViewPoint.y;
            rayhit.ray.org_z[lane] = oViewPoint.z;
            rayhit.ray.dir_x[lane] = ray.x();
            rayhit.ray.dir_y[lane] = ray.y();
            rayhit.ray.dir_z[lane] = ray.z();
            rayhit.ray.tnear[lane] = 0.1f;
            rayhit.ray.tfar[lane] = std::numeric_limits<float>::infinity();
            rayhit.ray.mask[lane] = -1;
            rayhit.ray.flags[lane] = 0;
            rayhit.hit.geomID[lane] = RTC_INVALID_GEOMETRY_ID;
            rayhit.hit.instID[0][lane] = RTC_INVALID_GEOMETRY_ID;
        }

        bool hasValidRay = false;
        for(size_t lane = 0; lane < count; ++lane)
            hasValidRay = hasValidRay || valid[lane] != 0;
        if(!hasValidRay)
            continue;

        rtcIntersect8(valid, scene, &rayhit, &intersectArguments);
        for(size_t lane = 0; lane < count; ++lane) {
            if(valid[lane] == 0 || rayhit.hit.geomID[lane] == RTC_INVALID_GEOMETRY_ID)
                continue;
            pcl::DistanceIoVoxel& query = vQueryPoints[base + lane];
            const float signedMargin = rayhit.ray.tfar[lane] - queryRange[lane];
            query.io = signedMargin >= 0.0f ? 1.0f : 0.0f;
            query.distance = std::abs(signedMargin);
            query.weight = 1.0f;
        }
    }
}
