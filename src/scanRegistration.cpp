// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.

#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne_ros2/common.h"
#include "aloam_velodyne_ros2/tic_toc.h"
#include <opencv2/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

using std::atan2;
using std::cos;
using std::sin;

const double scanPeriod = 0.1;
const int systemDelay = 0; 
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

bool PUB_EACH_LINE = false;
double MINIMUM_RANGE = 0.1; 

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + 
            cloud_in.points[i].y * cloud_in.points[i].y + 
            cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
        cloud_out.points.resize(j);

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

class ScanRegistration : public rclcpp::Node
{
public:
    ScanRegistration() : Node("scanRegistration")
    {
        // 读取参数
        this->declare_parameter<int>("scan_line", 16);
        this->declare_parameter<double>("minimum_range", 0.1);
        this->get_parameter("scan_line", N_SCANS);
        this->get_parameter("minimum_range", MINIMUM_RANGE);

        RCLCPP_INFO(this->get_logger(), "scan line number: %d", N_SCANS);

        if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
        {
            RCLCPP_ERROR(this->get_logger(), "only support velodyne with 16, 32 or 64 scan line!");
            rclcpp::shutdown();
            return;
        }

        // 订阅
        subLaserCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 100,
            std::bind(&ScanRegistration::laserCloudHandler, this, std::placeholders::_1));

        // 发布
        pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_2", 100);
        pubCornerPointsSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_sharp", 100);
        pubCornerPointsLessSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_less_sharp", 100);
        pubSurfPointsFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_flat", 100);
        pubSurfPointsLessFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_less_flat", 100);
        pubRemovePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_remove_points", 100);

        if(PUB_EACH_LINE)
        {
            for(int i = 0; i < N_SCANS; i++)
            {
                auto tmp = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/laser_scanid_" + std::to_string(i), 100);
                pubEachScan.push_back(tmp);
            }
        }
    }

private:
    void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        if (!systemInited)
        { 
            systemInitCount++;
            if (systemInitCount >= systemDelay)
                systemInited = true;
            else
                return;
        }

        TicToc t_whole;
        TicToc t_prepare;
        std::vector<int> scanStartInd(N_SCANS, 0);
        std::vector<int> scanEndInd(N_SCANS, 0);

        pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
        pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
        std::vector<int> indices;

        pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
        removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

        int cloudSize = laserCloudIn.points.size();
        float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
        float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                              laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

        if (endOri - startOri > 3 * M_PI)
            endOri -= 2 * M_PI;
        else if (endOri - startOri < M_PI)
            endOri += 2 * M_PI;

        bool halfPassed = false;
        int count = cloudSize;
        PointType point;
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);

        for (int i = 0; i < cloudSize; i++)
        {
            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (N_SCANS == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (N_SCANS - 1) || scanID < 0)
                { count--; continue; }
            }
            else if (N_SCANS == 32)
            {
                scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                if (scanID > (N_SCANS - 1) || scanID < 0)
                { count--; continue; }
            }
            else if (N_SCANS == 64)
            {   
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                { count--; continue; }
            }

            float ori = -atan2(point.y, point.x);
            if (!halfPassed)
            { 
                if (ori < startOri - M_PI / 2)
                    ori += 2 * M_PI;
                else if (ori > startOri + M_PI * 3 / 2)
                    ori -= 2 * M_PI;

                if (ori - startOri > M_PI)
                    halfPassed = true;
            }
            else
            {
                ori += 2 * M_PI;
                if (ori < endOri - M_PI * 3 / 2)
                    ori += 2 * M_PI;
                else if (ori > endOri + M_PI / 2)
                    ori -= 2 * M_PI;
            }

            float relTime = (ori - startOri) / (endOri - startOri);
            point.intensity = scanID + scanPeriod * relTime;
            laserCloudScans[scanID].push_back(point); 
        }

        cloudSize = count;
        RCLCPP_INFO(this->get_logger(), "points size %d", cloudSize);

        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < N_SCANS; i++)//16-dim to 1-dim 
        { 
            scanStartInd[i] = laserCloud->size() + 5;
            *laserCloud += laserCloudScans[i];
            scanEndInd[i] = laserCloud->size() - 6;
        }

        RCLCPP_INFO(this->get_logger(), "prepare time %f", t_prepare.toc());

        for (int i = 5; i < cloudSize - 5; i++)
        { 
            float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + 
                          laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + 
                          laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + 
                          laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + 
                          laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + 
                          laserCloud->points[i + 5].x;

            float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + 
                          laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + 
                          laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + 
                          laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + 
                          laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + 
                          laserCloud->points[i + 5].y;

            float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + 
                          laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + 
                          laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + 
                          laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + 
                          laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + 
                          laserCloud->points[i + 5].z;

            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[i] = i;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
        }

        TicToc t_pts;
        pcl::PointCloud<PointType> cornerPointsSharp;
        pcl::PointCloud<PointType> cornerPointsLessSharp;//diff level 
        pcl::PointCloud<PointType> surfPointsFlat;
        pcl::PointCloud<PointType> surfPointsLessFlat;

        float t_q_sort = 0;
        for (int i = 0; i < N_SCANS; i++)
        {
            if( scanEndInd[i] - scanStartInd[i] < 6)
                continue;

            pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
            for (int j = 0; j < 6; j++)//each line divided into 6 sections
            {
                int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
                int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

                TicToc t_tmp;
                std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
                t_q_sort += t_tmp.toc();

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSortInd[k]; 
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 2)
                        {                        
                            cloudLabel[ind] = 2;
                            cornerPointsSharp.push_back(laserCloud->points[ind]);
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                        else if (largestPickedNum <= 20)
                        {                        
                            cloudLabel[ind] = 1; 
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                        else
                            break;

                        cloudNeighborPicked[ind] = 1; 

                        for (int l = 1; l <= 5; l++)
                        {
                            float dx = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float dy = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float dz = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (dx*dx + dy*dy + dz*dz > 0.05) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float dx = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float dy = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float dz = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (dx*dx + dy*dy + dz*dz > 0.05) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSortInd[k];
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
                    {
                        cloudLabel[ind] = -1; 
                        surfPointsFlat.push_back(laserCloud->points[ind]);
                        smallestPickedNum++;
                        if (smallestPickedNum >= 4)
                            break;

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            float dx = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float dy = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float dz = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (dx*dx + dy*dy + dz*dz > 0.05) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float dx = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float dy = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float dz = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (dx*dx + dy*dy + dz*dz > 0.05) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0)
                        surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }

            pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(surfPointsLessFlatScanDS);
            surfPointsLessFlat += surfPointsLessFlatScanDS;
        }

        RCLCPP_INFO(this->get_logger(), "sort q time %f", t_q_sort);
        RCLCPP_INFO(this->get_logger(), "seperate points time %f", t_pts.toc());

        // 发布点云
        sensor_msgs::msg::PointCloud2 laserCloudOutMsg;
        pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
        laserCloudOutMsg.header = laserCloudMsg->header;
        laserCloudOutMsg.header.frame_id = "camera_init";
        pubLaserCloud->publish(laserCloudOutMsg);

        sensor_msgs::msg::PointCloud2 cornerSharpMsg;
        pcl::toROSMsg(cornerPointsSharp, cornerSharpMsg);
        cornerSharpMsg.header = laserCloudMsg->header;
        cornerSharpMsg.header.frame_id = "camera_init";
        pubCornerPointsSharp->publish(cornerSharpMsg);

        sensor_msgs::msg::PointCloud2 cornerLessSharpMsg;
        pcl::toROSMsg(cornerPointsLessSharp, cornerLessSharpMsg);
        cornerLessSharpMsg.header = laserCloudMsg->header;
        cornerLessSharpMsg.header.frame_id = "camera_init";
        pubCornerPointsLessSharp->publish(cornerLessSharpMsg);

        sensor_msgs::msg::PointCloud2 surfFlatMsg;
        pcl::toROSMsg(surfPointsFlat, surfFlatMsg);
        surfFlatMsg.header = laserCloudMsg->header;
        surfFlatMsg.header.frame_id = "camera_init";
        pubSurfPointsFlat->publish(surfFlatMsg);

        sensor_msgs::msg::PointCloud2 surfLessFlatMsg;
        pcl::toROSMsg(surfPointsLessFlat, surfLessFlatMsg);
        surfLessFlatMsg.header = laserCloudMsg->header;
        surfLessFlatMsg.header.frame_id = "camera_init";
        pubSurfPointsLessFlat->publish(surfLessFlatMsg);

        if(PUB_EACH_LINE)
        {
            for(int i = 0; i< N_SCANS; i++)
            {
                sensor_msgs::msg::PointCloud2 scanMsg;
                pcl::toROSMsg(laserCloudScans[i], scanMsg);
                scanMsg.header = laserCloudMsg->header;
                scanMsg.header.frame_id = "camera_init";
                pubEachScan[i]->publish(scanMsg);
            }
        }

        if(t_whole.toc() > 100)
            RCLCPP_WARN(this->get_logger(), "scan registration over 100ms");
    }

    // 订阅器 & 发布器
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsSharp;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsLessSharp;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsFlat;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsLessFlat;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRemovePoints;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubEachScan;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanRegistration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}