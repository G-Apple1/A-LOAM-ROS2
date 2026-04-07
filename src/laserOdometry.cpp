// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Zhang. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "aloam_velodyne_ros2/common.h"
#include "aloam_velodyne_ros2/tic_toc.h"
#include "lidarFactor.hpp"

#define DISTORTION 0

int corner_correspondence = 0, plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

int skipFrameNum = 5;
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);//
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);//

std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> cornerSharpBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> surfFlatBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> surfLessFlatBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> fullPointsBuf;
std::mutex mBuf;

void TransformToStart(PointType const *const pi, PointType *const po)
{
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;

    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

void TransformToEnd(PointType const *const pi, PointType *const po)
{
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    po->intensity = int(pi->intensity);
}

class LaserOdometry : public rclcpp::Node
{
public:
    LaserOdometry() : Node("laserOdometry")
    {
        this->declare_parameter<int>("mapping_skip_frame", 2);
        this->get_parameter("mapping_skip_frame", skipFrameNum);

        RCLCPP_INFO(this->get_logger(), "Mapping %d Hz", 10 / skipFrameNum);

        subCornerPointsSharp = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_sharp", 100,
            std::bind(&LaserOdometry::laserCloudSharpHandler, this, std::placeholders::_1));

        subCornerPointsLessSharp = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_less_sharp", 100,
            std::bind(&LaserOdometry::laserCloudLessSharpHandler, this, std::placeholders::_1));

        subSurfPointsFlat = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_flat", 100,
            std::bind(&LaserOdometry::laserCloudFlatHandler, this, std::placeholders::_1));

        subSurfPointsLessFlat = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_less_flat", 100,
            std::bind(&LaserOdometry::laserCloudLessFlatHandler, this, std::placeholders::_1));

        subLaserCloudFullRes = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_cloud_2", 100,
            std::bind(&LaserOdometry::laserCloudFullResHandler, this, std::placeholders::_1));

        pubLaserCloudCornerLast = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_corner_last", 100);
        pubLaserCloudSurfLast = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf_last", 100);
        pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_3", 100);
        pubLaserOdometry = this->create_publisher<nav_msgs::msg::Odometry>("/laser_odom_to_init", 100);
        pubLaserPath = this->create_publisher<nav_msgs::msg::Path>("/laser_odom_path", 100);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&LaserOdometry::runLoop, this));
    }

private:
    void laserCloudSharpHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cornerPointsSharp2)
    {
        std::lock_guard<std::mutex> lock(mBuf);
        cornerSharpBuf.push(cornerPointsSharp2);
    }

    void laserCloudLessSharpHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cornerPointsLessSharp2)
    {
        std::lock_guard<std::mutex> lock(mBuf);
        cornerLessSharpBuf.push(cornerPointsLessSharp2);
    }

    void laserCloudFlatHandler(const sensor_msgs::msg::PointCloud2::SharedPtr surfPointsFlat2)
    {
        std::lock_guard<std::mutex> lock(mBuf);
        surfFlatBuf.push(surfPointsFlat2);
    }

    void laserCloudLessFlatHandler(const sensor_msgs::msg::PointCloud2::SharedPtr surfPointsLessFlat2)
    {
        std::lock_guard<std::mutex> lock(mBuf);
        surfLessFlatBuf.push(surfPointsLessFlat2);
    }

    void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudFullRes2)
    {
        std::lock_guard<std::mutex> lock(mBuf);
        fullPointsBuf.push(laserCloudFullRes2);
    }

    void runLoop()
    {
        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty())
        {
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.sec;
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.sec;
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.sec;
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.sec;
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.sec;

            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                RCLCPP_ERROR(this->get_logger(), "unsync message!");
                rclcpp::shutdown();
                return;
            }

            mBuf.lock();
            cornerPointsSharp->clear();
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            cornerSharpBuf.pop();

            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();

            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();

            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();
            mBuf.unlock();

            TicToc t_whole;

            if (!systemInited)
            {
                systemInited = true;
                RCLCPP_INFO(this->get_logger(), "Initialization finished");
            }
            else
            {//基于特征匹配的迭代最近点（ICP）优化，用于计算相邻两帧激光雷达之间的相对运动（位姿变换）
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();

                TicToc t_opt;
                //双次迭代优化循环
                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
                {
                    corner_correspondence = 0;
                    plane_correspondence = 0;

                    /*Huber损失函数：鲁棒性优化，减少外点（误匹配点）的影响
                      四元数参数化：确保旋转矩阵的正交性约束
                      优化变量：
                        para_q：4维，旋转四元数 (qx, qy, qz, qw)
                        para_t：3维，平移向量 (x, y, z)*/
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    ceres::LocalParameterization *q_parameterization =
                        new ceres::EigenQuaternionParameterization();
                    ceres::Problem::Options problem_options;

                    ceres::Problem problem(problem_options);
                    problem.AddParameterBlock(para_q, 4, q_parameterization);
                    problem.AddParameterBlock(para_t, 3);

                    pcl::PointXYZI pointSel;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;

                    TicToc t_data;
                    for (int i = 0; i < cornerPointsSharpNum; ++i)
                    {
                        // 1. 变换到扫描起始时刻（去畸变）
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                        // 2. KD树搜索最近邻点
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1;
                        // 3. 在同一扫描线或相邻扫描线中寻找第二个最近邻点
                        //    构成一条直线（A-B），用于点到线的距离计算
                        /*当前角点 → 在上一帧中找最近点A → 
                                向前搜索 → 找同一条线上的点（同scan ID）
                                向后搜索 → 找同一条线上的点（同scan ID）
                            形成直线 A-B*/
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                            {
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)//intensity字段存储了相对时间（在0-1之间）
                                    continue;
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }
                        if (minPointInd2 >= 0)
                        {
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);

                            //当DISTORTION启用时，根据点在扫描中的时间戳进行线性插值
                            //s：归一化的时间比例（0表示扫描开始，1表示扫描结束）
                            double s = DISTORTION ? (cornerPointsSharp->points[i].intensity -
                                                     int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD : 1.0;

                            // 角点：点到线距离
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            corner_correspondence++;
                        }
                    }

                    for (int i = 0; i < surfPointsFlatNum; ++i)
                    {
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                            {
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }
                        }
                        if (minPointInd2 >= 0 && minPointInd3 >= 0)
                        {
                            Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                        surfPointsFlat->points[i].y,
                                                        surfPointsFlat->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                            laserCloudSurfLast->points[closestPointInd].y,
                                                            laserCloudSurfLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                            laserCloudSurfLast->points[minPointInd2].y,
                                                            laserCloudSurfLast->points[minPointInd2].z);
                            Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                            laserCloudSurfLast->points[minPointInd3].y,
                                                            laserCloudSurfLast->points[minPointInd3].z);

                            double s = DISTORTION ? (surfPointsFlat->points[i].intensity -
                                                     int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD : 1.0;

                            // 平面点：点到面距离
                            ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            plane_correspondence++;
                        }
                    }

                    RCLCPP_INFO(this->get_logger(), "data association time %f ms", t_data.toc());

                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        RCLCPP_WARN(this->get_logger(), "less correspondence!");
                    }

                    TicToc t_solver;
                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR; // 稠密QR分解
                    options.max_num_iterations = 4;               // 最大迭代次数 (平衡精度和计算效率)
                    options.minimizer_progress_to_stdout = false; // 不输出迭代信息
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);
                    RCLCPP_INFO(this->get_logger(), "solver time %f ms", t_solver.toc());
                }
                RCLCPP_INFO(this->get_logger(), "optimization twice time %f", t_opt.toc());

                // 注意平移变换需要先用当前旋转对位移进行旋转
                t_w_curr = t_w_curr + q_w_curr * t_last_curr;//平移
                q_w_curr = q_w_curr * q_last_curr;//旋转
            }

            TicToc t_pub;

            nav_msgs::msg::Odometry laserOdometry;
            laserOdometry.header.frame_id = "camera_init";
            laserOdometry.child_frame_id = "laser_odom";
            laserOdometry.header.stamp = rclcpp::Time(static_cast<int64_t>(timeSurfPointsLessFlat*1e9));
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaserOdometry->publish(laserOdometry);

            geometry_msgs::msg::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "camera_init";
            pubLaserPath->publish(laserPath);

            // 交换点云指针
            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;

            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();

            std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

            // 更新KD树
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

            if (frameCount % skipFrameNum == 0)
            {
                frameCount = 0;

                sensor_msgs::msg::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = rclcpp::Time(static_cast<int64_t>(timeSurfPointsLessFlat*1e9));
                laserCloudCornerLast2.header.frame_id = "camera";
                pubLaserCloudCornerLast->publish(laserCloudCornerLast2);

                sensor_msgs::msg::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = rclcpp::Time(static_cast<int64_t>(timeSurfPointsLessFlat*1e9));
                laserCloudSurfLast2.header.frame_id = "camera";
                pubLaserCloudSurfLast->publish(laserCloudSurfLast2);

                sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = rclcpp::Time(static_cast<int64_t>(timeSurfPointsLessFlat*1e9));
                laserCloudFullRes3.header.frame_id = "camera";
                pubLaserCloudFullRes->publish(laserCloudFullRes3);
            }

            RCLCPP_INFO(this->get_logger(), "publication time %f ms", t_pub.toc());
            RCLCPP_INFO(this->get_logger(), "whole laserOdometry time %f ms\n", t_whole.toc());

            if(t_whole.toc() > 100)
                RCLCPP_WARN(this->get_logger(), "odometry process over 100ms");

            frameCount++;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCornerPointsSharp;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCornerPointsLessSharp;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfPointsFlat;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfPointsLessFlat;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudCornerLast;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurfLast;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserPath;

    nav_msgs::msg::Path laserPath;
    int frameCount = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}