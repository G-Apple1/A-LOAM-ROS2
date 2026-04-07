// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Zhang. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.

/*
！！！ Debug 模式需要降低延时
1.1 增加下采样参数
	# 修改参数文件
	mapping_line_resolution: 0.8    # 从0.4改为0.8
	mapping_plane_resolution: 1.2   # 从0.8改为1.2
1.2 减少局部地图范围
	// 在laserMapping.cpp中
	// 将提取范围从±2改为±1
	for (int i = centerCubeI-1; i <= centerCubeI+1; i++)
	for (int j = centerCubeJ-1; j <= centerCubeJ+1; j++) 
	for (int k = centerCubeK-1; k <= centerCubeK+1; k++)
	从5×5×3=75个体素 → 3×3×3=27个体素
	特征点数量减少约2/3
1.3 降低优化迭代次数
	// 修改建图优化
	for (int iterCount = 0; iterCount < 1; iterCount++)  // 从2改为1
	ceres::Solver::Options options;
	options.max_num_iterations = 2;  // 从4改为2
*/

#include <math.h>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

#include "lidarFactor.hpp"
#include "aloam_velodyne_ros2/common.h"
#include "aloam_velodyne_ros2/tic_toc.h"

int frameCount = 0;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

/*每个网格（cube）大小：50×50×50米  地图网格范围 1050m 以上
总网格数：21×21×11 = 4851个
坐标原点在中心网格中心，不是网格边界

21×21×11网格是全局地图的存储结构：
	存储所有历史特征点的累积
	通过滑动窗口管理内存
	局部地图从中提取用于匹配
*/
// 中心网格索引
int laserCloudCenWidth = 10;// 中心那个网格的x索引
int laserCloudCenHeight = 10;
int laserCloudCenDepth = 5;

int SkipMapFrame = 5;//及时清理缓存
// 网格维度
const int laserCloudWidth = 21;// x轴方向网格数
const int laserCloudHeight = 21;
const int laserCloudDepth = 11;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851

int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];

pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);

Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);
Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);

std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> cornerLastBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> surfLastBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> fullResBuf;
std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf;
std::mutex mBuf;

pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;

std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;
PointType pointOri, pointSel;

nav_msgs::msg::Path laserAfterMappedPath;

std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMappedHighFrec;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserAfterMappedPath;

rclcpp::Node::SharedPtr node_;

void transformAssociateToMap()
{
	q_w_curr = q_wmap_wodom * q_wodom_curr;
	t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

void transformUpdate()
{
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}

void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
}

void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
	po->x = point_curr.x();
	po->y = point_curr.y();
	po->z = point_curr.z();
	po->intensity = pi->intensity;
}

void laserCloudCornerLastHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudCornerLast2)
{
	mBuf.lock();
	cornerLastBuf.push(laserCloudCornerLast2);
	mBuf.unlock();
}

void laserCloudSurfLastHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudSurfLast2)
{
	mBuf.lock();
	surfLastBuf.push(laserCloudSurfLast2);
	mBuf.unlock();
}

void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudFullRes2)
{
	mBuf.lock();
	fullResBuf.push(laserCloudFullRes2);
	mBuf.unlock();
}

void laserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(laserOdometry);
	mBuf.unlock();

	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;
	q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
	q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
	q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
	q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
	t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
	t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
	t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

	Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
	Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;

	nav_msgs::msg::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "camera_init";
	odomAftMapped.child_frame_id = "aft_mapped";
	odomAftMapped.header.stamp = laserOdometry->header.stamp;
	odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
	odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
	odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
	odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
	odomAftMapped.pose.pose.position.x = t_w_curr.x();
	odomAftMapped.pose.pose.position.y = t_w_curr.y();
	odomAftMapped.pose.pose.position.z = t_w_curr.z();
	pubOdomAftMappedHighFrec->publish(odomAftMapped);
}

void process()
{
	// static int skip_counter = 0;//Debug 模式

	while(rclcpp::ok())
	{	
		
		// if (skip_counter % SkipMapFrame == 0) {  // 每3帧处理1帧
		// 	// 清空队列，避免堆积
		// 	mBuf.lock();
		// 	while (!cornerLastBuf.empty()) cornerLastBuf.pop();
		// 	while (!surfLastBuf.empty()) surfLastBuf.pop();
		// 	while (!fullResBuf.empty()) fullResBuf.pop();
		// 	while (!odometryBuf.empty()) odometryBuf.pop();
		// 	mBuf.unlock();
		// 	skip_counter = 0;
		// }

		// skip_counter++;

		while (!cornerLastBuf.empty() && !surfLastBuf.empty() &&
			!fullResBuf.empty() && !odometryBuf.empty())
		{
			mBuf.lock();
			while (!odometryBuf.empty() && (odometryBuf.front()->header.stamp.sec) < (cornerLastBuf.front()->header.stamp.sec))
				odometryBuf.pop();
			if (odometryBuf.empty()) { mBuf.unlock(); break; }

			while (!surfLastBuf.empty() && (surfLastBuf.front()->header.stamp.sec) < (cornerLastBuf.front()->header.stamp.sec))
				surfLastBuf.pop();
			if (surfLastBuf.empty()) { mBuf.unlock(); break; }

			while (!fullResBuf.empty() && (fullResBuf.front()->header.stamp.sec) < (cornerLastBuf.front()->header.stamp.sec))
				fullResBuf.pop();
			if (fullResBuf.empty()) { mBuf.unlock(); break; }

			timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.sec;
			timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.sec;
			timeLaserCloudFullRes = fullResBuf.front()->header.stamp.sec;
			timeLaserOdometry = odometryBuf.front()->header.stamp.sec;

			if (timeLaserCloudCornerLast != timeLaserOdometry ||
				timeLaserCloudSurfLast != timeLaserOdometry ||
				timeLaserCloudFullRes != timeLaserOdometry)
			{
				RCLCPP_ERROR(node_->get_logger(), "unsync message!");
				mBuf.unlock();
				break;
			}

			laserCloudCornerLast->clear();
			pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
			cornerLastBuf.pop();

			laserCloudSurfLast->clear();
			pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
			surfLastBuf.pop();

			laserCloudFullRes->clear();
			pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
			fullResBuf.pop();

			q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
			q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
			q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
			q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
			t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
			t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
			t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
			odometryBuf.pop();

			while(!cornerLastBuf.empty()) {cornerLastBuf.pop();}
			mBuf.unlock();

			TicToc t_whole;
			transformAssociateToMap();

			
			TicToc t_shift;
			// 当前位姿所在的网格索引计算
			/*分解计算：
				1.t_w_curr.x() + 25.0：将世界坐标系平移到网格中心
					网格范围：[-25, 25]，中心是0
					加25后范围：[0, 50]
				2.(x+25)/50.0：计算归一化网格坐标
					结果范围：[0, 1.0)
				3. int()：取整得到相对于中心网格的偏移（中心网格索引为10: -25~25范围都是)
					例如：当前位置x=12.5 → (12.5+25)/50=0.75 → int=0
					当前位置x=-12.5 → (-12.5+25)/50=0.25 → int=0
				4. + laserCloudCenWidth：转换为绝对网格索引*/
			int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
			int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
			int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;
			// 处理int()函数对负数的截断问题 int(-1.5)=-1 应该是 -2
			if (t_w_curr.x() + 25.0 < 0) centerCubeI--;
			if (t_w_curr.y() + 25.0 < 0) centerCubeJ--;
			if (t_w_curr.z() + 25.0 < 0) centerCubeK--;

			// cube shift logic (unchanged)
			/*为什么要滑动？
				机器人不断移动，但地图存储空间有限
				需要保持机器人位于地图中心区域
				当机器人接近地图边界时，移动地图而不是扩展地图
				// 注意：laserCloudWidth=21，索引0-20，边界是索引3和18
			*/
			while (centerCubeI < 3) // 机器人太靠近左边界
			{
				for (int j = 0; j < laserCloudHeight; j++) for (int k = 0; k < laserCloudDepth; k++) 
				{
					int i = laserCloudWidth - 1;
					auto c = laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					auto s = laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					for (; i >= 1; i--) {
						laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
												laserCloudCornerArray[i-1 + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
						laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
												laserCloudSurfArray[i-1 + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					}
					laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = c;
					laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = s;
					c->clear(); 
					s->clear();
				}
				centerCubeI++; 
				laserCloudCenWidth++;
			}
			while (centerCubeI >= laserCloudWidth - 3) //18 机器人太靠近右边界
			{
				for (int j = 0; j < laserCloudHeight; j++) for (int k = 0; k < laserCloudDepth; k++) 
				{
					int i = 0;
					auto c = laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					auto s = laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					for (; i < laserCloudWidth-1; i++) {
						laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
						laserCloudCornerArray[i+1 + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
						laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
						laserCloudSurfArray[i+1 + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					}
					laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = c;
					laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = s;
					c->clear(); 
					s->clear();
				}
				centerCubeI--; 
				laserCloudCenWidth--;
			}
			while (centerCubeJ < 3) 
			{
				for (int i = 0; i < laserCloudWidth; i++) for (int k = 0; k < laserCloudDepth; k++) 
				{
					int j = laserCloudHeight-1;
					auto c = laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					auto s = laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					for (; j >=1; j--) {
						laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
						laserCloudCornerArray[i + laserCloudWidth*(j-1) + laserCloudWidth*laserCloudHeight*k];
						laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
						laserCloudSurfArray[i + laserCloudWidth*(j-1) + laserCloudWidth*laserCloudHeight*k];
					}
					laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = c;
					laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = s;
					c->clear(); 
					s->clear();
				}
				centerCubeJ++; 
				laserCloudCenHeight++;
			}
			while (centerCubeJ >= laserCloudHeight-3) 
			{
				for (int i = 0; i < laserCloudWidth; i++) for (int k = 0; k < laserCloudDepth; k++) 
				{
					int j = 0;
					auto c = laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					auto s = laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					for (; j < laserCloudHeight-1; j++) {
						laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
						laserCloudCornerArray[i + laserCloudWidth*(j+1) + laserCloudWidth*laserCloudHeight*k];
						laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
						laserCloudSurfArray[i + laserCloudWidth*(j+1) + laserCloudWidth*laserCloudHeight*k];
					}
					laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = c;
					laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = s;
					c->clear(); 
					s->clear();
				}
				centerCubeJ--; 
				laserCloudCenHeight--;
			}
			while (centerCubeK < 3) 
			{
				for (int i = 0; i < laserCloudWidth; i++) for (int j = 0; j < laserCloudHeight; j++) 
				{
					int k = laserCloudDepth-1;
					auto c = laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					auto s = laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					for (; k >=1; k--) {
						laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
						laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*(k-1)];
						laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
						laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*(k-1)];
					}
					laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = c;
					laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = s;
					c->clear(); 
					s->clear();
				}
				centerCubeK++; 
				laserCloudCenDepth++;
			}
			while (centerCubeK >= laserCloudDepth-3) 
			{
				for (int i = 0; i < laserCloudWidth; i++) for (int j = 0; j < laserCloudHeight; j++) 
				{
					int k = 0;
					auto c = laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					auto s = laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k];
					for (; k < laserCloudDepth-1; k++) {
						laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
						laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*(k+1)];
						laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] =
						laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*(k+1)];
					}
					laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = c;
					laserCloudSurfArray[i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k] = s;
					c->clear(); 
					s->clear();
				}
				centerCubeK--; 
				laserCloudCenDepth--;
			}

			// 局部地图提取 源代码512行
			/*x,y方向：±2个网格（共5×5）
			  z方向：±1个网格（共3层）
			  总网格数：5×5×3=75个（最大125个，边界时更少）*/
			int laserCloudValidNum = 0, laserCloudSurroundNum = 0;
			for (int i = centerCubeI-2; i <= centerCubeI+2; i++)
			for (int j = centerCubeJ-2; j <= centerCubeJ+2; j++)
			for (int k = centerCubeK-1; k <= centerCubeK+1; k++)
			{
				if (i>=0&&i<laserCloudWidth && 
					j>=0&&j<laserCloudHeight && 
					k>=0&&k<laserCloudDepth) 
				{
					laserCloudValidInd[laserCloudValidNum++] = i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k;
					laserCloudSurroundInd[laserCloudSurroundNum++] = i + laserCloudWidth*j + laserCloudWidth*laserCloudHeight*k;
				}
			}

			// 清空之前的局部地图
			laserCloudCornerFromMap->clear(); 
			laserCloudSurfFromMap->clear();
			// 收集有效区域的地图点
			/*laserCloudValidInd：之前计算的75个有效体素索引
				将机器人周围的有效体素中的点云合并到局部地图
				局部地图范围：±2网格（xy方向），±1网格（z方向）*/
			for (int i=0; i<laserCloudValidNum; i++) 
			{
				*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
				*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
			}

			
			//角点特征下采样
			pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
			downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
			downSizeFilterCorner.filter(*laserCloudCornerStack);

			//平面点特征下采样
			pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
			downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
			downSizeFilterSurf.filter(*laserCloudSurfStack);

			printf("map prepare time %f ms\n", t_shift.toc());
			printf("map corner num %d  surf num %d \n", laserCloudCornerFromMap->size(), laserCloudSurfFromMap->size());

			/*需要足够的地图特征点才能进行优化
			角点至少10个，平面点至少50个*/
			if (laserCloudCornerFromMap->size() > 10 && laserCloudSurfFromMap->size() > 50)
			{
				TicToc t_opt;
				TicToc t_tree;

				kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
				kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

				for (int iter=0; iter<2; iter++)
				{
					ceres::LossFunction *loss = new ceres::HuberLoss(0.1);
					ceres::LocalParameterization *q_param = new ceres::EigenQuaternionParameterization();
					ceres::Problem problem;
					problem.AddParameterBlock(parameters, 4, q_param);
					problem.AddParameterBlock(parameters+4, 3);

					TicToc t_data;

					for (auto &p : laserCloudCornerStack->points) {
						pointOri = p;
						// 转换到地图坐标系（使用当前估计的位姿）
						pointAssociateToMap(&p, &pointSel);
						// 搜索最近5个地图角点
						kdtreeCornerFromMap->nearestKSearch(pointSel,5,pointSearchInd,pointSearchSqDis);
						// 线特征判断 // 最大距离小于1米  确保匹配点足够近
						if (pointSearchSqDis[4] < 1.0) 
						{
							std::vector<Eigen::Vector3d> ns;
							Eigen::Vector3d cen(0,0,0);
							// 计算5个点的中心
							for (int j=0;j<5;j++) 
							{
								auto &pp = laserCloudCornerFromMap->points[pointSearchInd[j]];
								Eigen::Vector3d tmp(pp.x,pp.y,pp.z);
								cen += tmp; ns.push_back(tmp);
							}
							cen /=5;

							// 计算协方差矩阵
							Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
							for (auto &n : ns) 
							{ 
								auto t = n - cen; 
								cov += t*t.transpose(); 
							}
							// 特征值分解
							Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);
							// 线特征判断条件（构建点到线的距离残差）
							if (saes.eigenvalues()[2] > 3*saes.eigenvalues()[1]) 
							{
								// 线的方向向量
								auto dir = saes.eigenvectors().col(2);
								// 线上两点
								Eigen::Vector3d curr(p.x,p.y,p.z);
								Eigen::Vector3d a = cen + 0.1*dir;
								Eigen::Vector3d b = cen - 0.1*dir;
								// 构建残差
								auto cost = LidarEdgeFactor::Create(curr,a,b,1.0);
								problem.AddResidualBlock(cost, loss, parameters, parameters+4);
							}
						}
					}

					for (auto &p : laserCloudSurfStack->points) {
						pointOri = p;
						pointAssociateToMap(&p, &pointSel);
						kdtreeSurfFromMap->nearestKSearch(pointSel,5,pointSearchInd,pointSearchSqDis);
						if (pointSearchSqDis[4] <1.0) {
							// 拟合平面（ 构建线性方程组：n·x + d = 0）
							Eigen::Matrix<double,5,3> A;
							Eigen::Matrix<double,5,1> b = -Eigen::Matrix<double,5,1>::Ones();
							for (int j=0;j<5;j++) 
							{
								auto &pp = laserCloudSurfFromMap->points[pointSearchInd[j]];
								A(j,0)=pp.x; 
								A(j,1)=pp.y; 
								A(j,2)=pp.z;
							}
							// 求解平面法向量 n
							Eigen::Vector3d n = A.colPivHouseholderQr().solve(b);
							double inv = 1/n.norm(); // d = -1/|n|
							n.normalize(); // 单位化法向量
							// 平面拟合质量检查：
							bool ok = true;
							for (int j=0;j<5;j++) {
								auto &pp = laserCloudSurfFromMap->points[pointSearchInd[j]];
								if (fabs(n.x()*pp.x + n.y()*pp.y + n.z()*pp.z + inv) >0.2) {
									ok=false; break;
								}
							}
							// 构建点到面的距离残差：
							if (ok) {
								Eigen::Vector3d curr(p.x,p.y,p.z);
								auto cost = LidarPlaneNormFactor::Create(curr,n,inv);
								problem.AddResidualBlock(cost, loss, parameters, parameters+4);
							}
						}
					}
					printf("mapping data assosiation time %f ms \n", t_data.toc());

					//优化求解
					TicToc t_solver;
					ceres::Solver::Options opt;
					opt.linear_solver_type=ceres::DENSE_QR;
					opt.max_num_iterations=4;
					ceres::Solver::Summary sum;
					ceres::Solve(opt,&problem,&sum);
					printf("mapping solver time %f ms \n", t_solver.toc());
				}
				printf("mapping optimization time %f \n", t_opt.toc());
			}

			transformUpdate();

			TicToc t_add;
			// 将当前帧加入地图
			// 将优化后的角点加入地图
			for (auto &p : laserCloudCornerStack->points) {
				pointAssociateToMap(&p, &pointSel);// 用优化后的位姿转换
				// 计算体素索引并加入对应体素
				int ci = int((pointSel.x+25)/50) + laserCloudCenWidth;
				int cj = int((pointSel.y+25)/50) + laserCloudCenHeight;
				int ck = int((pointSel.z+25)/50) + laserCloudCenDepth;
				if (pointSel.x+25<0) ci--;
				if (pointSel.y+25<0) cj--;
				if (pointSel.z+25<0) ck--;
				// 索引计算和边界检查
				if (ci>=0&&ci<laserCloudWidth &&
					cj>=0&&cj<laserCloudHeight &&
					ck>=0&&ck<laserCloudDepth) 
				{
					int idx = ci + laserCloudWidth*cj + laserCloudWidth*laserCloudHeight*ck;
					laserCloudCornerArray[idx]->push_back(pointSel);
				}
			}
			for (auto &p : laserCloudSurfStack->points) {
				pointAssociateToMap(&p, &pointSel);
				int ci = int((pointSel.x+25)/50) + laserCloudCenWidth;
				int cj = int((pointSel.y+25)/50) + laserCloudCenHeight;
				int ck = int((pointSel.z+25)/50) + laserCloudCenDepth;
				if (pointSel.x+25<0) ci--;
				if (pointSel.y+25<0) cj--;
				if (pointSel.z+25<0) ck--;
				if (ci>=0&&ci<laserCloudWidth &&
					cj>=0&&cj<laserCloudHeight &&
					ck>=0&&ck<laserCloudDepth) {
					int idx = ci + laserCloudWidth*cj + laserCloudWidth*laserCloudHeight*ck;
					laserCloudSurfArray[idx]->push_back(pointSel);
				}
			}
			printf("add points time %f ms\n", t_add.toc());

			TicToc t_filter;
			// 地图维护与下采样
			for (int i=0; i<laserCloudValidNum; i++) 
			{
				int idx = laserCloudValidInd[i];

				pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>());
				downSizeFilterCorner.setInputCloud(laserCloudCornerArray[idx]);
				downSizeFilterCorner.filter(*tmp);
				laserCloudCornerArray[idx] = tmp;

				tmp.reset(new pcl::PointCloud<PointType>());
				downSizeFilterSurf.setInputCloud(laserCloudSurfArray[idx]);
				downSizeFilterSurf.filter(*tmp);
				laserCloudSurfArray[idx] = tmp;
			}
			printf("filter time %f ms \n", t_filter.toc());

			//局部地图发布（每5帧）
			TicToc t_pub;
			if (frameCount %5 ==0) {
				laserCloudSurround->clear();
				for (int i=0; i<laserCloudSurroundNum; i++) 
				{
					int idx = laserCloudSurroundInd[i];
					*laserCloudSurround += *laserCloudCornerArray[idx];
					*laserCloudSurround += *laserCloudSurfArray[idx];
				}
				sensor_msgs::msg::PointCloud2 msg;
				pcl::toROSMsg(*laserCloudSurround, msg);
				msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timeLaserOdometry*1e9));
				msg.header.frame_id = "camera_init";
				pubLaserCloudSurround->publish(msg);
			}

			//全局地图发布（每20帧）
			if (frameCount%20 ==0) {
				pcl::PointCloud<PointType> map;
				for (int i=0; i<laserCloudNum; i++) 
				{
					map += *laserCloudCornerArray[i]; 
					map += *laserCloudSurfArray[i];
				}
				sensor_msgs::msg::PointCloud2 msg;
				pcl::toROSMsg(map, msg);
				msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timeLaserOdometry*1e9));
				msg.header.frame_id = "camera_init";
				pubLaserCloudMap->publish(msg);
			}

			for (auto &p : laserCloudFullRes->points) pointAssociateToMap(&p, &p);


			sensor_msgs::msg::PointCloud2 msg_full;
			pcl::toROSMsg(*laserCloudFullRes, msg_full);
			msg_full.header.stamp = rclcpp::Time(static_cast<int64_t>(timeLaserOdometry*1e9));
			msg_full.header.frame_id = "camera_init";
			pubLaserCloudFullRes->publish(msg_full);

			printf("mapping pub time %f ms \n", t_pub.toc());

			printf("whole mapping time %f ms +++++\n", t_whole.toc());

			nav_msgs::msg::Odometry odom;
			odom.header.frame_id = "camera_init";
			odom.child_frame_id = "aft_mapped";
			odom.header.stamp = rclcpp::Time(static_cast<int64_t>(timeLaserOdometry*1e9));
			odom.pose.pose.orientation.x = q_w_curr.x();
			odom.pose.pose.orientation.y = q_w_curr.y();
			odom.pose.pose.orientation.z = q_w_curr.z();
			odom.pose.pose.orientation.w = q_w_curr.w();
			odom.pose.pose.position.x = t_w_curr.x();
			odom.pose.pose.position.y = t_w_curr.y();
			odom.pose.pose.position.z = t_w_curr.z();
			pubOdomAftMapped->publish(odom);

			geometry_msgs::msg::PoseStamped pose;
			pose.header = odom.header;
			pose.pose = odom.pose.pose;
			laserAfterMappedPath.header.stamp = odom.header.stamp;
			laserAfterMappedPath.header.frame_id = "camera_init";
			laserAfterMappedPath.poses.push_back(pose);
			pubLaserAfterMappedPath->publish(laserAfterMappedPath);

			geometry_msgs::msg::TransformStamped tf;
			tf.header.stamp = odom.header.stamp;
			tf.header.frame_id = "camera_init";
			tf.child_frame_id = "aft_mapped";
			tf.transform.translation.x = t_w_curr.x();
			tf.transform.translation.y = t_w_curr.y();
			tf.transform.translation.z = t_w_curr.z();
			tf.transform.rotation.x = q_w_curr.x();
			tf.transform.rotation.y = q_w_curr.y();
			tf.transform.rotation.z = q_w_curr.z();
			tf.transform.rotation.w = q_w_curr.w();
			tf_broadcaster_->sendTransform(tf);

			frameCount++;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	node_ = std::make_shared<rclcpp::Node>("laserMapping");

	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

	//增加下采样参数（去launch文件修改）
	float lineRes = 0.4; 
	float planeRes = 1.2; 
	node_->declare_parameter<float>("mapping_line_resolution", lineRes);
	node_->declare_parameter<float>("mapping_plane_resolution", planeRes);
	node_->get_parameter("mapping_line_resolution", lineRes);
	node_->get_parameter("mapping_plane_resolution", planeRes);
	RCLCPP_INFO(node_->get_logger(), "line %.2f plane %.2f", lineRes, planeRes);
	downSizeFilterCorner.setLeafSize(lineRes,lineRes,lineRes);
	downSizeFilterSurf.setLeafSize(planeRes,planeRes,planeRes);

	auto sub1 = node_->create_subscription<sensor_msgs::msg::PointCloud2>("/laser_cloud_corner_last",100,laserCloudCornerLastHandler);
	auto sub2 = node_->create_subscription<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf_last",100,laserCloudSurfLastHandler);
	auto sub3 = node_->create_subscription<nav_msgs::msg::Odometry>("/laser_odom_to_init",100,laserOdometryHandler);
	auto sub4 = node_->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_3",100,laserCloudFullResHandler);

	pubLaserCloudSurround = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surround",100);
	pubLaserCloudMap = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_map",100);
	pubLaserCloudFullRes = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_registered",100);
	pubOdomAftMapped = node_->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init",100);
	pubOdomAftMappedHighFrec = node_->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init_high_frec",100);//直接从里程计获取位姿
	pubLaserAfterMappedPath = node_->create_publisher<nav_msgs::msg::Path>("/aft_mapped_path",100);

	for (int i=0; i<laserCloudNum; i++) {
		laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
		laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
	}

	std::thread mapping_process{process};

	rclcpp::spin(node_);
	rclcpp::shutdown();
	return 0;
}