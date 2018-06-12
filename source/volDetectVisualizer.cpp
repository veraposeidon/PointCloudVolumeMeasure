//
// Created by shen on 18-6-5.
//
#include <iostream>
#include <string>

#include <pcl/io/openni2_grabber.h>         //OpenNI采集头文件

#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>.
#include <pcl/common/common_headers.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>                    //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>                     //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>    //统计滤波头文件
#include <pcl/filters/extract_indices.h>                //索引提取滤波器头文件
#include <pcl/filters/project_inliers.h>                //映射相关头文件

#include <pcl/segmentation/sac_segmentation.h>  //基于采样一致性分割的类的头文件
#include <pcl/ModelCoefficients.h>          //采样一致性模型相关类头文件

#include <pcl/sample_consensus/method_types.h>  //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件

#include <pcl/surface/concave_hull.h>           //提取凸（凹）多边形的头文件.

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

class VolumDetectViewer
{
public:
	boost::mutex cloud_mutex;
	pcl::visualization::PCLVisualizer viewer;

	// 直通滤波器对象
	pcl::PassThrough<pcl::PointXYZ> XpassFilter;
	pcl::PassThrough<pcl::PointXYZ> YpassFilter;

	// 下采样 VoxelGrid 滤波对象
	pcl::VoxelGrid<pcl::PointXYZ> VoxlFilter;

	// 统计滤波器
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisFilter;

	// 分割对象
	pcl::SACSegmentation<pcl::PointXYZ> segPlane;

	// 提取索引滤波器
	pcl::ExtractIndices<pcl::PointXYZ> extractPlane;

	// 点云投影滤波对象
	pcl::ProjectInliers<pcl::PointXYZ> projec;

	VolumDetectViewer():viewer("PCLVIEW")
	{
		// 1. 初始化 new viewer
		//pcl::visualization::PCLVisualizer viewer("PCLVIEW");
		viewer.setBackgroundColor(0, 0, 0);
		viewer.addCoordinateSystem(0.6);
		viewer.initCameraParameters();
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");

		// 2. 直通滤波设置
		// 滤波字段设为x轴方向.
		XpassFilter.setFilterFieldName("x");
		// 设定可接受范围，将不在范围内的点过滤掉或者保留（由setFilterLimitsNegative决定）.
		XpassFilter.setFilterLimits(-0.25, 0.05);  // TODO: X方向裁剪阈值需要调试 实际是0.44m.
		// 滤波字段设为y轴方向.
		YpassFilter.setFilterFieldName("y");
		// 设定可接受范围，将不在范围内的点过滤掉或者保留（由setFilterLimitsNegative决定）.
		YpassFilter.setFilterLimits(-0.25, 0.18);  // TODO: Y方向裁剪阈值需要调试 实际是0.69m.

		// 3. 体素滤波设置
		// 设置体素滤波时创建的体素体积为0.5*0.5*0.5cm的立方体
		VoxlFilter.setLeafSize(0.005f, 0.005f, 0.005f);

		// 4. 统计滤波初始设置
		statisFilter.setMeanK(50);  // 对每个点分析的临近点个数设为50
		statisFilter.setStddevMulThresh(1.0);   // 将标准差倍数设为1，意味着一个点的距离超出平均距离1个标准差以上，就会被标记为离群点，并被移除。

		// 5. 设置分割对象
		// 可选：设置模型系数需要优化
		segPlane.setOptimizeCoefficients(true);
		// 必选：设置分割的模型类型、所用的随机参数估计方法、距离阈值、迭代次数上限
		segPlane.setModelType(pcl::SACMODEL_PLANE);
		segPlane.setMethodType(pcl::SAC_RANSAC);
		segPlane.setDistanceThreshold(0.01);    // TODO: 距离阈值需要调试
		segPlane.setMaxIterations(1000);

		// 6. 设置投影模型为SACMODEL_PLANE
		projec.setModelType(pcl::SACMODEL_PLANE);


	}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &Prev_cloud, bool * new_cloud_available)
	{
		cloud_mutex.lock();

		*Prev_cloud = *cloud;

		*new_cloud_available = true;

		cloud_mutex.unlock();
	}


	void run()
	{
		// 1. 新建OpenNI设备的捕获器 grabber
		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		bool new_cloud_available = false;
		// 建立回调
		boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
			boost::bind(&VolumDetectViewer::cloud_cb_, this, _1, cloud, &new_cloud_available);

		interface->registerCallback(f);

		interface->start();

		//// 等待第一帧
		//while (!new_cloud_available)
		//{
		//	boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		//	new_cloud_available = false;
		//}

		while (!viewer.wasStopped())
		{

			if (new_cloud_available && cloud_mutex.try_lock())
			{
				new_cloud_available = false;

				// 1. 空间裁剪滤波
				pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filtered(new pcl::PointCloud<pcl::PointXYZ>);
				
				XpassFilter.setInputCloud(cloud);
				XpassFilter.filter(*pass_filtered);
				YpassFilter.setInputCloud(pass_filtered);
				YpassFilter.filter(*pass_filtered);
				// 滤波信息
				string originSize = "originSize: " + to_string(cloud->points.size());
				string passfiltered ="pass_filtered: " + to_string(pass_filtered->points.size());


				// 2. 下采样进行数据压缩
				// 下采样
				pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
				VoxlFilter.setInputCloud(pass_filtered);
				VoxlFilter.filter(*voxel_filtered);
				// 滤波信息
				string voxelfiltered = "voxel_filtered: " + to_string(voxel_filtered->points.size());

				// 3. 统计滤波 去除异常点
				pcl::PointCloud<pcl::PointXYZ>::Ptr statis_Filtered(new pcl::PointCloud<pcl::PointXYZ>);
				statisFilter.setInputCloud(voxel_filtered);
				statisFilter.filter(*statis_Filtered);
				// 滤波信息
				string statisFiltered = "statis_Filtered: " + to_string(statis_Filtered->points.size());

				// 4. 平面分割 获取地面
				// 创建分割时所需要的模型系数对象 coefficientsPlane
				// 创建存储模型内点的点索引集合对象 inliersPlane
				pcl::ModelCoefficients::Ptr coefficientsPlane(new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliersPlane(new pcl::PointIndices());
				// 平面分割
				segPlane.setInputCloud(statis_Filtered);
				segPlane.segment(*inliersPlane, *coefficientsPlane);
				string PlaneSeg;
				if (inliersPlane->indices.size() == 0) 
				{
					PlaneSeg = "No Plane detected";
				}
				else
				{
					PlaneSeg = "Plane Model: ax+by+cz+d=0, a = " + to_string(coefficientsPlane->values[0]) + ", b= " + to_string(coefficientsPlane->values[1]) +
						", c= " + to_string(coefficientsPlane->values[2]) + ", d= " + to_string(coefficientsPlane->values[3]);
				}
				// 拷贝一份平面点集,并着色
				pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneCloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*statis_Filtered, inliersPlane->indices, *PlaneCloud);
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Plane_color(PlaneCloud, 139, 139, 0);

				// 5. 根据索引提取除地面之外的点云
				pcl::PointCloud<pcl::PointXYZ>::Ptr extracted(new pcl::PointCloud<pcl::PointXYZ>);
				extractPlane.setInputCloud(statis_Filtered);
				extractPlane.setIndices(inliersPlane);
				extractPlane.setNegative(true); // 设置Negative是获取除了平面的点
				extractPlane.filter(*extracted);
				string extractedFromPlane = "extractedFromPlane Remains size: " + to_string(extracted->points.size());


				// 6. 平面分割 获取目标物上表面
				// 创建分割时所需要的模型系数对象 coefficientsTarget
				// 创建存储模型内点的点索引集合对象 inliersTarget
				pcl::ModelCoefficients::Ptr coefficientsTarget(new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliersTarget(new pcl::PointIndices());
				// 平面分割
				segPlane.setInputCloud(extracted);
				segPlane.segment(*inliersTarget, *coefficientsTarget);
				string TargetSeg;
				if (inliersTarget->indices.size() == 0)
				{
					TargetSeg = "No Plane detected";
					//break;
				}
				else
				{
					TargetSeg = "Target Model: ax+by+cz+d=0, a = " + to_string(coefficientsTarget->values[0]) + ", b= " + to_string(coefficientsTarget->values[1]) +
						", c= " + to_string(coefficientsTarget->values[2]) + ", d= " + to_string(coefficientsTarget->values[3]);
				}
				// 拷贝一份平面点集,并着色
				pcl::PointCloud<pcl::PointXYZ>::Ptr TargetCloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::copyPointCloud(*extracted, inliersTarget->indices, *TargetCloud);
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Target_color(TargetCloud, 205, 201, 201);

				// 7. 计算高度差
				float height = abs(coefficientsPlane->values[3] / coefficientsPlane->values[2] -
					coefficientsTarget->values[3] / coefficientsPlane->values[2]);
				string heightStr = "Height: " + to_string(height);


				// 8. 投影平面模型系数
				pcl::PointCloud<pcl::PointXYZ>::Ptr target_projected(new pcl::PointCloud<pcl::PointXYZ>);

				projec.setIndices(inliersTarget);
				projec.setInputCloud(extracted);
				projec.setModelCoefficients(coefficientsTarget);
				projec.filter(*target_projected);
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_projected_color(target_projected, 255, 69, 0);

				// 9. 为投影的点创建凹包表征
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::ConcaveHull<pcl::PointXYZ> chull;
				//polygons存储一系列顶点集，每组顶点集表示一个多边形 //应该取其中第0个顶点集
				vector<pcl::Vertices> polygons;
				chull.setInputCloud(target_projected);
				chull.setAlpha(0.1);
				// polygons对应的是前面point cloud_hull的索引
				chull.reconstruct(*cloud_hull, polygons);
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_hull_color(cloud_hull, 0, 245, 255);

				// 10. 获取二维轮廓供OpenCV处理
				// 生成二维轮廓
				vector<Point2f> hullContour;
				for (int j = 0; j < polygons[0].vertices.size(); ++j) {
					hullContour.push_back(Point2f(cloud_hull->points[polygons[0].vertices[j]].x,
						cloud_hull->points[polygons[0].vertices[j]].y));
				}

				RotatedRect minRect = minAreaRect(hullContour);

				// TODO：此处的长宽与实际需要进行换算

				float realWidth = minRect.size.width * 1.5;
				float realHeight = minRect.size.height * 1.5;
				float rectArea = realWidth* realHeight;
				string rectInfo = "length: " + to_string(realWidth) + ", width: " + to_string(realHeight);

				// 8. 计算凹包面积
				// 8.1 自带函数计算
				// 面积的计算方式是没有问题的
				float Hullarea = pcl::calculatePolygonArea(*cloud_hull) * (0.69*0.44) / (0.3*0.43);

				string AreaStr = "Rect Area: " + to_string(rectArea) + ", Hull Area: " + to_string(Hullarea);

				float Vol = rectArea * height;
				string VolStr = "Volume: " + to_string(Vol);

				// 更新点云显示和字符串显示
				viewer.removeAllPointClouds();
				viewer.removeAllShapes();

				// 添加圆球体
				//pcl::PointXYZ center(0.2, 0.2, 1.0);
				//viewer.addSphere(center,0.01,1.0,1.0,0, "sphere");
				
				// 添加方体
				//viewer.addCube(-0.1, 0.1, -0.1, 0.1, 0.9, 1.0,0.1,0.2,0.3, "cube");

				viewer.addText(PlaneSeg, 10, 1000, 20, 1, 1, 0, "PlaneSeg");
				viewer.addText(TargetSeg, 10, 950, 20, 1, 1, 0, "TargetSeg");
				viewer.addText(heightStr, 10, 900, 20, 1, 1, 0, "heightStr");
				viewer.addText(rectInfo, 10, 850, 20, 1, 1, 0, "rectInfo");
				viewer.addText(AreaStr, 10, 800, 20, 1, 1, 0, "AreaStr");
				viewer.addText(VolStr, 10, 750, 30, 1, 1, 0, "VolStr");

				viewer.addText(originSize, 10, 200, 15, 1, 1, 0, "originSize");
				viewer.addText(passfiltered, 10,180, 15, 1, 1, 0, "passfiltered");
				viewer.addText(voxelfiltered, 10, 160, 15, 1, 1, 0, "voxelfiltered");
				viewer.addText(statisFiltered, 10, 140, 15, 1, 1, 0, "statisFiltered");
				viewer.addText(extractedFromPlane, 10, 120, 15, 1, 1, 0, "extractedFromPlane");

				viewer.addPointCloud<pcl::PointXYZ>(statis_Filtered, "cloud");
				viewer.addPointCloud<pcl::PointXYZ>(PlaneCloud, Plane_color, "Plane");
				viewer.addPointCloud<pcl::PointXYZ>(TargetCloud, Target_color, "Target");
				viewer.addPointCloud<pcl::PointXYZ>(target_projected, target_projected_color, "target_projected");
				viewer.addPointCloud<pcl::PointXYZ>(cloud_hull, cloud_hull_color, "cloud_hull");


				viewer.spinOnce();

				cloud_mutex.unlock();
			}
		}


		interface->stop();
	}
};

int main()
{
	VolumDetectViewer v;
	v.run();
	return (0);
}