#include <iostream>                         //常用输入输出头文件
#include <vector>

#include <pcl/io/openni2_grabber.h>         //OpenNI采集头文件

#include <pcl/ModelCoefficients.h>          //采样一致性模型相关类头文件

#include <pcl/point_types.h>                //点云数据 点类型头文件

#include <pcl/visualization/cloud_viewer.h> //可视化头文件

#include <pcl/filters/passthrough.h>                    //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>                     //体素滤波器头文件
#include <pcl/filters/extract_indices.h>                //索引提取滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>    //统计滤波头文件
#include <pcl/filters/project_inliers.h>                //映射相关头文件

#include <pcl/sample_consensus/method_types.h>  //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件

#include <pcl/segmentation/sac_segmentation.h>  //基于采样一致性分割的类的头文件

#include <pcl/surface/concave_hull.h>           //提取凸（凹）多边形的头文件

//#include <opencv2/opencv.hpp>   //opencv头文件

using namespace std;
//using namespace cv;

class SimpleOpenNIProcessor
{
public:
	pcl::visualization::CloudViewer viewer; // Cloudviewer显示对象

											// 直通滤波器对象
	pcl::PassThrough<pcl::PointXYZ> XpassFilter;
	pcl::PassThrough<pcl::PointXYZ> YpassFilter;

	// 下采样 VoxelGrid 滤波对象
	pcl::VoxelGrid<pcl::PointXYZ> VoxlFilter;

	// 创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> segPlane;

	// 提取索引滤波器
	pcl::ExtractIndices<pcl::PointXYZ> extractPlane;

	// 统计滤波器
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisFilter;

	// 点云投影滤波对象
	pcl::ProjectInliers<pcl::PointXYZ> projec;
	int heightCount = 0;
	float heightSum = 0.0;

	int areaCount = 0;
	float areaSum = 0.0;


	SimpleOpenNIProcessor() : viewer("PCL OpenNI Viewer") {
		// 滤波字段设为x轴方向
		XpassFilter.setFilterFieldName("x");
		// 设定可接受范围，将不在范围内的点过滤掉或者保留（由setFilterLimitsNegative决定）
		XpassFilter.setFilterLimits(-0.25, 0.05);  // TODO: X方向裁剪阈值需要调试 实际是0.44m

												   // 滤波字段设为y轴方向
		YpassFilter.setFilterFieldName("y");
		// 设定可接受范围，将不在范围内的点过滤掉或者保留（由setFilterLimitsNegative决定）
		YpassFilter.setFilterLimits(-0.27, 0.18);  // TODO: Y方向裁剪阈值需要调试 实际是0.69m

												   // 设置体素滤波时创建的体素体积为0.5*0.5*0.5cm的立方体
		VoxlFilter.setLeafSize(0.005f, 0.005f, 0.005f);

		// 统计滤波初始设置
		statisFilter.setMeanK(50);  // 对每个点分析的临近点个数设为50
		statisFilter.setStddevMulThresh(1.0);   // 将标准差倍数设为1，意味着一个点的距离超出平均距离1个标准差以上，就会被标记为离群点，并被移除。

												// 可选：设置模型系数需要优化
		segPlane.setOptimizeCoefficients(true);
		// 必选：设置分割的模型类型、所用的随机参数估计方法、距离阈值、迭代次数上限
		segPlane.setModelType(pcl::SACMODEL_PLANE);
		segPlane.setMethodType(pcl::SAC_RANSAC);
		segPlane.setDistanceThreshold(0.01);    // TODO: 距离阈值需要调试
		segPlane.setMaxIterations(1000);

		// 设置投影模型为SACMODEL_PLANE
		projec.setModelType(pcl::SACMODEL_PLANE);
	}  // Construct a cloud viewer, with a window name

  // 定义回调函数cloud_cb_,获取到数据时对数据进行处理
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
	{
		heightCount++;
		areaCount++;

		cout << "原始点云大小：" << cloud->points.size() << endl;

		// 1. 空间裁剪
		pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		XpassFilter.setInputCloud(cloud);
		XpassFilter.filter(*pass_filtered);

		YpassFilter.setInputCloud(pass_filtered);
		YpassFilter.filter(*pass_filtered);

		cout << "空间裁剪后点云大小：" << pass_filtered->points.size() << endl;

		// 2. 下采样进行数据压缩
		// 下采样
		pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		VoxlFilter.setInputCloud(pass_filtered);
		VoxlFilter.filter(*voxel_filtered);

		cout << "下采样后点云大小：" << voxel_filtered->points.size() << endl;

		// 3. TODO: 添加一种滤波，删除异常点
		// 3. 统计滤波 去除异常点
		pcl::PointCloud<pcl::PointXYZ>::Ptr statisFiltered(new pcl::PointCloud<pcl::PointXYZ>);
		statisFilter.setInputCloud(pass_filtered);
		statisFilter.filter(*statisFiltered);

		// 3. 平面分割 获取地面
		// 创建分割时所需要的模型系数对象 coefficientsPlane
		// 创建存储模型内点的点索引集合对象 inliersPlane
		pcl::ModelCoefficients::Ptr coefficientsPlane(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliersPlane(new pcl::PointIndices());

		segPlane.setInputCloud(statisFiltered);
		segPlane.segment(*inliersPlane, *coefficientsPlane);
		if (inliersPlane->indices.size() == 0) {
			cout << "平面分割失败，没有找到平面" << endl;
		}
		//        cout << "平面分割，地面平面索引数目为" << inliersPlane->indices.size() << endl;
		//        cout << "平面分割，地面平面模型：ax+by+cz+d=0形式，其中" << ", a = " << coefficientsPlane->values[0]
		//             << ", b = " << coefficientsPlane->values[1]
		//             << ", c = " << coefficientsPlane->values[2]
		//             << ", d = " << coefficientsPlane->values[3] << endl;

		// 3.1 绘制平面颜色
		//        // 3.1.1 复制一份平面点云，用以作为底部轮廓提取的数据
		//        pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneCloud(new pcl::PointCloud<pcl::PointXYZ>);
		//        pcl::copyPointCloud(*voxel_filtered, inliersPlane->indices, *PlaneCloud);

		// 3.1.2 拷贝一份RGB点云，用以标记颜色
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::copyPointCloud(*statisFiltered, *ColorCloud);

		// 3.1.3 对底部平面内点进行颜色标记 // TODO: 有需要的话可以在这里将indcies保存一下
		for (int i = 0; i < inliersPlane->indices.size(); ++i) {
			ColorCloud->points[inliersPlane->indices[i]].r = 255;
			ColorCloud->points[inliersPlane->indices[i]].g = 0;
			ColorCloud->points[inliersPlane->indices[i]].b = 0;
		}

		// 4. 根据索引提取除地面之外的点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr extracted(new pcl::PointCloud<pcl::PointXYZ>);
		extractPlane.setInputCloud(statisFiltered);
		extractPlane.setIndices(inliersPlane);
		extractPlane.setNegative(true); // 设置Negative是获取除了平面的点
		extractPlane.filter(*extracted);
		cout << "索引滤波，除去地面后点云大小" << extracted->points.size() << endl;

		// 5. 平面分割 获取目标物上表面
		pcl::ModelCoefficients::Ptr coefficientsTarget(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliersTarget(new pcl::PointIndices());

		segPlane.setInputCloud(extracted);
		segPlane.segment(*inliersTarget, *coefficientsTarget);
		if (inliersPlane->indices.size() == 0) {
			cout << "平面分割失败，没有找到平面" << endl;
			return;
		}
		//        cout << "平面分割，目标顶面索引数目为" << inliersTarget->indices.size() << endl;
		//        cout << "平面分割，目标顶面平面模型：ax+by+cz+d=0形式，其中" << ", a = " << coefficientsTarget->values[0]
		//             << ", b = " << coefficientsTarget->values[1]
		//             << ", c = " << coefficientsTarget->values[2]
		//             << ", d = " << coefficientsTarget->values[3] << endl;


		// TODO: 此处可以计算物体的高度
		//        float height = abs(coefficientsPlane->values[3] - coefficientsTarget->values[3]);
		// 改进计算
		float height = abs(coefficientsPlane->values[3] / coefficientsPlane->values[2] -
			coefficientsTarget->values[3] / coefficientsPlane->values[2]);
		heightSum += height;
		float avheight = heightSum / heightCount;
		cout << "物体的高度为 height = " << avheight << "m" << endl;

		// 5.1 绘制目标顶面颜色
		//        // 5.1.1 复制一份目标顶面点云，用以作为底部轮廓提取的数据
		//        pcl::PointCloud<pcl::PointXYZ>::Ptr TargetCloud(new pcl::PointCloud<pcl::PointXYZ>);
		//        pcl::copyPointCloud(*extracted, inliersTarget->indices, *TargetCloud);

		// 5.1.2 顶面着色
		for (int i = 0; i < inliersTarget->indices.size(); ++i) {
			ColorCloud->points[inliersTarget->indices[i]].r = 0;
			ColorCloud->points[inliersTarget->indices[i]].g = 255;
			ColorCloud->points[inliersTarget->indices[i]].b = 0;
		}

		// 6. 投影平面模型系数
		pcl::PointCloud<pcl::PointXYZ>::Ptr target_projected(new pcl::PointCloud<pcl::PointXYZ>);

		projec.setIndices(inliersTarget);
		projec.setInputCloud(statisFiltered);
		projec.setModelCoefficients(coefficientsTarget);
		projec.filter(*target_projected);
		cout << "投影，投影后点数目为" << target_projected->points.size() << endl;

		// 7. 为投影的点创建凹包表征
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConcaveHull<pcl::PointXYZ> chull;

		//polygons存储一系列顶点集，每组顶点集表示一个多边形 //应该取其中第0个顶点集
		std::vector<pcl::Vertices> polygons;
		chull.setInputCloud(target_projected);
		chull.setAlpha(0.1);

		// polygons对应的是前面point cloud_hull的索引
		chull.reconstruct(*cloud_hull, polygons);

		cout << "凹包数目为" << cloud_hull->points.size() << endl;
		cout << "检测到的凹包数目为" << polygons.size() << endl;
		cout << "检测到的凹包点集数目为" << polygons[0].vertices.size() << endl;

		//       //  7.1 输出凹包点集查看
		//        for (int j = 0; j < cloud_hull->points.size(); ++j) {
		//            cout << cloud_hull->points[j].x << " " <<  cloud_hull->points[j].y << " " <<  cloud_hull->points[j].z << endl;
		//        }
		// 7.1.2 获取二维轮廓供OpenCV处理

		// 生成二维轮廓
		//std::vector<cv::Point2f> hullContour;
		//for (int j = 0; j < polygons[0].vertices.size(); ++j) {
		//	hullContour.push_back(cv::Point2f(cloud_hull->points[polygons[0].vertices[j]].x,
		//		cloud_hull->points[polygons[0].vertices[j]].y));
		//}

		//cv::RotatedRect minRect = cv::minAreaRect(hullContour);

		//// 这里的长宽有问题
		//cout << "最小外接矩形的长： " << minRect.size.width << " 宽： " << minRect.size.height << " 面积： " << minRect.size.area()
		//	<< " 角度： " << minRect.angle << endl;


		//         7.2 根据顶点集，在彩色点云中标出凹包 TODO: 问题：这里的点集与彩色空间中的点集对应不上

		//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorHull(new pcl::PointCloud<pcl::PointXYZRGB>);
		//        pcl::copyPointCloud(*cloud_hull, *ColorHull);
		//        for (int j = 0; j < polygons[0].vertices.size(); ++j) {
		//            ColorHull->points[polygons[0].vertices[j]].r = 0;
		//            ColorHull->points[polygons[0].vertices[j]].g = 0;
		//            ColorHull->points[polygons[0].vertices[j]].b = 255;
		//        }

		// 8. 计算凹包面积
		// 8.1 自带函数计算
		// 面积的计算方式是没有问题的，问题可能出在上表面的检测上
		float area = pcl::calculatePolygonArea(*cloud_hull);

		// 8.2数学原理计算
		//        float area = 0.0;
		//        float prevX = cloud_hull->points[0].x;
		//        float prevY = cloud_hull->points[0].y;
		//        for (int j = 0; j < cloud_hull->points.size(); ++j) {
		//            float nowX = cloud_hull->points[j].x;
		//            float nowY = cloud_hull->points[j].y;
		//            area += prevX*nowY - prevY*nowX;
		//            prevX = nowX;
		//            prevY = nowY;
		//        }
		//        area = area * 0.5;

		cout << "数值面积为 " << area << endl;
		float realArea = area / (0.3 * 0.45) * (0.69 * 0.43);

		areaSum += realArea;
		float avarea = areaSum / areaCount;

		cout << "实际面积为 " << avarea << "平方米" << endl;

		float realVolum = height * avarea;
		cout << "实际体积为 " << realVolum << "立方米" << endl << endl << endl;


		if (!viewer.wasStopped()) // Check if the gui was quit. true if the user signaled the gui to stop
			viewer.showCloud(ColorCloud);
	}

  void run ()
  {
	  // 新建OpenNI设备的捕获器 grabber
	  pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

	  // 建立回调
	  boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
		  boost::bind(&SimpleOpenNIProcessor::cloud_cb_, this, _1);

	  interface->registerCallback(f);

	  interface->start();

	  while (!viewer.wasStopped())
	  {
		  //boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	  }

	  interface->stop();
  }

};

int main ()
{
  SimpleOpenNIProcessor v;
  v.run ();
  return (0);
}
