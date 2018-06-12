# PointCloudVolumeMeasure

代码实现了Kinect V2 + PCL 实现简单地方体目标的体积测量，后续方案优化和界面优化将迁移至实习SVN库中。

## 驱动及PCL库安装

[Windows10下配置KinectV2以支持OpenNI2和PCL](https://shenxiaohai.me/2018/04/23/Win10-Kinect/) 

[Ubuntu16.04下编译libfreenect2和PCL以支持KinectV2点云处理](https://shenxiaohai.me/2018/04/25/Ubuntu-freenect2-PCL/)

## 流程

1. 捕获点云
   ![捕获点云](https://github.com/veraposeidon/PointCloudVolumeMeasure/blob/master/imgRecord/originCloud.png)
2. 空间裁剪
   ![空间裁剪](https://github.com/veraposeidon/PointCloudVolumeMeasure/blob/master/imgRecord/passThrough.png)
3. 下采样
   ![下采样](https://github.com/veraposeidon/PointCloudVolumeMeasure/blob/master/imgRecord/voxelFiltered.png)
4. 滤波
   ![滤波](https://github.com/veraposeidon/PointCloudVolumeMeasure/blob/master/imgRecord/statisFiltered.png)
5. 平面分割找地面
   ![平面分割](https://github.com/veraposeidon/PointCloudVolumeMeasure/blob/master/imgRecord/PlaneSeg.png)
6. 平面分割找目标顶面
7. 计算面积、高度及体积
   ![GIF](https://github.com/veraposeidon/PointCloudVolumeMeasure/blob/master/imgRecord/GIF.gif)
