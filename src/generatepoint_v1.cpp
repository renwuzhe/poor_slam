// C++ 标准库
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
using namespace std;
//slamBase
#include <slamBase.h>
//ros
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
//opencv
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// PCL 库
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
//g2o
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
//kinect bridge
#include <kinect2_bridge/kinect2_definitions.h>



using namespace std;
//using namespace cv;

// 定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 

//global variations

cv::Mat cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
cv::Mat cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

// 点云变量
// 使用智能指针，创建一个空点云。这种指针用完会自动释放。
PointCloud::Ptr cloud ( new PointCloud );
pcl::visualization::PCLVisualizer viewer ("PCLviewer");

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
{
	cv_bridge::CvImageConstPtr pCvImage;
	pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
	pCvImage->image.copyTo(image);
}
void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix)
{
	double *itC = cameraMatrix.ptr<double>(0, 0);
	for(size_t i = 0; i < 9; ++i, ++itC)
	{
		*itC = cameraInfo->K[i];
	}
}

int callback(const sensor_msgs::Image::ConstPtr imageColor, 
		const sensor_msgs::Image::ConstPtr imageDepth,
             	const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, 
             	const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
{
	cv::Mat  color, depth;
	readCameraInfo(cameraInfoColor, cameraMatrixColor);
	readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
	readImage(imageColor, color);
	readImage(imageDepth, depth);
	// cout<<"color_width:"<<color.rows<<endl;		//960
	// cout<<"depth_width:"<<depth.rows<<endl;		//960
	// cout<<"color_height:"<<color.cols<<endl;		//540
	// cout<<"depth_height:"<<depth.cols<<endl;		//540
	// imshow("img", depth);
	//go through the depth image
	PointCloud::Ptr framecloud ( new PointCloud );
	for (int m = 0; m < depth.rows; m++)
		for(int n = 0; n < depth.cols; n++)
		{
			ushort d = depth.ptr<ushort>(m)[n];
			if(d == 0)
				continue;
			PointT p;

			//compute positon
			p.z = double(d) / camera_factor;
			p.x = (n - camera_cx) * p.z / camera_fx;
			p.y = (m - camera_cy) * p.z / camera_fy;
			//compute color
			p.b = color.ptr<uchar>(m)[n*3];
			p.g = color.ptr<uchar>(m)[n*3+1];
			p.r = color.ptr<uchar>(m)[n*3+2];
			//join p into cloud
			framecloud->points.push_back( p );
			// cout<<"get a point"<<endl;
		}
	framecloud->height = 1;
	framecloud->width = framecloud->points.size();
	cout<<"point cloud size = "<<framecloud->points.size()<<endl;
	viewer.addPointCloud (framecloud,"cloud");
	viewer.addCoordinateSystem (1.0, "cloud", 0);
	viewer.setBackgroundColor(0.3, 0.3, 0.3, 0);
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	while (!viewer.wasStopped ()){
		viewer.spinOnce (100);
	}
	
	return 0;
}





int main(int argc, char **argv)
{
	// namedWindow("img");
	// startWindowThread();
	string topicColor = "/kinect2/qhd/image_color_rect";
	string topicDepth = "/kinect2/qhd/image_depth_rect";
	string topicCameraInfoColor = "/kinect2/qhd/camera_info";
	string topicCameraInfoDepth = "/kinect2/qhd/camera_info";
	
	ros::init(argc, argv, "imageGrabber");
	ros::NodeHandle nh;

	
	const size_t queueSize = 5;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;

	
	message_filters::Subscriber<sensor_msgs::Image> *subImageColor = new message_filters::Subscriber<sensor_msgs::Image>(nh, topicColor, queueSize);
	message_filters::Subscriber<sensor_msgs::Image> *subImageDepth = new message_filters::Subscriber<sensor_msgs::Image>(nh, topicDepth, queueSize);
	message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
	message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

	message_filters::Synchronizer<ExactSyncPolicy> *syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
	syncExact->registerCallback(boost::bind(callback, _1, _2, _3, _4));
	
	
	ros::spin();
	
	// destroyWindow("img");
	return 0;
}