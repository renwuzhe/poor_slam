// C++ 标准库
#include <iostream>
#include <string>
using namespace std;
//ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;
using namespace cv;

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 

Mat  rgb;
Mat  depth;
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;
// 点云变量
// 使用智能指针，创建一个空点云。这种指针用完会自动释放。
PointCloud::Ptr cloud ( new PointCloud );



int imageCallback_color(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr colorImage;
	try{
		colorImage = cv_bridge::toCvCopy(msg, "bgr8");
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return 0;
	}
	rgb = colorImage->image;
	// cout<<colorImage->header<<endl;
	// imshow("imgrgb", rgb);
	return 0;
}
int imageCallback_depth(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr depthImage;
	try{
		depthImage = cv_bridge::toCvCopy(msg);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return 0;
	}
	depth = depthImage->image;
	// cout<<depthImage->header<<endl;
	// imshow("imgdepth", depth);
	return 0;
}

void pclviewer(const pcl::PointCloud<PointT>::Ptr  &cloud)
{
	pcl::visualization::PCLVisualizer viewer ("PCLviewer");
	viewer.addPointCloud (cloud,"cloud");
	viewer.addCoordinateSystem (1.0, "cloud", 0);
	viewer.setBackgroundColor(0.3, 0.3, 0.3, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce (100);
		}

}

int main(int argc, char **argv){
	ros::init(argc, argv, "imageGrabber");
	ros::NodeHandle n;

	namedWindow("imgrgb");
	// namedWindow("imgdepth");
	// startWindowThread();
	//read image in
	image_transport::ImageTransport it_color(n);
	image_transport::Subscriber sub_rgb = it_color.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback_color);
	image_transport::ImageTransport it_depth(n);
	image_transport::Subscriber sub_depth = it_depth.subscribe("/kinect2/qhd/image_depth_rect", 1, imageCallback_depth);
	imshow("imgrgb", rgb);
	//go through the depth image
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
			p.b = rgb.ptr<uchar>(m)[n*3];
			p.g = rgb.ptr<uchar>(m)[n*3+1];
			p.r = rgb.ptr<uchar>(m)[n*3+2];
			//join p into cloud
			cloud->points.push_back( p );
			cout<<"get a point"<<endl;
		}
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout<<"point cloud size = "<<cloud->points.size()<<endl;
	// pclviewer(cloud);
	ros::spin();
	destroyWindow("imgrgb");
	// destroyWindow("imgdepth");
	return 0;
}