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
//pcl
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
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
int Index = 0;
vector<FRAME> Frames;
string detector;
string descriptor;
cv::Mat imgShow;

cv::BFMatcher matcher;
double minDis = 9999;
vector<cv::Point3f> pts_obj;
vector< cv::Point2f > pts_img;
// const double camera_factor = 1000;
// const double camera_cx = 325.5;
// const double camera_cy = 253.5;
// const double camera_fx = 518.0;
// const double camera_fy = 519.0;

// 点云变量
// 使用智能指针，创建一个空点云。这种指针用完会自动释放。
PointCloud::Ptr cloud ( new PointCloud );
// pcl::visualization::PCLVisualizer viewer ("PCLviewer");

CAMERA_INTRINSIC_PARAMETERS getCameraInfo(cv::Mat &cameraMatrix);

FRAME readFrame( int index, const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth);

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image);

void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix);

int isKeyFrame(FRAME& f1, FRAME& f2);

int callback(const sensor_msgs::Image::ConstPtr imageColor, 
		const sensor_msgs::Image::ConstPtr imageDepth,
             	const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, 
             	const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
{
	readCameraInfo(cameraInfoColor, cameraMatrixColor);
	readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
	FRAME currentFrame = readFrame(Index, imageColor, imageDepth);
	CAMERA_INTRINSIC_PARAMETERS camera = getCameraInfo(cameraMatrixDepth);
	computeKeyPointsAndDesp( currentFrame, detector, descriptor );
	
	if(Index == 0)
	{
		Frames.push_back(currentFrame);
		
	}
	else
	{
		vector<cv::DMatch> matches;
		vector< cv::DMatch > goodMatches;
		// int result = isKeyFrame(currentFrame, Frames[Index-1]);
		// cout<<"number of inliers :"<<result<<endl;
		matcher.match(Frames[Index-1].desp, currentFrame.desp, matches);
		// cout<<"find total "<<matches.size()<<" matches."<<endl;
		// cv::drawMatches(Frames[Index-1].rgb, Frames[Index-1].kp, currentFrame.rgb, currentFrame.kp, matches, imgShow);
		for ( size_t i=0; i<matches.size(); i++ )
		{
			if ( (matches[i].distance < minDis) && (matches[i].distance != 0) )
				minDis = matches[i].distance;
		}
		cout<<"min dis = "<<minDis<<endl;

		for ( size_t i=0; i<matches.size(); i++ )
		{
			if (matches[i].distance < 10*minDis)
				goodMatches.push_back( matches[i] );
		}
		cout<<"good matches="<<goodMatches.size()<<endl;
		cv::drawMatches(Frames[Index-1].rgb, Frames[Index-1].kp, currentFrame.rgb, currentFrame.kp, goodMatches, imgShow);
		cv::imshow("img", imgShow);

		for (size_t i=0; i<goodMatches.size(); i++)
		{
			// query 是第一个, train 是第二个
			cv::Point2f p = Frames[Index-1].kp[goodMatches[i].queryIdx].pt;
			// 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
			ushort d = Frames[Index-1].depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
			if (d == 0)
				continue;
			pts_img.push_back( cv::Point2f( currentFrame.kp[goodMatches[i].trainIdx].pt ) );

			// 将(u,v,d)转成(x,y,z)
			cv::Point3f pt ( p.x, p.y, d );
			cv::Point3f pd = point2dTo3d( pt, camera );
			pts_obj.push_back( pd );
		}
		cv::Mat rvec, tvec, inliers;
		// cout<<camera.fx<<endl;
		// cout<<camera.fy<<endl;
		// cout<<camera.cx<<endl;
		// cout<<camera.cy<<endl;
		// cout<<camera.scale<<endl;
		cv::solvePnPRansac( pts_obj, pts_img, cameraMatrixDepth, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );
		cout<<"inliers :"<<rvec<<endl;
		Frames.push_back(currentFrame);
	}
	Index += 1;
	cout<<Index<<endl;
	
	return 0;
}





int main(int argc, char **argv)
{
	ParameterReader pd;
	detector = pd.getData( "detector" );
	descriptor = pd.getData( "descriptor" );
	string topicColor = "/kinect2/qhd/image_color_rect";
	string topicDepth = "/kinect2/qhd/image_depth_rect";
	string topicCameraInfoColor = "/kinect2/qhd/camera_info";
	string topicCameraInfoDepth = "/kinect2/qhd/camera_info";
	
	cv::namedWindow("img");
	cv::startWindowThread();
	ros::init(argc, argv, "findKeyFrame");
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
	
	cv::destroyWindow("img");
	return 0;
}

CAMERA_INTRINSIC_PARAMETERS getCameraInfo(cv::Mat &cameraMatrix)
{
	CAMERA_INTRINSIC_PARAMETERS camera;
	camera.fx = *cameraMatrix.ptr<double>(0, 0);
	camera.fy = *cameraMatrix.ptr<double>(1, 1);
	camera.cx = *cameraMatrix.ptr<double>(0, 2);
	camera.cy = *cameraMatrix.ptr<double>(1, 2);
	camera.scale = *cameraMatrix.ptr<double>(0, 1);
	return camera;
}

FRAME readFrame( int index, const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth)
{
	FRAME f;
	readImage(imageColor, f.rgb);
	readImage(imageDepth, f.depth);
	f.frameID = index;
	return f;
}

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

int isKeyFrame(FRAME& f1, FRAME& f2)
{
	static ParameterReader pd;
	static int min_inliers = atoi( pd.getData("min_inliers").c_str() );
	static double max_norm = atof( pd.getData("max_norm").c_str() );
	static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
	static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
	static CAMERA_INTRINSIC_PARAMETERS camera = getCameraInfo(cameraMatrixDepth);
	RESULT_OF_PNP result = estimateMotion( f1, f2, camera );
	return result.inliers;
}
