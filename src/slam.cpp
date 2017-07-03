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
typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 
///////////////////////////////////////////////////////////////////////////////////
/******************global variations***********************/
//camerainfo
cv::Mat cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
cv::Mat cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
//point cloud
PointCloud::Ptr cloud ( new PointCloud() );		//global point cloud
PointCloud::Ptr tmp ( new PointCloud() );
PointCloud::Ptr currentCloud ( new PointCloud() );
//frames
vector< FRAME > keyframes;		//keyframe queue
int Index = 0;				//frame index
//g2o part
SlamLinearSolver* linearSolver;
SlamBlockSolver* blockSolver;
g2o::OptimizationAlgorithmLevenberg* solver;
g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
//keypoints part
string detector;
string descriptor;
//keyframe part
double keyframe_threshold;
bool check_loop_closure;
//point filter
pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
// pcl::visualization::PCLVisualizer viewer ("PCLviewer");

/**********************functions**************************/
//read camerainfo
CAMERA_INTRINSIC_PARAMETERS getCameraInfo(cv::Mat &cameraMatrix);
// 给定index，读取一帧数据
FRAME readFrame( int index, const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth);
// 估计一个运动的大小
double normofTransform( cv::Mat rvec, cv::Mat tvec );
//读取图像
void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image);
//读取摄像头信息
void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix);
// 检测两个帧，结果定义
enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME}; 
// 函数声明
CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops=false );
// 检测近距离的回环
void checkNearbyLoops( vector<FRAME>& frames, FRAME& currentFrame, g2o::SparseOptimizer& opti );
// 随机检测回环
void checkRandomLoops( vector<FRAME>& frames, FRAME& currentFrame, g2o::SparseOptimizer& opti );

//回调函数
int callback(const sensor_msgs::Image::ConstPtr imageColor, 
		const sensor_msgs::Image::ConstPtr imageDepth,
             	const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, 
             	const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
{
	
	readCameraInfo(cameraInfoColor, cameraMatrixColor);
	readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
	FRAME currentFrame = readFrame(Index, imageColor, imageDepth);
	CAMERA_INTRINSIC_PARAMETERS camera = getCameraInfo(cameraMatrixDepth);
	if(Index == 0)
	{
		computeKeyPointsAndDesp( currentFrame, detector, descriptor );
		cloud = image2PointCloud( currentFrame.rgb, currentFrame.depth, camera );
		// 向globalOptimizer增加第一个顶点
		g2o::VertexSE3* v = new g2o::VertexSE3();
		v->setId( Index );
		v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
		v->setFixed( true ); //第一个顶点固定，不用优化
		globalOptimizer.addVertex( v );
		keyframes.push_back( currentFrame );
	}
	else
	{
		computeKeyPointsAndDesp( currentFrame, detector, descriptor );
		CHECK_RESULT result = checkKeyframes( keyframes.back(), currentFrame, globalOptimizer ); //匹配该帧与keyframes里最后一帧
		switch (result) // 根据匹配结果不同采取不同策略
		{
			case NOT_MATCHED:
				//没匹配上，直接跳过
				cout<<"Not enough inliers."<<endl;
				break;
			case TOO_FAR_AWAY:
			// 太近了，也直接跳
				cout<<"Too far away, may be an error."<<endl;
				break;
			case TOO_CLOSE:
			// 太远了，可能出错了
				cout<<"Too close, not a keyframe"<<endl;
				break;
			case KEYFRAME:
				cout<<"This is a new keyframe"<<endl;
			// 不远不近，刚好
			// 检测回环
				if (check_loop_closure)
				{
					checkNearbyLoops( keyframes, currentFrame, globalOptimizer );
					checkRandomLoops( keyframes, currentFrame, globalOptimizer );
				}
				keyframes.push_back( currentFrame );

				break;
			default:
				break;
		}


	}
	if((Index % 5) == 0)
	{
		globalOptimizer.initializeOptimization();
		globalOptimizer.optimize( 10 ); //可以指定优化步数
		for(size_t i = 0; i < keyframes.size(); i++)
		{
			g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
			Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
			currentCloud = image2PointCloud( keyframes[i].rgb, keyframes[i].depth, camera ); //转成点云
			voxel.setInputCloud( currentCloud );
			voxel.filter( *tmp );
			pass.setInputCloud( tmp );
			pass.filter( *currentCloud );
			pcl::transformPointCloud( *currentCloud, *tmp, pose.matrix() );
			*cloud += *tmp;
			tmp->clear();
			currentCloud->clear();
		}
	}
	Index += 1;
	return 0;
}
/*
main()
*/
int main(int argc, char **argv)
{
	/**************read parameters***********************/
	ParameterReader pd;		
	//init detector and descriptor
	detector = pd.getData( "detector" );
	descriptor = pd.getData( "descriptor" );
	keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
	check_loop_closure = pd.getData("check_loop_closure")==string("yes");
	double gridsize = atof( pd.getData( "voxel_grid" ).c_str() ); //分辨图可以在parameters.txt里调
	/*************init topic*******************************/
	string topicColor = "/kinect2/qhd/image_color_rect";
	string topicDepth = "/kinect2/qhd/image_depth_rect";
	string topicCameraInfoColor = "/kinect2/qhd/camera_info";
	string topicCameraInfoDepth = "/kinect2/qhd/camera_info";
	/*************initialize g2o****************************/
	//初始化求解器
	linearSolver = new SlamLinearSolver();
	linearSolver->setBlockOrdering( false );
	blockSolver = new SlamBlockSolver( linearSolver );
	solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );
	globalOptimizer.setAlgorithm( solver );
	//turn off test message
	globalOptimizer.setVerbose( false );
	/*****************init point cloud**********************/	
	pass.setFilterFieldName("z");
	pass.setFilterLimits( 0.0, 4.0 ); //4m以上就不要了
	voxel.setLeafSize( gridsize, gridsize, gridsize );
	/****************************************************/
	ros::init(argc, argv, "poorSlam");
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
	voxel.setInputCloud( cloud );
	voxel.filter( *tmp );
	pcl::io::savePCDFile( "home/robot/catkin_ws/src/poor_slam/result.pcd", *tmp );
	cout<<"Final map is saved."<<endl;
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

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
	return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops)
{
	static ParameterReader pd;
	static int min_inliers = atoi( pd.getData("min_inliers").c_str() );
	static double max_norm = atof( pd.getData("max_norm").c_str() );
	static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
	static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
	static CAMERA_INTRINSIC_PARAMETERS camera = getCameraInfo(cameraMatrixDepth);
	// 比较f1 和 f2
	RESULT_OF_PNP result = estimateMotion( f1, f2, camera );
	if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
		return NOT_MATCHED;
	// 计算运动范围是否太大
	double norm = normofTransform(result.rvec, result.tvec);
	if ( is_loops == false )
	{
		if ( norm >= max_norm )
			return TOO_FAR_AWAY;   // too far away, may be error
	}
	else
	{
		if ( norm >= max_norm_lp)
			return TOO_FAR_AWAY;
	}

	if ( norm <= keyframe_threshold )
		return TOO_CLOSE;   // too adjacent frame
	// 向g2o中增加这个顶点与上一帧联系的边
	// 顶点部分
	// 顶点只需设定id即可
	if (is_loops == false)
	{
		g2o::VertexSE3 *v = new g2o::VertexSE3();
		v->setId( f2.frameID );
		v->setEstimate( Eigen::Isometry3d::Identity() );
		opti.addVertex(v);
	}
	// 边部分
	g2o::EdgeSE3* edge = new g2o::EdgeSE3();
	// 连接此边的两个顶点id
	edge->setVertex( 0, opti.vertex(f1.frameID ));
	edge->setVertex( 1, opti.vertex(f2.frameID ));
	edge->setRobustKernel( new g2o::RobustKernelHuber() );
	// 信息矩阵
	Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
	// 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
	// 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
	// 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
	information(0,0) = information(1,1) = information(2,2) = 100;
	information(3,3) = information(4,4) = information(5,5) = 100;
	// 也可以将角度设大一些，表示对角度的估计更加准确
	edge->setInformation( information );
	// 边的估计即是pnp求解之结果
	Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
	// edge->setMeasurement( T );
	edge->setMeasurement( T.inverse() );
	// 将此边加入图中
	opti.addEdge(edge);
	return KEYFRAME;
}

void checkNearbyLoops( vector<FRAME>& frames, FRAME& currentFrame, g2o::SparseOptimizer& opti )
{
	static ParameterReader pd;
	static int nearby_loops = atoi( pd.getData("nearby_loops").c_str() );
    
    // 就是把currentFrame和 frames里末尾几个测一遍
	if ( frames.size() <= nearby_loops )
	{
		// no enough keyframes, check everyone
		for (size_t i=0; i<frames.size(); i++)
		{
			checkKeyframes( frames[i], currentFrame, opti, true );
		}
	}
	else
	{
		// check the nearest ones
		for (size_t i = frames.size()-nearby_loops; i<frames.size(); i++)
		{
			checkKeyframes( frames[i], currentFrame, opti, true );
		}
	}
}

void checkRandomLoops( vector<FRAME>& frames, FRAME& currentFrame, g2o::SparseOptimizer& opti )
{
	static ParameterReader pd;
	static int random_loops = atoi( pd.getData("random_loops").c_str() );
	srand( (unsigned int) time(NULL) );
	// 随机取一些帧进行检测
    
	if ( frames.size() <= random_loops )
	{
		// no enough keyframes, check everyone
		for (size_t i=0; i<frames.size(); i++)
		{
			checkKeyframes( frames[i], currentFrame, opti, true );
		}
	}
	else
	{
		// randomly check loops
		for (int i=0; i<random_loops; i++)
		{
			int index = rand()%frames.size();
			checkKeyframes( frames[index], currentFrame, opti, true );
		}
	}
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
