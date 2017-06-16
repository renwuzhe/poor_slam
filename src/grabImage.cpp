#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
int imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try{
		imshow("img", cv_bridge::toCvShare(msg, "bgr8")->image);
	}catch(cv_bridge::Exception &e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	return 0;
}
int main(int argc, char **argv){
	ros::init(argc, argv, "imageGrabber");
	ros::NodeHandle n;

	namedWindow("img");
	startWindowThread();

	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback);
	ros::spin();
	destroyWindow("img");
	return 0;
}