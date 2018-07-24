#include <ros/ros.h>
#include "image_segmentation_node/ImageSet.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void segCallback(const image_segmentation_node::ImageSet set){
	sensor_msgs::Image msg1 =set.data[0];
	sensor_msgs::Image msg2 =set.data[1];
	sensor_msgs::Image msg3 =set.data[2];

	cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(msg1, "bgr8");
	cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(msg2, "bgr8");
	cv_bridge::CvImagePtr cv_ptr3 = cv_bridge::toCvCopy(msg3, "bgr8");
	
	cv::imshow("view5", cv_ptr1->image);
    cv::waitKey(30);

    cv::imshow("view6", cv_ptr2->image);
    cv::waitKey(30);

    cv::imshow("view7", cv_ptr3->image);
    cv::waitKey(30);
	

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "seg_listener");
  	ros::NodeHandle nh;
  	ros::Subscriber sub = nh.subscribe("seg_images", 50, segCallback);
  	cv::namedWindow("view5");
  	cv::namedWindow("view6");
  	cv::namedWindow("view7");

  	ros::Rate loop_rate(0.5);
  	while (nh.ok()){
    	ros::spinOnce();
    	loop_rate.sleep();
  	}
  cv::destroyWindow("view5");
  cv::destroyWindow("view6");
  cv::destroyWindow("view7");
}