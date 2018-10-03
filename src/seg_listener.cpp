#include <ros/ros.h>
//#include "image_segmentation_node/ImageSet.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_msgs/Image_Segments.h>

#include <string>

using namespace std;

void segCallback(const image_msgs::Image_Segments set){

  string window_name;

	

  for( int i=0; i<set.image_set.size(); i++ ){

    if(i>0){
      window_name="view"+(i-1);
      cv::destroyWindow(window_name);
    }

    sensor_msgs::Image msg =set.image_set[i];
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    window_name="view"+i;
    cv::namedWindow(window_name);
    cv::imshow(window_name, cv_ptr->image);
    cv::waitKey(30);

    static int image_count = 0;                            
    std::stringstream sstream;                               
    sstream << "my_image" << image_count << ".png" ;                 
    ROS_ASSERT( cv::imwrite( sstream.str(),  cv_ptr->image ) );      
    image_count++;                                      
  }
  window_name="view"+set.image_set.size();
  cv::destroyWindow(window_name);

  // sensor_msgs::Image msg1 =set.image_set[0];
  // sensor_msgs::Image msg2 =set.image_set[1];
  // sensor_msgs::Image msg3 =set.image_set[2];

	// cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(msg1, "bgr8");
	// cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(msg2, "bgr8");
	// cv_bridge::CvImagePtr cv_ptr3 = cv_bridge::toCvCopy(msg3, "bgr8");
	
	// cv::imshow("view5", cv_ptr1->image);
 //    cv::waitKey(30);

 //    cv::imshow("view6", cv_ptr2->image);
 //    cv::waitKey(30);

 //    cv::imshow("view7", cv_ptr3->image);
 //    cv::waitKey(30);
	

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "seg_listener");
  	ros::NodeHandle nh;
  	ros::Subscriber sub = nh.subscribe("seg_images", 50, segCallback);
  	// cv::namedWindow("view5");
  	// cv::namedWindow("view6");
  	// cv::namedWindow("view7");

  	ros::Rate loop_rate(0.5);
  	while (nh.ok()){
    	ros::spinOnce();
    	loop_rate.sleep();
  	}
  // cv::destroyWindow("view5");
  // cv::destroyWindow("view6");
  // cv::destroyWindow("view7");
}