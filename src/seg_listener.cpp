#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_msgs/Image_Segments.h>

#include <string>

using namespace std;
int message_count = 0; 

void segCallback(const image_msgs::Image_Segments set){
	
	int image_count = 0; 
	for( int i=0; i<set.image_set.size(); i++ ){

		sensor_msgs::Image msg =set.image_set[i];
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");                       
		std::stringstream sstream;                               
		sstream << "my_image_" << message_count << "_" << image_count << ".png" ;                 
		ROS_ASSERT( cv::imwrite( sstream.str(),  cv_ptr->image ) );      
		image_count++;                                      
	}

	std::cout << "Printing Header:\n" << set.header << std::endl;
	message_count++;

}


int main(int argc, char **argv){

	ros::init(argc, argv, "seg_listener");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("seg_images", 50, segCallback);
	ros::spin();

}