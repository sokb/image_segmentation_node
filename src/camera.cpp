#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <stdlib.h>

int main(int argc, char** argv)
{
  
  std::cout<<"hello!"<<std::endl;

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 2);

  //int* x;
  // if(argc>2){	//orismata:  prog path_eikonas x1 x2 ... xn
  // 	if(argc%2!=0){
  // 		std::cout<<"Please enter x-values in pairs"<<std::endl;
  // 	}
  // 	else{
  // 		x= (int*)malloc((argc-2)*sizeof(int));
  // 		int i=2;	//starts from first x-value
  // 		int j=0;
  // 		while(i<argc){
  // 			x[j]=atoi(argv[i]);
  // 			std::cout<<"x"<<j<<"= "<<x[j]<<std::endl;
  // 			i++;j++;
  // 		}
  // 	}
  // }
  
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  
  std::cout<<"Height is: "<< image.rows<<" and width is: "<<image.cols<<std::endl;

  // cv::Rect myROI(0, 0, image.cols/3,image.rows); 
  // cv::Mat roi = cv::Mat(image,myROI);

  // cv::Rect myROI2(image.cols-(image.cols/3), 0, image.cols/3,image.rows); 
  // cv::Mat roi2 = cv::Mat(image,myROI2);

  // cv::Rect myROI3(image.cols/3,0,image.cols/3,image.rows);
  // cv::Mat roi3= cv::Mat(image,myROI3);

  cv::Rect myROI(0, 0, image.cols/3,image.rows); 
  cv::Mat roi = cv::Mat(image,myROI);

  cv::Rect myROI2(image.cols/3, 0, image.cols/3,image.rows); 
  cv::Mat roi2 = cv::Mat(image,myROI2);

  cv::Rect myROI3(2*(image.cols/3),0,image.cols/3,image.rows);
  cv::Mat roi3= cv::Mat(image,myROI3);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi).toImageMsg();
  sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi2).toImageMsg();
  sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi3).toImageMsg();
  ros::Rate loop_rate(0.5);
  while (nh.ok()) {
    pub.publish(msg);
    pub.publish(msg2);
    pub.publish(msg3);
    ros::spinOnce();
    loop_rate.sleep();
  }
  //free(x);
}