#include <ros/ros.h>
#include "image_project/ImageSet.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// void pcl_Callback(const PointCloud::ConstPtr& msg)
// {
//   printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//   BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//     printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
// }

// void pcl_seg_Callback(const pointcloud_msgs::PointCloud2_Segments msg){
//   int i=0;
// }

// void imageCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//   std_msgs::Header h = msg->header;
//   try
//   {
//     if(h.seq%3==0){
//       cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
//       cv::waitKey(30);
//     }
//     else if(h.seq%3==1){
//       cv::imshow("view2", cv_bridge::toCvShare(msg, "bgr8")->image);
//       cv::waitKey(30);
//     }
//     else{
//       cv::imshow("view3", cv_bridge::toCvShare(msg, "bgr8")->image);
//       cv::waitKey(30);
//     }
    
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//   }
// }

ros::Publisher pub;
image_transport::Publisher tpub;

void videoCallback(const sensor_msgs::ImageConstPtr& msg){
	std_msgs::Header h = msg->header;
	sensor_msgs::Image new_msg;
  try
  {
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv::waitKey(30);
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::imshow("view",cv_ptr->image);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //cut a rectangle
    cv::Rect myROI(0, 0, (cv_ptr->image.cols)/3, cv_ptr->image.rows); 
    cv::Mat roi = cv::Mat(cv_ptr->image,myROI);

    cv::Rect myROI2(cv_ptr->image.cols/3, 0, cv_ptr->image.cols/3,cv_ptr->image.rows); 
    cv::Mat roi2 = cv::Mat(cv_ptr->image,myROI2);

    cv::Rect myROI3(2*(cv_ptr->image.cols/3),0,cv_ptr->image.cols/3,cv_ptr->image.rows);
    cv::Mat roi3= cv::Mat(cv_ptr->image,myROI3);

    image_project::ImageSet set;
    //convert to sensor_msgs/Image
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi).toImageMsg();
    set.data[0]= *msg1;
    msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi2).toImageMsg();
    set.data[1]= *msg1;
    msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi3).toImageMsg();
    set.data[2]= *msg1;
    new_msg=set.data[0];

    

    cv::imshow("view2", roi);
    cv::waitKey(30);

    cv::imshow("view3", roi2);
    cv::waitKey(30);

    cv::imshow("view4", roi3);
    cv::waitKey(30);

    //tpub.publish(msg1);
    tpub.publish(new_msg);
    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_segmentation_node");
  ros::NodeHandle nh;

  cv::namedWindow("view");
  cv::namedWindow("view2");
  cv::namedWindow("view3");
  cv::namedWindow("view4");
  //cv::startWindowThread(); //DO NOT INCLUDE
  image_transport::ImageTransport it(nh);

  //advertise to output topic
  //image_transport::Publisher pub = it.advertise("seg_images", 2);
  pub = nh.advertise<image_project::ImageSet>("seg_images", 2);
  tpub = it.advertise("normal_image", 2);

  // image_transport::Subscriber image_sub = it.subscribe("camera/image", 50, imageCallback);
  // ros::Subscriber pcl_sub = nh.subscribe<PointCloud>("points2", 1, pcl_Callback);
  // ros::Subscriber pcl_seg_sub = nh.subscribe<pointcloud_msgs::PointCloud2_Segments>("pointcloud2_clustering/clusters", 1, pcl_seg_Callback);
  image_transport::Subscriber video_sub = it.subscribe("usb_cam/image_raw", 50, videoCallback);

  //ros::spin();

  //PUBLISHER CODE
  
  //cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  //std::cout<<"Height is: "<< image.rows<<" and width is: "<<image.cols<<std::endl;

  // cv::Rect myROI(0, 0, image.cols/3,image.rows); 
  // cv::Mat roi = cv::Mat(image,myROI);

  // cv::Rect myROI2(image.cols-(image.cols/3), 0, image.cols/3,image.rows); 
  // cv::Mat roi2 = cv::Mat(image,myROI2);

  // cv::Rect myROI3(image.cols/3,0,image.cols/3,image.rows);
  // cv::Mat roi3= cv::Mat(image,myROI3);

  // cv::Rect myROI(0, 0, image.cols/3,image.rows); 
  // cv::Mat roi = cv::Mat(image,myROI);

  // cv::Rect myROI2(image.cols/3, 0, image.cols/3,image.rows); 
  // cv::Mat roi2 = cv::Mat(image,myROI2);

  // cv::Rect myROI3(2*(image.cols/3),0,image.cols/3,image.rows);
  // cv::Mat roi3= cv::Mat(image,myROI3);

  
  // sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi2).toImageMsg();
  // sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi3).toImageMsg();
  ros::Rate loop_rate(0.5);
  while (nh.ok()){
    // pub.publish(msg);
    // pub.publish(msg2);
    // pub.publish(msg3);
    ros::spinOnce();
    loop_rate.sleep();
  }
  cv::destroyWindow("view");
  cv::destroyWindow("view2");
  cv::destroyWindow("view3");
  cv::destroyWindow("view4");
}