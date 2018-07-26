#include <ros/ros.h>
#include <iostream>
#include "image_segmentation_node/ImageSet.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>

#include <pointcloud_msgs/PointCloud2_Segments.h>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// void pcl_Callback(const PointCloud::ConstPtr& msg)
// {
//   printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//   BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//     printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
// }

void pcl_seg_Callback(const pointcloud_msgs::PointCloud2_Segments& msg){
	//pointcloud_msgs::PointCloud2_Segments msg_out;
	double angle_min = msg.angle_min;
	double angle_max = msg.angle_max;
	double angle_increment = msg.angle_increment;

    pcl::PCLPointCloud2 pc2;
    pcl_conversions::toPCL ( msg.clusters[0] , pc2 );	//from sensor_msgs::pointcloud2 to pcl::pointcloud2

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromPCLPointCloud2 ( pc2 , pc );	//from pcl::pointcloud2 to pcl::pointcloud

    Eigen::Vector4f cluster_centroid;
    pcl::compute3DCentroid ( pc , cluster_centroid);	// (x,y,z,1)
    //cluster_centroid_vec.push_back( cluster_centroid );

    std::cout << "Centroid of first cluster is:\nx= " << cluster_centroid(0) << "\ny= " << cluster_centroid(1) << "\nz= " << cluster_centroid(2) << "\nlast= " << cluster_centroid(3) << "\n\n\n\n";

	// //pcl1 test
	// Eigen::Vector4f cluster_centroid;
	// sensor_msgs::PointCloud conv_msg;
	// convertPointCloud2ToPointCloud(msg.clusters[0], conv_msg);

    

 //    Eigen::Vector4f cluster_centroid;
	// convertPointCloud2ToPointCloud( msg.clusters[0], conv_msg);
 //    pcl::compute3DCentroid( cloud2 , cluster_centroid);

	// while(i < msg.cluster_id.size()){
	// 	std::cout << "i= " << i << ": " << msg.cluster_id[i] << std::endl << std::endl;
	// 	i++;
	// }

	//std::cout << "DATA OF FIRST CLUSTER: "<< std::endl << std::endl << msg.clusters[0] << std::endl << std::endl;
	//std::cout << "DATA OF SECOND CLUSTER: "<< std::endl << std::endl << msg.clusters[1] << std::endl << std::endl;
	
	// sensor_msgs::PointCloud conv_msg;
	//convertPointCloud2ToPointCloud(msg.clusters[0], conv_msg);

	// int i=0;
	// while(i< msg.clusters.size()){
	// 	std::cout << conv_msg.points[i] << std::endl;
	// 	i++;
	// }
	std::cout<<"\n\n\n\n";

}


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

    image_segmentation_node::ImageSet set;
    //convert to sensor_msgs/Image
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi).toImageMsg();
    set.data[0]= *msg1;
    msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi2).toImageMsg();
    set.data[1]= *msg1;
    msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi3).toImageMsg();
    set.data[2]= *msg1;
    new_msg=set.data[0];

    // cv::imshow("view2", roi);
    // cv::waitKey(30);

    // cv::imshow("view3", roi2);
    // cv::waitKey(30);

    // cv::imshow("view4", roi3);
    // cv::waitKey(30);

    //tpub.publish(msg1);

    //tpub.publish(new_msg);
    pub.publish(set);
    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	// ROS_INFO("angle_min is: %f\n",msg->angle_min);
	// ROS_INFO("angle_max is: %f\n",msg->angle_max);
	// ROS_INFO("angle_increment is: %f\n",msg->angle_increment);
	// ROS_INFO("time_increment is: %f\n",msg->time_increment);
	// ROS_INFO("scan_time is: %f\n",msg->scan_time);
	// ROS_INFO("range_min is: %f\n",msg->range_min);
	// ROS_INFO("range_max is: %f\n",msg->range_max);
	// unsigned long int i=0;
	// ROS_INFO("Size of ranges[] = %lu",msg->ranges.size()); //size=360
	// // std::cout << "[" << std::endl;
	// // while(i<msg->ranges.size()){
	// // 	std::cout << i << ": " << msg->ranges[i]<< " ";
	// // 	i++;
	// // }
	// // std::cout << "]" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_segmentation_node");
  ros::NodeHandle nh;

  //ROS_INFO("angle_max is 2.345\n");

  cv::namedWindow("view");
  // cv::namedWindow("view2");
  // cv::namedWindow("view3");
  // cv::namedWindow("view4");
  image_transport::ImageTransport it(nh);

  //advertise to output topic
  //image_transport::Publisher pub = it.advertise("seg_images", 2);
  pub = nh.advertise<image_segmentation_node::ImageSet>("seg_images", 2);
  tpub = it.advertise("normal_image", 2);

  // image_transport::Subscriber image_sub = it.subscribe("camera/image", 50, imageCallback);
  // ros::Subscriber pcl_sub = nh.subscribe<PointCloud>("points2", 1, pcl_Callback);

  ros::Subscriber pcl_seg_sub = nh.subscribe<const pointcloud_msgs::PointCloud2_Segments&>("pointcloud2_cluster_tracking/clusters", 1, pcl_seg_Callback);
  image_transport::Subscriber video_sub = it.subscribe("usb_cam/image_raw", 50, videoCallback);
  ros::Subscriber laser_sub = nh.subscribe("scan",50, laserCallback);

  ros::Rate loop_rate(0.5);
  while (nh.ok()){
    // pub.publish(msg);
    // pub.publish(msg2);
    // pub.publish(msg3);
    ros::spinOnce();
    loop_rate.sleep();
  }
  cv::destroyWindow("view");
  // cv::destroyWindow("view2");
  // cv::destroyWindow("view3");
  // cv::destroyWindow("view4");
}