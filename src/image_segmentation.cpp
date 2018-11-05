#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <fstream>
#include <algorithm>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>

#include <pointcloud_msgs/PointCloud2_Segments.h>
#include <image_msgs/Image_Segments.h>

#define LOGFILE	"imseg.log"
#define WIDENESS_ANGLE M_PI/3 	//total camera field of view (horizontal) in rads

using namespace std;
ros::Publisher pub;
image_transport::Publisher tpub;

std::vector<std::pair<double,double>> pair_vector;
sensor_msgs::Image latest_frame;

int first_frame = 0;

const double PI = 3.141592653589793;

void Log (uint32_t duration_nsecs, std::string message){	// logs a message to LOGFILE

	std::ofstream ofs;
	ofs.open(LOGFILE, std::ofstream::out | std::ios::app);
	ofs << duration_nsecs << " nsecs : " << message << std::endl;
	ofs.close();
}


void pcl_seg_Callback(const pointcloud_msgs::PointCloud2_Segments& msg){

	if(first_frame == 0){
		return;
	}

	image_msgs::Image_Segments out_msg;

	std::vector<std::pair<double,double>> pixel_vector;

	pair_vector.clear();
	double angle_min = msg.angle_min;
	double angle_max = msg.angle_max;
	double angle_increment = msg.angle_increment;

	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(latest_frame, "bgr8");

	
	ros::WallDuration dur;
	ros::WallDuration z_dur;
	uint32_t nsecs = 0;
	uint32_t z_nsecs = 0;

	//Timestamp: "Start" (Just got the pcl_segments message)
	ros::WallTime start = ros::WallTime::now();

	pcl::PointCloud<pcl::PointXYZ> pcz; 			//pcz contains all points with max z

	std::cout << "\n________NEW MESSAGE________\n" << std::endl;

	pcl::PCLPointCloud2 pc2_temp;
	pcl_conversions::toPCL ( msg.clusters[0] , pc2_temp );	//from sensor_msgs::pointcloud2 to pcl::pointcloud2

	pcl::PointCloud<pcl::PointXYZ> pc_temp;
	pcl::fromPCLPointCloud2 ( pc2_temp , pc_temp );				//from pcl::pointcloud2 to pcl::pointcloud

	// Timestamp: "z_start_time" (Starting maximum z pointcloud extraction process)
	ros::WallTime z_start_time = ros::WallTime::now();

	double max_z = pc_temp.points[0].z;
	for (int i=1; i < pc_temp.points.size(); i++){		//find max z of cluster	
		if(pc_temp.points[i].z > max_z){
			max_z = pc_temp.points[i].z;
		}
	}
	std::cout << "Max z = " << max_z << std::endl;

	// Timestamp: "z_stop_time" (End of maximum z pointcloud extraction process)
	ros::WallTime z_stop_time = ros::WallTime::now();
	z_dur = z_stop_time - z_start_time;
	z_nsecs = z_dur.toNSec();	// toNSec() returns uint64_t
	Log(z_nsecs,"Duration of Slice Extraction Process (all clusters)");

	std::cout << "\nz_start_time is: " << z_start_time.toNSec() << "\nz_stop_time is: " << z_stop_time.toNSec() << "\nz_dur is: " << z_dur << "\nz_nsecs is: " << z_nsecs << std::endl;

	for (int j=0; j < msg.clusters.size(); j++){		//for every cluster

		pcz.clear();

		int image_counter = 0;							//counter used for image_set array
		double angle_l, angle_r, c_angle_l, c_angle_r;
		std::pair<double,double> angle_pair(0,0);
		//std::pair<double,double> pixel_pair(0,0);

		pcl::PCLPointCloud2 pc2;
		pcl_conversions::toPCL ( msg.clusters[j] , pc2 );	//from sensor_msgs::pointcloud2 to pcl::pointcloud2

		pcl::PointCloud<pcl::PointXYZ> pc;
		pcl::fromPCLPointCloud2 ( pc2 , pc );				//from pcl::pointcloud2 to pcl::pointcloud
		//pc is clusters[j] in pointcloud format

		int counter = 0;
		for(int i=0; i < pc.points.size(); i++){		//add points with max z to a new pointcloud
			if(pc.points[i].z == max_z){
				counter++;
				pcz.push_back(pc.points[i]);
			}
		}

		pcl::PointXYZ min_point(pcz.points[0]);
		pcl::PointXYZ max_point(pcz.points[0]);
		std::cout << "\n\n\nSTART*****\n" << std::endl;
		//std::cout << "original min,max y: " << min_point.y << std::endl;

		for (int i=1; i < pcz.points.size(); i++){		//for every point in the cluster, find min y and max y	
			if(pcz.points[i].y < min_point.y){
				min_point.x = pcz.points[i].x;
				min_point.y = pcz.points[i].y;
				min_point.z = pcz.points[i].z;
			}
			if(pcz.points[i].y > max_point.y){
				max_point.x = pcz.points[i].x;
				max_point.y = pcz.points[i].y;
				max_point.z = pcz.points[i].z;
			}
		}

		std::cout << "Min y is: " << min_point.y << "\nMax y is: " << max_point.y << std::endl;
		std::cout << "Min x is: " << min_point.x << "\nMax x is: " << max_point.x << std::endl;	//min_x is the x of point with min y (not actual min_x of cluster)
	
		//angle_l= atan2(0, 105);		//atan2(-min_point.y, min_point.x);	//cuts right half of image
		//angle_r= atan2(100, 100);	//atan2(-max_point.y, max_point.x);

		angle_l = atan2(-min_point.y, min_point.x);
		angle_r = atan2(-max_point.y, max_point.x);

		std::cout << "Not centered angle_l is: " << angle_l << "\nNot centered angle_r is: " << angle_r << std::endl;

		//angle_pair= angle_calculation(angle_l, angle_r);	//conversion to center-based angles ; doing this for the purpose of printing the values only
		angle_pair.first = angle_l;
		angle_pair.second = angle_r;

		// std::cout << "Centered angle_l is: " << angle_pair.first << "\nCentered angle_r is: " << angle_pair.second << std::endl;

		std::cout << "Image.cols (width)= " << cv_ptr->image.cols << "\nImage.rows (height)= " << cv_ptr->image.rows << "\nWIDENESS ANGLE" << WIDENESS_ANGLE << std::endl;
		int ratio = (cv_ptr->image.cols) / (WIDENESS_ANGLE) ;	//	(width pixels) / (wideness)
		std::cout << "Ratio:\t" << ratio << std::endl;
		int center = (cv_ptr->image.cols)/2;
		int x_l, x_r;

		x_l = min(2*center, max(0, (int)ceil(min(angle_pair.first, angle_pair.second) * ratio + center)));
		x_r = max(0, min(2*center, (int)floor(max(angle_pair.first, angle_pair.second) * ratio + center)));

		int width_pixels, offset;
		width_pixels = x_r - x_l;
		if (width_pixels < 5){
			out_msg.has_image.push_back(0);
		}
		else{
			out_msg.has_image.push_back(1);

			// offset= center + x_l;
			offset = x_l;
			if(width_pixels < 0){
				std::cout << "width_pixels is negative for some reason..." << std::endl;
			}
			std::cout << "\n\nx_l= " << x_l << "\nx_r= " << x_r << "\nwidth_pixels= " << width_pixels << "\noffset= " << offset << "\n\n\n\n";

			cv::Rect myROIseg(offset, 0, width_pixels, cv_ptr->image.rows); 
			cv::Mat roiseg = cv::Mat(cv_ptr->image, myROIseg);

			//Image Segmentation

			//cv::Rect myROIout(offset, 0, width_pixels, cv_ptr->image.rows); 
			//cv::Mat roiout = cv::Mat(cv_ptr->image, myROIseg);
			sensor_msgs::ImagePtr imgptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roiseg).toImageMsg();

			tpub.publish(*imgptr);
			
			out_msg.image_set.push_back(*imgptr);	//insert images into message for publishing

			image_counter++;
		}
	}

	std::cout << "\tSize of image set: " << out_msg.image_set.size() << std::endl;
	std::cout << "No of clusters: " << msg.clusters.size() << std::endl;
	std::cout << "pair_vector size: " << pair_vector.size() << "\npixel_vector size: " << pixel_vector.size() << std::endl;
	if( pair_vector.size() == pixel_vector.size() && pixel_vector.size() == msg.clusters.size() ){
			std::cout << "Same size!\n" << std::endl;
	}
	else{
		std::cout << "Different size!\n" << std::endl;
	}
	
	std::cout << "has_image size: " << out_msg.has_image.size() << std::endl;
	std::cout << "has_image contents: [";
	for(int c=0; c<out_msg.has_image.size(); c++){
		std::cout << "," << out_msg.has_image[c];
	}
	std::cout << "]" << std::endl;

	//Message Creation

	out_msg.header.frame_id = msg.header.frame_id;
	out_msg.header.stamp = ros::Time::now();
	out_msg.factor = msg.factor;
	out_msg.first_stamp = msg.first_stamp;
	out_msg.overlap = msg.overlap;
	out_msg.num_scans = msg.num_scans;
	out_msg.angle_min = msg.angle_min;
	out_msg.angle_max = msg.angle_max;
	out_msg.angle_increment = msg.angle_increment;
	out_msg.time_increment = msg.time_increment;
	out_msg.range_min = msg.range_min;
	out_msg.range_max = msg.range_max;
	out_msg.scan_time = msg.scan_time;
	out_msg.clusters = msg.clusters;
	out_msg.cluster_id = msg.cluster_id;

	std::cout << "\nCHECKING^^^^^\nIN cluster_id contents: [";
	for(int c=0; c<msg.cluster_id.size(); c++){
		std::cout << "," << msg.cluster_id[c];
	}
	std::cout << "]" << std::endl;


	std::cout << "CHECKING^^^^^\nOUT cluster_id contents: [";
	for(int c=0; c<out_msg.cluster_id.size(); c++){
		std::cout << "," << out_msg.cluster_id[c];
	}
	std::cout << "]" << std::endl;

	// Timestamp: "End" (Just before publishing the message)
	ros::WallTime end = ros::WallTime::now();
	dur = end - start;
	nsecs = dur.toNSec();
	Log(nsecs,"Total Duration (from getting the message to just before publishing the new message)\n");

	pub.publish(out_msg);

}


void videoCallback(const sensor_msgs::ImageConstPtr& msg){

	std_msgs::Header h = msg->header;
	sensor_msgs::Image new_msg;
	latest_frame = *msg;
	first_frame = 1;

	try{
		cv_bridge::CvImagePtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		int center = (cv_ptr->image.cols)/2; 
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}


int main(int argc, char **argv){

	std::ofstream ofs;
	ofs.open(LOGFILE, std::ios::out | std::ios::trunc);		//clear log file
	ofs.close();

	ros::init(argc, argv, "image_segmentation_node");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

	//image_transport::Publisher pub = it.advertise("seg_images", 2);
	pub = nh.advertise<image_msgs::Image_Segments>("seg_images", 2);
	tpub = it.advertise("image_segmentation_node/seg_image", 1);

	ros::Subscriber pcl_seg_sub = nh.subscribe<const pointcloud_msgs::PointCloud2_Segments&>("pointcloud2_cluster_tracking/clusters", 1, pcl_seg_Callback);
	image_transport::Subscriber video_sub = it.subscribe("rear_cam/image_raw", 50, videoCallback);		//  camera/rgb/image_raw gia to rosbag me tous 3, rear_cam/image_raw gia to rosbag me emena, usb_cam/image_raw gia to rosbag me to video mono

	//ros::Rate loop_rate(0.5);

	while (nh.ok()){
		ros::spinOnce();
		//loop_rate.sleep();
	}
}
