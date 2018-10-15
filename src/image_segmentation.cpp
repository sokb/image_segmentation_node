#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <fstream> 

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
#include <image_msgs/Image_Segments.h>

#define LOGFILE	"imseg.log"     // all Log(); messages will be appended to this file

ros::Publisher pub;
image_transport::Publisher tpub;

std::vector<std::pair<double,double>> pair_vector;
sensor_msgs::Image latest_frame;

int first_frame= 0;

int oor= 0;
const double PI = 3.141592653589793;

void Log (uint64_t duration_secs, std::string message){	// logs a message to LOGFILE

	std::ofstream ofs;
	ofs.open(LOGFILE, std::ofstream::out | std::ios::app);
  	ofs << duration_secs << " nsecs : " << message << std::endl;
  	ofs.close();
}


std::pair<double,double> angle_calculation(double angle_l, double angle_r){
	//conversion to center-based angles			!A,B,C,D are the quadrants, starting from upper-left quadrant and moving clockwise.
		double c_angle_l, c_angle_r;
		if(angle_l > PI/2 && angle_l < PI){			//A
			c_angle_l= - ( angle_l - PI/2 );
		}
		else if(angle_l > -PI/2 && angle_l < PI/2){	//B,C
			c_angle_l= PI/2 - angle_l;
		}
		else if(angle_l > -PI && angle_l < -PI/2){	//D
			c_angle_l= -( 3*PI/2 + angle_l );
		}
		else if(angle_l == PI/2){			
			c_angle_l= 0;
		}
		else if(angle_l == PI || angle_l == -PI){			
			c_angle_l= -PI/2;
		}
		else if(angle_l == -PI/2){			
			c_angle_l= PI;
		}


		if(angle_r > PI/2 && angle_r < PI){			//A
			c_angle_r= - ( angle_r - PI/2 );
		}
		else if(angle_r > -PI/2 && angle_r < PI/2){	//B,C
			c_angle_r= PI/2 - angle_r;
		}
		else if(angle_r > -PI && angle_r < -PI/2){	//D
			c_angle_r= -( 3*PI/2 +angle_r );
		}
		else if(angle_r == PI/2){			
			c_angle_r= 0;
		}
		else if(angle_r == PI || angle_r == -PI){			
			c_angle_r= -PI/2;
		}
		else if(angle_r == -PI/2){			
			c_angle_r= PI;
		}
		std::pair<double,double> pair(c_angle_l, c_angle_r);
		return pair;

}

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// void pcl_Callback(const PointCloud::ConstPtr& msg)
// {
//   printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//   BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//     printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
// }



void pcl_seg_Callback(const pointcloud_msgs::PointCloud2_Segments& msg){

	if(first_frame == 0){
		return;
	}
	
	//pointcloud_msgs::PointCloud2_Segments msg_out;

	image_msgs::Image_Segments out_msg;

	std::vector<std::pair<double,double>> pixel_vector;

	pair_vector.clear();
	//std::vector<std::pair<double,double>> msg_vector;
	double angle_min = msg.angle_min;
	double angle_max = msg.angle_max;
	double angle_increment = msg.angle_increment;

	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(latest_frame, "bgr8");

	//Timestamp: "Start" (Just got the pcl_segments message)
	ros::Duration dur;
	ros::Duration z_dur;
	uint64_t secs=0;
	uint64_t z_secs=0;
	ros::Time start = ros::Time::now();

	cv::imshow("view",cv_ptr->image);
    cv::waitKey(30);

    pcl::PointCloud<pcl::PointXYZ> pcz; 			//pcz contains all points with max z

    std::cout << "\n________NEW MESSAGE________\n" << std::endl;

	for (int j=0; j < msg.clusters.size(); j++){		//for every cluster

		pcz.clear();

		int image_counter=0;							//counter used for image_set array
		double angle_l, angle_r, c_angle_l, c_angle_r;
		std::pair<double,double> angle_pair(0,0);
		std::pair<double,double> pixel_pair(0,0);

		pcl::PCLPointCloud2 pc2;
    	pcl_conversions::toPCL ( msg.clusters[j] , pc2 );	//from sensor_msgs::pointcloud2 to pcl::pointcloud2

    	pcl::PointCloud<pcl::PointXYZ> pc;
    	pcl::fromPCLPointCloud2 ( pc2 , pc );				//from pcl::pointcloud2 to pcl::pointcloud
    	//pc is clusters[j] in pointcloud format

   	//MAX Z******************************************************************************************************
    
    	// Timestamp: "z_start_time" (Start of maximum z pointcloud extraction process)
		ros::Time z_start_time = ros::Time::now();

    	double max_z=pc.points[0].z;
    	for (int i=1; i < pc.points.size(); i++){		//find max z of cluster	
		 	if(pc.points[i].z > max_z){
				max_z= pc.points[i].z;
			}
		}
		std::cout << "Max z = " << max_z << std::endl;
		int counter=0;
		for(int i=0; i < pc.points.size(); i++){		//add points with max z to a new pointcloud
			if(pc.points[i].z == max_z){
				counter++;
				pcz.push_back(pc.points[i]);
			}
		}

		// Timestamp: "z_stop_time" (End of maximum z pointcloud extraction process)
		ros::Time z_stop_time = ros::Time::now();
		z_dur = z_dur + (z_stop_time - z_start_time);
		z_secs =z_secs + z_dur.toNSec();	// toNSec() returns uint64_t

		if(counter == pcz.size() && pcz.size() == pcz.points.size() ){
			std::cout << "pcz size same as counter (" << counter << " / " << pc.points.size() << ") ! All is fine" << std::endl;
		}
		else{
			std::cout << "something's wrong" << std::endl;
		}

		pcl::PointXYZ min_point(pcz.points[0]);
    	pcl::PointXYZ max_point(pcz.points[0]);
		std::cout << "\n\n\nSTART*****\noriginal min,max y: " << min_point.y << std::endl;

		for (int i=1; i < pcz.points.size(); i++){		//for every point in the cluster, find min y and max y	
			//std::cout << "x= " << pcz.points[i].x << std::endl << "y= " << pcz.points[i].y << std::endl << "z= " << pcz.points[i].z << "\n\n\n";
			if(pcz.points[i].y < min_point.y){
				min_point.x= pcz.points[i].x;
				min_point.y= pcz.points[i].y;
				min_point.z= pcz.points[i].z;
			}
		 	if(pcz.points[i].y > max_point.y){
				max_point.x= pcz.points[i].x;
				max_point.y= pcz.points[i].y;
				max_point.z= pcz.points[i].z;
			}
		}

	//**********************************************************************************************************

	// //___OLD__________________________________________________________________________________________________________

	// 	pcl::PointXYZ min_point(pc.points[0]);
 //    	pcl::PointXYZ max_point(pc.points[0]);
	// 	std::cout << "\n\n\nSTART*****\noriginal min,max y: " << min_point.y << std::endl;

	// 	for (int i=1; i < pc.points.size(); i++){		//for every point in the cluster, find min y and max y	
	// 		//std::cout << "x= " << pc.points[i].x << std::endl << "y= " << pc.points[i].y << std::endl << "z= " << pc.points[i].z << "\n\n\n";
	// 		if(pc.points[i].y < min_point.y){
	// 			min_point.x= pc.points[i].x;
	// 			min_point.y= pc.points[i].y;
	// 			min_point.z= pc.points[i].z;
	// 		}
	// 	 	if(pc.points[i].y > max_point.y){
	// 			max_point.x= pc.points[i].x;
	// 			max_point.y= pc.points[i].y;
	// 			max_point.z= pc.points[i].z;
	// 		}
	// 	}
	// 	// std::cout << "MAX y= " << max_point.y << std::endl << "x= " << max_point.x << std::endl << "z= " << max_point.z << std::endl << std::endl;
	// 	// std::cout << "MIN y= " << min_point.y << std::endl << "x= " << min_point.x << std::endl << "z= " << min_point.z << std::endl << std::endl;

	// //________________________________________________________________________________________________________________

		std::cout << "Min y is: " << min_point.y << "\nMax y is: " << max_point.y << std::endl;
		std::cout << "Min x is: " << min_point.x << "\nMax x is: " << max_point.x << std::endl;	//min_x is the x of point with min y (not actual min_x of cluster)

		//angle calculation in rads [0,2pi] starting from upper-left quadrant. 		! x,y are reversed because of laserscan's reversed x,y axes
		angle_l= atan2(min_point.x, min_point.y);
		angle_r= atan2(max_point.x, max_point.y);

		std::cout << "Not centered angle_l is: " << angle_l << "\nNot centered angle_r is: " << angle_r << std::endl;

		angle_pair= angle_calculation(angle_l, angle_r);	//conversion to center-based angles

		std::cout << "Centered angle_l is: " << angle_pair.first << "\nCentered angle_r is: " << angle_pair.second << std::endl;

		//msg_vector.push_back( angle_calculation(angle_l, angle_r) );
		pair_vector.push_back( angle_calculation(angle_l, angle_r) );	

		double temp;
		if(pair_vector.at(j).first > pair_vector.at(j).second){		// case: ymin < ymax but angle_l > angle_r
			std::cout << "SWITCHED!\n";
			temp= pair_vector.at(j).first;	// switch them
			pair_vector.at(j).first= pair_vector.at(j).second;
			pair_vector.at(j).second= temp;
		}
		if( oor==1 && j!=0){
			std::cout << "*Previous (empty) cell*\n*first: " << pair_vector.at(j-1).first << "\n*second: " << pair_vector.at(j-1).second << std::endl;
			oor=0;
		}
		double a_l= pair_vector.at(j).first;	//first: center-based left angle, second: center_based right angle
		double a_r= pair_vector.at(j).second;
		std::cout << "ANGLES:\nLEFT: " << a_l << "\nRIGHT: " << a_r << std::endl;

		double cam_min= -PI/6, cam_max= PI/6;	// Orbbec Astra Pro wideness: PI/3 (60 degrees) total
		std::cout << "Image.cols (width)= " << cv_ptr->image.cols << "\nImage.rows (height)= " << cv_ptr->image.rows << "\nabs(cam_min)= " << abs(cam_min) << "\nabs(cam_max)= " << abs(cam_max) << std::endl;
		int ratio= (cv_ptr->image.cols) / (abs(cam_min)+abs(cam_max)) ;	//	(width pixels) / (wideness)
		std::cout << "Ratio:\t" << ratio << std::endl;
		int center = (cv_ptr->image.cols)/2;
		int x_l, x_r;

		// a_r not necessarily greater than a_l (a_l, a_r are the centered angles)
		if( a_l < a_r ){
			if( a_l < cam_min && a_r < cam_min ){	//out of range (left)
				std::cout << "Case: out of range! (left)" << std::endl;
				oor=1;

				pixel_pair.first= NAN;
				pixel_pair.second= NAN;
				pixel_vector.push_back( pixel_pair );
				//continue;
			}
			else if( a_l > cam_max && a_r > cam_max ){	//out of range (right)
				std::cout << "Case: out of range! (right)" << std::endl;
				oor=1;

				pixel_pair.first= NAN;
				pixel_pair.second= NAN;
				pixel_vector.push_back( pixel_pair );
				//continue;
			}
			else if( a_l < cam_min && a_r > cam_max ){				//bigger than image size
				std::cout << "Case: bigger than image size!" << std::endl;
				x_l= -center;
				x_r= center;

				pixel_pair.first= x_l;
				pixel_pair.second= x_r;
				pixel_vector.push_back( pixel_pair );
			}
			else if( a_l < cam_min && a_r > cam_min && a_r < cam_max ){	//left side out of range
				std::cout << "Case: left side out of range!" << std::endl;
				x_l= -center;
				x_r= a_r*ratio;		//x_r, x_l: pixel distance from center of image (can be either positive or negative)

				pixel_pair.first= x_l;
				pixel_pair.second= x_r;
				pixel_vector.push_back( pixel_pair );
			}
			else if( a_r > cam_max && a_l > cam_min && a_l < cam_max ){	//right side out of range
				std::cout << "Case: right side out of range!" << std::endl;
				x_r= center;
				x_l= a_l*ratio;

				pixel_pair.first= x_l;
				pixel_pair.second= x_r;
				pixel_vector.push_back( pixel_pair );
			}
			else if( a_l > cam_min && a_l < cam_max && a_r > cam_min && a_r < cam_max ){	//in range
				std::cout << "Case: in range!" << std::endl;
				x_l= a_l*ratio;
				x_r= a_r*ratio;

				pixel_pair.first= x_l;
				pixel_pair.second= x_r;
				pixel_vector.push_back( pixel_pair );
			}
			else{
				std::cout << "Case: ??????" << std::endl;
				oor=1;

				pixel_pair.first= NAN;
				pixel_pair.second= NAN;
				pixel_vector.push_back( pixel_pair );
				//continue;
			}
		}
		else{
			if( a_l == a_r ){
				if( angle_l != angle_r ){
					std::cout << "centered angle_l = centered angle_r and that's not ok" << std::endl;
				}
				else{
					std::cout << "centered angle_l = centered angle_r and that's ok" << std::endl;
				}
			}
			else{
				std::cout << "angle mistake" << std::endl;
			}
			oor=1;
			pixel_pair.first= NAN;
			pixel_pair.second= NAN;
			pixel_vector.push_back( pixel_pair );
			//continue;
		}

		int width_pixels, offset;

		//can include the following in later 'else'
		if( isnan(pixel_pair.first)==0 && isnan(pixel_pair.second)==0 ){
			width_pixels= x_r - x_l;
			offset= center + x_l;
			std::cout << "\n\nx_l= " << x_l << "\nx_r= " << x_r << "\nwidth_pixels= " << width_pixels << "\noffset= " << offset << "\n\n\n\n";
			cv::Rect myROIseg(offset, 0, width_pixels, cv_ptr->image.rows); 
	    	cv::Mat roiseg = cv::Mat(cv_ptr->image,myROIseg);
			cv::imshow("cluster1",roiseg);
	    	cv::waitKey(30);
	    }	

		//std::cout << "min: " << angle_l*180/PI << std::endl;
		//std::cout << "center based min: " << angle_pair.first*180/PI << std::endl;
		//std::cout << "center based min in msg_vector: " << msg_vector.at(j).first*180/PI << std::endl << std::endl;
		//std::cout << "center based min in global vector: " << pair_vector.at(j).first*180/PI << std::endl << std::endl;

		//std::cout << "max: " << angle_r*180/PI << std::endl;
		//std::cout << "center based max in msg_vector: " << msg_vector.at(j).second*180/PI << std::endl;

		//std::cout << "New Angle Pair!!!" << std::endl;

		//std::cout << "center based max: " << angle_pair.second*180/PI << std::endl;
		//std::cout << "center based max in global vector: " << pair_vector.at(j).second*180/PI << std::endl<< std::endl<< std::endl<< std::endl;
		
		if( isnan(pixel_pair.first) || isnan(pixel_pair.second ) ){	//tsekare an swsto
			out_msg.has_image.push_back(0);
		}
		else{								//create message for publishing
			out_msg.has_image.push_back(1);

			cv_bridge::CvImagePtr cv_ptr_in;
			cv_ptr_in = cv_bridge::toCvCopy(latest_frame, "bgr8");

			cv::Rect myROIout(offset, 0, width_pixels, cv_ptr_in->image.rows); 
	     	cv::Mat roiout = cv::Mat(cv_ptr_in->image,myROIout);
	  		sensor_msgs::ImagePtr imgptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roiout).toImageMsg();
	  		
	  		out_msg.image_set.push_back(*imgptr);	//insert images into message for publishing

			image_counter++;
		}
	}

	Log(z_secs,"Duration of Slice Extraction Process (all clusters)");

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

	/*
  header Header 	//copy this too?
  sensor_msgs/PointCloud2[] clusters	!!!
  int32 factor
  time first_stamp
  int32 overlap
  int32[] cluster_id 	!!!
  int32 num_scans
  float32 angle_min
  float32 angle_max
  float32 angle_increment
  float32 time_increment
  float32 range_min
  float32 range_max
  float32 scan_time
	*/

	// Timestamp: "End" (Just before publishing the message)
	ros::Time end = ros::Time::now();
	dur = end - start;
	secs =dur.toNSec();
	Log(secs,"Total Duration (from getting the message to just before publishing the new message)\n");

	pub.publish(out_msg);

	// std::cout << "HEADER INFO BELOW:\n\n" << msg.header<< std::endl;
	// std::cout << "FIRST STAMP:\n\n" << msg.first_stamp << std::endl;

	//std::cout << "DATA OF FIRST CLUSTER: "<< std::endl << std::endl << msg.clusters[0] << std::endl << std::endl;
	//std::cout << "DATA OF SECOND CLUSTER: "<< std::endl << std::endl << msg.clusters[1] << std::endl << std::endl;

	
	//sensor_msgs::PointCloud conv_msg;
	//convertPointCloud2ToPointCloud(msg.clusters[0], conv_msg);

	// int i=0;
	// while(i< msg.clusters.size()){
	// 	std::cout << conv_msg.points[i] << std::endl;
	// 	i++;
	// }

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


void videoCallback(const sensor_msgs::ImageConstPtr& msg){

	// std_msgs::Header h = msg->header;
	// sensor_msgs::Image new_msg;
	// latest_frame= *msg;	
	std_msgs::Header h = msg->header;
	sensor_msgs::Image new_msg;
	latest_frame= *msg;
	first_frame= 1;
  
  try
  {
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //cv::waitKey(30);
    cv_bridge::CvImagePtr cv_ptr;

    //std::cout << "New Video Frame!" << std::endl;

    try{
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      // cv::imshow("view",cv_ptr->image);
      // cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int center = (cv_ptr->image.cols)/2;

    //cut a rectangle
    //cv::Rect myROI(0, 0, (cv_ptr->image.cols)/3, cv_ptr->image.rows); 
    //cv::Mat roi = cv::Mat(cv_ptr->image,myROI);

    cv::Rect myROI2(cv_ptr->image.cols/3, 0, cv_ptr->image.cols/3,cv_ptr->image.rows); 
    cv::Mat roi2 = cv::Mat(cv_ptr->image,myROI2);

    cv::Rect myROI3(2*(cv_ptr->image.cols/3),0,cv_ptr->image.cols/3,cv_ptr->image.rows);
    cv::Mat roi3= cv::Mat(cv_ptr->image,myROI3);


    cv::Rect myROI4(0, 0, 150, 150); 
    cv::Mat roi4 = cv::Mat(cv_ptr->image,myROI4);
    // cv::imshow("view5", roi4);
    // cv::waitKey(30);

    // image_segmentation_node::ImageSet set;

    // cv::Rect myROI(0, 0, (cv_ptr->image.cols)/3, cv_ptr->image.rows); 
    // cv::Mat roi = cv::Mat(cv_ptr->image,myROI);
    // sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi).toImageMsg();
    // set.data[0]= *msg1;

 //    for (int i=0; i < pair_vector.size(); i++){
    	
 //    	// cv::Rect myROI(0, 0, 50, 50); 
 //    	// cv::Mat roi = cv::Mat(cv_ptr->image,myROI);

	// 	//convert to sensor_msgs/Image
 //    	sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi).toImageMsg();
 //    	set.data[i]= *msg1;

 //    	//...
 //    	// msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi2).toImageMsg();
 //    	// set.data[1]= *msg1;
 //    	// msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi3).toImageMsg();
 //    	// set.data[2]= *msg1;
 //    	//new_msg=set.data[0];

	// }

    // cv::imshow("view2", roi);
    // cv::waitKey(30);

    // cv::imshow("view3", roi2);
    // cv::waitKey(30);

    // cv::imshow("view4", roi3);
    // cv::waitKey(30);

    //tpub.publish(msg1);

    //tpub.publish(new_msg);

    //pub.publish(set);	//PUBLISH
    
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
	// std::cout << "[" << std::endl;
	// while(i<msg->ranges.size()){
	// 	std::cout << i << ": " << msg->ranges[i]<< " ";
	// 	i++;
	// }
	// std::cout << "]" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_segmentation_node");
  ros::NodeHandle nh;

  //ROS_INFO("angle_max is 2.345\n");

  cv::namedWindow("view");
  std::cout << "New Window: view" << std::endl;
  // cv::namedWindow("view2");
  // cv::namedWindow("view3");
  // cv::namedWindow("view4");
  //cv::namedWindow("view5");
  cv::namedWindow("cluster1");
  //std::cout << "New Window: view5" << std::endl;

  image_transport::ImageTransport it(nh);

  //advertise to output topic
  //image_transport::Publisher pub = it.advertise("seg_images", 2);
  pub = nh.advertise<image_msgs::Image_Segments>("seg_images", 2);
  tpub = it.advertise("normal_image", 2);

  // image_transport::Subscriber image_sub = it.subscribe("camera/image", 50, imageCallback);
  // ros::Subscriber pcl_sub = nh.subscribe<PointCloud>("points2", 1, pcl_Callback);

  std::cout << "reached subscribers point" << std::endl;
  ros::Subscriber pcl_seg_sub = nh.subscribe<const pointcloud_msgs::PointCloud2_Segments&>("pointcloud2_cluster_tracking/clusters", 1, pcl_seg_Callback);
  image_transport::Subscriber video_sub = it.subscribe("rear_cam/image_raw", 50, videoCallback);		//  camera/rgb/image_raw gia to rosbag me tous 3, usb_cam/image_raw gia to rosbag me to video mono, rear_cam/image_raw gia to rosbag me emena

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
  //cv::destroyWindow("view5");
  cv::destroyWindow("cluster1");
}