# Image Segmentation

## Description
*Image Segmentation* is a ROS package that transforms the 3D coordinates of pointcloud clusters to pixels and cuts an image that depicts the object of the specific cluster for every one of them, out of the equivalent camera frame.

### Files Included:
1. **image_segmentation_node:** This is the node that subscribes to the output topic of pointcloud2_cluster_tracking. It calculates the angle-to-pixel transformation and cuts an image out of the latest camera frame for every cluster for every instance of the pointcloud.
2. **seg_listener:** This is a node that subscribes to the output topic of image_segmentation_node. It saves the segmented images in the current directory as .png files.

### Topics
* **seg_images:** on which *image_segmentation_node* publishes messages of *image_msgs::Image_Segments* type.
* **image_segmentation_node/seg_image:** on which *image_segmentation_node* publishes the cropped images that correspond to a cluster with a specific ID.
* **test_pcl:** on which *image_segmentation_node* publishes the pointcloud of a cluster with a specific ID.

### Dependencies
This package is dependent on the following ROS packages:
* [Laserscan Stacker](https://github.com/roboskel/laserscan_stacker)
* [Pointcloud2 Clustering](https://github.com/roboskel/pointcloud2_clustering)
* [Pointcloud2 Cluster Tracking](https://github.com/roboskel/pointcloud2_cluster_tracking)
* [PointCloud2 Segments Viz](https://github.com/roboskel/pointcloud2_segments_viz)
* [Pointcloud msgs](https://github.com/roboskel/pointcloud_msgs)
* [Image msgs](https://github.com/roboskel/image_msgs)
* [hpr](https://github.com/roboskel/hpr/tree/rel3)

The package also requires the OpenCV library, which is included and automatically installed during the ROS installation process.

## Instructions
To test and run the demo, follow these steps:
1. Create a ROS workspace (let's assume it's named *catkin_ws*) and open a terminal inside the src directory of the workspace.
2. Clone all necessary packages from GitHub using the following commands:\
`git clone https://github.com/roboskel/pointcloud_msgs.git`\
 `git clone https://github.com/roboskel/laserscan_stacker.git`\
 `git clone https://github.com/roboskel/pointcloud2_clustering.git`\
 `git clone https://github.com/roboskel/pointcloud2_cluster_tracking.git`\
 `git clone https://github.com/roboskel/pointcloud2_segments_viz.git`\
 `git clone https://github.com/roboskel/hpr.git`\
 `git clone https://github.com/roboskel/image_segmentation_node.git`\
 `git clone https://github.com/roboskel/image_msgs.git`
 
3. Run `cd hpr`.  Switch to rel3 branch by running `git checkout rel3`
4. If you want to change the value of the safety_pixels parameter (defines the number of extra pixels added to each side of the cropped images), go to `src/image_segmentation_node/src` and open `image_segmentation.cpp`. In main function, change the following line:\
`nh.param<int>("safety_pixels", safety_pixels, 20);`\
to\
`nh.param<int>("safety_pixels", safety_pixels, <your_pixels>);`, where <your_pixels> is an integer of your choice (>=0).
5.  Go to `src/hpr/launch` and open `hpr_test.launch` file with a text editor. Change the following line:
`<param name="use_sim_time" value="False" />`\
to\
`<param name="use_sim_time" value="True" />`
6. Go to `src/laserscan_stacker/config` and open  `laserscan_stacker.yaml` file with a text editor. Change the *input topic* parameter value to `/rear_cam/image_raw`
7. Go to the root of your workspace directory and run `catkin_make` to build the packages.
8. Run `roscore`
9. In a new terminal, {go to directory with rosbag} and run: `rosbag play <name>.bag --clock`
10. In a new terminal, run `roslaunch hpr hpr_test.launch`
11. In a new terminal, run `rosrun image_segmentation_node image_segmentation`
12. In a new terminal, run `rviz`
13. {rviz steps here}
14. if you want to save all segmented images to the disk, open a new terminal **in the directory you want to save the images in** and run `rosrun image_segmentation_node seg_listener`.
15. Enjoy the demo!
