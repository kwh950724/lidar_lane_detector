#ifndef LIDARLANEDETECTOR_H
#define LIDARLANEDETECTOR_H
#define PCL_NO_PRECOMPILE

#include <iostream>
#include <cmath>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <point_os1.h>

#include <opencv2/opencv.hpp>

#define _USE_MATH_DEFINES

class LaneDetector {
public:
	LaneDetector(void);
private:
	struct Line {
		float a;
		float b;
	};
	typedef ouster_ros::OS1::PointOS1 PointT;
	typedef std::vector<PointT> VectorT;

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::Subscriber cloud_sub_;
	ros::Publisher cloud_pub_;
	ros::Publisher marker_pub_;

	int channel_threshold_;
	float angle_threshold_;
	int intensity_threshold_;	
	int lane_candidate_;
	float lane_width_;
	float x_std_dev_threshold_;
	float y_std_dev_threshold_;
	Line prev_line_[7];
	int is_line_[7];

	void init(void);
	void initParams(void);
	void cloudCallback(const boost::shared_ptr<sensor_msgs::PointCloud2>& cloud_msg);
	void extractDrivableRegion(const pcl::PointCloud<PointT>* cloud, std::vector<VectorT>* drivable_region, pcl::PointCloud<PointT>* filtered_cloud);
	void extractRoadMarkLine(const std::vector<VectorT>* drivable_region, std::vector<VectorT>* road_mark_line);
	void extractRoadLine(const std::vector<VectorT>* road_mark_line, Line prev_line[], int is_line[], pcl::PointCloud<PointT>* filtered_cloud);
	void publish(const ros::Publisher* marker_pub, const int is_line[], const Line prev_line[]);
};

#endif

