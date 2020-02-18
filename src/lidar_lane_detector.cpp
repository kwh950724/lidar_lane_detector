#include "lidar_lane_detector.h"

LaneDetector::LaneDetector(void):
	private_nh_("~") {
	initParams();
	init();
	std::cout<<"INITIALIZING LIDAR LANE DETECTOR NODE."<<std::endl;
}

void LaneDetector::init(void) {
	cloud_sub_ = nh_.subscribe("/points", 1, &LaneDetector::cloudCallback, this);
	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/road_line", 1);
}

void LaneDetector::initParams(void) {
	private_nh_.getParam("channel_threshold", channel_threshold_);
	private_nh_.getParam("angle_threshold", angle_threshold_);
	private_nh_.getParam("intensity_threshold", intensity_threshold_);
	private_nh_.getParam("lane_candidate", lane_candidate_);
	private_nh_.getParam("lane_width", lane_width_);
	private_nh_.getParam("x_std_dev_threshold", x_std_dev_threshold_);
	private_nh_.getParam("y_std_dev_threshold", y_std_dev_threshold_);

	prev_line_[0].a = 0.0;
	prev_line_[0].b = 1.5 * lane_width_;
	prev_line_[1].a = 0.0;
	prev_line_[1].b = 1.0 * lane_width_;
	prev_line_[2].a = 0.0;
	prev_line_[2].b = 0.5 * lane_width_;
	prev_line_[3].a = 0.0;
	prev_line_[3].b = 0.0;
	prev_line_[4].a = 0.0;
	prev_line_[4].b = -0.5 * lane_width_;
	prev_line_[5].a = 0.0;
	prev_line_[5].b = -1.0 * lane_width_;
	prev_line_[6].a = 0.0;
	prev_line_[6].b = -1.5 * lane_width_;

	is_line_[0], is_line_[1], is_line_[2], is_line_[3], is_line_[4], is_line_[5], is_line_[6] = 0;
}

void LaneDetector::cloudCallback(const boost::shared_ptr<sensor_msgs::PointCloud2> &cloud_msg) {
	pcl::PointCloud<PointT> cloud;
	pcl::PointCloud<PointT> filtered_cloud;

	std::vector<VectorT> drivable_region(1024);
	std::vector<VectorT> road_mark_line(1024);

	sensor_msgs::PointCloud2 filtered_cloud_msg;
	
	pcl::fromROSMsg(*cloud_msg, cloud);

	extractDrivableRegion(&cloud, &drivable_region, &filtered_cloud);
	extractRoadMarkLine(&drivable_region, &road_mark_line);
	extractRoadLine(&road_mark_line, prev_line_, is_line_, &filtered_cloud);
	publish(&marker_pub_, is_line_, prev_line_);

	toROSMsg(filtered_cloud, filtered_cloud_msg);
	filtered_cloud_msg.header.frame_id = "os1_lidar/lidar";

	cloud_pub_.publish(filtered_cloud_msg);
}

void LaneDetector::extractDrivableRegion(const pcl::PointCloud<PointT> *cloud, std::vector<VectorT> *drivable_region, pcl::PointCloud<PointT>* filtered_cloud) {
	for(int a=0;a<1024;a++) {
		for(int c=63;c>channel_threshold_;c--) {
			auto angle = (cloud->points[a*64 + c-1].z - cloud->points[a*64 + c].z) / sqrt(powf(cloud->points[a*64 + c].x - cloud->points[a*64 + c - 1].x, 2) + powf(cloud->points[a*64 + c].y - cloud->points[a*64 + c - 1].y, 2)) * 180.0 / M_PI;
			if(angle < angle_threshold_) (*drivable_region)[a].push_back(cloud->points[a*64 + c]);
			else break;
		}
		filtered_cloud->points.insert(filtered_cloud->points.end(), (*drivable_region)[a].begin(), (*drivable_region)[a].end());
	}	
}

void LaneDetector::extractRoadMarkLine(const std::vector<VectorT> *drivable_region, std::vector<VectorT> *road_mark_line) {
	for(int a=0;a<1024;a++) {
		for(int i=0;i<(*drivable_region)[a].size();i++) {
			if((*drivable_region)[a][i].intensity > intensity_threshold_) (*road_mark_line)[a].push_back((*drivable_region)[a][i]);
		}
	}
}

void LaneDetector::extractRoadLine(const std::vector<VectorT> *road_mark_line, Line prev_line[], int is_line[], pcl::PointCloud<PointT> *filtered_cloud) {
	std::vector<VectorT> closest_point(lane_candidate_*2 - 1);
	float min_distance = 0.25;
	float distance = 0.0;
	int index = -1;

	for(int a=0;a<1024;a++) {
		for(int i=0;i<(*road_mark_line)[a].size();i++) {
			min_distance = 0.25;
			index = -1;
			for(int j=0;j<lane_candidate_*2 - 1;j++) {
				distance = fabs(prev_line[j].a * (*road_mark_line)[a][i].x - (*road_mark_line)[a][i].y + prev_line[j].b) / sqrt(powf(prev_line[j].a, 2) + 1);
				if(distance < min_distance) {
					min_distance = distance;
					index = j;
				}
			}
			if(index != -1) {
				closest_point[index].push_back((*road_mark_line)[a][i]);
			}
		}
	}
	
	is_line[0], is_line[1], is_line[2], is_line[3], is_line[4], is_line[5], is_line[6] = 0;

	int even_count = 0;
	int odd_count = 0;

	float y_total = 0.0;
	float y_mean = 0.0;
	float y_square_total = 0.0;
	float y_std_dev = 0.0;
	float x_total = 0.0;
	float x_mean = 0.0;
	float x_square_total = 0.0;
	float x_std_dev = 0.0;

	for(int i=0;i<lane_candidate_*2 - 1;i++) {
		y_total = 0.0;
		y_square_total = 0.0;
		x_total = 0.0;
		x_square_total = 0.0;
		
		if(closest_point[i].size() != 0) {
			for(int j=0;j<closest_point[i].size();j++) {
				y_total += closest_point[i][j].y;
				x_total += closest_point[i][j].x;
			}
			y_mean = y_total / closest_point[i].size();
			x_mean = x_total / closest_point[i].size();
	

			for(int k=0;k<closest_point[i].size();k++) {
				y_square_total += powf(closest_point[i][k].y - y_mean, 2);	
				x_square_total += powf(closest_point[i][k].x - x_mean, 2);	
			}
			
			y_std_dev = sqrt(y_square_total / (closest_point[i].size() - 1));
			x_std_dev = sqrt(x_square_total / (closest_point[i].size() - 1));
			
			prev_line[i].b = y_mean;
	
			if(fabs(prev_line[i].b - y_mean) < 0.4 && x_std_dev > x_std_dev_threshold_ && y_std_dev < y_std_dev_threshold_) {
				is_line_[i] = 1;
				if(i % 2 == 0) {
					even_count += 1;
				} else {
					odd_count += 1;
				}
			}
		}
	}

	if(even_count > odd_count) {
		for(int i=1;i<lane_candidate_*2 - 1;i+=2) {
			is_line[i] = 0;	
		}
	} else {
		for(int i=0;i<lane_candidate_*2 - 1;i+=2) {
			is_line[i] = 0;
		}
	}

}

void LaneDetector::publish(const ros::Publisher *marker_pub, const int is_line[], const Line prev_line[]) {
	for(int i=0;i<lane_candidate_*2 - 1;i++) {
		if(is_line[i] == 1) {
			visualization_msgs::Marker line;
			std::vector<geometry_msgs::Point> line_points;
			geometry_msgs::Point temp_point;
			for(int j=-10;j<11;j++) {
				temp_point.x = j;
				temp_point.y = prev_line[i].b;
				temp_point.z = -2.0;
				line_points.push_back(temp_point);
			}
			line.header.frame_id = "os1_lidar/lidar";
			line.header.stamp.fromSec(ros::Time::now().toSec());
			line.ns = "lines";
			line.id = i;
			line.action = visualization_msgs::Marker::ADD;
			line.pose.orientation.w = 1.0;
			line.type = visualization_msgs::Marker::LINE_STRIP;
			line.scale.x = 0.2;
			line.color.b = 1.0;
			line.color.a = 1.0;
			line.points = line_points;
		
			marker_pub->publish(line);
		}
	}
}
