/*
 * RowExtractorNode.h
 *
 *  Created on: Aug 1, 2013
 *      Author: kent
 */

#ifndef ROWEXTRACTORNODE_H_
#define ROWEXTRACTORNODE_H_

#include <string.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <msgs/row.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "RowExtractor.h"

using namespace Extractors;

class RowExtractorNode
{
	//	Node handler
	ros::NodeHandle nodeHandler;
	int loopRate;
	bool debug;
	bool synchronous;

	//	Laser scan --> Point cloud and debug stuff
	tf::TransformListener tfListener;
	laser_geometry::LaserProjection laserProjection;
	sensor_msgs::PointCloud2 pointCloudMsg;
	//pcl::PointCloud<PointT> rawPointCloud;
	visualization_msgs::Marker marker;
	ros::Publisher pointCloudPublisher;
	ros::Publisher markerPublisher;

	//	System input
	struct
	{
		std::string scanTopic;
		std::string scanLink;
		ros::Subscriber scanSubscribe;
	} input;

	//	System output
	struct
	{
		msgs::row rowMessage;
		std::string rowTopic;
		ros::Publisher rowPublisher;
	} output;

	//	Row extractor
	RowExtractor rowExtractor;
	Input reInput;
	Output reOutput;
	Parameters reParameters;

	//	Callback
	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data);

	void updateDebugMarker (void);

public:
	RowExtractorNode();
	virtual ~RowExtractorNode();

	void makeItSpin (void);
};

#endif /* ROWEXTRACTORNODE_H_ */
