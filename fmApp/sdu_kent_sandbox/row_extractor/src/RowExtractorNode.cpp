/*
 * RowExtractorNode.cpp
 *
 *  Created on: Aug 1, 2013
 *      Author: kent
 */

#include "RowExtractorNode.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "RowExtractorNode");

	RowExtractorNode rowNode;
	rowNode.makeItSpin();

	return 0;
}

RowExtractorNode::RowExtractorNode() : nodeHandler("~")
{
	//	ROS node handler stuff
	//this->nodeHandler = ros::NodeHandle("~");
	this->nodeHandler.param<int>("loopRate", this->loopRate, 100);

	//	Setup laser scanner --> point cloud stuff
	this->tfListener.setExtrapolationLimit(ros::Duration(0.1));

	//	Setup debug publisher
	this->pointCloudPublisher = this->nodeHandler.advertise<sensor_msgs::PointCloud2>("pointCloudTest", 10);

	//  Setup system output
	this->nodeHandler.param<std::string>("rowTopic", this->output.rowTopic, "rowTopic");
	this->output.rowPublisher = this->nodeHandler.advertise<msgs::row>(this->output.rowTopic, 10);

	//	Setup system input
	this->nodeHandler.param<std::string>("scanTopic", this->input.scanTopic, "/fmSensors/laser_msg");
	this->nodeHandler.param<std::string>("scanLink", this->input.scanLink, "/laser_link");
	this->input.scanSubscribe = this->nodeHandler.subscribe<sensor_msgs::LaserScan>(this->input.scanTopic, 10, &RowExtractorNode::laserScanCallback, this);

	//	Setup marker
	this->marker.header.frame_id = this->input.scanLink;
	this->marker.ns = "RowExtractorMarker";
	this->marker.id = 0;
	this->marker.type = visualization_msgs::Marker::CUBE;
	this->marker.action = visualization_msgs::Marker::ADD;
	this->marker.lifetime = ros::Duration();
	this->marker.color.r = 0.0;
	this->marker.color.g = 0.0;
	this->marker.color.b = 1.0;
	this->marker.color.a = 0.8;

	this->markerPublisher = this->nodeHandler.advertise<visualization_msgs::Marker>("line_marker_pub", 10);

	//	Setup row extractor
	//	Preprocessor
	this->nodeHandler.param<bool>("rowExtractor/preprocessor/active", this->reParameters.preProcessor.active, true);
	this->nodeHandler.param<double>("rowExtractor/preprocessor/minimumClearZoneDistance", this->reParameters.preProcessor.minimumClearZoneDistance, .2);
	this->nodeHandler.param<double>("rowExtractor/preprocessor/maximumClearZoneDistance", this->reParameters.preProcessor.maximumClearZoneDistance, 5.0);

	//	Segmentation
	this->reParameters.segmentationProcessor.active = false;
	this->reParameters.segmentationProcessor.segmentationDistance = 0.0;

	//	Ransac
	this->nodeHandler.param<int>("rowExtractor/ransac/numberOfRansacTries", this->reParameters.ransacProcessor.numberOfRansacTries, 20);
	this->nodeHandler.param<int>("rowExtractor/ransac/numberOfPointsToAcceptLine", this->reParameters.ransacProcessor.numberOfPointsToAcceptLine, 50);
	this->nodeHandler.param<double>("rowExtractor/ransac/distanceFromLineThreshold", this->reParameters.ransacProcessor.distanceFromLineThreshold, 0.2);

	//	Instantiate row extractor with parameters
	this->rowExtractor = RowExtractor(this->reParameters);

	//	Setup row extractor input (empty)
	this->reInput.pointCloud = pcl::PointCloud<PointT>();

	//	Setup row extractor output (empty)
	this->reOutput.length = 0.0;
	this->reOutput.orientation = 0.0;
}

RowExtractorNode::~RowExtractorNode()
{
	// TODO Auto-generated destructor stub
}

void RowExtractorNode::makeItSpin()
{
	ros::Rate r(this->loopRate);

	while (ros::ok() && this->nodeHandler.ok())
	{
		//	Update event que
		ros::spinOnce();

		//	Update
		this->reOutput = this->rowExtractor.update(this->reInput);

		//	Publish debug cloud
		this->pointCloudPublisher.publish(this->pointCloudMsg);

		//	Publish debug marker
		this->marker.header.stamp = ros::Time::now();
		this->markerPublisher.publish(this->marker);

		//	Publish row data
		this->output.rowPublisher.publish(this->output.rowMessage);

		//	Delay looping according to rate
		r.sleep();
	}
}

void RowExtractorNode::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
	//	Transform laser scan to point cloud in the scanners own frame
	try
	{
		this->laserProjection.transformLaserScanToPointCloud(this->input.scanLink, *data, pointCloudMsg, this->tfListener);

		pcl::fromROSMsg(pointCloudMsg, this->rawPointCloud);
	}
	catch (tf::TransformException& e)
	{
		ROS_ERROR("TF Listener threw: %s", e.what());
	}
}
