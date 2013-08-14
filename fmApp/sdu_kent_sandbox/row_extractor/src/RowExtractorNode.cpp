/*
 * RowExtractorNode.cpp
 *
 *  Created on: Aug 1, 2013
 *      Author: kent
 */

#include "RowExtractorNode.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "rowExtractor");

	RowExtractorNode rowNode;
	rowNode.makeItSpin();

	return 0;
}

RowExtractorNode::RowExtractorNode() : nodeHandler("~")
{
	//	ROS node handler stuff
	this->nodeHandler.param<int>("loopRate", this->loopRate, 100);
	this->nodeHandler.param<bool>("debug", this->debug, true);
	this->nodeHandler.param<bool>("synchronous", this->synchronous, false);

	//	Setup laser scanner --> point cloud stuff
	this->tfListener.setExtrapolationLimit(ros::Duration(0.1));

	//  Setup system output
	this->nodeHandler.param<std::string>("rowTopic", this->output.rowTopic, "rowTopic");
	this->output.rowPublisher = this->nodeHandler.advertise<msgs::row>(this->output.rowTopic, 10);

	//	Setup system input
	this->nodeHandler.param<std::string>("scanTopic", this->input.scanTopic, "/fmSensors/laser_msg");
	this->nodeHandler.param<std::string>("scanLink", this->input.scanLink, "/laser_link");
	this->input.scanSubscribe = this->nodeHandler.subscribe<sensor_msgs::LaserScan>(this->input.scanTopic, 10, &RowExtractorNode::laserScanCallback, this);

	//	Setup debugging stuff
	//	Cloud publisher
	this->pointCloudPublisher = this->nodeHandler.advertise<sensor_msgs::PointCloud2>("pointCloudTest", 10);

	//	Marker
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
	this->nodeHandler.param<double>("timeLimit", this->reParameters.timeLimit, 1000);

	//	Preprocessor
	this->nodeHandler.param<bool>("preprocessor/active", this->reParameters.preProcessor.active, true);
	this->nodeHandler.param<double>("preprocessor/minimumClearZoneDistance", this->reParameters.preProcessor.minimumClearZoneDistance, .2);
	this->nodeHandler.param<double>("preprocessor/maximumClearZoneDistance", this->reParameters.preProcessor.maximumClearZoneDistance, 5.0);

	//	Ransac
	this->nodeHandler.param<int>("ransac/numberOfRansacTries", this->reParameters.ransacProcessor.numberOfRansacTries, 20);
	this->nodeHandler.param<int>("ransac/numberOfPointsToAcceptLine", this->reParameters.ransacProcessor.numberOfPointsToAcceptLine, 50);
	this->nodeHandler.param<double>("ransac/distanceFromLineThreshold", this->reParameters.ransacProcessor.distanceFromLineThreshold, 0.2);
	this->nodeHandler.param<double>("ransac/minimumRowLength", this->reParameters.ransacProcessor.minimumRowLength, 1.2);

	//	Instantiate row extractor with parameters
	this->rowExtractor = RowExtractor(this->reParameters);

	//	Setup row extractor input (empty)
	this->reInput.time_sec = 0;
	this->reInput.time_nsec = 0;
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

		//	Update (if asynchronous behavior have been selected)
		if (!this->synchronous)
		{
			this->reOutput = this->rowExtractor.update(this->reInput);
		}

		if (this->debug)
		{
			//	Publish raw cloud
			this->pointCloudPublisher.publish(this->pointCloudMsg);

			//	Publish debug marker
			this->updateDebugMarker();
			this->marker.header.stamp = ros::Time::now();
			this->markerPublisher.publish(this->marker);
		}

		//	Setup row output
		//	Extract info from vector<row>

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
		this->reInput.time_sec = data.get()->header.stamp.sec;
		this->reInput.time_nsec = data.get()->header.stamp.nsec;

		this->laserProjection.transformLaserScanToPointCloud(this->input.scanLink, *data, this->pointCloudMsg, this->tfListener);

		pcl::fromROSMsg(this->pointCloudMsg, this->reInput.pointCloud);

		if (this->synchronous)
		{
			this->reOutput = this->rowExtractor.update(this->reInput);
		}

	}
	catch (tf::TransformException& e)
	{
		ROS_ERROR("TF Listener threw: %s", e.what());
	}
}

void RowExtractorNode::updateDebugMarker (void)
{
	if (this->reOutput.rowFound)
	{
//		this->marker.pose.position.x = this->reOutput.center.x;
//		this->marker.pose.position.y = this->reOutput.center.y;
//
//		this->marker.pose.orientation = tf::createQuaternionMsgFromYaw(this->reOutput.orientation);
//
//		this->marker.scale.x = this->reOutput.length;
//		this->marker.scale.y = 2 * this->reParameters.ransacProcessor.distanceFromLineThreshold;
	}
	else
	{
//		this->marker.pose.position.x = 0.0;
//		this->marker.pose.position.y = 0.0;
//
//		this->marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
//
//		this->marker.scale.x = 0.0;
//		this->marker.scale.y = 0.0;
	}
}
