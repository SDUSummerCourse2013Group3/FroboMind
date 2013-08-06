/*
 * RowExtractor.h
 *
 *  Created on: Aug 3, 2013
 *      Author: kent
 */

#ifndef ROWEXTRACTOR_H_
#define ROWEXTRACTOR_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI PointT;

class RowExtractor
{
public:
	RowExtractor();
	virtual ~RowExtractor();

	void update (void);

	void getParameters (void);
	void setParameters (void);



	struct Parameters
	{
		struct
		{
			bool active;
			double minimumClearZoneDistance;
			double maximumClearZoneDistance;
		} preProcessor;

		struct
		{
			bool active;
			double segmentationDistance;
		} segmentationProcessor;

		struct
		{
			int numberOfRansacTries;
			int numberOfPointsToAcceptLine;
			double distanceFromLineThreshold;
		} ransacProcessor;
	};

	struct Input
	{
		pcl::PointCloud pointCLoud;
	};

	struct Output
	{
		double orientation;
	};

private:
	Input input;
	Output output;
	Parameters systemParameters;

	//	Processors
	void preProcess (void);
	void segmentationProcess (void);
	void ransacProcess (void);
};

#endif /* ROWEXTRACTOR_H_ */
