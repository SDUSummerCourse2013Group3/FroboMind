/*
 * RowExtractor.h
 *
 *  Created on: Aug 3, 2013
 *      Author: kent
 */

#ifndef ROWEXTRACTOR_H_
#define ROWEXTRACTOR_H_

#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace RowExtractor
{
	typedef pcl::PointXYZI PointT;

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
		pcl::PointCloud<PointT> pointCloud;
	};

	struct Output
	{
		double length;
		double orientation;
	};

	class RowExtractor
	{
	public:
		RowExtractor();
		RowExtractor(Parameters par);
		virtual ~RowExtractor();

		Parameters getParameters (void);
		void setParameters (Parameters par);

		Output update (Input in);

	private:
		Input input;
		Output output;
		Parameters systemParameters;

		//	Processed data
		pcl::PointCloud<PointT> preProcessedData;
		pcl::PointCloud<PointT> segmentedData;

		//	Processors
		void preProcess (void);
		void segmentationProcess (void);
		void ransacProcess (void);
	};
}

#endif /* ROWEXTRACTOR_H_ */
