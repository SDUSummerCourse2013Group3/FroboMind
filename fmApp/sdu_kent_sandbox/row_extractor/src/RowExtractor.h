/*
 * RowExtractor.h
 *
 *  Created on: Aug 3, 2013
 *      Author: kent
 */

#ifndef ROWEXTRACTOR_H_
#define ROWEXTRACTOR_H_

#include <stdio.h>

#include <limits>
#include <list>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

namespace Extractors
{
	typedef pcl::PointXYZI PointT;
	typedef Eigen::MatrixXd CovarianceMatrix;

	struct Row
	{
		bool rowFound;
		PointT pointInRow;
		double orientation;
		double lengthToRow;
		double varianceOfRes;
		CovarianceMatrix covariance;
	};

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
			int numberOfRansacTries;
			int numberOfPointsToAcceptLine;
			double distanceFromLineThreshold;
			double minimumRowLength;
		} ransacProcessor;

		double timeLimit;	//	[ms]
	};

	struct Input
	{
		uint32_t time_sec;
		uint32_t time_nsec;
		pcl::PointCloud<PointT> pointCloud;
	};

	struct Output
	{
		double length;
		double orientation;
		PointT center;

		bool rowFound;
		std::vector<Row> rows;
		CovarianceMatrix cumulativeCovariance;
	};

	class RowExtractor
	{
	public:
		RowExtractor();
		RowExtractor(Parameters par);
		virtual ~RowExtractor();

		Parameters getParameters (void);
		void setParameters (Parameters par);

		pcl::PointCloud<PointT> getPreProcessedCloud (void);

		Output update (Input in);

	private:
		Input input;
		Output output;
		Parameters systemParameters;

		//	Processed data
		pcl::PointCloud<PointT> preProcessedData;

		//	Processors
		void oldPreProcess (void);
		void oldRansacProcess (void);

		void preProcessor (void);
		void rowProcessor (void);
		Row ransacProcessor (void);
	};
}

#endif /* ROWEXTRACTOR_H_ */
