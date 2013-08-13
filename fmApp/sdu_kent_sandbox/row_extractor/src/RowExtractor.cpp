/*
 * RowExtractor.cpp
 *
 *  Created on: Aug 3, 2013
 *      Author: kent
 */

#include "RowExtractor.h"

Extractors::RowExtractor::RowExtractor()
{
	//	Setup default parameters
	this->systemParameters.timeLimit = 1000;

	//	Pre-Processor
	this->systemParameters.preProcessor.active = false;
	this->systemParameters.preProcessor.minimumClearZoneDistance = 0.0;
	this->systemParameters.preProcessor.maximumClearZoneDistance = 0.0;

	//	Ransac processor
	this->systemParameters.ransacProcessor.numberOfRansacTries = 0;
	this->systemParameters.ransacProcessor.numberOfPointsToAcceptLine = 0;
	this->systemParameters.ransacProcessor.distanceFromLineThreshold = 0.0;
	this->systemParameters.ransacProcessor.minimumRowLength = 0.0;

	//	Setup input (empty)
	this->input.time_sec = 0;
	this->input.time_nsec = 0;
	this->input.pointCloud = pcl::PointCloud<PointT>();

	//	Setup output (empty)
	this->output.rowFound = false;
	this->output.length = 0.0;
	this->output.orientation = 0.0;
	this->output.center = PointT();
}

Extractors::RowExtractor::RowExtractor(Parameters par)
{
	//	Setup default parameters
	this->systemParameters = par;

	//	Setup input (empty)
	this->input.time_sec = 0;
	this->input.time_nsec = 0;
	this->input.pointCloud = pcl::PointCloud<PointT>();

	//	Setup output (empty)
	this->output.rowFound = false;
	this->output.length = 0.0;
	this->output.orientation = 0.0;
	this->output.center = PointT();
}

Extractors::RowExtractor::~RowExtractor()
{
	// TODO Auto-generated destructor stub
}

Extractors::Parameters Extractors::RowExtractor::getParameters (void)
{
	return this->systemParameters;
}

void Extractors::RowExtractor::setParameters (Parameters par)
{
	this->systemParameters = par;
}

Extractors::Output Extractors::RowExtractor::oldUpdate (Input in)
{
	this->input = in;

	this->oldPreProcess();
	this->oldRansacProcess();

	return this->output;
}

void Extractors::RowExtractor::oldPreProcess (void)
{
	//	Copy data from input cloud to pre-processed cloud
	this->preProcessedData = this->input.pointCloud;

	//	Check if this processer is enabled
	if (this->systemParameters.preProcessor.active)
	{
		//	Tentative variables
		PointT p;	//	Point
		double l;	//	Length (from frame origin to point)

		//	Clear processed data
		this->preProcessedData.clear();

		//	Run through all points to check if they should be discarded
		for (int i = 0; i < this->input.pointCloud.size(); i++)
		{
			p = this->input.pointCloud.at(i);
			l = sqrt(pow(p.x, 2) + pow(p.y, 2));

			if (	l > this->systemParameters.preProcessor.minimumClearZoneDistance &&
					l < this->systemParameters.preProcessor.maximumClearZoneDistance)
				this->preProcessedData.push_back(p);
		}
	}
}

void Extractors::RowExtractor::oldRansacProcess (void)
{
	//	Seed randomizer
	srand(time(0));

	//	Variables
	int i = 0, pointCounter, bestCount = 0, randomPoints[2];
	PointT lookingAt[2], pointsOfInterest[2], directionVector, orthogonalVector, helpVector, projectionVector, lineVector, p;
	double dotProduct, length, projectionLength;

	//	Algorithm
	if (this->preProcessedData.size())
	{
		while (i < this->systemParameters.ransacProcessor.numberOfRansacTries)
		{
			//	Select two points in cloud randomly
			randomPoints[0] = rand() % this->preProcessedData.size();
			randomPoints[1] = rand() % this->preProcessedData.size();

			lookingAt[0] = this->preProcessedData.at(randomPoints[0]);
			lookingAt[1] = this->preProcessedData.at(randomPoints[1]);

			//	Calculate direction from two randomly selected points
			directionVector.x = lookingAt[1].x - lookingAt[0].x;
			directionVector.y = lookingAt[1].y - lookingAt[0].y;

			//	Calculate orthogonal vector on direction
			orthogonalVector.x = - directionVector.y;
			orthogonalVector.y = directionVector.x;

			//	Reset point counter
			pointCounter = 0;

			//	Check distance to line to each point in cloud
			for (int j = 0; j < this->preProcessedData.size(); j++)
			{
				//	Get point
				p = this->preProcessedData.at(j);

				//	Make help vector for projection
				helpVector.x = p.x - lookingAt[0].x;
				helpVector.y = p.y - lookingAt[0].y;

				//	Make projection of helpVector onto orthogonalVector
				dotProduct = orthogonalVector.x * helpVector.x + orthogonalVector.y * helpVector.y;
				length = std::sqrt(std::pow(orthogonalVector.x, 2) + std::pow(orthogonalVector.y, 2));

				projectionVector.x = dotProduct / std::pow(length, 2) * orthogonalVector.x;
				projectionVector.y = dotProduct / std::pow(length, 2) * orthogonalVector.y;

				//	Calculate length of projection and decide if it is an inlier
				projectionLength = std::sqrt(std::pow(projectionVector.x, 2) + std::pow(projectionVector.y, 2));

				if (projectionLength < this->systemParameters.ransacProcessor.distanceFromLineThreshold)
				{
					pointCounter++;
				}
			}

			if (pointCounter > bestCount)
			{
				bestCount = pointCounter;
				pointsOfInterest[0] = lookingAt[0];
				pointsOfInterest[1] = lookingAt[1];
			}

			//	Add one try
			i++;
		}
	}

	//	Output
	if (bestCount > this->systemParameters.ransacProcessor.numberOfPointsToAcceptLine)
	{
		PointT line, center;
		line.x = pointsOfInterest[1].x - pointsOfInterest[0].x;
		line.y = pointsOfInterest[1].y - pointsOfInterest[0].y;
		center.x = pointsOfInterest[0].x + line.x / 2;
		center.y = pointsOfInterest[0].y + line.y / 2;

		double lineLength, lineOrientation;
		lineLength = std::sqrt(std::pow(line.x, 2) + std::pow(line.y, 2));
		lineOrientation = std::atan2(line.y, line.x);

		this->output.rowFound = true;
		this->output.length = lineLength;
		this->output.orientation = lineOrientation;
		this->output.center = center;

		//ROS_INFO("Line found with %d inliers ... ", bestCount);
	}
	else
	{
		double nan = std::numeric_limits<double>::quiet_NaN();

		this->output.rowFound = false;
		this->output.length = nan;
		this->output.orientation = nan;
		this->output.center = PointT();

		//ROS_INFO("No line was found ... ");
	}
}

Extractors::Output Extractors::RowExtractor::update (Input in)
{
	this->input = in;

	this->preProcessor();
	this->ransacProcessor();

	return this->output;
}

void Extractors::RowExtractor::preProcessor (void)
{
	this->oldPreProcess();
}

void Extractors::RowExtractor::rowProcessor (PointT point, double orientation)
{

}

void Extractors::RowExtractor::ransacProcessor2 (void)
{
	printf("Ransac processing ...");

	//	Seed randomizer
	srand(time(0));

	//	Variables
	int i = 0, randomPoints[2];
	PointT lookingAt[2], pointsOfInterest[2], directionVector, orthogonalVector, helpVector, projectionVector, lineVector, p;
	double dotProduct, length, projectionLength, rowLength;
	pcl::PointCloud<PointT> inliers = pcl::PointCloud<PointT>(), inliersOfInterest = pcl::PointCloud<PointT>();
	std::list<double> pointDistribution, bestPointDistribution;

	double alpha, beta;
	double inliersPercentage, variance;

	//	Algorithm
	if (this->preProcessedData.size())
	{
		while (i < this->systemParameters.ransacProcessor.numberOfRansacTries)
		{
 			//	Select two points in cloud randomly ad check if distance between points are big enough
			int pointSelectCounter = 0;
			do
			{
				randomPoints[0] = rand() % this->preProcessedData.size();
				randomPoints[1] = rand() % this->preProcessedData.size();

				lookingAt[0] = this->preProcessedData.at(randomPoints[0]);
				lookingAt[1] = this->preProcessedData.at(randomPoints[1]);

				//	Calculate direction from two randomly selected points
				directionVector.x = lookingAt[1].x - lookingAt[0].x;
				directionVector.y = lookingAt[1].y - lookingAt[0].y;

				rowLength = sqrt(pow(directionVector.x, 2) + pow(directionVector.y, 2));

				pointSelectCounter++;
			} while (rowLength > this->systemParameters.ransacProcessor.minimumRowLength || pointSelectCounter > 9);

			//	Calculate orthogonal vector on direction
			orthogonalVector.x = - directionVector.y;
			orthogonalVector.y = directionVector.x;

			//	Reset point counter and inliers vector
			inliers = pcl::PointCloud<PointT>();

			//	Check distance to line to each point in cloud
			for (int j = 0; j < this->preProcessedData.size(); j++)
			{
				//	Get point
				p = this->preProcessedData.at(j);

				//	Make help vector for projection
				helpVector.x = p.x - lookingAt[0].x;
				helpVector.y = p.y - lookingAt[0].y;

				//	Make projection of helpVector onto orthogonalVector
				dotProduct = orthogonalVector.x * helpVector.x + orthogonalVector.y * helpVector.y;
				length = std::sqrt(std::pow(orthogonalVector.x, 2) + std::pow(orthogonalVector.y, 2));

				projectionVector.x = dotProduct / std::pow(length, 2) * orthogonalVector.x;
				projectionVector.y = dotProduct / std::pow(length, 2) * orthogonalVector.y;

				//	Calculate length of projection and decide if it is an inlier
				projectionLength = std::sqrt(std::pow(projectionVector.x, 2) + std::pow(projectionVector.y, 2));

				if (projectionLength < this->systemParameters.ransacProcessor.distanceFromLineThreshold)
				{
					inliers.push_back(p);

					//	Reuse dotproduct and length variable, for projection of helpvector onto direction vector
					dotProduct = directionVector.x * helpVector.x + directionVector.y * helpVector.y;
					length = std::sqrt(std::pow(directionVector.x, 2) + std::pow(directionVector.y, 2));

					projectionVector.x = dotProduct / std::pow(length, 2) * directionVector.x;
					projectionVector.y = dotProduct / std::pow(length, 2) * directionVector.y;

					projectionLength = std::sqrt(std::pow(projectionVector.x, 2) + std::pow(projectionVector.y, 2));

					if (dotProduct >= 0 && projectionLength <= length)
						pointDistribution.push_back(projectionLength);
				}
			}

			if (inliers.size() > inliersOfInterest.size())
			{
				inliersOfInterest = inliers;
				bestPointDistribution = pointDistribution;
				pointsOfInterest[0] = lookingAt[0];
				pointsOfInterest[1] = lookingAt[1];
			}

			//	Add one try
			i++;
		}

		//	Test row quality
		bool isRowQualityOkay = false;
		if (inliersOfInterest.size())
		{
			//	Calculate the percentage of inliers to line compared with the overall number of points
			inliersPercentage = (double)inliersOfInterest.size() / (double)this->preProcessedData.size();

			bestPointDistribution.sort();
			std::list<double> weights;

			double numberOfPoints = (double)bestPointDistribution.size();
			double weight = 1. / numberOfPoints;
			double cumulativeWeight = 0.0;

			for (std::list<double>::iterator it = bestPointDistribution.begin(); it != bestPointDistribution.end(); ++it)
			{
				cumulativeWeight += weight;
				weights.push_back(cumulativeWeight);
			}

			// Check distribution
			double sumX = 0, sumY = 0, sumXY = 0, sumSquareX = 0;
			double meanX, meanY, meanXY, meanSquareX;

			for (std::list<double>::iterator itx = bestPointDistribution.begin(), ity = weights.begin(); itx != bestPointDistribution.end(); ++itx, ++ity)
			{
				sumX += *itx;
				sumY += *ity;
				sumXY += (*itx) * (*ity);
				sumSquareX += pow(*itx, 2);
			}

			meanX = sumX / numberOfPoints;
			meanY = sumY / numberOfPoints;
			meanXY = sumXY / numberOfPoints;
			meanSquareX = sumSquareX / numberOfPoints;

			//	Calculate linear regression parameters
			alpha = (meanXY - meanX * meanY) / (meanSquareX - pow(meanX, 2));
			beta = meanY - alpha * meanX;

			//	Calculate variance of residuals
			//std::list<double> residuals;
			double residual, sumRes = 0, sumResSquared = 0;

			for (std::list<double>::iterator itx = bestPointDistribution.begin(), ity = weights.begin(); ity != weights.end(); ++itx, ++ity)
			{
				residual = *ity - (alpha * (*itx) + beta);

				sumRes += residual;
				sumResSquared += pow(residual, 2);
			}

			variance = (sumResSquared - pow(sumRes, 2)) / numberOfPoints;

			printf("Variance of residuals: %f", variance);
		}

		//	Output
		if (true)
		{
			printf(" Alpha: %f\n Beta: %f\n Variance: %f\n", alpha, beta, variance);
		}
		else
		{

		}
	}
	else
	{
		printf("No points received by the algorithm ...");
	}

//	//	Output
//	if (bestCount > this->systemParameters.ransacProcessor.numberOfPointsToAcceptLine)
//	{
//		PointT line, center;
//		line.x = pointsOfInterest[1].x - pointsOfInterest[0].x;
//		line.y = pointsOfInterest[1].y - pointsOfInterest[0].y;
//		center.x = pointsOfInterest[0].x + line.x / 2;
//		center.y = pointsOfInterest[0].y + line.y / 2;
//
//		double lineLength, lineOrientation;
//		lineLength = std::sqrt(std::pow(line.x, 2) + std::pow(line.y, 2));
//		lineOrientation = std::atan2(line.y, line.x);
//
//		this->output.rowFound = true;
//		this->output.length = lineLength;
//		this->output.orientation = lineOrientation;
//		this->output.center = center;
//	}
//	else
//	{
//		double nan = std::numeric_limits<double>::quiet_NaN();
//
//		this->output.rowFound = false;
//		this->output.length = nan;
//		this->output.orientation = nan;
//		this->output.center = PointT();
//	}
}

void Extractors::RowExtractor::ransacProcessor (void)
{
	const bool debug = true;

	if (debug) printf(" Ransac processing:\n=========================================\n");

	//	If points available in cloud, process them
	if (this->preProcessedData.size())
	{
		if (debug) printf(" Processing %d points in cloud\n", (int)this->preProcessedData.size());

		//	Seed randomizer
		srand(time(0));

		//	Variables
		int ransacTryCounter = 0;				//	Counter for ransac tries
		int randomPoints[2];					//	Random index from cloud
		int randomTryCounter;					//	Counter of the number of random points selected
		const int maxRandomCount = 10;			//	Maximum number of random tries (NB not ransac tries)
		PointT lookingAt[2];					//	Currently looking at points
		PointT pointsOfInterest[2];				//	Points selected for further analysis
		PointT lineDirectionVector;				//	Direction vector for line (vector between points of interest)
		double lineLength;						//	Length of direction vector
		bool lineFound;							//	Did ransac select points far enough away from each other
		bool rowFound = false;					//	Row found?

		PointT currentPoint;					//	Current point to test
		PointT currentVector;					//	Current vector (from lookingAt[0] to currentPoint)
		PointT orthogonalVector;				//	Vector perpendicular to direction vector
		PointT xProjection;						//	Projection of currentVector onto lineDirectionVector
		PointT yProjection;						//	Projection of currentVector onto orthogonalVector
		double xDotProduct;						//	Dot product between currentVector and lineDirectionVector
		double yDotProduct;						//	Dot product between currentVector and orthogonalVector
		double currentVecLen;					//	Length of current vector
		double orthoVecLen;						//	Length of orthogonal vector
		double xProjLen;						//	Length of projection of currentVector onto lineDirectionVector
		double yProjLen;						//	Length of projection of currentVector onto orthogonalVector

		int ransacInlierCounter;				//	Count inlier points
		int ransacBestCount = 0;				//	Best inlier ransac count

		std::list<double> xProjTestDist;		//	Tentative placeholder of length of vectors projected onto line
		std::list<double> xProjDistribution;	//	Store projection length
		std::list<double> weights;				//	Store weights from xProjDistribution
		std::list<double> residuals;			//	Store residuals from linear regression on CDF of points along the line

		double inliersPercentage;				//	Percentage of inliers compared to the total number of points in the pre-processed cloud

		//	Find possible row
		while (ransacTryCounter < this->systemParameters.ransacProcessor.numberOfRansacTries)
		{
			ransacTryCounter++;

			ransacBestCount = 0;
			randomTryCounter = 0;
			lineFound = false;

			do
			{
				randomPoints[0] = rand() % this->preProcessedData.size();
				randomPoints[1] = rand() % this->preProcessedData.size();

				lookingAt[0] = this->preProcessedData.at(randomPoints[0]);
				lookingAt[1] = this->preProcessedData.at(randomPoints[1]);

				//	Calculate direction from two randomly selected points
				lineDirectionVector.x = lookingAt[1].x - lookingAt[0].x;
				lineDirectionVector.y = lookingAt[1].y - lookingAt[0].y;

				lineLength = sqrt(pow(lineDirectionVector.x, 2) + pow(lineDirectionVector.y, 2));

				randomTryCounter++;

				if (lineLength >= this->systemParameters.ransacProcessor.minimumRowLength) lineFound = true;
			} while (!lineFound && randomTryCounter < maxRandomCount);

			//	If proper line is found continue further analysis
			if (lineFound)
			{
				//if (debug) printf(" Line was found for further analysis ... \n");

				ransacInlierCounter = 0;

				xProjTestDist.clear();
				weights.clear();

				orthogonalVector.x = - lineDirectionVector.y;
				orthogonalVector.y = lineDirectionVector.x;

				//	Traverse all points in cloud
				for (int i = 0; i < this->preProcessedData.size(); i++)
				{
					//	Get current point
					currentPoint = this->preProcessedData[i];

					//	Calculate "TLS" distance
					currentVector.x = currentPoint.x - lookingAt[0].x;
					currentVector.y = currentPoint.y - lookingAt[0].y;

					//	Make projection of currentVector onto directionVector and orthogonalVector
					xDotProduct = currentVector.x * lineDirectionVector.x + currentVector.y * lineDirectionVector.y;
					yDotProduct = currentVector.x * orthogonalVector.x + currentVector.y * orthogonalVector.y;

					currentVecLen = sqrt(pow(currentVector.x, 2) + pow(currentVector.y, 2));
					orthoVecLen = sqrt(pow(orthogonalVector.x, 2) + pow(orthogonalVector.y, 2));

					xProjection.x = xDotProduct / pow(currentVecLen, 2) * currentVector.x;
					xProjection.y = xDotProduct / pow(currentVecLen, 2) * currentVector.y;

					yProjection.x = yDotProduct / pow(orthoVecLen, 2) * orthogonalVector.x;
					yProjection.y = yDotProduct / pow(orthoVecLen, 2) * orthogonalVector.y;

					xProjLen = sqrt(pow(xProjection.x, 2) + pow(xProjection.y, 2));
					yProjLen = sqrt(pow(yProjection.x, 2) + pow(yProjection.y, 2));

					//if (debug) printf(" Y-Projection length: %f\n", yProjLen);

					if (yProjLen < this->systemParameters.ransacProcessor.distanceFromLineThreshold)
					{
						ransacInlierCounter++;

						//	If projection onto direction vector is in bound add length to distribution
						if (xDotProduct > 0 && xProjLen < lineLength)
						{
							xProjTestDist.push_back(xProjLen);
						}
					}
				}

				if (ransacInlierCounter > ransacBestCount)
				{
					ransacBestCount = ransacInlierCounter;
					xProjDistribution = xProjTestDist;
				}
			}
		}

		//	If any "best" line was found, analyse quality
		if (xProjDistribution.size())
		{
			if (debug) printf(" Some best fit was made!\n");

			//	Calculate the percentage of inliers to line compared with the overall number of points
			inliersPercentage = (double)xProjDistribution.size() / (double)this->preProcessedData.size();

			xProjDistribution.sort();

			double numberOfPoints = (double)xProjDistribution.size();
			double weight = 1. / numberOfPoints;
			double cumulativeWeight = 0.0;

			for (std::list<double>::iterator it = xProjDistribution.begin(); it != xProjDistribution.end(); ++it)
			{
				cumulativeWeight += weight;
				weights.push_back(cumulativeWeight);
			}

			// Check distribution
			double sumX = 0, sumY = 0, sumXY = 0, sumSquareX = 0;
			double meanX, meanY, meanXY, meanSquareX;
			double alpha, beta;

			for (std::list<double>::iterator itx = xProjDistribution.begin(), ity = weights.begin(); itx != xProjDistribution.end(); ++itx, ++ity)
			{
				sumX += *itx;
				sumY += *ity;
				sumXY += (*itx) * (*ity);
				sumSquareX += pow(*itx, 2);
			}

			meanX = sumX / numberOfPoints;
			meanY = sumY / numberOfPoints;
			meanXY = sumXY / numberOfPoints;
			meanSquareX = sumSquareX / numberOfPoints;

			//	Calculate linear regression parameters
			alpha = (meanXY - meanX * meanY) / (meanSquareX - pow(meanX, 2));
			beta = meanY - alpha * meanX;

			//	Calculate variance of residuals
			double residual, sumRes = 0, sumResSquared = 0, variance;

			for (std::list<double>::iterator itx = xProjDistribution.begin(), ity = weights.begin(); ity != weights.end(); ++itx, ++ity)
			{
				residual = *ity - (alpha * (*itx) + beta);

				sumRes += residual;
				sumResSquared += pow(residual, 2);
			}

			variance = (sumResSquared - pow(sumRes, 2)) / numberOfPoints;

			printf(" Variance of residuals: %f", variance);
		}
		else
		{
			if (debug) printf(" No fit was made from point cloud!\n");
		}
	}
	else
	{
		if (debug) printf(" No points in point cloud!\n");
	}

	if (debug) printf("=========================================\n Ransac processing ended\n\n\n");
}
