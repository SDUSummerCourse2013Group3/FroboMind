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

Extractors::Output Extractors::RowExtractor::update (Input in)
{
	this->input = in;

	this->preProcessor();
	this->ransacProcessor();

	return this->output;
}

void Extractors::RowExtractor::preProcessor (void)
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

void Extractors::RowExtractor::rowProcessor (void)
{
	//	Variables
	const bool debug = true;
	const double lowerBoundPercentage = 0.1;
	double totalSizeOfCloud = (double)this->preProcessedData.size();
	double percentageLeft = 1.0;
	Row ransacOutput;

	//	Clear output
	this->output.rows.clear();
	this->output.rowFound = false;

	//	Find row(s) in point cloud with ransac
	while (percentageLeft > lowerBoundPercentage)
	{
		ransacOutput = ransacProcessor();

		this->output.rows.push_back(ransacOutput);

		percentageLeft = (double)this->preProcessedData.size() / totalSizeOfCloud;
	}

	//	Process row(s) and finalize output
	for (int i = 0; i < this->output.rows.size(); i++)
	{
		this->output.rowFound |= this->output.rows[i].rowFound;

		if (this->output.rows[i].rowFound)
		{
			if (debug)
		}
	}
}

Extractors::Row Extractors::RowExtractor::ransacProcessor (void)
{
	const bool debug = true;

	//	Setup empty row
	Row row;
	row.rowFound = false;
	row.pointInRow = PointT();
	row.lengthToRow = 0.0;
	row.orientation = 0.0;
	row.varianceOfRes = 0.0;

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
		double lineRotation;					//	Line rotation
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

		std::vector<int> inlierIndecies;		//	Storing indecies of inlier points
		inlierIndecies = std::vector<int>();

		double inliersPercentage;				//	Percentage of inliers compared to the total number of points in the pre-processed cloud

		//	Find possible row
		while (ransacTryCounter < this->systemParameters.ransacProcessor.numberOfRansacTries)
		{
			ransacTryCounter++;

			inlierIndecies.clear();
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

					//	If it is an inlier
					if (yProjLen < this->systemParameters.ransacProcessor.distanceFromLineThreshold)
					{
						ransacInlierCounter++;

						//	Add index to inlier index
						inlierIndecies.push_back(i);

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
			//	Calculate the percentage of inliers to line compared with the overall number of points
			inliersPercentage = (double)xProjDistribution.size() / (double)this->preProcessedData.size();

			xProjDistribution.sort();

			if (debug)
			{
				printf(" Number of inliers: %d\n", (int)inlierIndecies.size());
				printf(" Before reduction of point cloud: %d\n", (int)this->preProcessedData.size());
			}

			//	Remove inliers from pre-processed cloud
			for (int index = inlierIndecies.size() - 1; index >= 0; index--)
			{
				this->preProcessedData.erase(preProcessedData.begin() + inlierIndecies[index]);
			}

			if (debug) printf(" After reduction of point cloud: %d\n", (int)this->preProcessedData.size());

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
			lineRotation = std::atan2(lineDirectionVector.y, lineDirectionVector.x);

			//	Setup output
			row.rowFound = true;
			row.pointInRow = lookingAt[0];
			row.lengthToRow = lineLength;
			row.orientation = lineRotation;
			row.varianceOfRes = variance;

			if (debug)
			{
				printf(" Point in line: (%f, %f)\n", lookingAt[0].x, lookingAt[0].y);
				printf(" Line length: %f\n", lineLength);
				printf(" Line rotation: %f\n", lineRotation);
				printf(" Variance of residuals: %f\n", variance);
			}
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

	return row;
}
