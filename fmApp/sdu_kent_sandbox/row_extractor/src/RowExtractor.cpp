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
	//	Pre-Processor
	this->systemParameters.preProcessor.active = false;
	this->systemParameters.preProcessor.minimumClearZoneDistance = 0.0;
	this->systemParameters.preProcessor.maximumClearZoneDistance = 0.0;

	//	Segmentation processor
	this->systemParameters.segmentationProcessor.active = false;
	this->systemParameters.segmentationProcessor.segmentationDistance = 0.0;

	//	Ransac processor
	this->systemParameters.ransacProcessor.numberOfRansacTries = 0;
	this->systemParameters.ransacProcessor.numberOfPointsToAcceptLine = 0;
	this->systemParameters.ransacProcessor.distanceFromLineThreshold = 0.0;

	//	Setup input (empty)
	this->input.pointCloud = pcl::PointCloud<PointT>();

	//	Setup output (empty)
	this->output.orientation = 0.0;
}

Extractors::RowExtractor::RowExtractor(Parameters par)
{
	//	Setup default parameters
	this->systemParameters = par;

	//	Setup input (empty)
	this->input.pointCloud = pcl::PointCloud<PointT>();

	//	Setup output (empty)
	this->output.orientation = 0.0;
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

	this->preProcess();
	this->ransacProcess();

	return this->output;
}

void Extractors::RowExtractor::preProcess (void)
{
	//	Cope data from input cloud to pre-processed cloud
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

void Extractors::RowExtractor::segmentationProcess (void)
{

}

void Extractors::RowExtractor::ransacProcess (void)
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

		this->output.length = lineLength;
		this->output.orientation = lineOrientation;
	}
	else
	{
		double nan = std::numeric_limits<double>::quiet_NaN();

		this->output.length = nan;
		this->output.orientation = nan;
	}
}
