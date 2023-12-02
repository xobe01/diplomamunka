#pragma once

#include <thread>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include "Vec3.cpp"
#include <algorithm>
#include <sstream>
#include <vector>
#include <chrono>

struct Point;

struct Point_CUDA;

struct Plane;

struct Plane_CUDA;

struct Point
{
	bool isMarked2{ false };
	size_t horizontalIndex;
	size_t verticalIndex;
	Vec3<double> position;
	Vec3<double> projected2DPosition;
	Plane* plane{ 0 };
	size_t cornerId{ 0 };
	size_t outlineId{ 0 };
	size_t cornerIndex{ 0 };
	Vec3<double> normal = { 0, 0, 0 };
	bool isMarked{ false };

	std::vector<Point*> neighbourPlaneNeighbours;
	std::vector<Point*> createdNeighbourPoints;
	std::vector<size_t> convexId;
	std::vector<size_t> convexIndex;
	bool isCorner;

	Point(Vec3<double> _position, size_t _horizontalIndex, size_t _verticalIndex, Plane* _plane) : position(_position), horizontalIndex(_horizontalIndex),
		verticalIndex(_verticalIndex), plane(_plane)
	{
		isCorner = false;
		projected2DPosition = { 100000, 100000, 100000 };
		neighbourPlaneNeighbours = { nullptr, nullptr, nullptr, nullptr };
		createdNeighbourPoints = { nullptr, nullptr, nullptr, nullptr };
		convexId = {};
		convexIndex = {};
	};
	std::string printCoordinates()
	{
		return std::to_string(horizontalIndex) + " " + std::to_string(verticalIndex) + '\n';
	}
};

struct Point_CUDA
{
	Vec3<double> position;
	Vec3<double> projected2DPosition;
	__host__ __device__ void copyFromPoint(Point* point)
	{
		position = point->position;
		projected2DPosition = point->projected2DPosition;
	}
};

struct Edge
{
	bool isMarked = false;
	bool isHole;
	bool wasFirstGenerated;
	bool isInUse = true;
	Point* startPoint;
	std::vector<std::pair<Vec3<double>, Vec3<double>>> closestNeighbourPoints;
	std::vector<std::pair<Point*, int>> pointsWithDir;
	std::pair<double, double> xBounds2D;
	std::pair<double, double> yBounds2D;
	Edge() : isHole(false), wasFirstGenerated(false), xBounds2D(100000, -100000), yBounds2D(100000, -100000) {}
	bool canIntersectWithEdge(Edge* edge)
	{
		return ((edge->xBounds2D.first >= xBounds2D.first && edge->xBounds2D.first <= xBounds2D.second) ||
			(edge->xBounds2D.second >= xBounds2D.first && edge->xBounds2D.first <= xBounds2D.first) ||
			(xBounds2D.first >= edge->xBounds2D.first && xBounds2D.first <= edge->xBounds2D.second) ||
			(xBounds2D.second >= edge->xBounds2D.first && xBounds2D.first <= edge->xBounds2D.first)) &&
			((edge->yBounds2D.first >= yBounds2D.first && edge->yBounds2D.first <= yBounds2D.second) ||
				(edge->yBounds2D.second >= yBounds2D.first && edge->yBounds2D.first <= yBounds2D.first) ||
				(yBounds2D.first >= edge->yBounds2D.first && yBounds2D.first <= edge->yBounds2D.second) ||
				(yBounds2D.second >= edge->yBounds2D.first && yBounds2D.first <= edge->yBounds2D.first));
	}
	std::vector<Point*> getPoints()
	{
		std::vector<Point*> newEdgePoints;
		for (size_t i = 0; i < pointsWithDir.size(); i++) {
			newEdgePoints.push_back(pointsWithDir[i].first);
		}
		return newEdgePoints;
	}
};

struct Edge_CUDA
{
	bool isHole;
	double xBounds2DMin;
	double xBounds2DMax;
	double yBounds2DMin;
	double yBounds2DMax;
	Point_CUDA** pointsWithDir;
	size_t pointsSize;

	__host__ __device__ void copyFromEdge(Edge* edge)
	{
		isHole = edge->isHole;
		cudaMallocManaged(&pointsWithDir, edge->pointsWithDir.size() * sizeof(Point_CUDA*));
		for (size_t i = 0; i < edge->pointsWithDir.size(); i++) {
			Point_CUDA* newPoint_CUDA;
			cudaMallocManaged(&newPoint_CUDA, sizeof(Point_CUDA*));
			pointsWithDir[i] = newPoint_CUDA;
			newPoint_CUDA->copyFromPoint(edge->pointsWithDir[i].first);
		}
		pointsSize = edge->pointsWithDir.size();
	}
	__device__ bool canIntersectWithEdge(Edge_CUDA* edge)
	{
		return ((edge->xBounds2DMin >= xBounds2DMin && edge->xBounds2DMin <= xBounds2DMax) ||
			(edge->xBounds2DMax >= xBounds2DMin && edge->xBounds2DMin <= xBounds2DMin) ||
			(xBounds2DMin >= edge->xBounds2DMin && xBounds2DMin <= edge->xBounds2DMax) ||
			(xBounds2DMax >= edge->xBounds2DMin && xBounds2DMin <= edge->xBounds2DMin)) &&
			((edge->yBounds2DMin >= yBounds2DMin && edge->yBounds2DMin <= yBounds2DMax) ||
				(edge->yBounds2DMax >= yBounds2DMin && edge->yBounds2DMin <= yBounds2DMin) ||
				(yBounds2DMin >= edge->yBounds2DMin && yBounds2DMin <= edge->yBounds2DMax) ||
				(yBounds2DMax >= edge->yBounds2DMin && yBounds2DMin <= edge->yBounds2DMin));
	}
	__device__ Vec3<double>* getPoints()
	{
		Vec3<double>* positions;
		positions = new Vec3<double>[pointsSize];
		for (size_t i = 0; i < pointsSize; i++) {
			positions[i] = pointsWithDir[i]->projected2DPosition;
		}
		return positions;
	}
};

struct Plane {
	std::vector<Point*> points;
	std::vector<Edge*> edges;
	Vec3<double> planePointPos;
	Vec3<double> normal;
	Vec3<double> pointDirections[2] = { {0,0,0}, {0,0,0} };
	size_t id;
	size_t presentInFrames = 1;
	std::vector<std::vector<Point*>> convexFaces;
	Vec3<double> furthestNormalPoints[2] = { {0,0,0}, {0,0,0} };
	std::vector<std::pair<Vec3<double>, Vec3<double>>> closestNeighbourPointsHole;
	std::vector<std::pair<Vec3<double>, Vec3<double>>> closestNeighbourPointsNonHole;
	void calculateAvaragePointPos()
	{
		std::pair<double, double> normalDistances = { 1000, -1000 };
		for (size_t i = 0; i < points.size(); i++) {
			double dist = Vec3<double>::dot_product(normal, points[i]->position - planePointPos);
			if (dist < normalDistances.first) {
				normalDistances.first = dist;
				furthestNormalPoints[0] = points[i]->position;
			}
			if (dist > normalDistances.second) {
				normalDistances.second = dist;
				furthestNormalPoints[1] = points[i]->position;
			}
		}
	}
};

struct Plane_CUDA
{
	Vec3<double> normal;
	Vec3<double> furthestNormalPoints[2] = { {0,0,0}, {0,0,0} };
	Vec3<double> planePointPos;
	Vec3<double> pointDirections[2] = { {0,0,0}, {0,0,0} };
	Edge_CUDA** edges;
	size_t edgesSize;

	__host__ __device__ void copyFromPlane(Plane* plane)
	{
		normal = plane->normal;
		furthestNormalPoints[0] = plane->furthestNormalPoints[0];
		furthestNormalPoints[1] = plane->furthestNormalPoints[1];
		planePointPos = plane->planePointPos;
		pointDirections[0] = plane->pointDirections[0];
		pointDirections[1] = plane->pointDirections[1];
		cudaMallocManaged(&edges, plane->edges.size() * sizeof(Edge_CUDA*));
		for (size_t i = 0; i < plane->edges.size(); i++) {
			Edge_CUDA* newEdge_CUDA;
			cudaMallocManaged(&newEdge_CUDA, sizeof(Edge_CUDA*));
			edges[i] = newEdge_CUDA;
			newEdge_CUDA->copyFromEdge(plane->edges[i]);
		}
		edgesSize = plane->edges.size();
	}
};

const double PI = 3.14159265359;
const std::pair<double, double> rayAngles = { -18, 20 };

double dbgTimeCounter = 0;
const size_t pointCloudCount = 10;
const size_t pointCloudBeginIndex = 0;
const int pointCloudTestIndex = -1;
std::vector<Vec3<double>> egoCarPos;
size_t currentFrame = 0;

bool checkIfBridge(Point* p, bool onlyMarked, size_t pointCloudIndex);
void writeData(size_t pointCloudIndex);
bool isEdgeTooTigth(Edge* edge);