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
#include "kernel.h"
#include <chrono>

struct Point
{
	size_t horizontalIndex;
	size_t verticalIndex;
	Vec3<double> position;
	Plane* plane{ 0 };
	size_t cornerId{ 0 };
	size_t outlineId{ 0 };
	size_t cornerIndex{ 0 };
	Vec3<double> normal = { 0, 0, 0 };
	bool isMarked{ false };
	bool isMarked2{ false };
	std::vector<Point*> neighbourPlaneNeighbours;
	std::vector<Point*> createdNeighbourPoints;
	bool isCorner;
	Point(Vec3<double> _position, size_t _horizontalIndex, size_t _verticalIndex, Plane* _plane) : position(_position), horizontalIndex(_horizontalIndex),
		verticalIndex(_verticalIndex), plane(_plane)
	{
		isCorner = false;
		neighbourPlaneNeighbours = { nullptr, nullptr, nullptr, nullptr };
		createdNeighbourPoints = { nullptr, nullptr, nullptr, nullptr };
	};
	std::string printCoordinates()
	{
		return std::to_string(horizontalIndex) + " " + std::to_string(verticalIndex) + '\n';
	}
};

struct Plane {
	std::vector<Point*> points;
	std::vector<std::pair<Point*, std::vector<std::pair<Point*, int>>>> edges;
	Vec3<double> planePointPos;
	Vec3<double> normal;
	std::pair<Vec3<double>, Vec3<double>> pointDirections;
	size_t id;
};

std::vector<Point*> points;
std::vector<Point*> addedPoints;
std::vector<Plane*> planes;
std::vector<int> verticalCounts;
size_t horizontalCount;
size_t verticalCount;
int currentCornerId = 1;
int currentSeparatedObjectId = 1;
int currentPlaneId = 1;
int currentOutlineId = 1;
int currentCornerIndex = 0;
const double objectPointDistance = 5;
const double planeDistanceTreshold = 0.05;

size_t getOffset(int horizontalIndex, int verticalIndex)
{
	if (horizontalIndex < 0) horizontalIndex = horizontalCount - 1 + horizontalIndex;
	else if (horizontalIndex > horizontalCount - 1) horizontalIndex = horizontalIndex - horizontalCount;
	if (verticalIndex < 0) verticalIndex = verticalCount - 1 + verticalIndex;
	else if (verticalIndex > verticalCount - 1) verticalIndex = verticalIndex - verticalCount;
	return horizontalIndex * verticalCount + verticalIndex;
}

void readData()
{
	verticalCounts.push_back(0);
    std::string myText;		  
    std::ifstream MyReadFile("C:/Users/ungbo/Desktop/BME/_Diplomamunka/Diplomamunka/Diplomamunka/Assets/Resources/points_raw.txt");
	getline(MyReadFile, myText);
	getline(MyReadFile, myText);
	horizontalCount = std::stoi(myText);
	getline(MyReadFile, myText);
	verticalCount = std::stoi(myText);
	points.resize(verticalCount * horizontalCount);
    while (getline(MyReadFile, myText)) {
		std::replace(myText.begin(), myText.end(), ',', '.');
		std::stringstream ss(myText);
		std::string _x, _y, _z, _horizontalIndex, _verticalIndex, _id;
		std::getline(ss, _x, ';');
		if (_x == myText) {
			verticalCounts.push_back(std::stoi(myText));
		}
		else {
			std::getline(ss, _y, ';');
			std::getline(ss, _z, ';');
			std::getline(ss, _horizontalIndex, ';');
			std::getline(ss, _verticalIndex, ';');
			std::getline(ss, _id, ';');
			double x = std::stof(_x);
			double y = std::stof(_y);
			double z = std::stof(_z);
			int id = std::stoi(_id);
			int horizontalIndex = std::stoi(_horizontalIndex);
			int verticalIndex = std::stoi(_verticalIndex);
			points[getOffset(horizontalIndex, verticalIndex)] = new Point({ x, y, z }, horizontalIndex, verticalIndex, nullptr);
		}		
	}
    MyReadFile.close();
}

void writePoints(const std::vector<Point*> points, std::ofstream& MyFile)
{
	for (size_t i = 0; i < points.size(); i++) {
		if (points[i])
			MyFile << points[i]->position.to_string() << ';' << points[i]->horizontalIndex << ';' << points[i]->verticalIndex <<
			';' << (points[i]->plane ? points[i]->plane->id : 0) << ';' << points[i]->outlineId << ';' << points[i]->cornerId
			<< ';' << points[i]->cornerIndex << std::endl;
	}
}

void writePlanes(std::ofstream& MyFile)
{
	MyFile << planes.size() << std::endl;
	for (size_t i = 0; i < planes.size(); i++) {
		MyFile << planes[i]->planePointPos.x << ';' << planes[i]->planePointPos.y << ';' << planes[i]->planePointPos.z << ';' 
			<< planes[i]->normal.x << ';' << planes[i]->normal.y << ';' << planes[i]->normal.z << std::endl;
	}
}

void writeData()
{
	std::ofstream MyFile("C:/Users/ungbo/Desktop/BME/_Diplomamunka/Diplomamunka/Diplomamunka/Assets/Resources/points_processed.txt");
	writePlanes(MyFile);
	writePoints(points, MyFile);
	writePoints(addedPoints, MyFile);
	MyFile.close();
}

void groundSegmentation() { //TODO point struktúra megvátozott
	double groundLevel = 100;
	for (size_t i = 0; i < points.size(); i++) {
		if (points[i] && points[i]->position.y < groundLevel) groundLevel = points[i]->position.y;
	}
	for (size_t i = 0; i < points.size(); i++) {
		if (points[i] && points[i]->position.y <= groundLevel) {
			points[i] = nullptr;
		}
	}
}

#include <random>

std::mt19937 gen(100);

Vec3<Point*> pick3Points(const std::vector<Point*>& nonProcessedPoints)
{
	std::uniform_int_distribution<int> distr(0, nonProcessedPoints.size() - 1);
	size_t index1 = distr(gen);
	size_t index2;
	size_t index3;
	do {
		index2 = distr(gen);
	} while (index1 == index2);

	do {
		index3 = distr(gen);
	} while (index1 == index3 || index2 == index3);

	Vec3<double> v1(1, 2, 3);
	Vec3<double> v2(3, 1, 1);
	Vec3<double> v3(1, 5, 2);
	auto normal = Vec3<double>::normalize(Vec3<double>::crossProduct(v1 -
		v2, v3 - v2));
	double dist = abs(Vec3<double>::dot_product(v3 - v2, normal));
	return Vec3<Point*>(nonProcessedPoints[index1], nonProcessedPoints[index2], nonProcessedPoints[index3]);
}

int spikeType(Point* p, int arriveDirection, bool onlyMarkedNeighbours)
{
	//arriveDirection
	// 0 - from left
	// 1 - from up
	// 2 - from rigth
	// 3 - from down
	size_t x = p->horizontalIndex;
	size_t y = p->verticalIndex;
	size_t neighbourCount = 0;
	bool isNeighbour[4] = { false, false, false, false };
	bool diagIsNeighbour[4] = { false, false, false, false };
	Point* neighbourPoints[4] = { points[getOffset(x, y - 1)], points[getOffset(x, y + 1)], points[getOffset(x - 1, y)],
				points[getOffset(x + 1, y)] };

	Point* diagNeighbourPoints[4] = { points[getOffset(x - 1, y - 1)], points[getOffset(x + 1, y - 1)], points[getOffset(x + 1, y + 1)], points[getOffset(x - 1, y + 1)] };
	for (size_t j = 0; j < 4; j++) {
		if (neighbourPoints[j] && (j > 0 || y > 0) && (j < 3 || y < verticalCount - 1) && neighbourPoints[j]->plane == p->plane && 
			(!onlyMarkedNeighbours || neighbourPoints[j]->isMarked2)) {
			neighbourCount++;
			isNeighbour[j] = true;
		}
	}
	for (size_t j = 0; j < 4; j++) {
		if (diagNeighbourPoints[j] && (j > 1 || y > 0) && (j < 2 || y < verticalCount - 1) && diagNeighbourPoints[j]->plane == p->plane &&
			(!onlyMarkedNeighbours || diagNeighbourPoints[j]->isMarked2)) {
			diagIsNeighbour[j] = true;
		}
	}
	if (neighbourCount == 2 && ((isNeighbour[0] && isNeighbour[2] && !diagIsNeighbour[0]) || (isNeighbour[0] && isNeighbour[3] && !diagIsNeighbour[1]) 
		|| (isNeighbour[1] && isNeighbour[2] && !diagIsNeighbour[3]) || (isNeighbour[1] && isNeighbour[3] && !diagIsNeighbour[2])))
		return -1;
	if ((neighbourCount == 2 && ((isNeighbour[0] && isNeighbour[1]) || (isNeighbour[2] && isNeighbour[3]))) || (neighbourCount > 1 && 
		((arriveDirection == 0 && !isNeighbour[1]) || (arriveDirection == 1 && !isNeighbour[2]) || (arriveDirection == 2 && !isNeighbour[0])
			|| (arriveDirection == 3 && !isNeighbour[3]))))
		return 0;
	if (neighbourCount == 1)
		return 1;
	if (neighbourCount > 1)
		return 2;
	return 3;
}

bool checkIfBridge(Point* p)
{
	if (spikeType(p, -1, false) == -1)
		return true;
	size_t x = p->horizontalIndex;
	size_t y = p->verticalIndex;
	int neighbourCount = 0;

	Point* neighbourPoints[4] = { points[getOffset(x, y - 1)], points[getOffset(x, y + 1)], points[getOffset(x - 1, y)],
				points[getOffset(x + 1, y)] };
	for (size_t j = 0; j < 4; j++) {
		if (neighbourPoints[j] && neighbourPoints[j]->plane == p->plane && (j > 0 || y > 0) && (j < 3 || y < verticalCount - 1) &&
			spikeType(neighbourPoints[j], -1, false) > 1)
			neighbourCount++;
	}
	if ((((y > 0 && (!points[getOffset(x - 1, y - 1)] || points[getOffset(x - 1, y - 1)]->plane != p->plane)) &&
		(y < verticalCount - 1 && (!points[getOffset(x + 1, y + 1)] || points[getOffset(x + 1, y + 1)]->plane != p->plane))) ||
		((y > 0 && (!points[getOffset(x + 1, y - 1)] || points[getOffset(x + 1, y - 1)]->plane != p->plane)) &&
			(y < verticalCount - 1 && (!points[getOffset(x - 1, y + 1)] || points[getOffset(x - 1, y + 1)]->plane != p->plane))))
		&& neighbourCount > 2)
		return true;
	return false;
}

bool isThereBridge(std::vector<Point*>& planePoints)
{
	std::vector<Point*> newPoints;
	bool theresBridge = false;
	for (auto p : planePoints)
		if (p->plane != nullptr && checkIfBridge(p)) {
			theresBridge = true;
			p->plane = nullptr;
		}
		else
			newPoints.push_back(p);
	planePoints = newPoints;
	return theresBridge;
}

Vec3<double> getNormal(Point* center, Point* p1, Point* p2)
{
	return Vec3<double>::crossProduct(p1->position - center->position, p2->position - center-> position);
}

void calculateNormal(Point* point)
{
	size_t x = point->horizontalIndex;
	size_t y = point->verticalIndex;
	Point* neighbourPoint1 = points[getOffset(x, y - 1)];
	Point* neighbourPoint2 = points[getOffset(x + 1, y)];
	Point* neighbourPoint3 = points[getOffset(x, y + 1)];
	Point* neighbourPoint4 = points[getOffset(x - 1, y)];
	if (y > 0 && neighbourPoint1 && neighbourPoint2) {
		point->normal = point->normal + getNormal(point, neighbourPoint1, neighbourPoint2);
	}
	if (y < verticalCount - 1 && neighbourPoint2 && neighbourPoint3) {
		point->normal = point->normal + getNormal(point, neighbourPoint2, neighbourPoint3);
	}
	if (y < verticalCount - 1 && neighbourPoint3 && neighbourPoint4) {
		point->normal = point->normal + getNormal(point, neighbourPoint3, neighbourPoint4);
	}
	if (y > 0 && neighbourPoint4 && neighbourPoint1) {
		point->normal = point->normal + getNormal(point, neighbourPoint4, neighbourPoint1);
	}
	point->normal = Vec3<double>::normalize(point->normal);
}

int areNeighbours(Point* p1, Point* p2)
{
	if (points[getOffset(p1->horizontalIndex + 1, p1->verticalIndex)] == p2)
		return 1;
	if (points[getOffset(p1->horizontalIndex, p1->verticalIndex + 1)] == p2)
		return 2;
	if (points[getOffset(p1->horizontalIndex - 1, p1->verticalIndex)] == p2)
		return 3;
	if (points[getOffset(p1->horizontalIndex, p1->verticalIndex - 1)] == p2)
		return 4;
	return 0;
}

void choosePoints(const Vec3<Point*> planePoints, Plane* basePlane, /*out*/ Plane* plane)
{
	if (basePlane) {
		plane->pointDirections = basePlane->pointDirections;
		plane->normal = basePlane->normal;
	}
	else 
	{
		auto normal = Vec3<double>::normalize(Vec3<double>::crossProduct(planePoints.y->position -
			planePoints.x->position, planePoints.z->position - planePoints.x->position));
		Vec3<double> horizontalDirection = { 0,0,0 };
		Vec3<double> verticalDirection = { 0,0,0 };
		Point* neighbours[2] = { planePoints.y, planePoints.z };
		for each (auto neighbour in neighbours) {
			switch (areNeighbours(planePoints.x, neighbour)) {
			case 1:
			horizontalDirection = neighbour->position - planePoints.x->position;
			break;
			case 2:
			verticalDirection = neighbour->position - planePoints.x->position;
			break;
			case 3:
			horizontalDirection = planePoints.x->position - neighbour->position;
			break;
			case 4:
			verticalDirection = planePoints.x->position - neighbour->position;
			break;
			default:
			break;
			}
		}
		plane->pointDirections = { horizontalDirection, verticalDirection };
		plane->normal = normal;
	}
	plane->planePointPos = planePoints.x->position;
	plane->id = currentPlaneId;
	std::vector<Point*>nextStepPoints;
	nextStepPoints.push_back(planePoints.x);
	planePoints.x->isMarked = false;
	planePoints.x->isMarked2 = false;
	planePoints.x->plane = plane;
	plane->points.push_back(planePoints.x);
	while (nextStepPoints.size() > 0) {
		std::vector<Point*> tempNextStepPoints;
		for (size_t i = 0; i < nextStepPoints.size(); i++) {
			size_t x = nextStepPoints[i]->horizontalIndex;
			size_t y = nextStepPoints[i]->verticalIndex;
			Point* neighbourPoints[4] = { points[getOffset(x, y - 1)], points[getOffset(x, y + 1)], points[getOffset(x - 1, y)], 
				points[getOffset(x + 1, y)] };
			for (size_t j = 0; j < 4; j++) {
				if (neighbourPoints[j] && (j > 0 || y > 0) && (j < 3 || y < verticalCount - 1) && neighbourPoints[j]->isMarked2) {
					double dist = abs(Vec3<double>::dot_product(plane->normal, neighbourPoints[j]->position - plane->planePointPos));
					if (dist <= planeDistanceTreshold) {
						plane->points.push_back(neighbourPoints[j]);
						neighbourPoints[j]->isMarked = false;
						neighbourPoints[j]->isMarked2 = false;
						neighbourPoints[j]->plane = plane;
						tempNextStepPoints.push_back(neighbourPoints[j]);
						if ((neighbourPoints[j]->normal - plane->normal).length() < 0.05) {
							plane->normal = ((plane->normal * (plane->points.size() - 1) + neighbourPoints[j]->normal)) / plane->points.size();
						}
					}
				}
			}
		}
		nextStepPoints = tempNextStepPoints;
	}
}

void findPlanes()
{
	size_t minPointCount = 10;
	size_t counter = 1;
	double normalTreshold = 0.01;
	for (size_t i = 0; i < points.size(); i++)
		if (points[i]) {
			points[i]->isMarked = true;
			points[i]->isMarked2 = true;
			calculateNormal(points[i]);
		}
	std::vector<Point*> nextStepPoints;
	for (size_t j = 0; j < points.size(); j++) {
		if (points[j] && points[j]->isMarked) {
			nextStepPoints.push_back(points[j]);
			calculateNormal(points[j]);
			while (nextStepPoints.size() > 0) {
				Plane* plane = new Plane();
				bool planeMerged;
				std::vector<Point*> tempNextStepPoints;
				for (size_t i = 0; i < nextStepPoints.size(); i++) {
					if (nextStepPoints[i]->isMarked2) {
						auto normal = nextStepPoints[i]->normal;
						Vec3<double> normals[4] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} };
						size_t x = nextStepPoints[i]->horizontalIndex;
						size_t y = nextStepPoints[i]->verticalIndex;
						Point* neighbourPoints[4] = { points[getOffset(x, y - 1)], points[getOffset(x + 1, y)], points[getOffset(x, y + 1)],
							points[getOffset(x - 1, y)] };
						for (size_t k = 0; k < 4; k++) {
							if (neighbourPoints[k] && (k > 0 || y > 0) && (k != 2 || y < verticalCount - 1) && neighbourPoints[k]->isMarked2) {
								if (neighbourPoints[k]->isMarked) {
									neighbourPoints[k]->isMarked = false;
									tempNextStepPoints.push_back(neighbourPoints[k]);
								}
								if (neighbourPoints[k]->normal.length() == 0) {
									calculateNormal(neighbourPoints[k]);
								}
								normals[k] = neighbourPoints[k]->normal;
							}
						}
						for (size_t k = 0; k < 4; k++) {
							if (normals[k].length() > 0 && normals[(k + 1) % 4].length() > 0 &&
								(normals[k] - normal).length() < normalTreshold && (normals[(k + 1) % 4] - normal).length() < normalTreshold) {
								choosePoints({ nextStepPoints[i], neighbourPoints[k], neighbourPoints[(k + 1) % 4] }, nullptr,
									plane);
								break;
							}
						}
					}
				}
				if (plane->points.size() > 0)
				{
					planes.push_back(plane);
					currentPlaneId++;
				}
				nextStepPoints = tempNextStepPoints;
			}
		}
	}
	for (size_t i = 0; i < points.size(); i++) if (points[i]) points[i]->isMarked = false;
	for (size_t i = 0; i < points.size(); i++) if (points[i]) points[i]->isMarked2 = false;
	for (size_t i = 0; i < planes.size(); i++) {
		int originalSize = planes[i]->points.size();
		while (isThereBridge(planes[i]->points)) {}		
		if (originalSize != planes[i]->points.size() && planes[i]->points.size() > 0) //cutting plane		
		{
			for (size_t j = 0; j < planes[i]->points.size(); j++) planes[i]->points[j]->isMarked2 = true;
			while (true) {
				Plane* plane = new Plane();
				choosePoints({ planes[i]->points[0], nullptr, nullptr }, planes[i], plane);
				if (plane->points.size() < planes[i]->points.size()) {
					for (size_t j = 0; j < planes[i]->points.size(); j++) {
						if (planes[i]->points[j]->plane != planes[i]) {
							planes[i]->points.erase(planes[i]->points.begin() + j);
							j--;
						}
					}
					planes.push_back(plane);
					currentPlaneId++;
				}
				else 
				{
					for (size_t j = 0; j < plane->points.size(); j++) {
						plane->points[j]->plane = planes[i];
					}
					delete plane;
					break;
				}
			}
		}
	}
	/*for (size_t i = 0; i < planes.size(); i++) {
		calculateBounds(planes[i]);	
	}/**/	
}

void findNextPoint(Point*& startPoint, size_t direction, bool isPreviousSpike, bool wasThereNonSpike, /*out*/ 
	std::vector<std::pair<Point*, int>>& currentEdge, size_t dbgPlaneIndex, std::vector<Plane*> dbgPlanes)
{
	Point* currentPoint = nullptr;
	std::pair<Point*, size_t> previousSavedPoint = {nullptr, 0};
	bool isFirstPoint = true;
	bool comeFromDeadEnd = false;
	while (currentPoint != startPoint || comeFromDeadEnd)
	{
		if (!currentPoint)
			currentPoint = startPoint;
		Point* neighbourPoint = nullptr;
		size_t x = currentPoint->horizontalIndex;
		size_t y = currentPoint->verticalIndex;
		if (spikeType(currentPoint, isFirstPoint ? -1 : ((direction + 1) % 4), false) == 0)
			isPreviousSpike = true;
		else {
			if (currentPoint->isMarked) {
				currentEdge.push_back({ currentPoint, direction });				
			}
		}
		currentPoint->isMarked = false;
		currentPoint->isMarked2 = false;
		isFirstPoint = false;
		for (size_t i = 0; i < 4; i++) {
			switch (direction) {
			case 0: //to right
			neighbourPoint = points[getOffset(x + 1, y)];
			break;
			case 1: //to down
			neighbourPoint = y == verticalCount - 1 ? nullptr : points[getOffset(x, y + 1)];
			break;
			case 2: //to left
			neighbourPoint = points[getOffset(x - 1, y)];
			break;
			case 3: //to up
			neighbourPoint = y == 0 ? nullptr : points[getOffset(x, y - 1)];
			break;
			default:
			break;
			}
			if (neighbourPoint == startPoint) {
				currentPoint = startPoint;
				comeFromDeadEnd = false;
				break;
			}
			if (neighbourPoint && neighbourPoint->plane == startPoint->plane && neighbourPoint->plane != nullptr && neighbourPoint->isMarked 
				&& (isPreviousSpike || spikeType(neighbourPoint, -1, false) <= 1 || spikeType(currentPoint, direction, false) > 0))
			{
					if (!wasThereNonSpike && spikeType(currentPoint, -1, false) > 1) {
					startPoint = currentPoint;
					isPreviousSpike = false;
					wasThereNonSpike = true;
				}
				if (!isPreviousSpike)
					previousSavedPoint = { currentPoint, (direction + (4 - i)) % 4 };
				if (spikeType(neighbourPoint, -1, false) == 1 || (wasThereNonSpike && currentEdge.size() > 1 && isPreviousSpike &&
					spikeType(neighbourPoint, -1, false) == 2 && neighbourPoint->isMarked)) {
					auto savedPoint = spikeType(neighbourPoint, -1, false) == 1 ? neighbourPoint : currentPoint;
					currentEdge.push_back({ savedPoint, (direction + 3) % 4 });
					savedPoint->isMarked = false;
					savedPoint->isMarked2 = false;
					currentPoint = previousSavedPoint.first;
					direction = previousSavedPoint.second;
					isPreviousSpike = false;
					comeFromDeadEnd = true;
					break;
				}
				else {
					comeFromDeadEnd = false;
					currentPoint = neighbourPoint;
				}
				direction = (direction + 3) % 4;
				break;
			}
			direction += direction == 3 ? -3 : 1;
			if (i == 3) {
				if (!wasThereNonSpike) {
					currentEdge.clear();
					return;
				}
				if (currentPoint == startPoint)
					return;
				currentPoint = currentEdge[currentEdge.size() - 2].first;
				direction = currentEdge[currentEdge.size() - 2].second;
				std::cout << "INVALID EDGE SEARCH" << std::endl;;
			}
		}
	}
}

bool isEdgePoint(Point* point)
{
	if (!point->isMarked)
		return false;
	size_t x = point->horizontalIndex;
	size_t y = point->verticalIndex;
	Point* neighbourPoint = points[getOffset(x, y - 1)];
	if (y == 0 || !neighbourPoint || !neighbourPoint->isMarked) {
		return true;
	}
	neighbourPoint = points[getOffset(x, y + 1)];
	if (y == verticalCount - 1 || !neighbourPoint || !neighbourPoint->isMarked) {
		return true;
	}
	neighbourPoint = points[getOffset(x - 1, y)];
	if (!neighbourPoint || !neighbourPoint->isMarked) {
		return true;
	}
	neighbourPoint = points[getOffset(x + 1, y)];
	if (!neighbourPoint || !neighbourPoint->isMarked) {
		return true;
	}
	return false;
}

void findEdgePoints()
{
	for (size_t i = 0; i < planes.size(); i++) 
	{
		std::vector<Point*> edgePointsInPlane;
		for (size_t j = 0; j < planes[i]->points.size(); j++) planes[i]->points[j]->isMarked = true;
		for (size_t j = 0; j < planes[i]->points.size(); j++) planes[i]->points[j]->isMarked2 = true;
		for (size_t j = 0; j < planes[i]->points.size(); j++) if(isEdgePoint(planes[i]->points[j])) edgePointsInPlane.push_back(planes[i]->points[j]);
		while (edgePointsInPlane.size() > 0) 
		{
			std::vector<Point*> tempEdgePointsInPlane;
			Point* startPoint = edgePointsInPlane[0];
			size_t minHorizontalCoord = startPoint->horizontalIndex;
			size_t minVerticalCoord = startPoint->verticalIndex;
			for (size_t j = 1; j < edgePointsInPlane.size(); j++) {
				if ((edgePointsInPlane[j]->horizontalIndex < minHorizontalCoord && minHorizontalCoord - edgePointsInPlane[j]->horizontalIndex < horizontalCount / 2)
					|| edgePointsInPlane[j]->horizontalIndex > minHorizontalCoord + horizontalCount / 2) {
					minHorizontalCoord = edgePointsInPlane[j]->horizontalIndex;
					minVerticalCoord = edgePointsInPlane[j]->verticalIndex;
					startPoint = edgePointsInPlane[j];
				}
				else if (edgePointsInPlane[j]->horizontalIndex == minHorizontalCoord && edgePointsInPlane[j]->verticalIndex < minVerticalCoord) {
					minVerticalCoord = edgePointsInPlane[j]->verticalIndex;
					startPoint = edgePointsInPlane[j];
				}
			}
			size_t direction = 0;
			if (spikeType(startPoint, -1, true) == -1) {
				size_t x = startPoint->horizontalIndex;
				size_t y = startPoint->verticalIndex;
				startPoint->plane = nullptr;
				startPoint->isMarked = false;
				for (size_t j = 0; j < planes[i]->points.size(); j++) {
					if (planes[i]->points[j] == startPoint) {
						planes[i]->points.erase(planes[i]->points.begin() + j);
						break;
					}
				}
				startPoint = points[getOffset(x + 1, y)];
				direction = 3;
			}
			else if(startPoint->verticalIndex > 0 && points[getOffset(startPoint->horizontalIndex, startPoint->verticalIndex - 1)] &&
				points[getOffset(startPoint->horizontalIndex, startPoint->verticalIndex - 1)]->isMarked2)
				direction = 1;
			std::vector<std::pair<Point*, int>> currentEdge;
			findNextPoint(startPoint, direction, false, false, currentEdge, i, planes);
			for (size_t j = 0; j < edgePointsInPlane.size(); j++) {
				if (edgePointsInPlane[j]->isMarked) tempEdgePointsInPlane.push_back(edgePointsInPlane[j]);
			}
			edgePointsInPlane = tempEdgePointsInPlane;
			if (currentEdge.size() > 0) {
				if (direction == 1) {
					currentEdge.insert(currentEdge.begin(), currentEdge[currentEdge.size() - 1]);
					currentEdge.pop_back();
				}
				for (size_t k = 0; k < currentEdge.size(); k++) {
					currentEdge[k].first->outlineId = currentOutlineId;
				}
				currentOutlineId++;
				planes[i]->edges.push_back({ startPoint, currentEdge });
			}
		}
		for (size_t j = 0; j < planes[i]->points.size(); j++) planes[i]->points[j]->isMarked = false;
		for (size_t j = 0; j < planes[i]->points.size(); j++) planes[i]->points[j]->isMarked2 = false;
	}
}

const double newPointAcceptTreshold = 0.95;
const double inf = 1000000;

bool isStraightPoint(size_t pointIndex, std::pair<Point*, std::vector<std::pair<Point*, int>>>& edge, size_t& previousNeighbourCount)
{
	Point* point = edge.second[pointIndex].first;
	if (point->isCorner) return false; // first point can modify last point
	size_t previousNeighbourCountStore = previousNeighbourCount;
	size_t neighbourCount = 0;
	size_t neighbourEdgeCount = 0;
	size_t x = point->horizontalIndex;
	size_t y = point->verticalIndex;
	Plane* plane = point->plane;
	bool isNeighbour[4] = { false, false, false, false };
	Point* neighbourPoints[4] = { points[getOffset(x, y - 1)], points[getOffset(x, y + 1)], points[getOffset(x - 1, y)],
		points[getOffset(x + 1, y)] };
	for (size_t i = 0; i < 4; i++) {
		if ((y > 0 || i > 0) && (y < verticalCount - 1 || i != 1) && neighbourPoints[i] && neighbourPoints[i]->plane &&
			neighbourPoints[i]->plane == plane) {
			neighbourCount++;
			isNeighbour[i] = true;
			if (neighbourPoints[i]->outlineId > 0)
				neighbourEdgeCount++;
		}
	}
	previousNeighbourCount = neighbourCount;
	if (areNeighbours(pointIndex < (edge.second.size() - 1) ? edge.second[pointIndex + 1].first : edge.first, point) == 0 && neighbourCount < 3) {
		(pointIndex > 0 ? edge.second[pointIndex - 1].first : edge.first)->isCorner = false;
		(pointIndex > 0 ? edge.second[pointIndex - 2].first : edge.second[edge.second.size() - 1].first)->isCorner = true;
		(pointIndex < (edge.second.size() - 1) ? edge.second[pointIndex + 1].first : edge.first)->isCorner = true;
		return false;
	}
	if ((neighbourCount == 3 && (neighbourEdgeCount == 2 && (pointIndex == edge.second.size() - 1 || areNeighbours(point, edge.second[pointIndex + 1].first)
		> 0)))) {
		return true;
	}
	if (neighbourCount == 4 && (pointIndex == 0 ? edge.second[edge.second.size() - 1].first : edge.second[pointIndex - 1].first)->isCorner &&
		previousNeighbourCountStore == 2)
		return true;
	if (previousNeighbourCountStore == 4) {
		edge.second[pointIndex - 1].first->isCorner = false;
	}
	return false;
}

void findCorners()
{
	for (size_t k = 0; k < planes.size(); k++) {
		for (size_t i = 0; i < planes[k]->edges.size(); i++) {
			size_t previousNeighbourCount = 0;
			for (size_t j = 0; j < planes[k]->edges[i].second.size(); j++) {
				if (!isStraightPoint(j, planes[k]->edges[i], previousNeighbourCount)) {
					planes[k]->edges[i].second[j].first->isCorner = true;
				}
			}
		}
	}
}

Point* createNewPoint(Vec3<double> newPointPos, Point* point, std::vector<Point*> neighbours, size_t addedCount, bool createBeforePoint = false)
{
	Point* newPoint = new Point(newPointPos, neighbours[0]->horizontalIndex, verticalCount, point->plane);
	addedPoints.push_back(newPoint);
	newPoint->isCorner = true;
	newPoint->outlineId = point->outlineId;
	for (size_t j = 0; j < point->plane->edges.size(); j++) {
		if (point->plane->edges[j].second[0].first->outlineId == point->outlineId) {
			for (size_t k = 0; k < point->plane->edges[j].second.size(); k++) {
				if (point->plane->edges[j].second[k].first == point) {
					point->plane->edges[j].second.insert(point->plane->edges[j].second.begin() + k + (createBeforePoint ? 0 : 1) + addedCount, 
						{ newPoint, -1 });
					break;
				}
			}
			break;
		}
	}
	for (size_t i = 0; i < neighbours.size(); i++) {
		newPoint->neighbourPlaneNeighbours[i] = neighbours[i];
	}
	return newPoint;
}

const double twoPointDifferenceTreshold = 0.5;

Point* addNewPoint(Point* point, Point*& neighbour, Plane* plane, size_t addedCount, size_t neighbourIndex)
{
	if (point->createdNeighbourPoints[neighbourIndex] != nullptr) { //created by other plane
		auto createdNeighbour = point->createdNeighbourPoints[neighbourIndex];
		for (size_t j = 0; j < point->plane->edges.size(); j++) {
			if (point->plane->edges[j].second[0].first->outlineId == point->outlineId) {
				for (size_t k = 0; k < point->plane->edges[j].second.size(); k++) {
					if (point->plane->edges[j].second[k].first == point) {
						for (size_t l = 0; l < 4; l++) 
						{
							if (point->plane->edges[j].second[k + l + 1].first == createdNeighbour) {
								point->plane->edges[j].second.insert(point->plane->edges[j].second.begin() + k + 1 + addedCount, { createdNeighbour, -1 });
								point->plane->edges[j].second.erase(point->plane->edges[j].second.begin() + k + l + 2);
								break;
							}
						}
						break;
					}
				}
				break;
			}
		}
		return createdNeighbour;
	}
	Vec3<double> dir = { 0,0,0 };
	switch (areNeighbours(point, neighbour)) {
	case 1:
	dir = point->plane->pointDirections.first;
	break;
	case 2:
	dir = point->plane->pointDirections.second;
	break;
	case 3:
	dir = point->plane->pointDirections.first * -1;
	break;
	case 4:
	dir = point->plane->pointDirections.second * -1;
	break;
	default:
	break;
	}
	Vec3<double> newPointPos = point->position - dir * Vec3<double>::dot_product(point->position
		- plane->planePointPos, plane->normal) / Vec3<double>::dot_product(dir, plane->normal);
	if (abs(newPointPos.x) > inf || abs(newPointPos.y) > inf || abs(newPointPos.z) > inf || isnan(newPointPos.x) || isnan(newPointPos.y) || isnan(newPointPos.z))
		return nullptr;
	Vec3<double> dirToNew = newPointPos - point->position;
	if (Vec3<double>::dot_product(Vec3<double>::normalize(dir), Vec3<double>::normalize(dirToNew)) < newPointAcceptTreshold && 
		(point->position - newPointPos).length() > planeDistanceTreshold)
		return nullptr;
	Vec3<double> neighbourDir = { 0,0,0 };
	switch (areNeighbours(neighbour, point)) {
	case 1:
	neighbourDir = plane->pointDirections.first;
	break;
	case 2:
	neighbourDir = plane->pointDirections.second;
	break;
	case 3:
	neighbourDir = plane->pointDirections.first * -1;
	break;
	case 4:
	neighbourDir = plane->pointDirections.second * -1;
	break;
	default:
	break;
	}	
	Vec3<double> neighbourNewPointPos = neighbour->position - neighbourDir * Vec3<double>::dot_product(neighbour->position
		- point->plane->planePointPos, point->plane->normal) / Vec3<double>::dot_product(neighbourDir, point->plane->normal);
	if (abs(neighbourNewPointPos.x) > inf || abs(neighbourNewPointPos.y) > inf || abs(neighbourNewPointPos.z) > inf || isnan(neighbourNewPointPos.x) ||
		isnan(neighbourNewPointPos.y) || isnan(neighbourNewPointPos.z))
		return nullptr;
	Vec3<double> dirToNewNeighbour = neighbourNewPointPos - neighbour->position;
	if (Vec3<double>::dot_product(Vec3<double>::normalize(neighbourDir), Vec3<double>::normalize(dirToNewNeighbour)) < newPointAcceptTreshold &&
		(neighbour->position - neighbourNewPointPos).length() > planeDistanceTreshold)
		return nullptr;
	if ((newPointPos - neighbourNewPointPos).length() > twoPointDifferenceTreshold)
		return nullptr;
	auto newPos = (newPointPos + neighbourNewPointPos) / 2;
	//bool isNeighbourEdge = false;
	Point* newPoint;
	Point* newNeighbourPoint;
	newPoint = createNewPoint(newPos, point, { neighbour }, addedCount);
	point->createdNeighbourPoints[neighbourIndex] = newPoint;
	for (size_t i = 0; i < neighbour->neighbourPlaneNeighbours.size(); i++) 
	{
		if (neighbour->neighbourPlaneNeighbours[i] == point) 		
		{
			//isNeighbourEdge = true;
			newNeighbourPoint = createNewPoint(newPos, neighbour, { point }, 0);
			neighbour->createdNeighbourPoints[i] = newNeighbourPoint;
			break;
		}
	}
	/*if (!isNeighbourEdge) {
		newNeighbourPoint = createNewPoint(newPos, neighbour, { point }, 0);
		neighbour->neighbourPlaneNeighbours.push_back(newNeighbourPoint);
	}*/
	newPoint->neighbourPlaneNeighbours[1] = newNeighbourPoint;
	newNeighbourPoint->neighbourPlaneNeighbours[1] = newPoint;
	return newPoint;
}

void findPlaneConnections()
{
	for (size_t i = 0; i < planes.size(); i++) 
	{
		for (size_t j = 0; j < planes[i]->edges.size(); j++) 
		{
			for (size_t k = 0; k < planes[i]->edges[j].second.size(); k++)
			{
				Point* point = planes[i]->edges[j].second[k].first;
				int direction = planes[i]->edges[j].second[k].second;
				size_t x = point->horizontalIndex;
				size_t y = point->verticalIndex;
				Point* neighbourPoints[4] = { points[getOffset(x + 1, y)], points[getOffset(x, y + 1)], points[getOffset(x - 1, y)],
					points[getOffset(x, y - 1)] };
				for (size_t i = 0; i < 4; i++) {
					if ((y > 0 || direction != 3) && (y < verticalCount - 1 || direction != 1) && neighbourPoints[direction] && 
						neighbourPoints[direction]->outlineId > 0 && neighbourPoints[direction]->outlineId != point->outlineId && 
						neighbourPoints[direction]->plane != point->plane) {
						point->neighbourPlaneNeighbours[i] = neighbourPoints[direction];
					}
					else if((y == 0 && direction == 3) || (y == verticalCount - 1 && direction == 1) || !neighbourPoints[direction] ||
						neighbourPoints[direction]->plane == nullptr)
						point->neighbourPlaneNeighbours[i] = nullptr;
					direction += direction == 3 ? -3 : 1;
				}
			}
		}
	}
}

void createPlaneCorner(Point* point, std::vector<Point*> point1, std::vector<Point*> point2)
{
	Plane* p1 = point->plane;
	Plane* p2 = point1[2]->plane;
	Plane* p3 = point2[2]->plane;
	auto cross1 = Vec3<double>::crossProduct(p2->normal, p3->normal);
	auto cross2 = Vec3<double>::crossProduct(p3->normal, p1->normal);
	auto cross3 = Vec3<double>::crossProduct(p1->normal, p2->normal);

	auto denom = Vec3<double>::dot_product(p1->normal, cross1);

	auto planeDist1 = Vec3<double>::dot_product(p1->planePointPos, p1->normal);
	auto planeDist2 = Vec3<double>::dot_product(p2->planePointPos, p2->normal);
	auto planeDist3 = Vec3<double>::dot_product(p3->planePointPos, p3->normal);

	cross1 = cross1 * planeDist1;
	cross2 = cross2 * planeDist2;
	cross3 = cross3 * planeDist3;

	auto cornerPoint = (cross1 + cross2 + cross3) / denom;

	createNewPoint(cornerPoint, point1[0], {point1[2], point2[2]}, 0);
	createNewPoint(cornerPoint, point1[1], { point }, 0, true);
	createNewPoint(cornerPoint, point2[1], { point }, 0);
}

void connectPlanes()
{
	std::vector<std::vector<bool>> wasFirstGeneratedVec;
	std::vector<Point*> createdPoints;
	for (size_t i = 0; i < planes.size(); i++) {
		wasFirstGeneratedVec.push_back({});
		for (size_t j = 0; j < planes[i]->edges.size(); j++) {
			wasFirstGeneratedVec[i].push_back({false});
			for (size_t k = 0; k < planes[i]->edges[j].second.size(); k++) {
				auto point = planes[i]->edges[j].second[k].first;
				if (point->verticalIndex == verticalCount)
				{
					continue;
				}
				size_t addedCount = 0;
				if(point->isCorner)
				{
					for (size_t l = 0; l < point->neighbourPlaneNeighbours.size(); l++) {
						Point* newPoint = nullptr;
						if (point->neighbourPlaneNeighbours[l])
						{
							newPoint = addNewPoint(point, point->neighbourPlaneNeighbours[l], point->neighbourPlaneNeighbours[l]->plane, addedCount, l);
							if (newPoint) {
								if (k == 0 && ((planes[i]->edges[j].second[k].first->horizontalIndex -
									point->neighbourPlaneNeighbours[l]->horizontalIndex + horizontalCount) %
									horizontalCount == 1)) wasFirstGeneratedVec[i][wasFirstGeneratedVec[i].size() - 1] = true;
								planes[i]->edges[j].second[k].first->isCorner = false;
								createdPoints.push_back(newPoint);
								addedCount++;
							}
							else {
								planes[i]->edges[j].second.insert(planes[i]->edges[j].second.begin() + k + 1 + addedCount, { nullptr, -1 });
								addedCount++;
							}
						}						
					}
					k += addedCount;
				}
			}			
		}
	}

	for (size_t i = 0; i < planes.size(); i++) {
		for (size_t j = 0; j < planes[i]->edges.size(); j++) {
			for (size_t k = 0; k < planes[i]->edges[j].second.size(); k++) {
				auto point = planes[i]->edges[j].second[k].first;
				if (point) {
					for (size_t l = 0; l < planes[i]->edges[j].second[k].first->createdNeighbourPoints.size(); l++) {
						auto createdPoint1 = planes[i]->edges[j].second[k].first->createdNeighbourPoints[l];
						auto createdPoint2 = planes[i]->edges[j].second[k].first->createdNeighbourPoints[l == 3 ? 0 : (l + 1)];
						if (createdPoint1 && createdPoint2 && createdPoint1->neighbourPlaneNeighbours[1]->plane !=
							createdPoint2->neighbourPlaneNeighbours[1]->plane) {
							createPlaneCorner(planes[i]->edges[j].second[k].first, { createdPoint1, createdPoint1->neighbourPlaneNeighbours[1],
								point->neighbourPlaneNeighbours[l] }, { createdPoint2, createdPoint2->neighbourPlaneNeighbours[1],
								point->neighbourPlaneNeighbours[l == 3 ? 0 : (l + 1)] });
						}
					}
				}
			}
		}
	}

	std::vector<std::vector<std::vector<std::pair<size_t, size_t>>>> cornersToDelete;
	for (size_t i = 0; i < planes.size(); i++) {
		cornersToDelete.push_back({});
		for (size_t j = 0; j < planes[i]->edges.size(); j++) {
			cornersToDelete[i].push_back({});
			for (size_t k = 0; k < planes[i]->edges[j].second.size(); k++) {
				if (planes[i]->edges[j].second[k].first) {
					if (planes[i]->edges[j].second[k].first->verticalIndex == verticalCount) {
						for (size_t l = 0; l < planes[i]->edges[j].second[k].first->neighbourPlaneNeighbours.size(); l++) {
							if (planes[i]->edges[j].second[k].first->neighbourPlaneNeighbours[l] &&
								planes[i]->edges[j].second[k].first->neighbourPlaneNeighbours[l]->verticalIndex < verticalCount)
								cornersToDelete[i][j].push_back({ k,  planes[i]->edges[j].second[k].first->neighbourPlaneNeighbours[l]->plane->id });
						}
					}						
				}
				else 
				{
					cornersToDelete[i][j].push_back({ 0, 0 });
					planes[i]->edges[j].second.erase(planes[i]->edges[j].second.begin() + k);
					k--;
				}
			}
			int newPointStartIndex = wasFirstGeneratedVec[i][j] ? 1 : 0;
			int newPointEndIndex = newPointStartIndex;
			while (newPointEndIndex < (int)cornersToDelete[i][j].size() - 1) {
				if (cornersToDelete[i][j][newPointStartIndex].second > 0) {
					while (newPointEndIndex + 1 < cornersToDelete[i][j].size()
						&& cornersToDelete[i][j][newPointStartIndex].second == cornersToDelete[i][j][newPointEndIndex + 1].second) {
						newPointEndIndex++;
					}
					for (size_t k = cornersToDelete[i][j][newPointStartIndex].first + 1; k < cornersToDelete[i][j][newPointEndIndex].first; k++) {
						planes[i]->edges[j].second[k].first->isCorner = false;
					}
					if(newPointEndIndex == cornersToDelete[i][j].size() - 1 && newPointStartIndex != newPointEndIndex && wasFirstGeneratedVec[i][j] 
						&& cornersToDelete[i][j][0].second == cornersToDelete[i][j][newPointStartIndex].second)
						planes[i]->edges[j].second[cornersToDelete[i][j][cornersToDelete[i][j].size() - 1].first].first->isCorner = false;
				}
				newPointStartIndex = newPointEndIndex + 1;
				newPointEndIndex = newPointStartIndex;
			}
		}
	}
	for (size_t i = 0; i < createdPoints.size(); i++) {
		if (createdPoints[i]->isCorner)
			createdPoints[i]->neighbourPlaneNeighbours[1]->isCorner = true;
	}
}

void egoCarSegmentation()
{
	for (size_t i = 0; i < points.size(); i++) {
		if (points[i] && points[i]->position.x <= 1 && points[i]->position.x >= -1 &&
			points[i]->position.y <= 1 && points[i]->position.y >= -1 &&
			points[i]->position.z <= 2.5 && points[i]->position.z >= -2.5)
			points[i] = nullptr;
	}
}

void exportObjects()
{
	for (size_t i = 0; i < planes.size(); i++) {
		for (size_t j = 0; j < planes[i]->edges.size(); j++) {
			std::ofstream MyFile("C:/Users/ungbo/Desktop/BME/_Diplomamunka/Diplomamunka/Diplomamunka/Assets/Resources/Generated_Models/processed_obj_" 
				+ std::to_string(currentCornerId - 1) + ".obj");
			MyFile << "o Mesh" << std::endl;
			std::vector<Point*> corners;
			currentCornerIndex = 0;
			for (size_t k = 0; k < planes[i]->edges[j].second.size(); k++) {
				if (planes[i]->edges[j].second[k].first->isCorner) {
					planes[i]->edges[j].second[k].first->cornerId = currentCornerId;
					planes[i]->edges[j].second[k].first->cornerIndex = currentCornerIndex;
					corners.push_back(planes[i]->edges[j].second[k].first);
					currentCornerIndex++;
				}
			}
			for (size_t k = 0; k < corners.size(); k++) {
				MyFile << "v " << -corners[k]->position.x << " " << corners[k]->position.y << " " << corners[k]->position.z << std::endl;
			}
			MyFile << "f ";
			for (size_t j = 1; j < corners.size() + 1; j++) {
				MyFile << j << " ";
			}
			MyFile << std::endl;
			MyFile << "f ";
			for (size_t j = corners.size(); j > 0; j--) {
				MyFile << j << " ";
			}
			MyFile << std::endl;
			MyFile.close();
			currentCornerId++;
		}
	}
}

void processData() {
	groundSegmentation();
	egoCarSegmentation();
	findPlanes();
	findEdgePoints();
	findCorners();
	findPlaneConnections();
	connectPlanes();
	exportObjects();
	writeData();
}

int main()
{
    readData();
	auto start = std::chrono::steady_clock::now();
	processData();
	auto end = std::chrono::steady_clock::now();
	std::cout << "Elapsed time in seconds: "
		<< std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
		<< " sec" << std::endl;
    return 0;
}