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
	std::vector<size_t> convexId;
	std::vector<size_t> convexIndex;
	bool isCorner;
	Point(Vec3<double> _position, size_t _horizontalIndex, size_t _verticalIndex, Plane* _plane) : position(_position), horizontalIndex(_horizontalIndex),
		verticalIndex(_verticalIndex), plane(_plane)
	{
		isCorner = false;
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

struct Edge
{
	bool isHole;
	bool wasFirstGenerated;
	Point* startPoint;
	std::vector<std::pair<Point*, int>> pointsWithDir;
	Edge() : isHole(false), wasFirstGenerated(false) {}
};

struct Plane {
	std::vector<Point*> points;
	std::vector<Edge*> edges; //wasFirstGenerated, startpoint, points, direction
	Vec3<double> planePointPos;
	Vec3<double> normal;
	std::pair<Vec3<double>, Vec3<double>> pointDirections;
	size_t id;
	std::vector<std::vector<Point*>> convexFaces;
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

const size_t pointCloudCount = 100;

size_t getOffset(int horizontalIndex, int verticalIndex)
{
	if (horizontalIndex < 0) horizontalIndex = horizontalCount + horizontalIndex;
	else if (horizontalIndex > horizontalCount - 1) horizontalIndex = horizontalIndex - horizontalCount;
	if (verticalIndex < 0) verticalIndex = verticalCount + verticalIndex;
	else if (verticalIndex > verticalCount - 1) verticalIndex = verticalIndex - verticalCount;
	return horizontalIndex * verticalCount + verticalIndex;
}

void readData(size_t pointCloudIndex)
{
	points.clear();
	addedPoints.clear();
	planes.clear();
	verticalCounts.clear();
	currentCornerId = 1;
	currentSeparatedObjectId = 1;
	currentPlaneId = 1;
	currentOutlineId = 1;
	currentCornerIndex = 0;
	verticalCounts.push_back(0);
    std::string myText;		  
    std::ifstream MyReadFile("C:/Users/ungbo/Desktop/BME/_Diplomamunka/Diplomamunka/Diplomamunka/Assets/Resources/points_raw_" + 
		(pointCloudTestIndex == -1 ? (pointCloudCount == 0 ? "test" : std::to_string(pointCloudIndex)) : std::to_string(pointCloudTestIndex)) + ".txt");
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
		if (points[i]) {
			MyFile << points[i]->position.to_string() << ';' << points[i]->horizontalIndex << ';' << points[i]->verticalIndex <<
				';' << (points[i]->plane ? points[i]->plane->id : 0) << ';' << points[i]->outlineId << ';' << points[i]->cornerId
				<< ';' << points[i]->cornerIndex;
			for (size_t j = 0; j < points[i]->convexId.size(); j++) {
				MyFile << ";" << points[i]->convexId[j] << ";" << points[i]->convexIndex[j];
			}
			MyFile << std::endl;
		}
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

void writeData(size_t pointCloudIndex)
{
	std::cout << pointCloudIndex << std::endl;
	std::ofstream MyFile("C:/Users/ungbo/Desktop/BME/_Diplomamunka/Diplomamunka/Diplomamunka/Assets/Resources/points_processed_" +
		((pointCloudCount == 0 || pointCloudTestIndex != -1) ? "test" : std::to_string(pointCloudIndex)) + ".txt");
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
		if (points[i] && points[i]->position.y <= groundLevel + 0.1) {
			points[i] = nullptr;
		}
	}
}

#include <random>

std::mt19937 gen(100);

void setPointsMarked(std::vector<Point*> points, bool isMarked, bool isMarked2)
{
	for (size_t i = 0; i < points.size(); i++) {
		points[i]->isMarked = isMarked;
		points[i]->isMarked2 = isMarked2;
	}
}

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
			(!onlyMarkedNeighbours || neighbourPoints[j]->isMarked)) {
			neighbourCount++;
			isNeighbour[j] = true;
		}
	}
	for (size_t j = 0; j < 4; j++) {
		if (diagNeighbourPoints[j] && (j > 1 || y > 0) && (j < 2 || y < verticalCount - 1) && diagNeighbourPoints[j]->plane == p->plane &&
			(!onlyMarkedNeighbours || diagNeighbourPoints[j]->isMarked)) {
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

bool checkIfBridge(Point* p, bool onlyMarked)
{
	size_t x = p->horizontalIndex;
	size_t y = p->verticalIndex;
	int neighbourCount = 0;

	Point* neighbourPoints[4] = { points[getOffset(x, y - 1)], points[getOffset(x, y + 1)], points[getOffset(x - 1, y)],
				points[getOffset(x + 1, y)] };
	for (size_t j = 0; j < 4; j++) {
		if (neighbourPoints[j] && neighbourPoints[j]->plane == p->plane && (j > 0 || y > 0) && (j < 3 || y < verticalCount - 1) &&
			spikeType(neighbourPoints[j], -1, false) > 1 && (!onlyMarked || neighbourPoints[j]->isMarked))
			neighbourCount++;
	}
	bool diagIsNeighbour[4] = { false, false, false, false };
	Point* diagNeighbourPoints[4] = { points[getOffset(x - 1, y - 1)], points[getOffset(x + 1, y - 1)], points[getOffset(x + 1, y + 1)],
		points[getOffset(x - 1, y + 1)] };
	for (size_t j = 0; j < 4; j++) {
		if (diagNeighbourPoints[j] && (j > 1 || y > 0) && (j < 2 || y < verticalCount - 1) && diagNeighbourPoints[j]->plane == p->plane &&
			(!onlyMarked || diagNeighbourPoints[j]->isMarked)) {
			diagIsNeighbour[j] = true;
		}
	}


	if (((!diagIsNeighbour[0] && !diagIsNeighbour[2]) || (!diagIsNeighbour[1] && !diagIsNeighbour[3])) && neighbourCount > 2)
		return true;
	return false;
}

bool isThereBridge(std::vector<Point*>& planePoints)
{
	std::vector<Point*> newPoints;
	bool theresBridge = false;
	for (auto p : planePoints)
		if (p->plane != nullptr && checkIfBridge(p, false)) {
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
							plane->normal = Vec3<double>::normalize(plane->normal * (plane->points.size() - 1) + neighbourPoints[j]->normal);
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
						if (plane->points.size() > 0)
							break;
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
}

bool hasNonSpykeNeighbour(size_t x, size_t y)
{
	Point* neighbourPoint = nullptr;
	for (size_t i = 0; i < 4; i++) {
		switch (i) {
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
		if (neighbourPoint && spikeType(neighbourPoint, -1, false) == 2)
			return true;
	}
	return false;
}

void findNextPoint(Point*& startPoint, size_t direction, /*out*/ 
	std::vector<std::pair<Point*, int>>& currentEdge, std::vector<Point*>& spikePoints, size_t dbgPlaneIndex, std::vector<Plane*> dbgPlanes)
{
	//isMarked -- turned off after the edge is complete
	//isMarked -- turned off when point added to edge
	
	Point* currentPoint = nullptr;
	std::pair<Point*, size_t> previousSavedPoint = {nullptr, 0};
	bool isFirstPoint = true;
	bool comeFromDeadEnd = false;
	bool isPreviousSpike = false;
	bool wasThereNonSpike = false;
	bool isHole = direction == 1;
	while (currentPoint != startPoint || comeFromDeadEnd)
	{
		
		if (!currentPoint)
			currentPoint = startPoint;
		if (currentPoint->horizontalIndex == 77 && currentPoint->verticalIndex == 95) {
			std::cout << "asd";
		}
		Point* neighbourPoint = nullptr;
		size_t x = currentPoint->horizontalIndex;
		size_t y = currentPoint->verticalIndex;
		isPreviousSpike = spikeType(currentPoint, (!wasThereNonSpike || currentPoint == startPoint) ? -1 : ((direction + 1) % 4), false) == 0;
		if (!isPreviousSpike || currentEdge.size() == 0) {
			if (currentPoint->isMarked2) {
				currentEdge.push_back({ currentPoint, direction });
			}
		}
		else spikePoints.push_back(currentPoint);
		if(true || !isHole) currentPoint->isMarked2 = false;		
		for (size_t i = 0; i < 4; i++) {
			/*if (checkIfBridge(currentPoint)) {
				i += 2;
				direction = (direction + 2) % 4;
			}*/
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
				currentEdge[0].second = (direction + 3) % 4;
				comeFromDeadEnd = false;
				break;
			}
			if (neighbourPoint && neighbourPoint->plane == startPoint->plane && neighbourPoint->plane != nullptr && neighbourPoint->isMarked2
				&& (isPreviousSpike || spikeType(neighbourPoint, -1, false) <= 1 || spikeType(currentPoint, direction, false) > 0))
			{
				if (isFirstPoint) {
					currentEdge[0].second = (direction + 1) % 4 ;
					isFirstPoint = false;
				}
				if (!wasThereNonSpike && spikeType(currentPoint, -1, false) > 1) {
					if (currentEdge.size() > 1) 
					{
						auto helper = currentEdge[0];
						currentEdge[0] = currentEdge[1];
						currentEdge[1] = helper;
						startPoint = currentPoint;
						if (!hasNonSpykeNeighbour(startPoint->horizontalIndex, startPoint->verticalIndex)) {
							for (size_t j = 0; j < currentEdge.size(); j++) {
								spikePoints.push_back(currentEdge[j].first);
								currentEdge[j].first->plane = nullptr;
							}
							currentEdge.clear();
							return;
						}
					}
					isPreviousSpike = false;
					wasThereNonSpike = true;
				}
				if (!isPreviousSpike)
					previousSavedPoint = { currentPoint, (direction + (4 - i)) % 4 };
				auto neighbourSpikeType = spikeType(neighbourPoint, -1, false);
				if ((!isHole && checkIfBridge(neighbourPoint, true)) || neighbourSpikeType == -1 || neighbourSpikeType == 1 || (wasThereNonSpike && 
					currentEdge.size() > 1 && isPreviousSpike &&  neighbourSpikeType == 2 && neighbourPoint->isMarked)) {
					auto savedPoint = neighbourSpikeType == 1 ? neighbourPoint : currentPoint;
					if(savedPoint->isMarked2)
						currentEdge.push_back({ savedPoint, (direction + 3) % 4 });
					if (neighbourSpikeType == -1 || (!isHole && checkIfBridge(neighbourPoint, true)))
					{
						neighbourPoint->isMarked = false;
						neighbourPoint->isMarked2 = false;
						neighbourPoint->plane = nullptr;
					}
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
					for (size_t j = 0; j < currentEdge.size(); j++) {
						currentEdge[j].first->plane = nullptr;
						spikePoints.push_back(currentEdge[j].first);
					}
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
		std::vector<Point*> outerConnectedEdgePoints;
		std::vector<Point*> holeConnectedEdgePoints;
		std::vector<Point*> edgePointsInPlane;
		for (size_t j = 0; j < planes[i]->points.size(); j++) planes[i]->points[j]->isMarked = true;
		for (size_t j = 0; j < planes[i]->points.size(); j++) planes[i]->points[j]->isMarked2 = true;
		for (size_t j = 0; j < planes[i]->points.size(); j++) if(isEdgePoint(planes[i]->points[j])) edgePointsInPlane.push_back(planes[i]->points[j]);
		while (edgePointsInPlane.size() > 0) 
		{
			Edge* currentEdge = new Edge();
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
			size_t direction = 2;
			if (spikeType(startPoint, -1, true) == -1) {
				size_t x = startPoint->horizontalIndex;
				size_t y = startPoint->verticalIndex;
				startPoint->plane = nullptr;
				startPoint->isMarked = false;
				startPoint->isMarked2 = false;
				for (size_t j = 0; j < planes[i]->points.size(); j++) {
					if (planes[i]->points[j] == startPoint) {
						planes[i]->points[j]->plane = nullptr;
						planes[i]->points.erase(planes[i]->points.begin() + j);
						break;
					}
				}
				minVerticalCoord = points[getOffset(x + 1, y)]->verticalIndex;
				for (size_t j = 1; j < edgePointsInPlane.size(); j++) {
					if (edgePointsInPlane[j]->horizontalIndex == x + 1 && edgePointsInPlane[j]->verticalIndex < minVerticalCoord) 			
					{
						minVerticalCoord = edgePointsInPlane[j]->verticalIndex;
						startPoint = edgePointsInPlane[j];
					}
				}
			}
			else if (startPoint->verticalIndex > 0 && points[getOffset(startPoint->horizontalIndex, startPoint->verticalIndex - 1)] &&
				points[getOffset(startPoint->horizontalIndex, startPoint->verticalIndex - 1)]->plane == startPoint->plane &&
				points[getOffset(startPoint->horizontalIndex - 1, startPoint->verticalIndex - 1)] &&
				points[getOffset(startPoint->horizontalIndex - 1, startPoint->verticalIndex - 1)]->plane == startPoint->plane &&
				points[getOffset(startPoint->horizontalIndex - 1, startPoint->verticalIndex)] &&
				points[getOffset(startPoint->horizontalIndex - 1, startPoint->verticalIndex)]->plane == startPoint->plane)
			{
				direction = 1;
				currentEdge->isHole = true;
			}			
			if (i == 28) {
				std::cout << "asd";
			}
			std::vector<Point*> spikePoints;
			setPointsMarked(currentEdge->isHole ? holeConnectedEdgePoints : outerConnectedEdgePoints, false, false);
			setPointsMarked(currentEdge->isHole ? outerConnectedEdgePoints : holeConnectedEdgePoints, true, false);
			findNextPoint(startPoint, direction, currentEdge->pointsWithDir, spikePoints, i, planes);
			setPointsMarked(outerConnectedEdgePoints, false, false);
			setPointsMarked(holeConnectedEdgePoints, false, false);
			for (size_t j = 0; j < currentEdge->pointsWithDir.size(); j++) {
				currentEdge->pointsWithDir[j].first->isMarked = false;
				(currentEdge->isHole ? holeConnectedEdgePoints : outerConnectedEdgePoints).push_back(currentEdge->pointsWithDir[j].first);
			}
			for (size_t j = 0; j < spikePoints.size(); j++) {
				spikePoints[j]->isMarked = false;
				(currentEdge->isHole ? holeConnectedEdgePoints : outerConnectedEdgePoints).push_back(spikePoints[j]);
			}
			for (size_t j = 0; j < edgePointsInPlane.size(); j++) {
				if (edgePointsInPlane[j]->isMarked) tempEdgePointsInPlane.push_back(edgePointsInPlane[j]);
			}
			edgePointsInPlane = tempEdgePointsInPlane;
			if (currentEdge->pointsWithDir.size() > 3) {
				if (direction == 1) {
					currentEdge->pointsWithDir.insert(currentEdge->pointsWithDir.begin(),
						currentEdge->pointsWithDir[currentEdge->pointsWithDir.size() - 1]);
					currentEdge->pointsWithDir.pop_back();
				}
				for (size_t k = 0; k < currentEdge->pointsWithDir.size(); k++) {
					currentEdge->pointsWithDir[k].first->outlineId = currentOutlineId;
				}
				currentOutlineId++;
				currentEdge->startPoint = startPoint;
				planes[i]->edges.push_back(currentEdge);
			}
		}
		for (size_t j = 0; j < planes[i]->points.size(); j++) planes[i]->points[j]->isMarked = false;
		for (size_t j = 0; j < planes[i]->points.size(); j++) planes[i]->points[j]->isMarked2 = false;
	}
}

const double newPointAcceptTreshold = 0.95;
const double inf = 1000000;

bool isStraightPoint(size_t pointIndex, Edge*& edge, size_t& previousNeighbourCount, 
	Vec3<double>& straigthDir)
{
	const double newDirTreshold = 0.1;
	Point* point = edge->pointsWithDir[pointIndex].first;
	if (point->horizontalIndex == 666 && point->verticalIndex == 29) {
		//std::cout << "asd";
	}
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
	if (pointIndex > 0 && areNeighbours(pointIndex < (edge->pointsWithDir.size() - 1) ? edge->pointsWithDir[pointIndex + 1].first : edge->startPoint,
		point) == 0 && neighbourCount < 3) { //deadend
		auto previousPoint = edge->pointsWithDir[pointIndex - 1];
		auto previousPreviousPoint = pointIndex > 1 ? edge->pointsWithDir[pointIndex - 2] : edge->pointsWithDir[edge->pointsWithDir.size() - 2 
			+ pointIndex];
		straigthDir = { 0,0,0 };
		if (areNeighbours(previousPoint.first, previousPreviousPoint.first) > 0 && ((previousPoint.first->verticalIndex ==
			previousPreviousPoint.first->verticalIndex && previousPoint.first->horizontalIndex == point->horizontalIndex &&
			previousPoint.first->verticalIndex != point->verticalIndex) || (previousPoint.first->horizontalIndex ==
				previousPreviousPoint.first->horizontalIndex && previousPoint.first->verticalIndex == point->verticalIndex &&
				previousPoint.first->horizontalIndex != point->horizontalIndex))) //if curve swap with previous
		{
			point->isCorner = true;
			edge->pointsWithDir[pointIndex - 1] = edge->pointsWithDir[pointIndex];
			edge->pointsWithDir[pointIndex] = previousPoint;
			return true;
		}
		return false;
	}
	if (neighbourCount == 3 && (neighbourEdgeCount == 2 && (pointIndex == edge->pointsWithDir.size() - 1 || 
		areNeighbours(point, edge->pointsWithDir[pointIndex + 1].first)
		> 0)))
	{
		if (straigthDir.length() == 0) {
			if (pointIndex > 0)
				straigthDir = Vec3<double>::normalize(point->position - edge->pointsWithDir[pointIndex - 1].first->position);
		}
		else
		{
			auto newDir = Vec3<double>::normalize(point->position - edge->pointsWithDir[pointIndex - 1].first->position);
			if ((straigthDir - newDir).length() > newDirTreshold) {
				straigthDir = newDir;
				return false;
			}
		}
		return true;
	}
	straigthDir = { 0,0,0 };
	if (neighbourCount == 4 && (pointIndex == 0 ? edge->pointsWithDir[edge->pointsWithDir.size() - 1].first : 
		edge->pointsWithDir[pointIndex - 1].first)->isCorner &&
		previousNeighbourCountStore == 2)
		return true;
	if (previousNeighbourCountStore == 4) {
		edge->pointsWithDir[pointIndex - 1].first->isCorner = false;
	}
	return false;
}

void findCorners()
{
	for (size_t k = 0; k < planes.size(); k++) {
		for (size_t i = 0; i < planes[k]->edges.size(); i++) {
			size_t previousNeighbourCount = 0;
			Vec3<double> straightDir = { 0,0,0 };
			for (size_t j = 0; j < planes[k]->edges[i]->pointsWithDir.size(); j++) {
				if (k == 86 && i == 0) {
				//	std::cout << "asd";
				}
				if (!isStraightPoint(j, planes[k]->edges[i], previousNeighbourCount, straightDir)) {
					planes[k]->edges[i]->pointsWithDir[j].first->isCorner = true;
				}
			}
		}
	}
}

Point* createNewPoint(Vec3<double> newPointPos, Point* point, std::vector<Point*> neighbours, size_t addedCount, bool createBeforePoint = false, 
	bool isCornerPoint = false)
{
	double deleteDurroundingCornersTreshold = 0.1;
	Point* newPoint = new Point(newPointPos, neighbours[0]->horizontalIndex, isCornerPoint ? verticalCount + 1 : verticalCount, point->plane);
	addedPoints.push_back(newPoint);
	newPoint->isCorner = true;
	newPoint->outlineId = point->outlineId;
	for (size_t j = 0; j < point->plane->edges.size(); j++) {
		if (point->plane->edges[j]->pointsWithDir[0].first->outlineId == point->outlineId) {
			for (size_t k = 0; k < point->plane->edges[j]->pointsWithDir.size(); k++) {
				if (point->plane->edges[j]->pointsWithDir[k].first == point) {
					auto index = k + (createBeforePoint ? 0 : 1) + addedCount;
					point->plane->edges[j]->pointsWithDir.insert(point->plane->edges[j]->pointsWithDir.begin() + index, { newPoint, -1 });
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
		createdNeighbour->isCorner = true;
		for (size_t j = 0; j < point->plane->edges.size(); j++) {
			if (point->plane->edges[j]->pointsWithDir[0].first->outlineId == point->outlineId) {
				for (size_t k = 0; k < point->plane->edges[j]->pointsWithDir.size(); k++) {
					if (point->plane->edges[j]->pointsWithDir[k].first == point) {
						for (size_t l = 0; l < 4; l++) 
						{
							if (point->plane->edges[j]->pointsWithDir[k + l + 1].first == createdNeighbour) {
								point->plane->edges[j]->pointsWithDir.insert(point->plane->edges[j]->pointsWithDir.begin() + k + 1 + addedCount,
									{ createdNeighbour, -1 });
								point->plane->edges[j]->pointsWithDir.erase(point->plane->edges[j]->pointsWithDir.begin() + k + l + 2);
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
	Point* newPoint;
	Point* newNeighbourPoint;
	newPoint = createNewPoint(newPos, point, { neighbour }, addedCount);
	point->createdNeighbourPoints[neighbourIndex] = newPoint;
	for (size_t i = 0; i < neighbour->neighbourPlaneNeighbours.size(); i++) 
	{
		if (neighbour->neighbourPlaneNeighbours[i] == point) 		
		{
			newNeighbourPoint = createNewPoint(newPos, neighbour, { point }, 0, false);
			neighbour->createdNeighbourPoints[i] = newNeighbourPoint;
			break;
		}
	}
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
			for (size_t k = 0; k < planes[i]->edges[j]->pointsWithDir.size(); k++)
			{
				Point* point = planes[i]->edges[j]->pointsWithDir[k].first;
				if (point->horizontalIndex == 339 && point->verticalIndex == 14) 	
				{
					//std::cout << "asd";
				}
				int direction = planes[i]->edges[j]->pointsWithDir[k].second;
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

	createNewPoint(cornerPoint, point1[0], {point1[2], point2[2]}, 0, false, true);
	createNewPoint(cornerPoint, point1[1], { point }, 0, true, true);
	createNewPoint(cornerPoint, point2[1], { point }, 0, false, true);
}

void connectPlanes()
{
	std::vector<Point*> createdPoints;
	for (size_t i = 0; i < planes.size(); i++) {
		for (size_t j = 0; j < planes[i]->edges.size(); j++) {
			for (size_t k = 0; k < planes[i]->edges[j]->pointsWithDir.size(); k++) {
				auto point = planes[i]->edges[j]->pointsWithDir[k].first;
				if (point->horizontalIndex == 666 && point->verticalIndex == 29) {
					//std::cout << "asd";
				}
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
								if (k == 0 && l == 0) planes[i]->edges[planes[i]->edges.size() - 1]->wasFirstGenerated = true;
								planes[i]->edges[j]->pointsWithDir[k].first->isCorner = false;
								createdPoints.push_back(newPoint);
								addedCount++;
							}
							else {
								planes[i]->edges[j]->pointsWithDir.insert(planes[i]->edges[j]->pointsWithDir.begin() + k + 1 + addedCount, { nullptr, -1 });
								addedCount++;
							}
						}						
					}
					k += addedCount;
				}
			}			
		}
	}
}

void createCorners()
{
	const double normalDiffTreshold = 0.1;
	for (size_t i = 0; i < planes.size(); i++) {
		for (size_t j = 0; j < planes[i]->edges.size(); j++) {	
			for (size_t k = 0; k < planes[i]->edges[j]->pointsWithDir.size(); k++) {
				auto point = planes[i]->edges[j]->pointsWithDir[k].first;
				if (point) {
					for (size_t l = 0; l < planes[i]->edges[j]->pointsWithDir[k].first->createdNeighbourPoints.size(); l++) {
						auto createdPoint1 = planes[i]->edges[j]->pointsWithDir[k].first->createdNeighbourPoints[l];
						auto createdPoint2 = planes[i]->edges[j]->pointsWithDir[k].first->createdNeighbourPoints[l == 3 ? 0 : (l + 1)];
						if (createdPoint1 && createdPoint2 && createdPoint1->neighbourPlaneNeighbours[1]->plane !=
							createdPoint2->neighbourPlaneNeighbours[1]->plane
							&& (planes[i]->edges[j]->pointsWithDir[k].first->plane->normal -
							createdPoint1->neighbourPlaneNeighbours[1]->plane->normal).length() > normalDiffTreshold && 
							(planes[i]->edges[j]->pointsWithDir[k].first->plane->normal -
								createdPoint2->neighbourPlaneNeighbours[1]->plane->normal).length() > normalDiffTreshold && 
							(createdPoint1->neighbourPlaneNeighbours[1]->plane->normal -
									createdPoint2->neighbourPlaneNeighbours[1]->plane->normal).length() > normalDiffTreshold) {
							createPlaneCorner(planes[i]->edges[j]->pointsWithDir[k].first, { createdPoint1, createdPoint1->neighbourPlaneNeighbours[1],
								point->neighbourPlaneNeighbours[l] }, { createdPoint2, createdPoint2->neighbourPlaneNeighbours[1],
								point->neighbourPlaneNeighbours[l == 3 ? 0 : (l + 1)] });
							break;
						}
					}
				}
			}
		}
	}
}

void filterEdgePoints()
{

	std::vector<std::vector<std::vector< std::pair<bool, std::pair<size_t, size_t>>>>> cornersToDelete;
	for (size_t i = 0; i < planes.size(); i++) {
		cornersToDelete.push_back({});
		for (size_t j = 0; j < planes[i]->edges.size(); j++) {
			cornersToDelete[i].push_back({});
			for (size_t k = 0; k < planes[i]->edges[j]->pointsWithDir.size(); k++) {
				if (planes[i]->edges[j]->pointsWithDir[k].first) {
					if (planes[i]->edges[j]->pointsWithDir[k].first->verticalIndex >= verticalCount) {
						for (size_t l = 0; l < planes[i]->edges[j]->pointsWithDir[k].first->neighbourPlaneNeighbours.size(); l++) {
							if (planes[i]->edges[j]->pointsWithDir[k].first->neighbourPlaneNeighbours[l] &&
								planes[i]->edges[j]->pointsWithDir[k].first->neighbourPlaneNeighbours[l]->verticalIndex < verticalCount)
								cornersToDelete[i][j].push_back({ planes[i]->edges[j]->pointsWithDir[k].first->verticalIndex > verticalCount,
									{ k,  planes[i]->edges[j]->pointsWithDir[k].first->neighbourPlaneNeighbours[l]->plane->id } });
						}
					}
				}
				else {
					cornersToDelete[i][j].push_back({ false, { 0, 0 } });
					planes[i]->edges[j]->pointsWithDir.erase(planes[i]->edges[j]->pointsWithDir.begin() + k);
					k--;
				}
			}
			for (int k = 0; k < cornersToDelete[i][j].size(); k++) 
			{
				if (k < ((int)cornersToDelete[i][j].size() - 2) && cornersToDelete[i][j][k + 1].first && cornersToDelete[i][j][k + 2].first) 
				{
					cornersToDelete[i][j][k + 1].second = { cornersToDelete[i][j][k + 1].second.first, cornersToDelete[i][j][k].second.second };
					size_t index = k + 2;
					while (index < cornersToDelete[i][j].size() && cornersToDelete[i][j][index].first)
					{
						cornersToDelete[i][j].erase(cornersToDelete[i][j].begin() + index);
					}
					cornersToDelete[i][j].insert(cornersToDelete[i][j].begin() + index, { true, {cornersToDelete[i][j][k + 1].second.first,
						index < cornersToDelete[i][j].size() ? cornersToDelete[i][j][index].second.second : cornersToDelete[i][j][0].second.second} });
				}
			}
			int newPointStartIndex = planes[i]->edges[j]->wasFirstGenerated ? 1 : 0;
			if (planes[i]->edges[j]->pointsWithDir[newPointStartIndex + 1].first->horizontalIndex < planes[i]->edges[j]->startPoint->horizontalIndex &&
				planes[i]->edges[j]->pointsWithDir[newPointStartIndex + 1].first->createdNeighbourPoints[0])
				newPointStartIndex++;
			int newPointEndIndex = newPointStartIndex;
			if (i == 165) {
				std::cout << "asd";
			}
			while (newPointEndIndex < (int)cornersToDelete[i][j].size() - 1) {
				if (cornersToDelete[i][j][newPointStartIndex].second.second > 0) {
					while (newPointEndIndex + 1 < cornersToDelete[i][j].size()
						&& cornersToDelete[i][j][newPointStartIndex].second.second == cornersToDelete[i][j][newPointEndIndex + 1].second.second) {
						newPointEndIndex++;
					}
					for (size_t k = cornersToDelete[i][j][newPointStartIndex].second.first + 1; k <
						cornersToDelete[i][j][newPointEndIndex].second.first; k++) {
						planes[i]->edges[j]->pointsWithDir[k].first->isCorner = false;
					}
					auto startPoint = planes[i]->edges[j]->pointsWithDir[cornersToDelete[i][j][newPointStartIndex].second.first].first;
					auto endPoint = planes[i]->edges[j]->pointsWithDir[cornersToDelete[i][j][newPointEndIndex].second.first].first;
					if ((startPoint->position - endPoint->position).length() < 0.1) {
						if (startPoint->verticalIndex == verticalCount + 1 && endPoint->verticalIndex != verticalCount + 1)
							endPoint->isCorner = false;
						else if (endPoint->verticalIndex == verticalCount + 1 && startPoint->verticalIndex != verticalCount + 1)
							startPoint->isCorner = false;
					}
					if (newPointEndIndex == cornersToDelete[i][j].size() - 1 && newPointStartIndex != newPointEndIndex && 
						planes[i]->edges[j]->wasFirstGenerated && cornersToDelete[i][j][0].second.second ==
						cornersToDelete[i][j][newPointStartIndex].second.second)
						planes[i]->edges[j]->pointsWithDir[cornersToDelete[i][j][cornersToDelete[i][j].size() - 1].second.first].first->isCorner = false;
				}
				newPointStartIndex = newPointEndIndex + 1;
				newPointEndIndex = newPointStartIndex;
			}
		}
	}
	/*for (size_t i = 0; i < planes.size(); i++) {
		for (size_t j = 0; j < planes[i]->edges.size(); j++) {
			for (size_t k = 0; k < planes[i]->edges[j]->pointsWithDir.size(); k++) {
				if (planes[i]->edges[j]->pointsWithDir[k].first->verticalIndex == verticalCount && planes[i]->edges[j]->pointsWithDir[k].first->isCorner
					&& planes[i]->edges[j]->pointsWithDir[k].first->neighbourPlaneNeighbours[1] &&
					planes[i]->edges[j]->pointsWithDir[k].first->neighbourPlaneNeighbours[1]->verticalIndex == verticalCount) {
					planes[i]->edges[j]->pointsWithDir[k].first->neighbourPlaneNeighbours[1]->isCorner = true;
				}
			}
		}
	}*/
}

void egoCarSegmentation(size_t frameIndex)
{
	float egoCarPosZ = -(int)frameIndex * (50.0 / 36.0);
	for (size_t i = 0; i < points.size(); i++) {
		if (points[i] && points[i]->position.x <= 1 && points[i]->position.x >= -1 &&
			points[i]->position.y <= 1 && points[i]->position.y >= -1 &&
			points[i]->position.z <= 2.5 + egoCarPosZ && points[i]->position.z >= -2.5 + egoCarPosZ)
			points[i] = nullptr;
	}
}

const double PI = 3.14159265359;

double angleOfVectors(Vec3<double> v1, Vec3<double> v2, bool isBackward)
{
	auto v1Angle = atan2(v1.x, v1.y) / PI;
	auto v2Angle = atan2(v2.x, v2.y) / PI;
	auto angle = abs(v1Angle - v2Angle);
	if ((!isBackward && v1Angle < v2Angle) || (isBackward && v1Angle > v2Angle)) angle = 2 - angle;
	return angle * 180;
}

Vec3<double> intersectionOfLines(Vec3<double> p1, Vec3<double> p2, Vec3<double> q1, Vec3<double> q2, bool& isOnEdge)
{
	auto dir1 = p2 - p1;
	auto dir2 = q2 - q1;
	double R = (p1.y * dir1.x + q1.x * dir1.y - p1.x * dir1.y - q1.y * dir1.x) / (dir2.y * dir1.x - dir1.y * dir2.x);
	Vec3<double> intersectionPos = { q1.x + R * dir2.x, q1.y + R * dir2.y, 0 };
	auto side1Length = (p2 - p1).length();
	auto side2Length = (q2 - q1).length();
	if ((intersectionPos - p1).length() < side1Length && (intersectionPos - p2).length() < side1Length &&
		(intersectionPos - q1).length() < side2Length && (intersectionPos - q2).length() < side2Length)
		isOnEdge = true;
	return intersectionPos;
}

bool isPointInsidePolygon(std::vector<std::pair<Vec3<double>, Point*>>polygon, Vec3<double> point, std::pair<double, double> xBounds,
	std::pair<double, double> yBounds)
{
	bool notInUse = false;
	if (point.x > xBounds.first && point.x < xBounds.second &&
		point.y > yBounds.first && point.y < yBounds.second) 
	{
		int rigthCounter = 0;
		for (size_t i = 0; i < polygon.size(); i++) 
		{
			auto p1 = polygon[i].first;
			auto p2 = polygon[(i + 1) % polygon.size()].first;
			if ((p1.x < point.x && p2.x < point.x) || (p1.y > point.y && p2.y > point.y) || (p1.y < point.y && p2.y < point.y) || (p1.y == p2.y ))
				continue;
			else if (point == p1)
				return false;
			else
			{
				/*auto smallerAngle = (p1.x < p2.x ? p1 - point : (p2 - point));
				auto biggerAngle = (p1.x < p2.x ? p2 - point : (p1 - point));
				if((p1.x > point.x && p2.x > point.x) || angleOfVectors(smallerAngle, biggerAngle, smallerAngle.y > point.y || biggerAngle.y < point.y)
					< 180)*/
				auto intersection = intersectionOfLines(point, point + Vec3<double>({ 1,0,0 }), p1, p2, notInUse);
				if (intersection.x <= point.x)
					continue;
				else
				{
					rigthCounter++;
					if (p2.y == point.y)
						i++;
				}
			}
		}
		return rigthCounter % 2 == 1;
	}
	return false;
}

void changeBaseTo2D(std::vector<std::pair<Vec3<double>, Point*>>& points)
{
	auto normal = points[0].second->plane->normal;
	auto x = points[0].second->plane->pointDirections.first;
	x = Vec3<double>::normalize(x - normal * Vec3<double>::dot_product(x, normal));
	auto y = Vec3<double>::crossProduct(x, normal);
	for (size_t i = 0; i < points.size(); i++) {
		points[i].first = { Vec3<double>::dot_product(points[i].second->position, x), Vec3<double>::dot_product(points[i].second->position, y), 0 };
	}
	for (int i = 0; i < points.size(); i++) {
		for (int j = (i == (points.size() - 1) ? 1 : 0); j < i - 1; j++) {
			bool isOnEdge = false;
			auto intersection = intersectionOfLines(points[j].first, points[(j + 1) % points.size()].first, points[i].first,
				points[(i + 1) % points.size()].first, isOnEdge);
			if(isOnEdge)
			{
				points[i].second->isCorner = false;
				points.erase(points.begin() + i);
				i-= 2;
				break;
			}
		}
	}
}

bool isClockwise(std::vector<std::pair<Vec3<double>, Point*>>& points)
{
	double angleSum = 0;
	for (size_t i = 0; i < points.size(); i++) {
		auto  a = angleOfVectors(points[(i + points.size() - 1) % points.size()].first - points[i].first,
			points[(i + 1) % points.size()].first - points[i].first, false);
		angleSum += angleOfVectors(points[(i + points.size() - 1) % points.size()].first - points[i].first,
			points[(i + 1) % points.size()].first - points[i].first, false);
	}
	return angleSum < (double)points.size() * 360.0 / 2;
}

void convexSegmentation()
{
	const std::pair<double, double> acceptAngle = { 181, 359 };
	size_t currentConvexId = 1;
	for (size_t i = 0; i < planes.size(); i++) {
		std::vector<std::vector<std::pair<Vec3<double>, Point*>>> holeEdges;
		size_t outerCounter = 0;
		for (size_t j = 0; j < planes[i]->edges.size(); j++) {
			if (planes[i]->edges[j]->isHole) {
				outerCounter++;
				std::vector<std::pair<Vec3<double>, Point*>> holeEdge;
				size_t counter = 0;
				for (size_t k = 0; k < planes[i]->edges[j]->pointsWithDir.size(); k++) {
					if (planes[i]->edges[j]->pointsWithDir[k].first->isCorner) {
						holeEdge.push_back({ {0,0,0}, planes[i]->edges[j]->pointsWithDir[k].first });
						counter++;
					}
				}
				if (holeEdge.size() < 4) {
					for (size_t k = 0; k < holeEdge.size(); k++) {
						holeEdge[k].second->isCorner = false;
					}
				}
				else {
					changeBaseTo2D(holeEdge);
					holeEdges.push_back(holeEdge);
				}
			}
		}
		if (i == 51) 			
		{
			std::cout << "asd";
			//return;
		}
		for (size_t x = 0; x < planes[i]->edges.size(); x++) {
			if (!planes[i]->edges[x]->isHole) {
				//std::vector<Vec3<double>> remainingPoints;
				std::vector<std::pair<Vec3<double>, Point*>> remainingPoints;
				size_t counter = 0;
				Vec3<double> pivotPoint = { 10000, 10000, 10000 };
				for (size_t j = 0; j < planes[i]->edges[x]->pointsWithDir.size(); j++) {
					if (planes[i]->edges[x]->pointsWithDir[j].first->isCorner) {
						if (pivotPoint == Vec3<double>({ 10000, 10000, 10000 }))
						{
							pivotPoint = planes[i]->edges[x]->pointsWithDir[j].first->position;
						}
						remainingPoints.push_back({ {},	planes[i]->edges[x]->pointsWithDir[j].first });
						counter++;
					}
				}
				if (remainingPoints.size() < 4) {
					for (size_t k = 0; k < remainingPoints.size(); k++) {
						remainingPoints[k].second->isCorner = false;
					}
					continue;
				}				
				changeBaseTo2D(remainingPoints);
				if (!isClockwise(remainingPoints)) 
				{
					for (size_t k = 0; k < remainingPoints.size(); k++) 
					{
						remainingPoints.push_back(remainingPoints[remainingPoints.size() - 1 - k]);
						remainingPoints.erase(remainingPoints.begin() + remainingPoints.size() - 2 - k);
					}
				}
				while (remainingPoints.size() > 3) {
					if (i == 16) {
						int a = 1;
					}
					//std::vector<Vec3<double>> remainingPointsHelper(remainingPoints);
					//std::vector<Vec3<double>> L = { remainingPointsHelper[0], remainingPointsHelper[1] };

					std::vector<std::pair<Vec3<double>, Point*>> remainingPointsHelper(remainingPoints);
					std::vector<std::pair<Vec3<double>, Point*>> L = { remainingPointsHelper[0], remainingPointsHelper[1] };
						std::vector<std::pair<Vec3<double>, Point*>> remainingPointsHelperSave;
					std::vector<std::pair<Vec3<double>, Point*>> LSave;

					remainingPointsHelper.erase(remainingPointsHelper.begin(), remainingPointsHelper.begin() + 2);
					std::pair<double, double> xBounds = { std::min(L[0].first.x, L[1].first.x), std::max(L[0].first.x, L[1].first.x) };
					std::pair<double, double> yBounds = { std::min(L[0].first.y, L[1].first.y), std::max(L[0].first.y, L[1].first.y) };

					for (int j = 0; j < 2; j++) {
						bool isForward = j == 0;
						while (remainingPointsHelper.size() > 0) {
							auto newPoint = isForward ? remainingPointsHelper[0] : remainingPointsHelper[remainingPointsHelper.size() - 1];
							auto v1 = isForward ? L[L.size() - 1].first - L[L.size() - 2].first : (L[0].first - L[1].first);
							auto v2 = isForward ? newPoint.first - L[L.size() - 1].first : (newPoint.first - L[0].first);
							auto vecToBegin = isForward ? L[0].first - newPoint.first : (L[L.size() - 1].first - newPoint.first);
							auto vecAtBegin = isForward ? L[1].first - L[0].first : (L[L.size() - 2].first - L[L.size() - 1].first);
							auto temp = angleOfVectors(v1 * -1, v2, !isForward);
							auto temp2 = angleOfVectors(v2 * -1, vecToBegin, !isForward);
							auto temp3 = angleOfVectors(vecToBegin * -1, vecAtBegin, !isForward);
							if ((angleOfVectors(v1 * -1, v2, !isForward) <= acceptAngle.first || angleOfVectors(v1 * -1, v2, !isForward) >= 
								acceptAngle.second) &&
								(angleOfVectors(v2 * -1, vecToBegin, !isForward) <= acceptAngle.first || angleOfVectors(v2 * -1, vecToBegin, !isForward) >=
								acceptAngle.second) && 
								(angleOfVectors(vecToBegin * -1, vecAtBegin, !isForward) <= acceptAngle.first || angleOfVectors(vecToBegin * -1, vecAtBegin, !isForward) >=
								acceptAngle.second)) {
								L.insert(isForward ? L.end() : L.begin(), newPoint);
								remainingPointsHelper.erase(isForward ? remainingPointsHelper.begin() : remainingPointsHelper.end() - 1);
								if (newPoint.first.x < xBounds.first)
									xBounds.first = newPoint.first.x;
								if (newPoint.first.x > xBounds.second)
									xBounds.second = newPoint.first.x;
								if (newPoint.first.y < yBounds.first)
									yBounds.first = newPoint.first.y;
								if (newPoint.first.y > yBounds.second)
									yBounds.second = newPoint.first.y;
							}
							else {
								if (L.size() > 2) {
									bool containsCorner = true;
									while (containsCorner && L.size() > 2) {
										containsCorner = false;
										for (size_t k = 0; k < remainingPointsHelper.size(); k++) {
											if (isPointInsidePolygon(L, remainingPointsHelper[k].first, xBounds, yBounds)) {
												containsCorner = true;
												break;
											}
										}
										if (containsCorner) {
											remainingPointsHelper.insert(isForward ? remainingPointsHelper.begin() : remainingPointsHelper.end(),
												isForward ? L[L.size() - 1] : L[0]);
											L.erase(isForward ? L.end() - 1 : L.begin());
										}
									}
									int absoluteClosestEdgeIndex = -1;
									size_t absoluteClosestPointIndex = 0;
									bool isNewPointFound = true;
									auto lastPointPos = L[L.size() - 1].first;
									if (L.size() > 2) {
										while (isNewPointFound) {
											double minIntersectionDistance = 1000;
											int closestEdgeIndex = -1;
											size_t closestPointIndex = 0;
											auto LTemp = L;
											if (absoluteClosestEdgeIndex != -1)
												LTemp.insert(LTemp.begin(), holeEdges[absoluteClosestEdgeIndex][absoluteClosestPointIndex]);
											isNewPointFound = false;
											for (size_t l = 0; l < holeEdges.size(); l++) {
												std::vector<bool> isPointsInside;
												auto edgePoints = holeEdges[l];
												size_t holeType = 0;
												for (size_t m = 0; m < edgePoints.size(); m++) {
													bool isHolePointInside = isPointInsidePolygon(LTemp, edgePoints[m].first, xBounds, yBounds);
													isPointsInside.push_back(isHolePointInside);
													if (isHolePointInside && holeType == 0)
														holeType = 1;
													if (m > 0 && isPointsInside[m - 1] != isPointsInside[m])
														holeType = 2;
												}
												if (holeType > 0) {
													for (size_t m = 0; m < edgePoints.size(); m++) {
														if (holeType == 2 && isPointsInside[m] != isPointsInside[(m + 1) % isPointsInside.size()]) {
															bool isOnEdge = false;
															auto intersection = intersectionOfLines(LTemp[0].first, lastPointPos, edgePoints[m].first,
																edgePoints[(m + 1) % isPointsInside.size()].first, isOnEdge);
															if (isOnEdge && (intersection - lastPointPos).length() < minIntersectionDistance) {
																minIntersectionDistance = (intersection - lastPointPos).length();
																closestEdgeIndex = l;
																if (((edgePoints[m].first - lastPointPos).length() <
																	(edgePoints[(m + 1) % isPointsInside.size()].first - lastPointPos).length() &&
																	isPointInsidePolygon(LTemp, edgePoints[m].first, xBounds, yBounds)) ||
																	((edgePoints[m].first - lastPointPos).length() >=
																		(edgePoints[(m + 1) % isPointsInside.size()].first - lastPointPos).length() &&
																		!isPointInsidePolygon(LTemp, edgePoints[(m + 1) % isPointsInside.size()].first,
																			xBounds, yBounds)))
																	closestPointIndex = m;
																else
																	closestPointIndex = ((m + 1) % isPointsInside.size());
															}
														}
														else if (holeType == 1 && absoluteClosestEdgeIndex == -1) {
															if ((edgePoints[m].first - lastPointPos).length() < minIntersectionDistance) {
																minIntersectionDistance = (edgePoints[m].first - lastPointPos).length();
																closestEdgeIndex = l;
																closestPointIndex = m;
															}
														}
													}
												}
											}
											if (closestEdgeIndex != -1) {
												absoluteClosestEdgeIndex = closestEdgeIndex;
												absoluteClosestPointIndex = closestPointIndex;
												isNewPointFound = true;
											}
										}
										if (absoluteClosestEdgeIndex >= 0) {
											if (!isForward) {
												remainingPointsHelper = remainingPointsHelperSave;
												L = LSave;
											}
											remainingPointsHelper.insert(remainingPointsHelper.begin(), L[L.size() - 1]);
											for (size_t l = 0; l < holeEdges[absoluteClosestEdgeIndex].size() + 1; l++) {
												remainingPointsHelper.insert(remainingPointsHelper.begin() + l,
													holeEdges[absoluteClosestEdgeIndex][(absoluteClosestPointIndex + l) % holeEdges[absoluteClosestEdgeIndex].size()]);
											}
											holeEdges.erase(holeEdges.begin() + absoluteClosestEdgeIndex);
											j = -1;
											break;
										}
									}
								}
								if (isForward) {
									LSave = L;
									remainingPointsHelperSave = remainingPointsHelper;
								}
								else {
									if (L.size() > 2) {
										remainingPointsHelper.insert(remainingPointsHelper.begin(), L[L.size() - 1]);
										remainingPointsHelper.insert(remainingPointsHelper.end(), L[0]);
									}
									else {
										L.clear();
										remainingPoints.push_back(remainingPoints[0]);
										remainingPoints.erase(remainingPoints.begin());
										remainingPointsHelper = remainingPoints;
									}
								}
								break;
							}
						}
					}
					if (remainingPointsHelper.size() == 0) //last convex polygon hole test
					{
						double minDistance = 1000;
						int closestEdgeIndex = -1;
						size_t closestPointIndex = 0;
						for (size_t l = 0; l < holeEdges.size(); l++) {
							auto edgePoints = holeEdges[l];
							for (size_t m = 0; m < edgePoints.size(); m++) {
								if (isPointInsidePolygon(L, edgePoints[m].first, xBounds, yBounds) && minDistance >
									(L[0].first - edgePoints[m].first).length()) {
									minDistance = (L[0].first - edgePoints[m].first).length();
									closestEdgeIndex = l;
									closestPointIndex = m;
								}
							}
						}
						if (closestEdgeIndex >= 0) {
							remainingPointsHelper.insert(remainingPointsHelper.begin(), L[0]);
							for (size_t l = 0; l < holeEdges[closestEdgeIndex].size() + 1; l++) {
								remainingPointsHelper.insert(remainingPointsHelper.begin() + l + 1,
									holeEdges[closestEdgeIndex][(closestPointIndex + l) % holeEdges[closestEdgeIndex].size()]);
							}
							for (size_t l = 0; l < L.size(); l++) {
								remainingPointsHelper.insert(remainingPointsHelper.end(), L[l]);
							}
							L.clear();
							remainingPoints = remainingPointsHelper;
						}
					}
					if (L.size() > 0) {
						std::vector<Point*> convexFace;
						for (size_t j = 0; j < L.size(); j++) {
							L[j].second->convexId.push_back(currentConvexId);
							L[j].second->convexIndex.push_back(j);
							convexFace.push_back(L[j].second);
						}
						planes[i]->convexFaces.push_back(convexFace);
						if (i == 51 && planes[i]->convexFaces.size() == 9) {
							int a = 1;
							//return;
						}
						remainingPoints = remainingPointsHelper;
						currentConvexId++;
					}
				}
				if (remainingPoints.size() == 3) 
				{
					std::vector<Point*> convexFace;
					for (size_t j = 0; j < remainingPoints.size(); j++) {
						remainingPoints[j].second->convexId.push_back(currentConvexId);
						remainingPoints[j].second->convexIndex.push_back(j);
						convexFace.push_back(remainingPoints[j].second);
					}
					planes[i]->convexFaces.push_back(convexFace);
					currentConvexId++;
				}
			}
		}
	}
}

void exportObjects(size_t pointCloudIndex)
{
	std::string name = "C:\\Users\\ungbo\\Desktop\\BME\\_Diplomamunka\\Diplomamunka\\Diplomamunka\\Assets\\Resources\\Generated_Models_" + 
		((pointCloudCount == 0 || pointCloudTestIndex != -1) ? "test" : std::to_string(pointCloudIndex)) + "\\processed_obj_0.obj";
	std::ifstream f(name);
	size_t counter = 0;
	while (f.good())
	{
		f.close();
		remove(name.c_str());
		counter++;
		name = "C:\\Users\\ungbo\\Desktop\\BME\\_Diplomamunka\\Diplomamunka\\Diplomamunka\\Assets\\Resources\\Generated_Models_" +
			((pointCloudCount == 0 || pointCloudTestIndex != -1) ? "test" : std::to_string(pointCloudIndex)) + "\\processed_obj_"
			+ std::to_string(counter) + ".obj";
		f = std::ifstream(name.c_str());
	}
	size_t objCounter = 0;
	for (size_t i = 0; i < planes.size(); i++) {
		if (objCounter == 37) 			{
			std::cout << "asd";
		}
		std::vector<Point*> corners;
		currentCornerIndex = 0;
		
		for (size_t j = 0; j < planes[i]->edges.size(); j++) {
			for (size_t k = 0; k < planes[i]->edges[j]->pointsWithDir.size(); k++) {
				if (planes[i]->edges[j]->pointsWithDir[k].first->isCorner) {
					planes[i]->edges[j]->pointsWithDir[k].first->cornerId = currentCornerId;
					planes[i]->edges[j]->pointsWithDir[k].first->cornerIndex = currentCornerIndex;
					corners.push_back(planes[i]->edges[j]->pointsWithDir[k].first);
					currentCornerIndex++;
				}
			}
			currentCornerId++;
		}
		if (corners.size() == 0)
			continue;
		std::ofstream MyFile("C:/Users/ungbo/Desktop/BME/_Diplomamunka/Diplomamunka/Diplomamunka/Assets/Resources/Generated_Models_" +
			((pointCloudCount == 0 || pointCloudTestIndex != -1) ? "test" : std::to_string(pointCloudIndex)) + "/processed_obj_"
			+ std::to_string(objCounter) + ".obj");
		MyFile << "o Mesh" << std::endl;
		for (size_t k = 0; k < corners.size(); k++) {
			MyFile << "v " << -corners[k]->position.x << " " << corners[k]->position.y << " " << corners[k]->position.z << std::endl;
		}

		for (size_t j = 0; j < planes[i]->convexFaces.size(); j++) {
			MyFile << "f ";
			for (size_t k = 0; k < planes[i]->convexFaces[j].size(); k++) {
				MyFile << planes[i]->convexFaces[j][k]->cornerIndex + 1 << " ";
			}
			MyFile << std::endl;
			MyFile << "f ";
			for (int k = planes[i]->convexFaces[j].size() - 1; k >= 0; k--) {
				MyFile << planes[i]->convexFaces[j][k]->cornerIndex + 1 << " ";
			}
			MyFile << std::endl;
		}
		
		for (size_t j = 0; j < planes[i]->edges.size(); j++) {
			int indexShift = -1;
			for (size_t k = 0; k < planes[i]->edges[j]->pointsWithDir.size(); k++) {
				if (planes[i]->edges[j]->pointsWithDir[k].first->isCorner) {
					if (indexShift == -1)
						indexShift = planes[i]->edges[j]->pointsWithDir[k].first->cornerIndex;
					planes[i]->edges[j]->pointsWithDir[k].first->cornerIndex -= indexShift;
				}
			}
		}
		MyFile << std::endl;
		MyFile.close();
		objCounter++;
	}
}

void processData(size_t frameIndex) {
	groundSegmentation();
	egoCarSegmentation(frameIndex);
	findPlanes();
	findEdgePoints();
	findCorners();
	findPlaneConnections();
	connectPlanes();
	createCorners();
	filterEdgePoints();
	convexSegmentation();
}

int main()
{
	for (size_t i = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0); i < (pointCloudTestIndex == -1 ? std::max<size_t>(1, pointCloudCount) : 1); i++) {
		readData(i);
		auto start = std::chrono::steady_clock::now();
		processData(std::max<int>(pointCloudTestIndex, i));
		auto end = std::chrono::steady_clock::now();
		std::cout << "It. " + std::to_string(i) + " Elapsed time in seconds : "
			<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
			<< " sec" << std::endl;
		exportObjects(i);
		writeData(i);
	}
	return 0;
}