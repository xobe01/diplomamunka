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
	bool isCorner;
	Point(Vec3<double> _position, size_t _horizontalIndex, size_t _verticalIndex, Plane* _plane) : position(_position), horizontalIndex(_horizontalIndex),
		verticalIndex(_verticalIndex), plane(_plane)
	{
		isCorner = false;
	};
};

struct Plane {
	std::vector<Point*> points;
	std::pair<double, double> horizontalBounds;
	std::pair<double, double> verticalBounds;
	std::vector<std::vector<Point*>> edges;
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

void writeData()
{
	std::ofstream MyFile("C:/Users/ungbo/Desktop/BME/_Diplomamunka/Diplomamunka/Diplomamunka/Assets/Resources/points_processed.txt");
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

bool isSpike(Point* p)
{
	size_t x = p->horizontalIndex;
	size_t y = p->verticalIndex;

	//fel-le
	bool isUpNotNeightbour = y == 0 || !points[getOffset(x, y - 1)] || points[getOffset(x, y - 1)]->plane != p->plane;
	bool isDownNotNeightbour = y == verticalCount - 1 || !points[getOffset(x, y + 1)] || points[getOffset(x, y + 1)]->plane != p->plane;

	//jobbra-balra
	bool isLeftNotNeightbour = !points[getOffset(x - 1, y)] || points[getOffset(x - 1, y)]->plane != p->plane;
	bool isRighttNotNeightbour = !points[getOffset(x + 1, y)] || points[getOffset(x + 1, y)]->plane != p->plane;
	if (((isLeftNotNeightbour && isRighttNotNeightbour) || (isUpNotNeightbour && isDownNotNeightbour))) {
		return true;
	}
	return false;
}

bool checkIfBridge(Point* p)
{
	size_t x = p->horizontalIndex;
	size_t y = p->verticalIndex;
	int neighbourCount = 0;

	Point* neighbourPoints[4] = { points[getOffset(x, y - 1)], points[getOffset(x, y + 1)], points[getOffset(x - 1, y)],
				points[getOffset(x + 1, y)] };
	for (size_t j = 0; j < 4; j++) {
		if (neighbourPoints[j] && (j > 0 || y > 0) && (j < 3 || y < verticalCount - 1) && isSpike(neighbourPoints[j]))
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

void checkForGaps(/*out*/ std::vector<Point*>& chosenPoints)
{  
	for (auto p : chosenPoints) p->isMarked = true;
	while (isThereBridge(chosenPoints)) {}
	std::vector<Point*> nonProcessedPoints(chosenPoints);
	std::vector<Point*> nextStepPoints;
	std::vector<Point*> bestPartitionPoints;
	while (bestPartitionPoints.size() < nonProcessedPoints.size()) {
		std::vector<Point*> currentPartitionPoints;
		currentPartitionPoints.push_back(nonProcessedPoints[0]);
		nextStepPoints.push_back(nonProcessedPoints[0]);
		nonProcessedPoints[0]->isMarked = false;
		while (nextStepPoints.size() > 0) {
			std::vector<Point*> tempNextStepPoints;
			for (size_t i = 0; i < nextStepPoints.size(); i++) {
				size_t x = nextStepPoints[i]->horizontalIndex;
				size_t y = nextStepPoints[i]->verticalIndex;
				Point* neighbourPoint = points[getOffset(x, y - 1)];
				if (y > 0 && neighbourPoint && neighbourPoint->isMarked) {
					neighbourPoint->isMarked = false;
					currentPartitionPoints.push_back(neighbourPoint);
					tempNextStepPoints.push_back(neighbourPoint);
				}
				neighbourPoint = points[getOffset(x, y + 1)];
				if (y < verticalCount - 1 && neighbourPoint && neighbourPoint->isMarked) {
					neighbourPoint->isMarked = false;
					currentPartitionPoints.push_back(neighbourPoint);
					tempNextStepPoints.push_back(neighbourPoint);
				}
				neighbourPoint = points[getOffset(x - 1, y)];
				if (neighbourPoint && neighbourPoint->isMarked) {
					neighbourPoint->isMarked = false;
					currentPartitionPoints.push_back(neighbourPoint);
					tempNextStepPoints.push_back(neighbourPoint);
				}
				neighbourPoint = points[getOffset(x + 1, y)];
				if (neighbourPoint && neighbourPoint->isMarked) {
					neighbourPoint->isMarked = false;
					currentPartitionPoints.push_back(neighbourPoint);
					tempNextStepPoints.push_back(neighbourPoint);
				}
			}
			nextStepPoints = tempNextStepPoints;
		}
		if (currentPartitionPoints.size() > bestPartitionPoints.size()) bestPartitionPoints = currentPartitionPoints;
		std::vector<Point*> tempNonProcessedPoints;
		for (size_t i = 0; i < nonProcessedPoints.size(); i++) {
			if (nonProcessedPoints[i]->isMarked) tempNonProcessedPoints.push_back(nonProcessedPoints[i]);
		}
		nonProcessedPoints = tempNonProcessedPoints;
	}
	for (size_t i = 0; i < chosenPoints.size(); i++) {
		chosenPoints[i]->isMarked = false;
	}
	chosenPoints = bestPartitionPoints;
}

#include <chrono>

Vec3<double> getNormal(Point* center, Point* p1, Point* p2)
{
	return Vec3<double>::crossProduct(center->position - p1->position, center->position - p2->position);
}

void calculateNormal(Point* point)
{
	size_t x = point->horizontalIndex;
	size_t y = point->verticalIndex;
	Point* neighbourPoint1 = points[getOffset(x, y - 1)];
	Point* neighbourPoint2 = points[getOffset(x - 1, y)];
	Point* neighbourPoint3 = points[getOffset(x, y + 1)];
	Point* neighbourPoint4 = points[getOffset(x + 1, y)];
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
	if (points[getOffset(p1->horizontalIndex - 1, p1->verticalIndex)] == p2)
		return 2;
	if (points[getOffset(p1->horizontalIndex, p1->verticalIndex + 1)] == p2)
		return 3;
	if (points[getOffset(p1->horizontalIndex, p1->verticalIndex - 1)] == p2)
		return 4;
	return 0;
}

void choosePoints(const Vec3<Point*> planePoints, double acceptTreshold, Vec3<double> normal, /*out*/ Plane* plane)
{
	if (normal == Vec3<double>{0, 0, 0}) {
		normal = Vec3<double>::normalize(Vec3<double>::crossProduct(planePoints.x->position -
			planePoints.y->position, planePoints.z->position - planePoints.y->position));
	}
	Vec3<double> horizontalDirection = { 0,0,0 };
	Vec3<double> verticalDirection = { 0,0,0 };
	Point* neighbours[2] = { planePoints.y, planePoints.z };
	for each (auto neighbour in neighbours) {
		switch (areNeighbours(planePoints.x, neighbour)) {
		case 1:
		horizontalDirection = neighbour->position - planePoints.x->position;
		break;
		case 2:
		horizontalDirection = planePoints.x->position - neighbour->position;
		break;
		case 3:
		verticalDirection = neighbour->position - planePoints.x->position;
		break;
		case 4:
		verticalDirection = planePoints.x->position - neighbour->position;
		break;
		default:
		break;
		}
	}
	plane->pointDirections = {horizontalDirection, verticalDirection };
	plane->normal = normal;
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
					double dist = abs(Vec3<double>::dot_product(normal, neighbourPoints[j]->position - planePoints.x ->position));
					if (dist <= acceptTreshold) {
						plane->points.push_back(neighbourPoints[j]);
						neighbourPoints[j]->isMarked = false;
						neighbourPoints[j]->isMarked2 = false;
						neighbourPoints[j]->plane = plane;
						tempNextStepPoints.push_back(neighbourPoints[j]);
					}
				}
			}
		}
		nextStepPoints = tempNextStepPoints;
	}
}

void calculateBounds(Plane& plane)
{
	plane.horizontalBounds = std::make_pair(plane.points[0]->horizontalIndex, plane.points[0]->horizontalIndex);
	plane.verticalBounds = std::make_pair(plane.points[0]->verticalIndex, plane.points[0]->verticalIndex);
	for (size_t i = 1; i < plane.points.size(); i++) 
	{
		if (plane.points[i]->horizontalIndex > plane.horizontalBounds.second)
			plane.horizontalBounds.second = plane.points[i]->horizontalIndex;
		else if (plane.points[i]->horizontalIndex < plane.horizontalBounds.first)
			plane.horizontalBounds.first = plane.points[i]->horizontalIndex;
		if (plane.points[i]->verticalIndex > plane.verticalBounds.second)
			plane.verticalBounds.second = plane.points[i]->verticalIndex;
		else if (plane.points[i]->verticalIndex < plane.verticalBounds.first)
			plane.verticalBounds.first = plane.points[i]->verticalIndex;
	}
}

void findPlanes()
{
	auto start = std::chrono::steady_clock::now();
	size_t minPointCount = 10;
	size_t counter = 1;
	double planeDistanceTreshold = 0.1;
	double normalTreshold = 0.1;
	for (size_t i = 0; i < points.size(); i++) if (points[i]) points[i]->isMarked = true;
	for (size_t i = 0; i < points.size(); i++) if (points[i]) points[i]->isMarked2 = true;
	std::vector<Point*> nextStepPoints;
	for (size_t j = 0; j < points.size(); j++) {
		if (points[j] && points[j]->isMarked) {
			nextStepPoints.push_back(points[j]);
			calculateNormal(points[j]);
			while (nextStepPoints.size() > 0) {
				Plane* plane = new Plane();
				std::vector<Point*> tempNextStepPoints;
				for (size_t i = 0; i < nextStepPoints.size(); i++) {
					if (nextStepPoints[i]->isMarked2) {
						auto normal = nextStepPoints[i]->normal;
						Vec3<double> normals[4] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} };
						size_t x = nextStepPoints[i]->horizontalIndex;
						size_t y = nextStepPoints[i]->verticalIndex;
						Point* neighbourPoints[4] = { points[getOffset(x, y - 1)], points[getOffset(x - 1, y)], points[getOffset(x, y + 1)],
							points[getOffset(x + 1, y)] };
						for (size_t k = 0; k < 4; k++) {
							if (neighbourPoints[k] && (k > 0 || y > 0) && (k < verticalCount - 1 || y < 3) && neighbourPoints[k]->isMarked2) {
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
							if ((normals[k] - normal).length() < normalTreshold && (normals[(k + 1) % 4] - normal).length() < normalTreshold) {

								choosePoints({ nextStepPoints[i], neighbourPoints[k], neighbourPoints[(k + 1) % 4] }, planeDistanceTreshold, {0,0,0}, 
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
				choosePoints({ planes[i]->points[0], nullptr, nullptr }, planeDistanceTreshold, planes[i]->normal,
					plane);
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
	auto end = std::chrono::steady_clock::now();
	std::cout << "Elapsed time in seconds: "
		<< std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
		<< " sec" << std::endl;
}

void findNextPoint(Point* startPoint, bool isOuterEdge, bool isPreviousSpike, bool wasThereNonSpike, /*out*/ std::vector<Point*>& currentEdge)
{
	size_t direction = isOuterEdge ? 0 : 1;
	Point* currentPoint = nullptr;
	while (currentPoint != startPoint) 
	{
		if (!currentPoint)
			currentPoint = startPoint;
		Point* neighbourPoint = nullptr;
		size_t x = currentPoint->horizontalIndex;
		size_t y = currentPoint->verticalIndex;
		if (currentPoint->isMarked) {
			currentEdge.push_back(currentPoint);
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
				if (neighbourPoint && neighbourPoint->plane != startPoint->plane && neighbourPoint->plane != nullptr) {
					currentPoint->neighbourPlaneNeighbours.push_back(neighbourPoint);
				}
				direction += direction == 3 ? -3 : 1;
			}
		}
		size_t initialDirection = direction;
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
				currentPoint->isMarked = false;
				currentPoint = startPoint;
				break;
			}
			if (neighbourPoint && neighbourPoint->plane == startPoint->plane && neighbourPoint->plane != nullptr) {
				if (wasThereNonSpike && currentEdge.size() > 1 && isPreviousSpike && !isSpike(currentPoint) && currentPoint->isMarked) {
					currentEdge.pop_back();
					currentPoint = currentEdge[currentEdge.size() - 1];
					direction = (initialDirection + 2) % 4;
					break;
				}
				else {
					if (!wasThereNonSpike && !isSpike(currentPoint)) {
						startPoint = currentPoint;
						wasThereNonSpike = true;
					}
					currentPoint->isMarked = false;
					currentPoint = neighbourPoint;
					direction = (direction + 3) % 4;
					break;
				}
			}
			direction += direction == 3 ? -3 : 1;
		}
	}
}

bool isPoint(Point* point)
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

bool isStraightPoint(size_t pointIndex, std::vector<Point*>& edge, bool& isPreviousConcave)
{
	bool previousConcaveStore = isPreviousConcave;
	size_t neighbourCount = 0;
	Point* point = edge[pointIndex];
	size_t x = point->horizontalIndex;
	size_t y = point->verticalIndex;
	Plane* plane = point->plane;
	bool isNeighbour[4] = { false, false, false, false };
	Point* neighbourPoints[4] = { points[getOffset(x, y - 1)], points[getOffset(x, y + 1)], points[getOffset(x - 1, y)],
		points[getOffset(x + 1, y)] };
	for (size_t i = 0; i < 4; i++) {
		if ((y > 0 || i > 0) && (y < verticalCount - 1 || i < 3) && neighbourPoints[i] && neighbourPoints[i]->plane &&
			neighbourPoints[i]->plane == plane && (neighbourPoints[i]->outlineId == 0 || neighbourPoints[i]->outlineId == point->outlineId)) {
			neighbourCount++;
			isNeighbour[i] = true;
		}
	}
	isPreviousConcave = neighbourCount == 4;
	if (neighbourCount == 3 || (neighbourCount == 2 && ((isNeighbour[0] && isNeighbour[1]) || (isNeighbour[2] && isNeighbour[3])))) {
		return true;
	}
	if (neighbourCount == 4 && (pointIndex == 0 ? edge[edge.size() - 1] : edge[pointIndex - 1])->isCorner)
		return true;
	if (previousConcaveStore) {
		edge[pointIndex - 1] -> isCorner = false;
	}
	return false;
}

void findPoints()
{
	for (size_t i = 0; i < planes.size(); i++) 
	{
		std::vector<Point*> PointsInPlane;
		for (size_t j = 0; j < planes[i]->points.size(); j++) planes[i]->points[j]->isMarked = true;
		for (size_t j = 0; j < planes[i]->points.size(); j++) if(isPoint(planes[i]->points[j])) PointsInPlane.push_back(planes[i]->points[j]);
		bool isFirstEdge = true;
		while (PointsInPlane.size() > 0) 
		{
			std::vector<Point*> tempPointsInPlane;
			Point* startPoint = PointsInPlane[0];
			size_t minHorizontalCoord = startPoint->horizontalIndex;
			size_t minVerticalCoord = startPoint->verticalIndex;
			for (size_t j = 1; j < PointsInPlane.size(); j++) {
				if ((PointsInPlane[j]->horizontalIndex < minHorizontalCoord && minHorizontalCoord - PointsInPlane[j]->horizontalIndex < horizontalCount / 2)
					|| PointsInPlane[j]->horizontalIndex > minHorizontalCoord + horizontalCount / 2) {
					minHorizontalCoord = PointsInPlane[j]->horizontalIndex;
					minVerticalCoord = PointsInPlane[j]->verticalIndex;
					startPoint = PointsInPlane[j];
				}
				else if (PointsInPlane[j]->horizontalIndex == minHorizontalCoord && PointsInPlane[j]->verticalIndex < minVerticalCoord) {
					minVerticalCoord = PointsInPlane[j]->verticalIndex;
					startPoint = PointsInPlane[j];
				}
			}
			std::vector<Point*> currentEdge;
			findNextPoint(startPoint, isFirstEdge, false, false, currentEdge);
			if (!isFirstEdge) {
				currentEdge.insert(currentEdge.begin(), currentEdge[currentEdge.size() - 1]);
				currentEdge.pop_back();
			}
			for (size_t k = 0; k < currentEdge.size(); k++) {
				currentEdge[k]->outlineId = currentOutlineId;
			}
			for (size_t j = 0; j < PointsInPlane.size(); j++) {
				if (PointsInPlane[j]->isMarked) tempPointsInPlane.push_back(PointsInPlane[j]);
			}
			PointsInPlane = tempPointsInPlane;
			currentOutlineId++;
			planes[i]->edges.push_back(currentEdge);
			isFirstEdge = false;
		}

		for (size_t j = 0; j < planes[i]->points.size(); j++) planes[i]->points[j]->isMarked = false;
	}
}

bool arePlanesNeighbours(Plane p1, Plane p2, std::pair<int, int>& horizontalCommonBounds, std::pair<int, int>& verticalCommonBounds)
{
	if (p1.horizontalBounds.first > p1.horizontalBounds.second)
		p1.horizontalBounds.second += horizontalCount;
	double p1HorizontalSize = p1.horizontalBounds.second - p1.horizontalBounds.first;
	double p1VerticalSize = p1.verticalBounds.second - p1.verticalBounds.first;
	std::pair<int, int> p1Center = {(int)(p1.horizontalBounds.first + p1HorizontalSize / 2) % horizontalCount, 
		p1.verticalBounds.first + p1VerticalSize / 2 };

	if (p2.horizontalBounds.first > p2.horizontalBounds.second)
		p2.horizontalBounds.second += horizontalCount;
	double p2HorizontalSize = p2.horizontalBounds.second - p2.horizontalBounds.first;
	double p2VerticalSize = p2.verticalBounds.second - p2.verticalBounds.first;
	std::pair<int, int> p2Center = { (int)(p2.horizontalBounds.first + p2HorizontalSize / 2) % horizontalCount,
		p2.verticalBounds.first + p2VerticalSize / 2 };

	if (abs(p1Center.first - p2Center.first) > horizontalCount / 2) 
	{
		if (p1Center.first > p2Center.first)
			p2Center.first += horizontalCount;
		if (p1Center.first < p2Center.first)
			p1Center.first += horizontalCount;
	}
	if (abs(p1Center.first - p2Center.first) <= (p1HorizontalSize + p2HorizontalSize) / 2 + 1 &&
		abs(p1Center.second - p2Center.second) <= (p1VerticalSize + p2VerticalSize) / 2 + 1) 
	{
		horizontalCommonBounds = {abs(p1.horizontalBounds.first - p2.horizontalBounds.first) > horizontalCount / 2
			? std::min(p1.horizontalBounds.first, p2.horizontalBounds.first) : std::max(p1.horizontalBounds.first, p2.horizontalBounds.first),
			abs(p1.horizontalBounds.second - p2.horizontalBounds.second) > horizontalCount / 2
			? std::max(p1.horizontalBounds.second, p2.horizontalBounds.second) : std::min(p1.horizontalBounds.second, p2.horizontalBounds.second)};
		if (horizontalCommonBounds.second < horizontalCommonBounds.first)
			horizontalCommonBounds = { horizontalCommonBounds.second, horizontalCommonBounds.first };

		verticalCommonBounds = { std::max(p1.verticalBounds.first, p2.verticalBounds.first),
			std::min(p1.verticalBounds.second, p2.verticalBounds.second) };
		if (verticalCommonBounds.second < verticalCommonBounds.first)
			verticalCommonBounds = { verticalCommonBounds.second, verticalCommonBounds.first };
		return true;
	}
	return false;
}

const double newPointAcceptTreshold = 0.95;
const double inf = 1000000;

Point* createNewPoint(Vec3<double> newPointPos, Point* point, size_t addedCount)
{
	Point* newPoint = new Point(newPointPos, point->horizontalIndex, verticalCount, point->plane);
	addedPoints.push_back(newPoint);
	newPoint->isCorner = true;
	newPoint->outlineId = point->outlineId;
	for (size_t j = 0; j < point->plane->edges.size(); j++) {
		if (point->plane->edges[j][0]->outlineId == point->outlineId) {
			for (size_t k = 0; k < point->plane->edges[j].size(); k++) {
				if (point->plane->edges[j][k] == point) {
					point->plane->edges[j].insert(point->plane->edges[j].begin() + k + 1 + addedCount, newPoint);
					break;
				}
			}
			break;
		}
	}
	return newPoint;
}

bool addNewPoint(Point* point, Point*& neighbour, Plane* plane, size_t addedCount)
{
	if (neighbour->verticalIndex == verticalCount) { //created by other plane
		for (size_t j = 0; j < point->plane->edges.size(); j++) {
			if (point->plane->edges[j][0]->outlineId == point->outlineId) {
				for (size_t k = 0; k < point->plane->edges[j].size(); k++) {
					if (point->plane->edges[j][k] == point) {
						for (size_t l = 0; l < 4; l++) 
						{
							if (point->plane->edges[j][k + l + 1] == neighbour) {
								point->plane->edges[j].insert(point->plane->edges[j].begin() + k + 1 + addedCount, neighbour);
								point->plane->edges[j].erase(point->plane->edges[j].begin() + k + l + 2);
								break;
							}
						}
						break;
					}
				}
				break;
			}
		}
		return true;
	}
	Vec3<double> dir = { 0,0,0 };
	switch (areNeighbours(point, neighbour)) {
	case 1:
	dir = point->plane->pointDirections.first;
	break;
	case 2:
	dir = point->plane->pointDirections.first * -1;
	break;
	case 3:
	dir = point->plane->pointDirections.second;
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
		return false;
	Vec3<double> dirToNew = newPointPos - point->position;
	if (Vec3<double>::dot_product(Vec3<double>::normalize(dir), Vec3<double>::normalize(dirToNew)) < newPointAcceptTreshold)
		return false;
	Vec3<double> neighbourDir = { 0,0,0 };
	switch (areNeighbours(neighbour, point)) {
	case 1:
	neighbourDir = plane->pointDirections.first;
	break;
	case 2:
	neighbourDir = plane->pointDirections.first * -1;
	break;
	case 3:
	neighbourDir = plane->pointDirections.second;
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
		return false;
	Vec3<double> dirToNewNeighbour = neighbourNewPointPos - neighbour->position;
	if (Vec3<double>::dot_product(Vec3<double>::normalize(neighbourDir), Vec3<double>::normalize(dirToNewNeighbour)) < newPointAcceptTreshold) 
		return false;
	auto newPos = (newPointPos + neighbourNewPointPos) / 2;
	for (size_t i = 0; i < neighbour->neighbourPlaneNeighbours.size(); i++) 
	{
		if (neighbour->neighbourPlaneNeighbours[i] == point) 		
		{
			auto newPoint = createNewPoint(newPointPos, neighbour, 0);
			neighbour->neighbourPlaneNeighbours[i] = newPoint;
			break;
		}
	}
	std::cout << point->plane->edges[0].size() << std::endl;
	createNewPoint(newPointPos, point, addedCount);
	std::cout << point->plane->edges[0].size() << std::endl;
	std::cout << newPointPos.x << std::endl;
	return true;
}

void connectPlanes()
{
	for (size_t i = 0; i < planes.size(); i++) 
	{
		for (size_t j = 0; j < planes[i]->edges.size(); j++) 
		{
			bool wasFirstPointGenerated = false;
			bool wasPreviousSelected = true;
			Point* previousPoint = nullptr;
			Point* previousNeighbourPoint = nullptr;
			size_t previousIndex = 0;
			size_t previousAddedCount = 0;
			size_t previousId = 0;
			std::vector<std::pair<size_t, size_t>> cornersToDelete;
			for (size_t k = 0; k < planes[i]->edges[j].size(); k++) {
				if (planes[i]->edges[j][k]->verticalIndex == verticalCount)
					continue;
				size_t addedCount = 0;
				for (auto neighbour : planes[i]->edges[j][k]->neighbourPlaneNeighbours) 
				{
					if (!previousNeighbourPoint || neighbour->plane->id != previousId) 
					{
						if (!wasPreviousSelected)
						{
							auto newPoint = addNewPoint(previousPoint, previousNeighbourPoint, previousNeighbourPoint->plane, previousAddedCount);
							if (newPoint) {
								cornersToDelete.push_back({ k,  previousNeighbourPoint->plane->id});
								k++;
							}
						}
						auto newPoint = addNewPoint(planes[i]->edges[j][k], neighbour, neighbour->plane, addedCount);
						if (newPoint) 
						{
							if (k == 0 && ((planes[i]->edges[j][k]->horizontalIndex - neighbour->horizontalIndex + horizontalCount) % horizontalCount
								== 1)) wasFirstPointGenerated = true;
							cornersToDelete.push_back({ k, neighbour->plane->id });
							k++;
							addedCount++;
							previousId = neighbour->plane->id;
						}
						wasPreviousSelected = true;
					}
					else
						wasPreviousSelected = false;
					previousPoint = planes[i]->edges[j][k];
					previousIndex = k;
					previousAddedCount = addedCount;
					if(k > 0 || !wasFirstPointGenerated)
						previousNeighbourPoint = neighbour;
				}
			}
			if (!wasPreviousSelected && (cornersToDelete.size() == 0 || previousNeighbourPoint->plane->id != cornersToDelete[0].second || !wasFirstPointGenerated)) {
				auto newPoint = addNewPoint(previousPoint, previousNeighbourPoint, previousNeighbourPoint->plane, previousAddedCount);
				if (newPoint) {
					cornersToDelete.push_back({ previousIndex, previousNeighbourPoint->plane->id});
				}
			}
			for (size_t k = 0; k < cornersToDelete.size(); k++)
			{
				if (cornersToDelete[k].second == cornersToDelete[k == cornersToDelete.size() - 1 ? 0 : k + 1].second && (wasFirstPointGenerated || k < cornersToDelete.size() - 1) &&
					(!wasFirstPointGenerated || k > 0))
				{
					for (size_t l = cornersToDelete[k].first; l <= (k == cornersToDelete.size() - 1 ? planes[i]->edges[j].size() - 1 : cornersToDelete[k + 1].first); l++) {
						if(planes[i]->edges[j][l]->verticalIndex < verticalCount)
							planes[i]->edges[j][l]->isCorner = false;
					}
				}
				else planes[i]->edges[j][cornersToDelete[k].first]->isCorner = false;
			}
		}
	}
}

void findCorners()
{
	for (size_t k = 0; k < planes.size(); k++) {
		for (size_t i = 0; i < planes[k]->edges.size(); i++) {
			bool isPreviousConcave = false;
			for (size_t j = 0; j < planes[k]->edges[i].size(); j++) {
				if (!isStraightPoint(j, planes[k]->edges[i], isPreviousConcave)) {
					planes[k]->edges[i][j]->isCorner = true;
				}
			}
		}
	}
}

double distancePointFromLine(Vec3<double> p, std::pair<Vec3<double>, Vec3<double>> line)
{
	Vec3<double> directionVector = line.first - line.second;
	return Vec3<double>::crossProduct(p - line.first, directionVector).length() / directionVector.length();
}

bool isOnLine(Vec3<double> p, std::pair<Vec3<double>, Vec3<double>> line, double treshold)
{
	return distancePointFromLine(p, line) < treshold;
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
			for (size_t k = 0; k < planes[i]->edges[j].size(); k++) {
				if (planes[i]->edges[j][k]->isCorner) {
					planes[i]->edges[j][k]->cornerId = currentCornerId;
					planes[i]->edges[j][k]->cornerIndex = currentCornerIndex;
					corners.push_back(planes[i]->edges[j][k]);
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
	findPoints();
	findCorners();
	connectPlanes();
	exportObjects();
	writeData();
}

int main()
{
    readData();
	processData();
    return 0;
}