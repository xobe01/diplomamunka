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

struct Point
{
	Vec3<double> position;
	size_t horizontalIndex;
	size_t verticalIndex;
	size_t id{ 0 };
	bool deleted{ false };
	bool isMarked{ false };
	Point(Vec3<double> _position, size_t _horizontalIndex, size_t _verticalIndex, size_t _id) : position(_position), horizontalIndex(_horizontalIndex),
		verticalIndex(_verticalIndex), id(_id) {};
};

std::vector<Point*> points;
std::vector<std::vector<Point*>> separatedPoints;
std::vector<int> verticalCounts;
size_t pointCount;
size_t horizontalCount;
size_t verticalCount;
int currentId = 1;
const double objectPointDistance = 5;

size_t getOffset(int horizontalIndex, int verticalIndex)
{
	if (horizontalIndex < 0) horizontalIndex = horizontalCount - 1 + horizontalIndex;
	else if (horizontalIndex > horizontalCount - 1) horizontalIndex = horizontalIndex - horizontalCount;
	if (verticalIndex < 0) verticalIndex = verticalCount - 1 + verticalIndex;
	else if (verticalIndex > verticalCount - 1) verticalIndex = verticalIndex - verticalCount;
	return horizontalIndex * verticalCount + verticalIndex;
}

void ReadData()
{
	verticalCounts.push_back(0);
    std::string myText;
    std::ifstream MyReadFile("C:/Users/ungbo/Desktop/BME/_Diplomaterv/Diplomamunka/Diplomamunka/Assets/Resources/points_raw.txt");
	getline(MyReadFile, myText);
	pointCount = std::stoi(myText);
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
			points[getOffset(horizontalIndex, verticalIndex)] = new Point({ x, y, z }, horizontalIndex, verticalIndex, id);
		}		
	}
    MyReadFile.close();
}

void WriteData(std::vector<Point*> exportPoints)
{
	std::ofstream MyFile("C:/Users/ungbo/Desktop/BME/_Diplomaterv/Diplomamunka/Diplomamunka/Assets/Resources/points_processed.txt");
	for (size_t i = 0; i < exportPoints.size(); i++) {
		if (exportPoints[i] && !exportPoints[i]->deleted)
			MyFile << exportPoints[i]->position.to_string() << ';' << exportPoints[i]->horizontalIndex << ';' << exportPoints[i]->verticalIndex <<
			';' << exportPoints[i]->id << std::endl;
	}
	MyFile.close();
}

void GroundSegmentation() { //TODO point struktúra megvátozott
	/*double groundLevel = points[0].position.y;
	for (size_t i = 0; i < pointCount; i++) {
		if (points[i].position.y < groundLevel) groundLevel = points[i].position.y;
	}

	for (size_t i = 0; i < pointCount; i++) {
		if (points[i].position.y == groundLevel)
			points[i].deleted = true;
	}*/
}

void choosePoints(const Vec3<Point*> planePoints, const std::vector<Point*>& nonProcessedPoints, double acceptTreshold, 
	/*out*/ std::vector<Point*>& chosenPoints, /*out*/ double std)
{
	std = 0;
	std::vector<double> distances;
	double avg = 0;
	auto normal = Vec3<double>::normalize(Vec3<double>::crossProduct(planePoints.x->position - 
		planePoints.y->position, planePoints.z->position -planePoints.y->position));
	for (size_t i = 0; i < nonProcessedPoints.size(); i++)
	{
		double dist = abs(Vec3<double>::dot_product(normal, nonProcessedPoints[i]->position - planePoints.y->position));
		if (dist <= acceptTreshold) 
		{
			avg += dist;
			distances.push_back(dist);
			chosenPoints.push_back(nonProcessedPoints[i]);
		}
	}
	avg /= chosenPoints.size();
	for (size_t i = 0; i < chosenPoints.size(); i++)
	{
		std += pow(distances[i] - avg, 2);
	}
	std /= chosenPoints.size();
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

bool checkIfBridge(Point* p)
{
	if (!p->isMarked) return false;
	size_t x = p->horizontalIndex;
	size_t y = p->verticalIndex;
	int neighbourCount = 0;
	//fel-le
	bool isUpNotNeightbour = y == 0 || !points[getOffset(x, y - 1)] || !points[getOffset(x, y - 1)]->isMarked;
	bool isDownNotNeightbour = y == verticalCount - 1 || !points[getOffset(x, y + 1)] || !points[getOffset(x, y + 1)]->isMarked;
	if (isUpNotNeightbour && isDownNotNeightbour) return true;
	if (!isUpNotNeightbour)
		neighbourCount++;
	if (!isDownNotNeightbour)
		neighbourCount++;
	//jobbra-balra
	bool isLeftNotNeightbour = !points[getOffset(x - 1, y)] || !points[getOffset(x - 1, y)]->isMarked;
	bool isRighttNotNeightbour = !points[getOffset(x + 1, y)] || !points[getOffset(x + 1, y)]->isMarked;
	if (isLeftNotNeightbour && isRighttNotNeightbour) return true;
	if (!isLeftNotNeightbour)
		neighbourCount++;
	if (!isRighttNotNeightbour)
		neighbourCount++;
	//átlósan
	if ((((y > 0 && (!points[getOffset(x - 1, y - 1)] || !points[getOffset(x - 1, y - 1)]->isMarked)) &&
		(y < verticalCount - 1 && (!points[getOffset(x + 1, y + 1)] || !points[getOffset(x + 1, y + 1)]->isMarked))) ||
		((y > 0 && (!points[getOffset(x + 1, y - 1)] || !points[getOffset(x + 1, y - 1)]->isMarked)) &&
			(y < verticalCount - 1 && (!points[getOffset(x - 1, y + 1)] || !points[getOffset(x - 1, y + 1)]->isMarked))))
		&& neighbourCount > 2)
		return true;
	return false;
}

bool isThereBridge(const std::vector<Point*>& checkPoints)
{
	bool theresBridge = false;
	for (auto p : checkPoints) 
		if (checkIfBridge(p)) {
			theresBridge = true;
			p->isMarked = false;
		}
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

void RANSAC(std::vector<Point*>& nonProcessedPoints)
{
	/*size_t maxPointInPLane = nonProcessedPoints.size()/10;
	double findChance = 0.9;
	double eps = 1 - (double)maxPointInPLane / nonProcessedPoints.size();
	size_t N = log(1 - findChance) / log(1 - pow(1 - eps, 3));*/
	size_t N = 100;
	double bestStd = 100000;
	double acceptTreshold = 0.1;
	std::vector<Point*> bestPoints;
	for (size_t i = 0; i < N; i++) 
	{
		std::vector<Point*> chosenPoints;
		std::vector<Point*> chosenPoints2;
		double std = 0;
		auto pickedPoints = pick3Points(nonProcessedPoints);
		choosePoints(pickedPoints, nonProcessedPoints, acceptTreshold, chosenPoints, std);
		bool temptt = currentId == 13 && chosenPoints.size() == 1748;
		checkForGaps(chosenPoints); //TODO std
		if (chosenPoints.size() >= 3) {
			choosePoints(pick3Points(chosenPoints), nonProcessedPoints, acceptTreshold, chosenPoints2, std);
			checkForGaps(chosenPoints2); //TODO std
			chosenPoints = chosenPoints2;
		}		
		if (chosenPoints.size() > bestPoints.size() || (chosenPoints.size() == bestPoints.size() && std < bestStd))
		{
			bestPoints = chosenPoints;
			bestStd = std;
		}
	}
	for (size_t i = 0; i < bestPoints.size(); i++) {
		bestPoints[i]->isMarked = true;
	}
}

void ObjectSegmentation()
{
	double treshold = 1;
	for (size_t i = 0; i < points.size(); i++) if(points[i]) points[i]->isMarked = true;
	std::vector<Point*>nextStepPoints;
	for (size_t j = 0; j < points.size(); j++) 
	{
		if (points[j] && points[j]->isMarked) {
			std::vector<Point*> objectPoints;
			objectPoints.push_back(points[j]);
			nextStepPoints.push_back(points[j]);
			points[j]->isMarked = false;
			while (nextStepPoints.size() > 0) {
				std::vector<Point*> tempNextStepPoints;
				for (size_t i = 0; i < nextStepPoints.size(); i++) {
					size_t x = nextStepPoints[i]->horizontalIndex;
					size_t y = nextStepPoints[i]->verticalIndex;
					Point* neighbourPoint = points[getOffset(x, y - 1)];
					if (y > 0 && neighbourPoint && neighbourPoint->isMarked && Vec3<double>::distance(nextStepPoints[i]->position, neighbourPoint->position) < treshold) {
						neighbourPoint->isMarked = false;
						tempNextStepPoints.push_back(neighbourPoint);
						objectPoints.push_back(neighbourPoint);
					}
					neighbourPoint = points[getOffset(x, y + 1)];
					if (y < verticalCount - 1 && neighbourPoint && neighbourPoint->isMarked && Vec3<double>::distance(nextStepPoints[i]->position, neighbourPoint->position) < treshold) {
						neighbourPoint->isMarked = false;
						tempNextStepPoints.push_back(neighbourPoint);
						objectPoints.push_back(neighbourPoint);
					}
					neighbourPoint = points[getOffset(x - 1, y)];
					if (neighbourPoint && neighbourPoint->isMarked && Vec3<double>::distance(nextStepPoints[i]->position, neighbourPoint->position) < treshold) {
						neighbourPoint->isMarked = false;
						tempNextStepPoints.push_back(neighbourPoint);
						objectPoints.push_back(neighbourPoint);
					}
					neighbourPoint = points[getOffset(x + 1, y)];
					if (neighbourPoint && neighbourPoint->isMarked && Vec3<double>::distance(nextStepPoints[i]->position, neighbourPoint->position) < treshold) {
						neighbourPoint->isMarked = false;
						tempNextStepPoints.push_back(neighbourPoint);
						objectPoints.push_back(neighbourPoint);
					}
				}
				nextStepPoints = tempNextStepPoints;
			}
			separatedPoints.push_back(objectPoints);
		}
	}
	for (size_t i = 0; i < points.size(); i++) if (points[i]) points[i]->isMarked = false;
}

#include <chrono>

void findPlanes(std::vector<std::vector<Point*>>& planes)
{
	auto start = std::chrono::steady_clock::now();
	std::vector<Point*> nonProcessedPoints;
	for (size_t i = 0; i < points.size(); i++) {
		if (points[i] && !points[i]->deleted)
			nonProcessedPoints.push_back(points[i]);
	}

	size_t planeSearchIter = 100;
	for (size_t j = 0; j < separatedPoints.size(); j++) {
		nonProcessedPoints = separatedPoints[j];
	
		while (true) {
			std::vector<Point*> plane;
			if (nonProcessedPoints.size() >= 50)
				RANSAC(nonProcessedPoints);
			else break;
			std::vector<Point*> tempPoints;
			for (size_t i = 0; i < nonProcessedPoints.size(); i++) {
				if (nonProcessedPoints[i]->isMarked) {
					nonProcessedPoints[i]->isMarked = false;
					plane.push_back(nonProcessedPoints[i]);
				}
				else {
					tempPoints.push_back(nonProcessedPoints[i]);
				}
			}
			if (plane.size() < 50) break;
			planes.push_back(plane);
			for (size_t i = 0; i < plane.size(); i++) {
				plane[i]->id = currentId;
			}
			currentId++;
			nonProcessedPoints = tempPoints;
			std::cout << nonProcessedPoints.size() << std::endl;
		}
	}
	auto end = std::chrono::steady_clock::now();
	std::cout << "Elapsed time in seconds: "
		<< std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
		<< " sec";
}

void findNextEdgePoint(Point* startPoint, Point* currentPoint, std::vector<Point*>& edgePoints, size_t direction)
{
	edgePoints.push_back(currentPoint);
	size_t x = currentPoint->horizontalIndex;
	size_t y = currentPoint->verticalIndex;
	Point* neighbourPoint = nullptr;
	for (size_t i = 0; i < 3; i++) {
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
		if (neighbourPoint && neighbourPoint->isMarked) {
			neighbourPoint->isMarked = false;
			if (neighbourPoint == startPoint) {
			}
			else 
			{
				findNextEdgePoint(startPoint, neighbourPoint, edgePoints, (direction + 3) % 4);
			}
			break;
		}
		direction += direction == 3 ? -3 : 1;
	}
	if (neighbourPoint == nullptr) {
		std::cout << "PLANE WITHOUT RELEVANT OUTLINE!" << std::endl;
	}
}

bool isEdgePoint(Point* point)
{
	if (!point->isMarked) return false;
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

void findEdgePoints(std::vector<std::vector<Point*>>& planes, /*out*/ std::vector<std::vector<std::vector<Point*>>>& edges)
{
	for (size_t i = 0; i < planes.size(); i++) 
	{
		std::vector<std::vector<Point*>> edgesInPlane;
		std::vector<Point*> edgePointsInPlane;
		for (size_t j = 0; j < planes[i].size(); j++) planes[i][j]->isMarked = true;
		for (size_t j = 0; j < planes[i].size(); j++) if(isEdgePoint(planes[i][j])) edgePointsInPlane.push_back(planes[i][j]);
		bool isFirstEdge = true;
		for (size_t j = 0; j < edgePointsInPlane.size(); j++) {
			if (isEdgePoint(edgePointsInPlane[j])) {
				std::vector<Point*> edgePoints;
				findNextEdgePoint(edgePointsInPlane[j], edgePointsInPlane[j], edgePoints, isFirstEdge ? 0 : 1);
				if (!isFirstEdge) {
					edgePoints.insert(edgePoints.begin(), edgePoints[edgePoints.size() - 1]);
					edgePoints.pop_back();
				}
				edgesInPlane.push_back(edgePoints);
				for (auto p : edgePoints) {
					//if (p) p->id = currentId;
				}
				//currentId++;
				isFirstEdge = false;
				break; //TODO remove
			}
		}
		for (size_t j = 0; j < planes[i].size(); j++) planes[i][j]->isMarked = false;
		edges.push_back(edgesInPlane);
	}
}

double distancePointFromLine(Vec3<double> p, std::pair<Vec3<double>, Vec3<double>> line)
{
	Vec3<double> directionVector = line.first - line.second;
	return Vec3<double>::length(Vec3<double>::crossProduct(p - line.first, directionVector)) / Vec3<double>::length(directionVector);
}

bool isOnLine(Vec3<double> p, std::pair<Vec3<double>, Vec3<double>> line, double treshold)
{
	return distancePointFromLine(p, line) < treshold;
}

void createEdgeSegments(const std::vector<std::vector<std::vector<Point*>>>& edges, /*out*/ std::vector < std::vector<Point*>>& corners)
{
	for (auto plane : edges) {
		for (auto edge : plane) {
			std::pair<Vec3<double>, Vec3<double>> linePoints;
			std::vector<Point*> currentCorners;
			bool newSegment = true;
			for (size_t i = 0; i < edge.size(); i++) {
				if (i == 0 || !isOnLine(edge[i]->position, linePoints, 0.1)) newSegment = true;
				if (newSegment) {
					newSegment = false;
					currentCorners.push_back(edge[i == 0 ? i : (i - 1)]);
					edge[i == 0 ? i : (i - 1)]->id = currentId;
					if (i == 0) {
						linePoints = std::make_pair(edge[i]->position, edge[i + 1]->position);
						i++;
					}
					else {
						linePoints = std::make_pair(edge[i-1]->position, edge[i]->position);
					}
				}
			}
			currentId++;
			corners.push_back(currentCorners);
		}
	}
}

void egoCarSegmentation()
{
	for (size_t i = 0; i < points.size(); i++) {
		if (points[i] && points[i]->position.x <= 1 && points[i]->position.x >= -1 &&
			points[i]->position.y <= 1 && points[i]->position.y >= -1 &&
			points[i]->position.z <= 2.5 && points[i]->position.z >= -2.5)
			points[i]->deleted = true;
	}
}

void filterCornerPoints(const std::vector<std::vector<Point*>>& corners, std::vector<std::vector<Point*>>& filteredCorners)
{
	for (auto currentEdgeCorners : corners) {
		std::vector<Point*> currentCorners;
		currentEdgeCorners.push_back(currentEdgeCorners[0]);
		currentCorners.push_back(currentEdgeCorners[0]);
		size_t startCornerIndex = 0;
		for (size_t i = startCornerIndex + 2; i < currentEdgeCorners.size(); i++) {
			std::pair<Vec3<double>, Vec3<double>> direction = std::make_pair(currentEdgeCorners[startCornerIndex]->position, currentEdgeCorners[i]->position);
			for (size_t j = startCornerIndex + 1; j < i; j++) {
				if (!isOnLine(currentEdgeCorners[j]->position, direction, 0.0)) {
					currentCorners.push_back(currentEdgeCorners[i - 1]);
					startCornerIndex = i - 1;
					i = startCornerIndex + 1;
					break;
				}
			}
		}
		filteredCorners.push_back(currentCorners);
	}
}

void exportObjects(std::vector<std::vector<Point*>> filteredCorners)
{
	for (size_t i = 0; i < filteredCorners.size(); i++) {
		std::ofstream MyFile("C:/Users/ungbo/Desktop/BME/_Diplomaterv/Diplomamunka/Diplomamunka/Assets/Resources/processed_obj_" + std::to_string(i) + ".obj");
		MyFile << "o Mesh" << std::endl;
		for (size_t j = 0; j < filteredCorners[i].size(); j++) {
			MyFile << "v " << -filteredCorners[i][j]->position.x << " " << filteredCorners[i][j]->position.y << " " <<
				filteredCorners[i][j]->position.z << std::endl;
		}
		MyFile << "f ";
		for (size_t j = 1; j < filteredCorners[i].size() + 1; j++) {
			MyFile << j << " ";
		}
		MyFile << std::endl;
		MyFile << "f ";
		for (size_t j = filteredCorners[i].size(); j > 0; j--) {
			MyFile << j << " ";
		}
		MyFile << std::endl;
		MyFile.close();
	}
}

void ProcessData() {
	egoCarSegmentation();
	//GroundSegmentation();
	ObjectSegmentation();
	//WriteData(points);

	std::vector<std::vector<Point*>> planes;
	findPlanes(planes);

	WriteData(points);

	/*std::vector<std::vector<std::vector<Point*>>> edges;
	findEdgePoints(planes, edges);
	std::vector<std::vector<Point*>> corners;
	createEdgeSegments(edges, corners);
	std::vector<std::vector<Point*>> filteredCorners;
	filterCornerPoints(corners, filteredCorners);
	exportObjects(filteredCorners);
	std::vector<Point*> writePoints;
	for (size_t i = 0; i < filteredCorners.size(); i++) {
		for (size_t j = 0; j < filteredCorners[i].size(); j++) {
			writePoints.push_back(filteredCorners[i][j]);
		}
	}
	WriteData(writePoints);*/
}



int main()
{
    ReadData();
	ProcessData();
	//WriteData();
    return 0;
}