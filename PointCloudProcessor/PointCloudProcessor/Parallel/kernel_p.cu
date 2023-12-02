#include "kernel_p.h"

std::vector<Plane*> allPlanes;
std::vector<Point*> fitAddedPoints;
std::vector<Edge*> fitEdges;
std::vector<std::vector<Point*>> points;
std::vector<std::vector<Point*>> addedPoints;
std::vector<std::vector<Edge*>> edges;
std::vector<std::vector<Plane*>> planes;

size_t horizontalCount = 0;
size_t verticalCount = 0;
std::vector<int> currentPlaneId;
std::vector<int> currentOutlineId;
const double objectPointDistance = 5;
const double planeDistanceTreshold = 0.05;

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
	points.push_back({});
	addedPoints.push_back({});
	edges.push_back({});
	planes.push_back({});
	currentPlaneId.push_back(1);
	currentOutlineId.push_back(1);
	std::string myText;
	std::ifstream MyReadFile("C:/Users/ungbo/Desktop/BME/_Diplomamunka/Diplomamunka/Diplomamunka/Assets/Resources/points_raw_" +
		(pointCloudTestIndex == -1 ? (pointCloudCount == 0 ? "test" : std::to_string(pointCloudIndex)) : std::to_string(pointCloudTestIndex)) + ".txt");
	getline(MyReadFile, myText);
	std::replace(myText.begin(), myText.end(), ',', '.');
	std::stringstream ss(myText);
	std::string _x, _y, _z;
	std::getline(ss, _x, ';');
	std::getline(ss, _y, ';');
	std::getline(ss, _z, ';');
	egoCarPos.push_back({ std::stof(_x),  std::stof(_y),  std::stof(_z) });
	getline(MyReadFile, myText);
	horizontalCount = std::stoi(myText);
	getline(MyReadFile, myText);
	verticalCount = std::stoi(myText);
	getline(MyReadFile, myText);
	std::vector<std::string> stringContainer(std::stoi(myText));
	points[pointCloudIndex].resize(verticalCount * horizontalCount);
	while (getline(MyReadFile, myText))
	{
		std::replace(myText.begin(), myText.end(), ',', '.');
		std::stringstream ss(myText);
		std::string _x, _y, _z, _horizontalIndex, _verticalIndex, _id;
		std::getline(ss, _x, ';');
		if (_x != myText) {
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
			points[pointCloudIndex][getOffset(horizontalIndex, verticalIndex)] = new Point({ x, y, z }, horizontalIndex, verticalIndex, nullptr);
		}
	}
	MyReadFile.close();
}

void groundSegmentation(size_t pointCloudIndex)
{ //TODO point struktúra megvátozott
	double groundLevel = 100;
	for (size_t i = 0; i < points[pointCloudIndex].size(); i++) {
		if (points[pointCloudIndex][i] && points[pointCloudIndex][i]->position.y < groundLevel) groundLevel = points[pointCloudIndex][i]->position.y;
	}

	for (size_t i = 0; i < points[pointCloudIndex].size(); i++) {
		if (points[pointCloudIndex][i] && points[pointCloudIndex][i]->position.y <= groundLevel + 0.1) {
			points[pointCloudIndex][i] = nullptr;
		}
	}
}

void setPointsMarked(std::vector<Point*> points, bool isMarked, bool isMarked2)
{
	for (size_t i = 0; i < points.size(); i++) {
		points[i]->isMarked = isMarked;
		points[i]->isMarked2 = isMarked2;
	}
}

int spikeType(Point* p, int arriveDirection, bool onlyMarkedNeighbours, size_t pointCloudIndex)
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
	Point* neighbourPoints[4] = { points[pointCloudIndex][getOffset(x, y - 1)], points[pointCloudIndex][getOffset(x, y + 1)], points[pointCloudIndex][getOffset(x - 1, y)],
				points[pointCloudIndex][getOffset(x + 1, y)] };

	Point* diagNeighbourPoints[4] = { points[pointCloudIndex][getOffset(x - 1, y - 1)], points[pointCloudIndex][getOffset(x + 1, y - 1)], points[pointCloudIndex][getOffset(x + 1, y + 1)],
		points[pointCloudIndex][getOffset(x - 1, y + 1)] };
	for (size_t j = 0; j < 4; j++) {
		if (neighbourPoints[j] && (j != 0 || y > 0) && (j != 1 || y < verticalCount - 1) && neighbourPoints[j]->plane == p->plane &&
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

bool has2NonSpikeNeighbours(Point* p, bool onlyMarked, size_t pointCloudIndex)
{
	size_t x = p->horizontalIndex;
	size_t y = p->verticalIndex;
	int neighbourCount = 0;

	Point* neighbourPoints[4] = { points[pointCloudIndex][getOffset(x, y - 1)], points[pointCloudIndex][getOffset(x, y + 1)], points[pointCloudIndex][getOffset(x - 1, y)],
				points[pointCloudIndex][getOffset(x + 1, y)] };
	for (size_t j = 0; j < 4; j++) {
		if (neighbourPoints[j] && neighbourPoints[j]->plane == p->plane && (j != 0 || y > 0) && (j != 1 || y < verticalCount - 1) &&
			spikeType(neighbourPoints[j], -1, onlyMarked, pointCloudIndex) > 1 && (!onlyMarked || neighbourPoints[j]->isMarked))
			neighbourCount++;
	}
	return neighbourCount > 1;
}

bool checkIfBridge(Point* p, bool onlyMarked, size_t pointCloudIndex)
{
	size_t x = p->horizontalIndex;
	size_t y = p->verticalIndex;
	int neighbourCount = 0;

	Point* neighbourPoints[4] = { points[pointCloudIndex][getOffset(x, y - 1)], points[pointCloudIndex][getOffset(x, y + 1)], points[pointCloudIndex][getOffset(x - 1, y)],
				points[pointCloudIndex][getOffset(x + 1, y)] };
	for (size_t j = 0; j < 4; j++) {
		if (neighbourPoints[j] && neighbourPoints[j]->plane == p->plane && (j != 0 || y > 0) && (j != 1 || y < verticalCount - 1) &&
			spikeType(neighbourPoints[j], -1, onlyMarked, pointCloudIndex) > 1 && (!onlyMarked || neighbourPoints[j]->isMarked) && has2NonSpikeNeighbours(neighbourPoints[j], onlyMarked,
				pointCloudIndex))
			neighbourCount++;
	}
	bool diagIsNeighbour[4] = { false, false, false, false };
	Point* diagNeighbourPoints[4] = { points[pointCloudIndex][getOffset(x - 1, y - 1)], points[pointCloudIndex][getOffset(x + 1, y - 1)], points[pointCloudIndex][getOffset(x + 1, y + 1)],
		points[pointCloudIndex][getOffset(x - 1, y + 1)] };
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

bool isThereBridge(std::vector<Point*>& planePoints, size_t pointCloudIndex)
{
	std::vector<Point*> newPoints;
	bool theresBridge = false;
	for (auto p : planePoints)
		if (p->plane != nullptr && checkIfBridge(p, false, pointCloudIndex)) {
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
	return Vec3<double>::crossProduct(p1->position - center->position, p2->position - center->position);
}

void calculateNormal(Point* point, size_t pointCloudIndex)
{
	size_t x = point->horizontalIndex;
	size_t y = point->verticalIndex;
	Point* neighbourPoint1 = points[pointCloudIndex][getOffset(x, y - 1)];
	Point* neighbourPoint2 = points[pointCloudIndex][getOffset(x + 1, y)];
	Point* neighbourPoint3 = points[pointCloudIndex][getOffset(x, y + 1)];
	Point* neighbourPoint4 = points[pointCloudIndex][getOffset(x - 1, y)];
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

int areNeighbours(Point* p1, Point* p2, size_t pointCloudIndex)
{
	if (points[pointCloudIndex][getOffset(p1->horizontalIndex + 1, p1->verticalIndex)] == p2)
		return 1;
	if (points[pointCloudIndex][getOffset(p1->horizontalIndex, p1->verticalIndex + 1)] == p2)
		return 2;
	if (points[pointCloudIndex][getOffset(p1->horizontalIndex - 1, p1->verticalIndex)] == p2)
		return 3;
	if (points[pointCloudIndex][getOffset(p1->horizontalIndex, p1->verticalIndex - 1)] == p2)
		return 4;
	return 0;
}

void choosePoints(const Vec3<Point*> planePoints, Plane* basePlane, /*out*/ Plane* plane, size_t pointCloudIndex)
{
	if (basePlane) {
		plane->pointDirections[0] = basePlane->pointDirections[0];
		plane->pointDirections[1] = basePlane->pointDirections[1];
		plane->normal = basePlane->normal;
	}
	else {
		auto normal = Vec3<double>::normalize(Vec3<double>::crossProduct(planePoints.y->position -
			planePoints.x->position, planePoints.z->position - planePoints.x->position));
		Vec3<double> horizontalDirection = { 0,0,0 };
		Vec3<double> verticalDirection = { 0,0,0 };
		Point* neighbours[2] = { planePoints.y, planePoints.z };
		for each (auto neighbour in neighbours) {
			switch (areNeighbours(planePoints.x, neighbour, pointCloudIndex)) {
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
		plane->pointDirections[0] = horizontalDirection;
		plane->pointDirections[1] = verticalDirection;
		plane->normal = normal;
	}
	plane->planePointPos = planePoints.x->position;
	plane->id = currentPlaneId[pointCloudIndex];
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
			Point* neighbourPoints[4] = { points[pointCloudIndex][getOffset(x, y - 1)], points[pointCloudIndex][getOffset(x, y + 1)], points[pointCloudIndex][getOffset(x - 1, y)],
				points[pointCloudIndex][getOffset(x + 1, y)] };
			for (size_t j = 0; j < 4; j++) {
				if (neighbourPoints[j] && (j != 0 || y > 0) && (j != 1 || y < verticalCount - 1) && neighbourPoints[j]->isMarked2) {
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

void findPlanes(size_t pointCloudIndex)
{
	size_t minPointCount = 10;
	size_t counter = 1;
	double normalTreshold = 0.01;
	
	for (size_t i = 0; i < points[pointCloudIndex].size(); i++)
		if (points[pointCloudIndex][i]) {
			points[pointCloudIndex][i]->isMarked = true;
			points[pointCloudIndex][i]->isMarked2 = true;
			calculateNormal(points[pointCloudIndex][i], pointCloudIndex);
		}
	
	
	std::vector<Point*> nextStepPoints;
	for (size_t j = 0; j < points[pointCloudIndex].size(); j++) {
		if (points[pointCloudIndex][j] && points[pointCloudIndex][j]->isMarked) 
		{
			auto start = std::chrono::steady_clock::now();
			nextStepPoints.push_back(points[pointCloudIndex][j]);
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
						Point* neighbourPoints[4] = { points[pointCloudIndex][getOffset(x, y - 1)], points[pointCloudIndex][getOffset(x + 1, y)], points[pointCloudIndex][getOffset(x, y + 1)],
							points[pointCloudIndex][getOffset(x - 1, y)] };
						for (size_t k = 0; k < 4; k++) {
							if (neighbourPoints[k] && (k != 0 || y > 0) && (k != 2 || y < verticalCount - 1) && neighbourPoints[k]->isMarked2) {
								if (neighbourPoints[k]->isMarked) {
									neighbourPoints[k]->isMarked = false;
									tempNextStepPoints.push_back(neighbourPoints[k]);
								}
								normals[k] = neighbourPoints[k]->normal;
							}
						}
						for (size_t k = 0; k < 4; k++) {
							if (normals[k].length() > 0 && normals[(k + 1) % 4].length() > 0 &&
								(normals[k] - normal).length() < normalTreshold && (normals[(k + 1) % 4] - normal).length() < normalTreshold) 
							{
								
								choosePoints({ nextStepPoints[i], neighbourPoints[k], neighbourPoints[(k + 1) % 4] }, nullptr,
									plane, pointCloudIndex);
								
								break;
							}
						}
						if (plane->points.size() > 0)
							break;
					}
				}
				if (plane->points.size() > 0) {
					planes[pointCloudIndex].push_back(plane);
					currentPlaneId[pointCloudIndex]++;
				}
				else
					delete plane;
				nextStepPoints = tempNextStepPoints;
				
			}		
			auto end = std::chrono::steady_clock::now();
			dbgTimeCounter += (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000;
		}		
	}
	
	for (size_t i = 0; i < points[pointCloudIndex].size(); i++) if (points[pointCloudIndex][i]) points[pointCloudIndex][i]->isMarked = false;
	for (size_t i = 0; i < points[pointCloudIndex].size(); i++) if (points[pointCloudIndex][i]) points[pointCloudIndex][i]->isMarked2 = false;
	
	for (size_t i = 0; i < planes[pointCloudIndex].size(); i++) {
		
		int originalSize = planes[pointCloudIndex][i]->points.size();
		
		while (isThereBridge(planes[pointCloudIndex][i]->points, pointCloudIndex)) {}
		
		if (originalSize != planes[pointCloudIndex][i]->points.size() && planes[pointCloudIndex][i]->points.size() > 0) //cutting plane		
		{
			for (size_t j = 0; j < planes[pointCloudIndex][i]->points.size(); j++) planes[pointCloudIndex][i]->points[j]->isMarked2 = true;
			while (true) {
				Plane* plane = new Plane();
				choosePoints({ planes[pointCloudIndex][i]->points[0], nullptr, nullptr }, planes[pointCloudIndex][i], plane, pointCloudIndex);
				if (plane->points.size() < planes[pointCloudIndex][i]->points.size()) {
					for (size_t j = 0; j < planes[pointCloudIndex][i]->points.size(); j++) {
						if (planes[pointCloudIndex][i]->points[j]->plane != planes[pointCloudIndex][i]) {
							planes[pointCloudIndex][i]->points.erase(planes[pointCloudIndex][i]->points.begin() + j);
							j--;
						}
					}
					planes[pointCloudIndex].push_back(plane);
					currentPlaneId[pointCloudIndex]++;
				}
				else {
					for (size_t j = 0; j < plane->points.size(); j++) {
						plane->points[j]->plane = planes[pointCloudIndex][i];
					}
					delete plane;
					break;
				}
			}
		}		
	}	
	for (size_t i = 0; i < planes[pointCloudIndex].size(); i++)
		planes[pointCloudIndex][i]->calculateAvaragePointPos();
}

bool hasNonSpykeNeighbour(size_t x, size_t y, size_t pointCloudIndex)
{
	Point* neighbourPoint = nullptr;
	for (size_t i = 0; i < 4; i++) {
		switch (i) {
		case 0: //to right
		neighbourPoint = points[pointCloudIndex][getOffset(x + 1, y)];
		break;
		case 1: //to down
		neighbourPoint = y == verticalCount - 1 ? nullptr : points[pointCloudIndex][getOffset(x, y + 1)];
		break;
		case 2: //to left
		neighbourPoint = points[pointCloudIndex][getOffset(x - 1, y)];
		break;
		case 3: //to up
		neighbourPoint = y == 0 ? nullptr : points[pointCloudIndex][getOffset(x, y - 1)];
		break;
		default:
		break;
		}
		if (neighbourPoint && spikeType(neighbourPoint, -1, false, pointCloudIndex) == 2)
			return true;
	}
	return false;
}

void findNextPoint(Point*& startPoint, size_t direction, size_t pointCloudIndex, /*out*/
	std::vector<std::pair<Point*, int>>& currentEdge, std::vector<Point*>& spikePoints, size_t dbgPlaneIndex, std::vector<Plane*> dbgPlanes)
{
	//isMarked -- turned off after the edge is complete
	//isMarked2 -- turned off when point added to edge

	Point* currentPoint = nullptr;
	std::pair<Point*, size_t> previousSavedPoint = { nullptr, 0 };
	bool isFirstPoint = true;
	bool comeFromDeadEnd = false;
	bool isPreviousSpike = false;
	bool wasThereNonSpike = false;
	bool isHole = direction == 1;
	while (currentPoint != startPoint || comeFromDeadEnd) {

		if (!currentPoint)
			currentPoint = startPoint;
		if (currentPoint->horizontalIndex == 109 && currentPoint->verticalIndex == 63) {
			std::cout << "asd";
		}
		Point* neighbourPoint = nullptr;
		size_t x = currentPoint->horizontalIndex;
		size_t y = currentPoint->verticalIndex;
		isPreviousSpike = currentPoint != previousSavedPoint.first && spikeType(currentPoint, (!wasThereNonSpike || currentPoint == startPoint) ? -1 : ((direction + 1) % 4), !isHole,
			pointCloudIndex) == 0;
		if (!isPreviousSpike || currentEdge.size() == 0) {
			if (currentPoint->isMarked2) {
				currentEdge.push_back({ currentPoint, direction });
			}
		}
		else spikePoints.push_back(currentPoint);
		if (isHole && isPreviousSpike) {
			currentPoint = previousSavedPoint.first;
			direction = (previousSavedPoint.second + 1) % 4;
			continue;
		}
		if (true || !isHole) currentPoint->isMarked2 = false;
		for (size_t i = 0; i < 4; i++) {
			/*if (checkIfBridge(currentPoint)) {
				i += 2;
				direction = (direction + 2) % 4;
			}*/
			switch (direction) {
			case 0: //to right
			neighbourPoint = points[pointCloudIndex][getOffset(x + 1, y)];
			break;
			case 1: //to down
			neighbourPoint = y == verticalCount - 1 ? nullptr : points[pointCloudIndex][getOffset(x, y + 1)];
			break;
			case 2: //to left
			neighbourPoint = points[pointCloudIndex][getOffset(x - 1, y)];
			break;
			case 3: //to up
			neighbourPoint = y == 0 ? nullptr : points[pointCloudIndex][getOffset(x, y - 1)];
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
				&& (isPreviousSpike || spikeType(neighbourPoint, -1, !isHole, pointCloudIndex) <= 1 || spikeType(currentPoint, direction, !isHole, pointCloudIndex) > 0)) {
				if (isFirstPoint) {
					currentEdge[0].second = (direction + 1) % 4;
					isFirstPoint = false;
				}
				if (!wasThereNonSpike && spikeType(currentPoint, -1, !isHole, pointCloudIndex) > 1) {
					if (currentEdge.size() > 1) {
						auto helper = currentEdge[0];
						currentEdge[0] = currentEdge[1];
						currentEdge[1] = helper;
						startPoint = currentPoint;
						if (!hasNonSpykeNeighbour(startPoint->horizontalIndex, startPoint->verticalIndex, pointCloudIndex)) {
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
				auto neighbourSpikeType = spikeType(neighbourPoint, -1, !isHole, pointCloudIndex);
				if ((!isHole && checkIfBridge(neighbourPoint, true, pointCloudIndex)) || neighbourSpikeType == -1 || neighbourSpikeType == 1 || (wasThereNonSpike &&
					currentEdge.size() > 1 && isPreviousSpike && neighbourSpikeType == 2 && neighbourPoint->isMarked)) {
					auto savedPoint = neighbourSpikeType == 1 ? neighbourPoint : currentPoint;
					if (savedPoint->isMarked2 || isPreviousSpike)
						currentEdge.push_back({ savedPoint, (direction + 3) % 4 });
					if (neighbourSpikeType == -1 || (!isHole && checkIfBridge(neighbourPoint, true, pointCloudIndex))) {
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
				currentPoint = previousSavedPoint.first;
				direction = previousSavedPoint.second;
				std::cout << "INVALID EDGE SEARCH" << std::endl;;
			}
		}
	}
}

bool isEdgePoint(Point* point, size_t pointCloudIndex)
{
	if (!point->isMarked)
		return false;
	size_t x = point->horizontalIndex;
	size_t y = point->verticalIndex;
	Point* neighbourPoint = points[pointCloudIndex][getOffset(x, y - 1)];
	if (y == 0 || !neighbourPoint || !neighbourPoint->isMarked) {
		return true;
	}
	neighbourPoint = points[pointCloudIndex][getOffset(x, y + 1)];
	if (y == verticalCount - 1 || !neighbourPoint || !neighbourPoint->isMarked) {
		return true;
	}
	neighbourPoint = points[pointCloudIndex][getOffset(x - 1, y)];
	if (!neighbourPoint || !neighbourPoint->isMarked) {
		return true;
	}
	neighbourPoint = points[pointCloudIndex][getOffset(x + 1, y)];
	if (!neighbourPoint || !neighbourPoint->isMarked) {
		return true;
	}
	return false;
}

Vec3<double> getRay(size_t hoizontalIndex, size_t verticalIndex)
{
	hoizontalIndex = (hoizontalIndex + horizontalCount) % horizontalCount;
	hoizontalIndex = (hoizontalIndex + horizontalCount) % horizontalCount;
	return  Vec3<double>::normalize({ sin(2 * PI * ((double)hoizontalIndex / horizontalCount)),
				rayAngles.second / 45 + (rayAngles.first - rayAngles.second) / 45 / verticalCount * (double)verticalIndex,
				cos(2 * PI * ((double)hoizontalIndex / horizontalCount)) });
}

void saveEdgeNeighbours(Edge* edge, size_t pointCloudIndex)
{
	const double backgroundPlaneDistTreshold = 0.5;
	auto normal = Vec3<double>::crossProduct(edge->pointsWithDir[0].first->plane->pointDirections[0],
		edge->pointsWithDir[0].first->plane->pointDirections[1]);//edge->pointsWithDir[0].first->plane->normal;
	auto planePointPos = edge->pointsWithDir[0].first->plane->planePointPos;
	for (size_t i = 0; i < edge->pointsWithDir.size(); i++) {
		auto point = edge->pointsWithDir[i].first;
		size_t x = point->horizontalIndex;
		size_t y = point->verticalIndex;
		Point* neighbourPoints[4] = { points[pointCloudIndex][getOffset(x + 1, y)], points[pointCloudIndex][getOffset(x, y + 1)], points[pointCloudIndex][getOffset(x - 1, y)],
			points[pointCloudIndex][getOffset(x, y - 1)] };
		Vec3<double> neighbourRays[4] = { getRay(x + 1, y), getRay(x, y + 1), getRay(x - 1, y), getRay(x, y - 1) };
		for (size_t j = 0; j < 4; j++) {
			if ((y > 0 || j != 3) && (y < verticalCount - 1 || j != 1) && (!neighbourPoints[j] || neighbourPoints[j]->plane !=
				edge->pointsWithDir[i].first->plane)) {
				double d = Vec3<double>::dot_product(normal, planePointPos);
				if (Vec3<double>::dot_product(normal, neighbourRays[j]) == 0) { // No intersection, the line is parallel to the plane
					continue;
				}
				float x = (Vec3<double>::dot_product(normal, planePointPos) - Vec3<double>::dot_product(normal, egoCarPos[pointCloudIndex])) /
					Vec3<double>::dot_product(normal, neighbourRays[j]);
				auto intersection = egoCarPos[pointCloudIndex] + neighbourRays[j] * x;
				if (!neighbourPoints[j] || ((intersection - egoCarPos[pointCloudIndex]).length() + backgroundPlaneDistTreshold) <
					(neighbourPoints[j]->position - egoCarPos[pointCloudIndex]).length()) {
					edge->closestNeighbourPoints.push_back({ intersection, {0,0,0} });
				}
			}
		}
	}
}

void findEdgePoints(size_t pointCloudIndex)
{
	for (size_t i = 0; i < planes[pointCloudIndex].size(); i++) {
		std::vector<Point*> outerConnectedEdgePoints;
		std::vector<Point*> holeConnectedEdgePoints;
		std::vector<Point*> edgePointsInPlane;
		for (size_t j = 0; j < planes[pointCloudIndex][i]->points.size(); j++) planes[pointCloudIndex][i]->points[j]->isMarked = true;
		for (size_t j = 0; j < planes[pointCloudIndex][i]->points.size(); j++) planes[pointCloudIndex][i]->points[j]->isMarked2 = true;
		for (size_t j = 0; j < planes[pointCloudIndex][i]->points.size(); j++) if (isEdgePoint(planes[pointCloudIndex][i]->points[j], pointCloudIndex))
			edgePointsInPlane.push_back(planes[pointCloudIndex][i]->points[j]);
		while (edgePointsInPlane.size() > 0) {
			Edge* currentEdge = new Edge();
			edges[pointCloudIndex].push_back(currentEdge);
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
			if (spikeType(startPoint, -1, true, pointCloudIndex) == -1) {
				size_t x = startPoint->horizontalIndex;
				size_t y = startPoint->verticalIndex;
				startPoint->plane = nullptr;
				startPoint->isMarked = false;
				startPoint->isMarked2 = false;
				for (size_t j = 0; j < planes[pointCloudIndex][i]->points.size(); j++) {
					if (planes[pointCloudIndex][i]->points[j] == startPoint) {
						planes[pointCloudIndex][i]->points[j]->plane = nullptr;
						planes[pointCloudIndex][i]->points.erase(planes[pointCloudIndex][i]->points.begin() + j);
						break;
					}
				}
				startPoint = points[pointCloudIndex][getOffset(x + 1, y)];
				minVerticalCoord = y;
				for (size_t j = 1; j < edgePointsInPlane.size(); j++) {
					if (edgePointsInPlane[j]->horizontalIndex == x + 1 && edgePointsInPlane[j]->verticalIndex < minVerticalCoord) {
						minVerticalCoord = edgePointsInPlane[j]->verticalIndex;
						startPoint = edgePointsInPlane[j];
					}
				}
			}
			else if (startPoint->verticalIndex > 0 && points[pointCloudIndex][getOffset(startPoint->horizontalIndex, startPoint->verticalIndex - 1)] &&
				points[pointCloudIndex][getOffset(startPoint->horizontalIndex, startPoint->verticalIndex - 1)]->plane == startPoint->plane &&
				points[pointCloudIndex][getOffset(startPoint->horizontalIndex - 1, startPoint->verticalIndex - 1)] &&
				points[pointCloudIndex][getOffset(startPoint->horizontalIndex - 1, startPoint->verticalIndex - 1)]->plane == startPoint->plane &&
				points[pointCloudIndex][getOffset(startPoint->horizontalIndex - 1, startPoint->verticalIndex)] &&
				points[pointCloudIndex][getOffset(startPoint->horizontalIndex - 1, startPoint->verticalIndex)]->plane == startPoint->plane) {
				direction = 1;
				currentEdge->isHole = true;
			}

			if (i == 19 && planes[pointCloudIndex][i]->edges.size() == 0) {
				std::cout << "asd";
				//return;
			}
			std::vector<Point*> spikePoints;
			setPointsMarked(currentEdge->isHole ? holeConnectedEdgePoints : outerConnectedEdgePoints, false, false);
			setPointsMarked(currentEdge->isHole ? outerConnectedEdgePoints : holeConnectedEdgePoints, true, false);
			findNextPoint(startPoint, direction, pointCloudIndex, currentEdge->pointsWithDir, spikePoints, i, planes[pointCloudIndex]);
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
					currentEdge->pointsWithDir[k].first->outlineId = currentOutlineId[pointCloudIndex];
				}
				currentOutlineId[pointCloudIndex]++;
				currentEdge->startPoint = startPoint;
				planes[pointCloudIndex][i]->edges.push_back(currentEdge);
				saveEdgeNeighbours(currentEdge, pointCloudIndex);
			}
		}
		for (size_t j = 0; j < planes[pointCloudIndex][i]->points.size(); j++) planes[pointCloudIndex][i]->points[j]->isMarked = false;
		for (size_t j = 0; j < planes[pointCloudIndex][i]->points.size(); j++) planes[pointCloudIndex][i]->points[j]->isMarked2 = false;
	}
}

const double newPointAcceptTreshold = 0.95;
const double inf = 1000000;

bool isStraightPoint(size_t pointIndex, Edge* edge, size_t& previousNeighbourCount,
	Vec3<double>& straigthDir, size_t pointCloudIndex)
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
	Point* neighbourPoints[4] = { points[pointCloudIndex][getOffset(x, y - 1)], points[pointCloudIndex][getOffset(x, y + 1)], points[pointCloudIndex][getOffset(x - 1, y)],
		points[pointCloudIndex][getOffset(x + 1, y)] };
	for (size_t i = 0; i < 4; i++) {
		if ((y > 0 || i != 0) && (y < verticalCount - 1 || i != 1) && neighbourPoints[i] && neighbourPoints[i]->plane &&
			neighbourPoints[i]->plane == plane) {
			neighbourCount++;
			isNeighbour[i] = true;
			if (neighbourPoints[i]->outlineId > 0)
				neighbourEdgeCount++;
		}
	}
	previousNeighbourCount = neighbourCount;
	if (pointIndex > 0 && areNeighbours(pointIndex < (edge->pointsWithDir.size() - 1) ? edge->pointsWithDir[pointIndex + 1].first : edge->startPoint,
		point, pointCloudIndex) == 0 && neighbourCount < 3) { //deadend
		auto previousPoint = edge->pointsWithDir[pointIndex - 1];
		auto previousPreviousPoint = pointIndex > 1 ? edge->pointsWithDir[pointIndex - 2] : edge->pointsWithDir[edge->pointsWithDir.size() - 2
			+ pointIndex];
		straigthDir = { 0,0,0 };
		if (areNeighbours(previousPoint.first, previousPreviousPoint.first, pointCloudIndex) > 0 && ((previousPoint.first->verticalIndex ==
			previousPreviousPoint.first->verticalIndex && previousPoint.first->horizontalIndex == point->horizontalIndex &&
			previousPoint.first->verticalIndex != point->verticalIndex) || (previousPoint.first->horizontalIndex ==
				previousPreviousPoint.first->horizontalIndex && previousPoint.first->verticalIndex == point->verticalIndex &&
				previousPoint.first->horizontalIndex != point->horizontalIndex))) //if curve swap with previous
		{
			point->isCorner = true;
			edge->pointsWithDir[pointIndex - 1] = edge->pointsWithDir[pointIndex];
			edge->pointsWithDir[pointIndex] = previousPoint;
			if (pointIndex > 1) edge->pointsWithDir[pointIndex - 2].first->isCorner = true;
			return true;
		}
		return false;
	}
	if (neighbourCount == 3 && (neighbourEdgeCount == 2 && (pointIndex == edge->pointsWithDir.size() - 1 ||
		areNeighbours(point, edge->pointsWithDir[pointIndex + 1].first, pointCloudIndex)
				> 0))) {
		if (straigthDir.length() == 0) {
			if (pointIndex > 0)
				straigthDir = Vec3<double>::normalize(point->position - edge->pointsWithDir[pointIndex - 1].first->position);
		}
		else {
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

void findCorners(size_t pointCloudIndex)
{
	for (size_t k = 0; k < planes[pointCloudIndex].size(); k++) {
		for (size_t i = 0; i < planes[pointCloudIndex][k]->edges.size(); i++) {
			size_t previousNeighbourCount = 0;
			Vec3<double> straightDir = { 0,0,0 };
			for (size_t j = 0; j < planes[pointCloudIndex][k]->edges[i]->pointsWithDir.size(); j++) {
				if (!isStraightPoint(j, planes[pointCloudIndex][k]->edges[i], previousNeighbourCount, straightDir, pointCloudIndex)) {
					planes[pointCloudIndex][k]->edges[i]->pointsWithDir[j].first->isCorner = true;
				}
			}
		}
	}
}

Point* createNewPoint(Vec3<double> newPointPos, Point* point, std::vector<Point*> neighbours, size_t addedCount, size_t pointCloudIndex, bool createBeforePoint = false,
	bool isCornerPoint = false)
{
	double deleteDurroundingCornersTreshold = 0.1;
	Point* newPoint = new Point(newPointPos, neighbours[0]->horizontalIndex, isCornerPoint ? verticalCount + 1 : verticalCount, point->plane);
	addedPoints[pointCloudIndex].push_back(newPoint);
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

Point* addNewPoint(Point* point, Point*& neighbour, Plane* plane, size_t addedCount, size_t neighbourIndex, size_t pointCloudIndex)
{
	if (point->createdNeighbourPoints[neighbourIndex] != nullptr) { //created by other plane
		auto createdNeighbour = point->createdNeighbourPoints[neighbourIndex];
		createdNeighbour->isCorner = true;
		for (size_t j = 0; j < point->plane->edges.size(); j++) {
			if (point->plane->edges[j]->pointsWithDir[0].first->outlineId == point->outlineId) {
				for (size_t k = 0; k < point->plane->edges[j]->pointsWithDir.size(); k++) {
					if (point->plane->edges[j]->pointsWithDir[k].first == point) {
						for (size_t l = 0; l < 4; l++) {
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
	switch (areNeighbours(point, neighbour, pointCloudIndex)) {
	case 1:
	dir = point->plane->pointDirections[0];
	break;
	case 2:
	dir = point->plane->pointDirections[1];
	break;
	case 3:
	dir = point->plane->pointDirections[0] * -1;
	break;
	case 4:
	dir = point->plane->pointDirections[1] * -1;
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
	switch (areNeighbours(neighbour, point, pointCloudIndex)) {
	case 1:
	neighbourDir = plane->pointDirections[0];
	break;
	case 2:
	neighbourDir = plane->pointDirections[1];
	break;
	case 3:
	neighbourDir = plane->pointDirections[0] * -1;
	break;
	case 4:
	neighbourDir = plane->pointDirections[1] * -1;
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
	newPoint = createNewPoint(newPos, point, { neighbour }, addedCount, pointCloudIndex);
	point->createdNeighbourPoints[neighbourIndex] = newPoint;
	for (size_t i = 0; i < neighbour->neighbourPlaneNeighbours.size(); i++) {
		if (neighbour->neighbourPlaneNeighbours[i] == point) {
			newNeighbourPoint = createNewPoint(newPos, neighbour, { point }, 0, false);
			neighbour->createdNeighbourPoints[i] = newNeighbourPoint;
			break;
		}
	}
	newPoint->neighbourPlaneNeighbours[1] = newNeighbourPoint;
	newNeighbourPoint->neighbourPlaneNeighbours[1] = newPoint;
	return newPoint;
}

void findPlaneConnections(size_t pointCloudIndex)
{
	for (size_t i = 0; i < planes[pointCloudIndex].size(); i++) {
		for (size_t j = 0; j < planes[pointCloudIndex][i]->edges.size(); j++) {
			for (size_t k = 0; k < planes[pointCloudIndex][i]->edges[j]->pointsWithDir.size(); k++) {
				Point* point = planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first;
				if (point->horizontalIndex == 339 && point->verticalIndex == 14) {
					//std::cout << "asd";
				}
				int direction = planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].second;
				size_t x = point->horizontalIndex;
				size_t y = point->verticalIndex;
				Point* neighbourPoints[4] = { points[pointCloudIndex][getOffset(x + 1, y)], points[pointCloudIndex][getOffset(x, y + 1)], points[pointCloudIndex][getOffset(x - 1, y)],
					points[pointCloudIndex][getOffset(x, y - 1)] };
				for (size_t i = 0; i < 4; i++) {
					if ((y > 0 || direction != 3) && (y < verticalCount - 1 || direction != 1) && neighbourPoints[direction]) {
						point->neighbourPlaneNeighbours[i] = neighbourPoints[direction];
					}
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

	createNewPoint(cornerPoint, point1[0], { point1[2], point2[2] }, 0, false, true);
	createNewPoint(cornerPoint, point1[1], { point }, 0, true, true);
	createNewPoint(cornerPoint, point2[1], { point }, 0, false, true);
}

void connectPlanes(size_t pointCloudIndex)
{
	std::vector<Point*> createdPoints;
	for (size_t i = 0; i < planes[pointCloudIndex].size(); i++) {
		for (size_t j = 0; j < planes[pointCloudIndex][i]->edges.size(); j++) {
			for (size_t k = 0; k < planes[pointCloudIndex][i]->edges[j]->pointsWithDir.size(); k++) {
				auto point = planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first;
				if (point->horizontalIndex == 666 && point->verticalIndex == 29) {
					//std::cout << "asd";
				}
				if (point->verticalIndex == verticalCount) {
					continue;
				}
				size_t addedCount = 0;
				if (point->isCorner) {
					for (size_t l = 0; l < point->neighbourPlaneNeighbours.size(); l++) {
						Point* newPoint = nullptr;
						auto neighbourPoint = point->neighbourPlaneNeighbours[l];
						if (neighbourPoint) {
							if (neighbourPoint->plane && neighbourPoint->outlineId > 0 && neighbourPoint->outlineId != point->outlineId &&
								neighbourPoint->plane != point->plane) {
								newPoint = addNewPoint(point, neighbourPoint, neighbourPoint->plane, addedCount, l, pointCloudIndex);
								if (newPoint) {
									if (k == 0 && l == 0) planes[pointCloudIndex][i]->edges[planes[pointCloudIndex][i]->edges.size() - 1]->wasFirstGenerated = true;
									planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->isCorner = false;
									createdPoints.push_back(newPoint);
									addedCount++;
								}
								else {
									planes[pointCloudIndex][i]->edges[j]->pointsWithDir.insert(planes[pointCloudIndex][i]->edges[j]->pointsWithDir.begin() + k + 1 + addedCount, { nullptr, -1 });
									addedCount++;
								}
							}
						}
						if (!neighbourPoint || !neighbourPoint->plane) {
							planes[pointCloudIndex][i]->edges[j]->pointsWithDir.insert(planes[pointCloudIndex][i]->edges[j]->pointsWithDir.begin() + k + 1 + addedCount, { nullptr, -1 });
							addedCount++;
						}
					}
					k += addedCount;
				}
			}
		}
	}
}

void createCorners(size_t pointCloudIndex)
{
	const double normalDiffTreshold = 0.1;
	for (size_t i = 0; i < planes[pointCloudIndex].size(); i++) {
		for (size_t j = 0; j < planes[pointCloudIndex][i]->edges.size(); j++) {
			for (size_t k = 0; k < planes[pointCloudIndex][i]->edges[j]->pointsWithDir.size(); k++) {
				auto point = planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first;
				if (point) {
					for (size_t l = 0; l < planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->createdNeighbourPoints.size(); l++) {
						auto createdPoint1 = planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->createdNeighbourPoints[l];
						auto createdPoint2 = planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->createdNeighbourPoints[l == 3 ? 0 : (l + 1)];
						if (createdPoint1 && createdPoint2 && createdPoint1->neighbourPlaneNeighbours[1]->plane !=
							createdPoint2->neighbourPlaneNeighbours[1]->plane
							&& (planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->plane->normal -
								createdPoint1->neighbourPlaneNeighbours[1]->plane->normal).length() > normalDiffTreshold &&
							(planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->plane->normal -
								createdPoint2->neighbourPlaneNeighbours[1]->plane->normal).length() > normalDiffTreshold &&
							(createdPoint1->neighbourPlaneNeighbours[1]->plane->normal -
								createdPoint2->neighbourPlaneNeighbours[1]->plane->normal).length() > normalDiffTreshold) {
							createPlaneCorner(planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first, { createdPoint1, createdPoint1->neighbourPlaneNeighbours[1],
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

void filterEdgePoints(size_t pointCloudIndex)
{
	std::vector<std::vector<std::vector< std::pair<bool, std::pair<size_t, size_t>>>>> cornersToDelete;
	for (size_t i = 0; i < planes[pointCloudIndex].size(); i++) {
		cornersToDelete.push_back({});
		for (size_t j = 0; j < planes[pointCloudIndex][i]->edges.size(); j++) {
			cornersToDelete[i].push_back({});
			for (size_t k = 0; k < planes[pointCloudIndex][i]->edges[j]->pointsWithDir.size(); k++) {
				if (planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first) {
					if (planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->verticalIndex >= verticalCount) {
						for (size_t l = 0; l < planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->neighbourPlaneNeighbours.size(); l++) {
							if (planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->neighbourPlaneNeighbours[l] &&
								planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->neighbourPlaneNeighbours[l]->verticalIndex < verticalCount)
								cornersToDelete[i][j].push_back({ planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->verticalIndex > verticalCount,
									{ k,  planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->neighbourPlaneNeighbours[l]->plane->id } });
						}
					}
				}
				else {
					cornersToDelete[i][j].push_back({ false, { 0, 0 } });
					planes[pointCloudIndex][i]->edges[j]->pointsWithDir.erase(planes[pointCloudIndex][i]->edges[j]->pointsWithDir.begin() + k);
					k--;
				}
			}
			for (int k = 0; k < cornersToDelete[i][j].size(); k++) {
				if (k < ((int)cornersToDelete[i][j].size() - 2) && cornersToDelete[i][j][k + 1].first && cornersToDelete[i][j][k + 2].first) {
					cornersToDelete[i][j][k + 1].second = { cornersToDelete[i][j][k + 1].second.first, cornersToDelete[i][j][k].second.second };
					size_t index = k + 2;
					while (index < cornersToDelete[i][j].size() && cornersToDelete[i][j][index].first) {
						cornersToDelete[i][j].erase(cornersToDelete[i][j].begin() + index);
					}
					cornersToDelete[i][j].insert(cornersToDelete[i][j].begin() + index, { true, {cornersToDelete[i][j][k + 1].second.first,
						index < cornersToDelete[i][j].size() ? cornersToDelete[i][j][index].second.second : cornersToDelete[i][j][0].second.second} });
				}
			}
			int newPointStartIndex = planes[pointCloudIndex][i]->edges[j]->wasFirstGenerated ? 1 : 0;
			if ((planes[pointCloudIndex][i]->edges[j]->pointsWithDir[newPointStartIndex + 1].first->horizontalIndex < planes[pointCloudIndex][i]->edges[j]->startPoint->horizontalIndex ||
				planes[pointCloudIndex][i]->edges[j]->pointsWithDir[newPointStartIndex + 1].first->verticalIndex < planes[pointCloudIndex][i]->edges[j]->startPoint->verticalIndex) &&
				planes[pointCloudIndex][i]->edges[j]->pointsWithDir[newPointStartIndex + 1].first->createdNeighbourPoints[0])
				newPointStartIndex++;
			int newPointEndIndex = newPointStartIndex;
			if (currentFrame == 4 && i == 0) {
				std::cout << "asd";
			}
			Point* endPoint = nullptr;
			while (newPointEndIndex < (int)cornersToDelete[i][j].size() - 1) {
				if (cornersToDelete[i][j][newPointStartIndex].second.second > 0) {
					while (newPointEndIndex + 1 < cornersToDelete[i][j].size()
						&& cornersToDelete[i][j][newPointStartIndex].second.second == cornersToDelete[i][j][newPointEndIndex + 1].second.second) {
						newPointEndIndex++;
					}
					for (size_t k = cornersToDelete[i][j][newPointStartIndex].second.first + 1; k <
						cornersToDelete[i][j][newPointEndIndex].second.first; k++) {
						planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->isCorner = false;
					}

					auto startPoint = planes[pointCloudIndex][i]->edges[j]->pointsWithDir[cornersToDelete[i][j][newPointStartIndex].second.first].first;
					for (size_t k = 0; k < 2; k++) {
						if (endPoint && (startPoint->position - endPoint->position).length() < 0.1) {
							if (startPoint->verticalIndex == verticalCount + 1 && endPoint->verticalIndex != verticalCount + 1)
								endPoint->isCorner = false;
							else if (endPoint->verticalIndex == verticalCount + 1 && startPoint->verticalIndex != verticalCount + 1)
								startPoint->isCorner = false;
						}
						endPoint = planes[pointCloudIndex][i]->edges[j]->pointsWithDir[cornersToDelete[i][j][newPointEndIndex].second.first].first;
					}
					if (newPointEndIndex == cornersToDelete[i][j].size() - 1 && newPointStartIndex != newPointEndIndex &&
						planes[pointCloudIndex][i]->edges[j]->wasFirstGenerated && cornersToDelete[i][j][0].second.second ==
						cornersToDelete[i][j][newPointStartIndex].second.second)
						planes[pointCloudIndex][i]->edges[j]->pointsWithDir[cornersToDelete[i][j][cornersToDelete[i][j].size() - 1].second.first].first->isCorner = false;
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

void egoCarSegmentation(size_t pointCloudIndex)
{
	for (size_t i = 0; i < points[pointCloudIndex].size(); i++) {
		if (points[pointCloudIndex][i] && points[pointCloudIndex][i]->position.x <= 1 && points[pointCloudIndex][i]->position.x >= -1 &&
			points[pointCloudIndex][i]->position.y <= 1 && points[pointCloudIndex][i]->position.y >= -1 &&
			points[pointCloudIndex][i]->position.z <= 2.5 + egoCarPos[pointCloudIndex].z && points[pointCloudIndex][i]->position.z >= -2.5 + egoCarPos[pointCloudIndex].z)
			points[pointCloudIndex][i] = nullptr;
	}
}

double angleOfVectors(Vec3<double> v1, Vec3<double> v2, bool isBackward)
{
	auto v1Angle = atan2(v1.x, v1.y) / PI;
	auto v2Angle = atan2(v2.x, v2.y) / PI;
	auto angle = abs(v1Angle - v2Angle);
	if ((!isBackward && v1Angle < v2Angle) || (isBackward && v1Angle > v2Angle)) angle = 2 - angle;
	return angle * 180;
}

__host__ __device__
Vec3<double> intersectionOfLines(Vec3<double> p1, Vec3<double> p2, Vec3<double> q1, Vec3<double> q2, size_t& onEdgeType, double& distanceRatioOnEdge)
{
	const double acceptTreshold = 0.0000001;
	auto dir1 = p2 - p1;
	auto dir2 = q2 - q1;
	double R = (p1.y * dir1.x + q1.x * dir1.y - p1.x * dir1.y - q1.y * dir1.x) / (dir2.y * dir1.x - dir1.y * dir2.x);
	Vec3<double> intersectionPos = { q1.x + R * dir2.x, q1.y + R * dir2.y, 0 };
	auto side1Length = (p2 - p1).length() + acceptTreshold;
	auto side2Length = (q2 - q1).length() + acceptTreshold;
	if ((intersectionPos - p1).length() < side1Length && (intersectionPos - p2).length() < side1Length &&
		(intersectionPos - q1).length() < side2Length && (intersectionPos - q2).length() < side2Length) {
		if ((intersectionPos - p1).length() < acceptTreshold)
			onEdgeType = 2;
		else if ((intersectionPos - p2).length() < acceptTreshold)
			onEdgeType = 3;
		else if ((intersectionPos - q1).length() < acceptTreshold)
			onEdgeType = 4;
		else if ((intersectionPos - q2).length() < acceptTreshold)
			onEdgeType = 5;
		else onEdgeType = 1;
		distanceRatioOnEdge = (intersectionPos - p1).length() / (p2 - p1).length();
	}
	return intersectionPos;
}

size_t isPointInsidePolygon(std::vector<Point*>polygon, Vec3<double> point, std::pair<double, double> xBounds, std::pair<double, double> yBounds,
	bool checkOnEdge = false, double onEdgetreshold = 0.0000001)
{
	// 0 - outside
	// 1 - inside
	// 2 - onEdge
	size_t notInUse = 0;
	double notInUseRatio = 0;
	if (point.x > xBounds.first && point.x < xBounds.second &&
		point.y > yBounds.first && point.y < yBounds.second) {
		int rigthCounter = 0;
		for (size_t i = 0; i < polygon.size(); i++) {
			auto p1 = polygon[i]->projected2DPosition;
			auto p2 = polygon[(i + 1) % polygon.size()]->projected2DPosition;
			if ((p1.x < point.x && p2.x < point.x) || (p1.y > point.y && p2.y > point.y) || (p1.y < point.y && p2.y < point.y) || (p1.y == p2.y))
				continue;
			else if (point == p1)
				return false;
			else {
				/*auto smallerAngle = (p1.x < p2.x ? p1 - point : (p2 - point));
				auto biggerAngle = (p1.x < p2.x ? p2 - point : (p1 - point));
				if((p1.x > point.x && p2.x > point.x) || angleOfVectors(smallerAngle, biggerAngle, smallerAngle.y > point.y || biggerAngle.y < point.y)
					< 180)*/
				auto intersection = intersectionOfLines(point, point + Vec3<double>({ 1,0,0 }), p1, p2, notInUse, notInUseRatio);
				if (abs(intersection.x - point.x) < onEdgetreshold && checkOnEdge)
					return 2;
				if (intersection.x <= point.x)
					continue;
				else {
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

void changeBaseTo2D(Edge* edge, std::pair<Vec3<double>, Vec3<double>> newBase = { {0,0,0}, {0,0,0} })
{
	edge->xBounds2D = { 100000, -100000 };
	edge->yBounds2D = { 100000, -100000 };
	auto normal = newBase.first == Vec3<double>({ 0,0,0 }) ? edge->pointsWithDir[0].first->plane->normal : newBase.first;
	auto x = newBase.first == Vec3<double>({ 0,0,0 }) ? edge->pointsWithDir[0].first->plane->pointDirections[0] : newBase.second;
	x = Vec3<double>::normalize(x - normal * Vec3<double>::dot_product(x, normal));
	auto y = Vec3<double>::crossProduct(x, normal);
	for (size_t i = 0; i < edge->pointsWithDir.size(); i++) {
		edge->pointsWithDir[i].first->projected2DPosition = { Vec3<double>::dot_product(edge->pointsWithDir[i].first->position, x),
			Vec3<double>::dot_product(edge->pointsWithDir[i].first->position, y), 0 };
		auto pos2D = edge->pointsWithDir[i].first->projected2DPosition;
		if (pos2D.x < edge->xBounds2D.first) edge->xBounds2D.first = pos2D.x;
		if (pos2D.x > edge->xBounds2D.second) edge->xBounds2D.second = pos2D.x;
		if (pos2D.y < edge->yBounds2D.first) edge->yBounds2D.first = pos2D.y;
		if (pos2D.y > edge->yBounds2D.second) edge->yBounds2D.second = pos2D.y;
	}
	for (size_t i = 0; i < edge->closestNeighbourPoints.size(); i++) {
		edge->closestNeighbourPoints[i].second = { Vec3<double>::dot_product(edge->closestNeighbourPoints[i].first, x),
			Vec3<double>::dot_product(edge->closestNeighbourPoints[i].first, y), 0 };
	}
}

void deleteSelfIntersections(std::vector<Plane*> planes)
{
	double notInUseRatio = 0;
	for (size_t k = 0; k < planes.size(); k++) {
		for (size_t l = 0; l < planes[k]->edges.size(); l++) {
			for (int i = 0; i < planes[k]->edges[l]->pointsWithDir.size(); i++) {
				for (int j = (i == (planes[k]->edges[l]->pointsWithDir.size() - 1) ? 1 : 0); j < i - 1; j++) {
					size_t onEdgeType = 0;
					auto intersection = intersectionOfLines(planes[k]->edges[l]->pointsWithDir[j].first->projected2DPosition,
						planes[k]->edges[l]->pointsWithDir[j + 1].first->projected2DPosition,
						planes[k]->edges[l]->pointsWithDir[i].first->projected2DPosition, planes[k]->edges[l]->pointsWithDir[(i + 1) %
						planes[k]->edges[l]->pointsWithDir.size()].first->projected2DPosition,
						onEdgeType, notInUseRatio);
					if (onEdgeType > 0) {
						double routeLength1 = 0;
						for (size_t m = j + 1; m < i; m++) {
							routeLength1 += (planes[k]->edges[l]->pointsWithDir[m + 1].first->projected2DPosition -
								planes[k]->edges[l]->pointsWithDir[m].first->projected2DPosition).length();
						}
						double routeLength2 = 0;
						size_t m = (i + 1) % planes[k]->edges[l]->pointsWithDir.size();
						while (m != j) {
							routeLength2 += (planes[k]->edges[l]->pointsWithDir[(m + 1) % planes[k]->edges[l]->pointsWithDir.size()].first->projected2DPosition -
								planes[k]->edges[l]->pointsWithDir[m].first->projected2DPosition).length();
							m = (m + 1) % planes[k]->edges[l]->pointsWithDir.size();
						}
						if (routeLength1 < routeLength2) {
							if (planes[k]->edges[l]->pointsWithDir[j + 1].first->verticalIndex == verticalCount + 1) {
								planes[k]->edges[l]->pointsWithDir[i].first->isCorner = false;
								planes[k]->edges[l]->pointsWithDir.erase(planes[k]->edges[l]->pointsWithDir.begin() + i);
							}
							else {
								planes[k]->edges[l]->pointsWithDir[j + 1].first->isCorner = false;
								planes[k]->edges[l]->pointsWithDir.erase(planes[k]->edges[l]->pointsWithDir.begin() + j + 1);
							}
						}
						else {
							if (planes[k]->edges[l]->pointsWithDir[j].first->verticalIndex == verticalCount + 1) {
								planes[k]->edges[l]->pointsWithDir[(i + 1) % planes[k]->edges[l]->pointsWithDir.size()].first->isCorner = false;
								planes[k]->edges[l]->pointsWithDir.erase(planes[k]->edges[l]->pointsWithDir.begin() + ((i + 1) % planes[k]->edges[l]->pointsWithDir.size()));
							}
							else {
								planes[k]->edges[l]->pointsWithDir[j].first->isCorner = false;
								planes[k]->edges[l]->pointsWithDir.erase(planes[k]->edges[l]->pointsWithDir.begin() + j);
							}
						}
						i--;
						break;
					}
				}
			}
		}
	}
}

void deleteTigthEdges(size_t pointCloudIndex)
{
	return;
	for (size_t k = 0; k < planes[pointCloudIndex].size(); k++) {
		for (size_t l = 0; l < planes[pointCloudIndex][k]->edges.size(); l++) {
			if (isEdgeTooTigth(planes[pointCloudIndex][k]->edges[l])) {
				planes[pointCloudIndex][k]->edges.erase(planes[pointCloudIndex][k]->edges.begin() + l);
				l--;
			}
		}
		if (planes[pointCloudIndex][k]->edges.size() == 0) 
		{
			planes[pointCloudIndex].erase(planes[pointCloudIndex].begin() + k);
			k--;
		}
	}
}

bool isClockwise(std::vector<Point*>& points)
{
	double angleSum = 0;
	for (size_t i = 0; i < points.size(); i++) {
		auto  a = angleOfVectors(points[(i + points.size() - 1) % points.size()]->projected2DPosition - points[i]->projected2DPosition,
			points[(i + 1) % points.size()]->projected2DPosition - points[i]->projected2DPosition, false);
		angleSum += angleOfVectors(points[(i + points.size() - 1) % points.size()]->projected2DPosition - points[i]->projected2DPosition,
			points[(i + 1) % points.size()]->projected2DPosition - points[i]->projected2DPosition, false);
	}
	return angleSum < (double)points.size() * 360.0 / 2;
}

void extract2DPolygon(size_t pointCloudIndex)
{
	for (size_t i = 0; i < planes[pointCloudIndex].size(); i++) {
		for (size_t j = 0; j < planes[pointCloudIndex][i]->edges.size(); j++) {
			for (size_t k = 0; k < planes[pointCloudIndex][i]->edges[j]->pointsWithDir.size(); k++) {
				if (!planes[pointCloudIndex][i]->edges[j]->pointsWithDir[k].first->isCorner) {
					planes[pointCloudIndex][i]->edges[j]->pointsWithDir.erase(planes[pointCloudIndex][i]->edges[j]->pointsWithDir.begin() + k);
					k--;
				}
			}
			changeBaseTo2D(planes[pointCloudIndex][i]->edges[j]);
		}
	}
}

bool isDesiredEdge(std::vector<Point*> savedEdge, std::vector<Point*> newEdge, std::pair<double, double> newEdgeXBounds2D, std::pair<double, double> newEdgeYBounds2D,
	size_t vertexIndex, bool isInner)
{
	if (savedEdge[vertexIndex]->isMarked2) {
		auto isMiddlePointDesired = isPointInsidePolygon(newEdge, (savedEdge[vertexIndex]->projected2DPosition +
			savedEdge[(vertexIndex + 1)
			% savedEdge.size()]->projected2DPosition) / 2, newEdgeXBounds2D, newEdgeYBounds2D, true, 0.00001) == isInner;
		return isMiddlePointDesired;
	}
	return false;
}

Point* decideIfDesiredEdgeGood(Edge* savedEdge, Edge* newEdge, Point* currentPoint, std::vector<Point*> savedEdgePoints, std::vector<Point*> newEdgePoints,
	bool isOnSavedEdge, bool checkForInsidePoints, std::vector<std::pair<Vec3<double>, Vec3<double>>> allNeighbours, /*out*/ std::vector<Point*>& pointsOnDesiredEdge)
{
	std::pair<double, double> xBounds = { 100000, -1000000 };
	std::pair<double, double> yBounds = { 100000, -1000000 };
	std::vector<Point*> pointsOnAddedPolygonPart;
	Point* endPoint = nullptr;
	size_t currentIndex = 0;
	bool isForward = true;
	do {
		for (size_t i = 0; i < (isOnSavedEdge ? newEdgePoints : savedEdgePoints).size(); i++) {
			if ((isOnSavedEdge ? newEdgePoints : savedEdgePoints)[i] == currentPoint) {
				currentIndex = i;
				currentPoint = (isOnSavedEdge ? newEdgePoints : savedEdgePoints)[currentIndex];
				break;
			}
		}
		isOnSavedEdge = !isOnSavedEdge;
		do {
			if (isForward && pointsOnDesiredEdge.size() == pointsOnAddedPolygonPart.size()) {
				pointsOnDesiredEdge.push_back(currentPoint);
				currentPoint->isMarked = false;
			}
			pointsOnAddedPolygonPart.push_back(currentPoint);
			auto pos2D = currentPoint->projected2DPosition;
			if (pos2D.x < xBounds.first) xBounds.first = pos2D.x;
			if (pos2D.x > xBounds.second) xBounds.second = pos2D.x;
			if (pos2D.y < yBounds.first) yBounds.first = pos2D.y;
			if (pos2D.y > yBounds.second) yBounds.second = pos2D.y;
			currentIndex = (currentIndex + (isForward ? 1 : ((isOnSavedEdge ? savedEdgePoints : newEdgePoints).size() - 1))) % (isOnSavedEdge ?
				savedEdgePoints : newEdgePoints).size();
			currentPoint = (isOnSavedEdge ? savedEdgePoints : newEdgePoints)[currentIndex];
		} while (!currentPoint->isMarked2);
		if (pointsOnDesiredEdge.size() == pointsOnAddedPolygonPart.size()) endPoint = currentPoint;
		isForward = !isForward;
	} while (currentPoint != pointsOnDesiredEdge[0] && checkForInsidePoints);
	if (checkForInsidePoints) {
		for (const auto p : allNeighbours) {
			if (isPointInsidePolygon(pointsOnAddedPolygonPart, p.second, xBounds, yBounds, true, savedEdge->isHole ? 0.00 : 0.0) == 1) {
				/*points.push_back(new Point(p.first, 0, 0, nullptr));
				for (const auto p2 : (isOnSavedEdge ? savedEdge : newEdge)->closestNeighbourPoints) {
					//points.push_back(new Point(p2.first, 0, 0, nullptr));
				}*/
				return nullptr;
			}
		}
	}
	return endPoint;
}

void calculateNewNeighbours(std::vector<Edge*> createdEdges, std::vector<std::pair<Vec3<double>, Vec3<double>>> neighbours)
{
	std::vector < std::vector<std::pair<std::vector<std::pair<std::pair<Vec3<double>, Vec3<double>>, double>>, double>>>
		neighboursClosestToEdge(createdEdges.size());
	for (size_t i = 0; i < createdEdges.size(); i++) {
		neighboursClosestToEdge[i] = std::vector< std::pair<std::vector<std::pair<std::pair<Vec3<double>, Vec3<double>>, double>>, double>>(
			createdEdges[i]->pointsWithDir.size(), { {}, 100000 });
	}
	for (size_t i = 0; i < neighbours.size(); i++) {
		double minDist = 100000;
		size_t closestEdgeIndex = 0;
		size_t closestPointIndex = 0;
		for (size_t k = 0; k < createdEdges.size(); k++) {
			for (size_t j = 0; j < createdEdges[k]->pointsWithDir.size(); j++) {
				double newDist = (createdEdges[k]->pointsWithDir[j].first->projected2DPosition - neighbours[i].second).length();
				if (newDist < minDist) {
					minDist = newDist;
					closestEdgeIndex = k;
					closestPointIndex = j;
				}
			}
		}
		neighboursClosestToEdge[closestEdgeIndex][closestPointIndex].first.push_back({ neighbours[i] , minDist });
		if (minDist < neighboursClosestToEdge[closestEdgeIndex][closestPointIndex].second)
			neighboursClosestToEdge[closestEdgeIndex][closestPointIndex].second = minDist;
	}
	for (size_t i = 0; i < neighboursClosestToEdge.size(); i++) {
		createdEdges[i]->closestNeighbourPoints.clear();
		for (size_t j = 0; j < neighboursClosestToEdge[i].size(); j++) {
			for (size_t k = 0; k < neighboursClosestToEdge[i][j].first.size(); k++) {
				if (neighboursClosestToEdge[i][j].first[k].second < neighboursClosestToEdge[i][j].second * 2)
					createdEdges[i]->closestNeighbourPoints.push_back(neighboursClosestToEdge[i][j].first[k].first);
			}
		}
	}
}

bool areEdgesIntersect(Plane* savedPlane, Plane* newPlane, Edge* savedEdge, Edge* newEdge)
{
	bool hasFoundIntersection = false;
	auto savedEdgePoints = savedEdge->getPoints();
	auto newEdgePoints = newEdge->getPoints();
	for (size_t j = 0; j < newEdge->pointsWithDir.size(); j++) {
		if (isPointInsidePolygon(savedEdgePoints, newEdge->pointsWithDir[j].first->projected2DPosition, savedEdge->xBounds2D, savedEdge->yBounds2D, true)) {
			hasFoundIntersection = true;
			break;
		}
		for (size_t i = 0; i < savedEdge->pointsWithDir.size(); i++) {
			if (isPointInsidePolygon(newEdgePoints, savedEdge->pointsWithDir[i].first->projected2DPosition, newEdge->xBounds2D, newEdge->yBounds2D, true)) {
				hasFoundIntersection = true;
				break;
			}
			size_t onEdgeType = 0;
			double distanceRatio = 0;
			auto intersectionPos = intersectionOfLines(savedEdge->pointsWithDir[i].first->projected2DPosition, savedEdge->pointsWithDir[(i + 1) %
				savedEdge->pointsWithDir.size()].first->projected2DPosition, newEdge->pointsWithDir[j].first->projected2DPosition,
				newEdge->pointsWithDir[(j + 1) % newEdge->pointsWithDir.size()].first->projected2DPosition, onEdgeType, distanceRatio);
			if (onEdgeType > 0) {
				hasFoundIntersection = true;
				break;
			}
		}
		if (hasFoundIntersection)
			break;
	}
	return hasFoundIntersection;
}

bool hasNewEdgeNeighbourInsideSavedEdge(Edge* savedEdge, Edge* newEdge)
{
	for (size_t i = 0; i < newEdge->closestNeighbourPoints.size(); i++) {
		if (isPointInsidePolygon(savedEdge->getPoints(), newEdge->closestNeighbourPoints[i].second, savedEdge->xBounds2D, savedEdge->yBounds2D, true) == 1) {
			return true;
		}
	}
	return false;
}

void mergePolygons(Plane* savedPlane, Edge* savedEdge, Edge* newEdge, std::vector<std::pair<Vec3<double>, Vec3<double>>> allNeighbours,
	/*out*/ std::vector<Edge*>& outputEdges, bool& createdNewPolygon)
{
	std::vector<Point*> savedEdgePoints = savedEdge->getPoints();
	std::vector<Point*> newEdgePoints = newEdge->getPoints();
	bool isHole = newEdge->isHole;
	for (size_t j = 0; j < newEdgePoints.size(); j++) {
		for (size_t i = 0; i < savedEdgePoints.size(); i++) {
			size_t onEdgeType = 0;
			double distanceRatio = 0;
			if ((newEdgePoints[j]->isMarked2 && (newEdgePoints[j] == savedEdgePoints[i] || newEdgePoints[j] == savedEdgePoints[(i + 1) % savedEdgePoints.size()])) ||
				(newEdgePoints[(j + 1) % newEdgePoints.size()]->isMarked2 && (newEdgePoints[(j + 1) % newEdgePoints.size()] == savedEdgePoints[i] ||
					newEdgePoints[(j + 1) % newEdgePoints.size()] == savedEdgePoints[(i + 1) % savedEdgePoints.size()])))
				continue;
			auto intersectionPos = intersectionOfLines(savedEdgePoints[i]->projected2DPosition, savedEdgePoints[(i + 1) %
				savedEdgePoints.size()]->projected2DPosition, newEdgePoints[j]->projected2DPosition,
				newEdgePoints[(j + 1) % newEdgePoints.size()]->projected2DPosition, onEdgeType, distanceRatio);
			if (onEdgeType > 0) {
				if (onEdgeType == 1) {
					Vec3<double> pos3D = savedEdgePoints[i]->position + (savedEdgePoints[(i + 1) %
						savedEdgePoints.size()]->position - savedEdgePoints[i]->position) * distanceRatio;
					Point* intersectionPoint = new Point(pos3D, horizontalCount + 2, verticalCount + 2, savedEdge->startPoint->plane);
					intersectionPoint->projected2DPosition = intersectionPos;
					intersectionPoint->isCorner = true;
					intersectionPoint->isMarked2 = true;
					savedEdgePoints.insert(savedEdgePoints.begin() + i + 1, intersectionPoint);
					newEdgePoints.insert(newEdgePoints.begin() + j + 1, intersectionPoint);
					fitAddedPoints.push_back(intersectionPoint);
					i++;
				}
				else {
					bool isOnSavedEdge = onEdgeType < 4;
					size_t selectedIndex = (onEdgeType % 2) == 0 ? (isOnSavedEdge ? i : j) : (isOnSavedEdge ?
						((i + 1) % savedEdgePoints.size()) : ((j + 1) % newEdgePoints.size()));
					auto selectedEdgePoints = isOnSavedEdge ? savedEdgePoints : newEdgePoints;
					auto otherEdgePoints = isOnSavedEdge ? newEdgePoints : savedEdgePoints;
					auto selectedPoint = selectedEdgePoints[selectedIndex];
					bool isAlreadyAdded = false;
					for (size_t k = 0; k < otherEdgePoints.size(); k++) {
						if (otherEdgePoints[k] == selectedPoint) {
							isAlreadyAdded = true;
							break;
						}
					}
					if (!isAlreadyAdded) {
						selectedPoint->isMarked2 = true;
						(isOnSavedEdge ? newEdgePoints : savedEdgePoints).insert((isOnSavedEdge ? newEdgePoints : savedEdgePoints).begin() + (isOnSavedEdge ? j : i) + 1,
							selectedPoint);
					}
				}
			}
		}
	}
	if (newEdgePoints[0]->horizontalIndex == 168 && newEdgePoints[0]->verticalIndex == 18) {
		std::cout << "asd";
	}
	std::vector<Point*> desiredEdges;
	Point* currentPoint = nullptr;
	for (size_t i = 0; i < 2; i++) //filtering
	{
		std::vector<Point*>& currentEdge = (i == 0 ? savedEdgePoints : newEdgePoints);
		for (size_t j = 0; j < currentEdge.size(); j++) {
			if (currentEdge[j]->projected2DPosition == currentEdge[(j + 1) % currentEdge.size()]->projected2DPosition) {
				if (!currentEdge[j]->isMarked2) {
					currentEdge.erase(currentEdge.begin() + j);
					j--;
				}
				else if (!currentEdge[(j + 1) % currentEdge.size()]->isMarked2) {
					currentEdge.erase(currentEdge.begin() + (j + 1) % currentEdge.size());
					j--;
				}
			}
		}
	}
	for (size_t i = 0; i < savedEdgePoints.size(); i++) //filtering
	{
		if (savedEdgePoints[i]->isMarked2) {
			auto pos1 = (savedEdgePoints[i]->projected2DPosition + savedEdgePoints[(i +
				savedEdgePoints.size() - 1) % savedEdgePoints.size()]->projected2DPosition) / 2;
			auto pos2 = (savedEdgePoints[i]->projected2DPosition + savedEdgePoints[(i + 1) %
				savedEdgePoints.size()]->projected2DPosition) / 2;
			if (isPointInsidePolygon(newEdgePoints, pos1, newEdge->xBounds2D, newEdge->yBounds2D, true) ==
				isPointInsidePolygon(newEdgePoints, pos2, newEdge->xBounds2D, newEdge->yBounds2D, true)) {
				savedEdgePoints[i]->isMarked2 = false;
			}
		}
		if (savedEdgePoints[i]->isMarked2 && savedEdgePoints[(i + 1) % savedEdgePoints.size()]->isMarked2)
			//&& savedEdgePoints[i]->verticalIndex != verticalCount + 2 && savedEdgePoints[(i + 1) % 
			//savedEdgePoints.size()]->verticalIndex != verticalCount + 2)
		{
			for (size_t j = 0; j < newEdgePoints.size(); j++) {
				if ((newEdgePoints[j] == savedEdgePoints[i] && newEdgePoints[(j + 1) %
					newEdgePoints.size()] == savedEdgePoints[(i + 1) % savedEdgePoints.size()]) || (newEdgePoints[j] == savedEdgePoints[(i + 1) % savedEdgePoints.size()]
						&& newEdgePoints[(j + 1) % newEdgePoints.size()] == savedEdgePoints[i])) {
					auto pos1 = (newEdgePoints[(j + newEdgePoints.size() - 1) % newEdgePoints.size()]->projected2DPosition + newEdgePoints[j]->projected2DPosition) / 2;
					auto pos2 = (newEdgePoints[(j + 2) % newEdgePoints.size()]->projected2DPosition + newEdgePoints[(j + 1) % newEdgePoints.size()]->projected2DPosition) / 2;
					if (isPointInsidePolygon(savedEdgePoints, pos1, savedEdge->xBounds2D, savedEdge->yBounds2D, true, 0.01) ==
						isPointInsidePolygon(savedEdgePoints, pos2, savedEdge->xBounds2D, savedEdge->yBounds2D, true, 0.01)) {
						newEdgePoints[j]->isMarked2 = false;
					}
					newEdgePoints[(j + 1) % newEdgePoints.size()]->isMarked2 = false;
					break;
				}
			}
		}
	}
	int dbgCounter = 0;
	for (size_t i = 0; i < savedEdgePoints.size(); i++) {
		if (savedEdgePoints[i]->isMarked2)
			dbgCounter++;
		if (isDesiredEdge(savedEdgePoints, newEdgePoints, newEdge->xBounds2D, newEdge->yBounds2D, i, isHole)) {
			desiredEdges.push_back(savedEdgePoints[i]);
			savedEdgePoints[i]->isMarked = true;
			if (savedEdgePoints[i]->isMarked2 && !currentPoint) {
				currentPoint = savedEdgePoints[i];
			}
		}
	}
	if (dbgCounter % 2 != 0) {
		/*/for (size_t y = 0; y < savedEdge->pointsWithDir.size(); y++) {
			savedEdge->pointsWithDir[y].first->cornerId = currentCornerId;
			savedEdge->pointsWithDir[y].first->cornerIndex = currentCornerIndex;
			savedPoints.push_back(savedEdge->pointsWithDir[y].first);
			savedEdge->pointsWithDir[y].first->position = savedEdge->pointsWithDir[y].first->projected2DPosition;
			currentCornerIndex++;
		}
		currentCornerId++;
		currentCornerIndex = 0;
		for (size_t y = 0; y < newEdge->pointsWithDir.size(); y++) {
			newEdge->pointsWithDir[y].first->cornerId = currentCornerId;
			newEdge->pointsWithDir[y].first->cornerIndex = currentCornerIndex;
			savedPoints.push_back(newEdge->pointsWithDir[y].first);
			newEdge->pointsWithDir[y].first->position = newEdge->pointsWithDir[y].first->projected2DPosition;
			currentCornerIndex++;
		}
		currentCornerId++;
		writeData(4);*/
		std::cout << "INTERSECTION COUNT IS NOT EVEN!" << std::endl;
	}

	while (desiredEdges.size() > 0) {
		bool isOnSavedEdge = false;
		Edge* unionPolygon = new Edge();
		fitEdges.push_back(unionPolygon);
		unionPolygon->startPoint = currentPoint;
		unionPolygon->isHole = isHole;
		do {
			std::vector<Point*> pointsOnNextEdge;
			auto helperPoint = currentPoint;
			bool helperIsOnEdge = isOnSavedEdge;
			currentPoint = decideIfDesiredEdgeGood(savedEdge, newEdge, currentPoint, savedEdgePoints, newEdgePoints, isOnSavedEdge, true, allNeighbours, pointsOnNextEdge);
			if (!currentPoint) {
				pointsOnNextEdge.clear();
				isOnSavedEdge = !isOnSavedEdge;
				currentPoint = decideIfDesiredEdgeGood(savedEdge, newEdge, helperPoint, savedEdgePoints, newEdgePoints, isOnSavedEdge, false, allNeighbours,
					pointsOnNextEdge);
			}
			for (size_t i = 0; i < pointsOnNextEdge.size(); i++) {
				pointsOnNextEdge[i]->isMarked = false;
				unionPolygon->pointsWithDir.push_back({ pointsOnNextEdge[i], -1 });
				auto pos2D = pointsOnNextEdge[i]->projected2DPosition;
				if (pos2D.x < unionPolygon->xBounds2D.first) unionPolygon->xBounds2D.first = pos2D.x;
				if (pos2D.x > unionPolygon->xBounds2D.second) unionPolygon->xBounds2D.second = pos2D.x;
				if (pos2D.y < unionPolygon->yBounds2D.first) unionPolygon->yBounds2D.first = pos2D.y;
				if (pos2D.y > unionPolygon->yBounds2D.second) unionPolygon->yBounds2D.second = pos2D.y;
			}
			isOnSavedEdge = !helperIsOnEdge;
		} while (currentPoint != unionPolygon->pointsWithDir[0].first);
		outputEdges.push_back(unionPolygon);
		currentPoint = nullptr;
		for (size_t i = 0; i < desiredEdges.size(); i++) {
			if (!desiredEdges[i]->isMarked) {
				desiredEdges.erase(desiredEdges.begin() + i);
				i--;
			}
			else if (currentPoint == nullptr && desiredEdges[i]->isMarked2)
				currentPoint = desiredEdges[i];
		}
	}
	if (outputEdges.size() > 0) {
		if (!isHole) {
			while (outputEdges.size() > 1) {
				if (isPointInsidePolygon(outputEdges[1]->getPoints(), outputEdges[0]->pointsWithDir[0].first->projected2DPosition, outputEdges[1]->xBounds2D,
					outputEdges[1]->yBounds2D))
					outputEdges.erase(outputEdges.begin());
				else
					outputEdges.erase(outputEdges.begin() + 1);
			}
		}
		else {
			double maxLength = 0;
			size_t maxIndex = 0;
			for (size_t i = 0; i < outputEdges.size(); i++) {
				double length = 0;
				for (size_t j = 0; j < outputEdges[i]->pointsWithDir.size(); j++) {
					length += (outputEdges[i]->pointsWithDir[j].first->projected2DPosition - outputEdges[i]->pointsWithDir[(j + 1) %
						outputEdges[i]->pointsWithDir.size()].first->projected2DPosition).length();
				}
				if (length > maxLength) {
					maxIndex = i;
					maxLength = length;
				}
			}
			auto helper = outputEdges[maxIndex];
			outputEdges.clear();
			outputEdges.push_back(helper);
		}
	}
	else {
		bool hasFoundPointInside = false;
		for (size_t j = 0; j < 2; j++) {
			for (size_t i = 0; i < (j == 0 ? newEdgePoints : savedEdgePoints).size(); i++) {
				if (isPointInsidePolygon((j == 0 ? savedEdgePoints : newEdgePoints), (j == 0 ? newEdgePoints :
					savedEdgePoints)[i]->projected2DPosition, (j == 0 ? savedEdge : newEdge)->xBounds2D, (j == 0 ? savedEdge : newEdge)->yBounds2D,
					true) == 1) {
					bool hasFoundNeighbourInside = false;
					for (size_t k = 0; k < (((j == 0 && !newEdge->isHole) || (j == 1 && newEdge->isHole)) ? newEdge : savedEdge)->closestNeighbourPoints.size(); k++) {
						if (isPointInsidePolygon((((j == 0 && !newEdge->isHole) || (j == 1 && newEdge->isHole)) ? savedEdge : newEdge)->getPoints(),
							(((j == 0 && !newEdge->isHole) || (j == 1 && newEdge->isHole)) ? newEdge : savedEdge)->closestNeighbourPoints[k].second,
							(((j == 0 && !newEdge->isHole) || (j == 1 && newEdge->isHole)) ? savedEdge : newEdge)->xBounds2D,
							(((j == 0 && !newEdge->isHole) || (j == 1 && newEdge->isHole)) ? savedEdge : newEdge)->yBounds2D, true) == 1) {
							hasFoundNeighbourInside = true;
							break;
						}
					}
					hasFoundPointInside = true;
					auto acceptedEdge = (((j == 0 && !newEdge->isHole) || (j == 1 && newEdge->isHole)) && !hasFoundNeighbourInside) ? savedEdge : newEdge;
					Edge* unionPolygon = new Edge();
					fitEdges.push_back(unionPolygon);
					unionPolygon->startPoint = acceptedEdge->startPoint;
					unionPolygon->isHole = acceptedEdge->isHole;
					unionPolygon->pointsWithDir = acceptedEdge->pointsWithDir;
					unionPolygon->xBounds2D = acceptedEdge->xBounds2D;
					unionPolygon->yBounds2D = acceptedEdge->yBounds2D;
					outputEdges.push_back(unionPolygon);
					break;
				}
			}
			if (hasFoundPointInside) break;
		}
		if (!hasFoundPointInside) {
			createdNewPolygon = false;
			outputEdges.push_back(savedEdge);
			outputEdges.push_back(newEdge);
		}
	}
	if (createdNewPolygon) {
		std::vector<std::pair<Vec3<double>, Vec3<double>>> neighbours;
		neighbours.reserve(savedEdge->closestNeighbourPoints.size() + newEdge->closestNeighbourPoints.size());
		neighbours.insert(neighbours.end(), savedEdge->closestNeighbourPoints.begin(), savedEdge->closestNeighbourPoints.end());
		neighbours.insert(neighbours.end(), newEdge->closestNeighbourPoints.begin(), newEdge->closestNeighbourPoints.end());
		calculateNewNeighbours(outputEdges, neighbours);
		for (size_t i = 0; i < outputEdges.size(); i++) {
			for (size_t j = 0; j < outputEdges[i]->pointsWithDir.size(); j++) {
				outputEdges[i]->pointsWithDir[j].first->plane = savedPlane;
			}
		}
	}
	setPointsMarked(savedEdgePoints, false, false);
	setPointsMarked(newEdgePoints, false, false);
}

bool isEdgeTooTigth(Edge* edge)
{
	double acceptTreshold = 0.1;
	double maxDistance = 0;
	Vec3<double> normal = { 0,0,0 };
	Vec3<double> linePoint = { 0,0,0 };
	for (size_t i = 0; i < edge->pointsWithDir.size() - 1; i++) {
		for (size_t j = i + 1; j < edge->pointsWithDir.size(); j++) {
			double currentDistance = (edge->pointsWithDir[i].first->projected2DPosition - edge->pointsWithDir[j].first->projected2DPosition).length();
			if (currentDistance > maxDistance) {
				maxDistance = currentDistance;
				normal = edge->pointsWithDir[i].first->projected2DPosition - edge->pointsWithDir[j].first->projected2DPosition;
				linePoint = edge->pointsWithDir[i].first->projected2DPosition;
			}
		}
	}
	normal = { -normal.y, normal.x, 0 };
	for (size_t i = 0; i < edge->pointsWithDir.size() - 1; i++) {
		if (abs(Vec3<double>::dot_product(normal, edge->pointsWithDir[i].first->projected2DPosition - linePoint)) > acceptTreshold)
			return false;
	}
	return true;
}

void mergeArrayOfEdges(Plane* savedPlane, std::vector<Edge*>& createdEdges, std::vector<std::pair<Vec3<double>, Vec3<double>>> neighbours, size_t pointCloudIndex, bool dbg = false)
{
	std::vector<Edge*> tempCreatedEdges;
	bool createdNewPolygon = true;
	bool foundNewIntersection = true;
	int counter = 0;
	while (foundNewIntersection) {
		foundNewIntersection = false;
		while (createdEdges.size() > 0) {

			for (size_t i = 1; i < createdEdges.size(); i++) {
				if (createdEdges[0]->isHole == createdEdges[i]->isHole) {
					std::vector<Edge*> innerCreatedEdges;
					/*if (false && dbg && i == 24 && counter == 13) {// currentFrame == 4 && savedEdge->isHole && counter == 3) {
						savedPoints.clear();
						addedPoints.clear();
						points.clear();
						changeBaseTo2D(createdEdges[0], { savedPlane->normal, savedPlane->pointDirections.first });
						changeBaseTo2D(createdEdges[i], { savedPlane->normal, savedPlane->pointDirections.first });
						for (size_t y = 0; y < createdEdges[0]->pointsWithDir.size(); y++) {
							createdEdges[0]->pointsWithDir[y].first->cornerId = currentCornerId;
							createdEdges[0]->pointsWithDir[y].first->cornerIndex = currentCornerIndex;
							savedPoints.push_back(createdEdges[0]->pointsWithDir[y].first);
							createdEdges[0]->pointsWithDir[y].first->position = createdEdges[0]->pointsWithDir[y].first->projected2DPosition;
							currentCornerIndex++;
						}
						currentCornerId++;
						currentCornerIndex = 0;
						for (size_t y = 0; y < createdEdges[i]->pointsWithDir.size(); y++) {
							createdEdges[i]->pointsWithDir[y].first->cornerId = currentCornerId;
							createdEdges[i]->pointsWithDir[y].first->cornerIndex = currentCornerIndex;
							savedPoints.push_back(createdEdges[i]->pointsWithDir[y].first);
							createdEdges[i]->pointsWithDir[y].first->position = createdEdges[i]->pointsWithDir[y].first->projected2DPosition;
							currentCornerIndex++;
						}

						for (size_t x = 0; x < createdEdges.size(); x++) {
							for (size_t y = 0; y < createdEdges[x]->closestNeighbourPoints.size(); y++) {
								savedPoints.push_back(new Point(createdEdges[x]->closestNeighbourPoints[y].first, 0, 0, nullptr));
							}
							currentCornerIndex = 0;
							for (size_t y = 0; y < createdEdges[x]->pointsWithDir.size(); y++) {
								createdEdges[x]->pointsWithDir[y].first->cornerId = 0;
								createdEdges[x]->pointsWithDir[y].first->cornerIndex = 0;
							}
							for (size_t y = 0; y < createdEdges[x]->pointsWithDir.size(); y++) {
								if (createdEdges[x]->pointsWithDir[y].first->isCorner && createdEdges[x]->pointsWithDir[y].first->cornerId == 0) {
									createdEdges[x]->pointsWithDir[y].first->cornerId = currentCornerId;
									createdEdges[x]->pointsWithDir[y].first->cornerIndex = currentCornerIndex;
									savedPoints.push_back(createdEdges[x]->pointsWithDir[y].first);
									currentCornerIndex++;
								}
								else if (createdEdges[x]->pointsWithDir[y].first->cornerId != 0) {
									std::cout << "asd";
								}
							}
							currentCornerId++;
						}
						writeData(4);
						std::cout << "asd";
					}*/
					createdNewPolygon = true;
					mergePolygons(savedPlane, createdEdges[0], createdEdges[i], neighbours, innerCreatedEdges, createdNewPolygon);

					if (createdNewPolygon) {
						foundNewIntersection = true;
						for (size_t m = 0; m < innerCreatedEdges.size(); m++) {
							if (!isEdgeTooTigth(innerCreatedEdges[m]))
								tempCreatedEdges.push_back(innerCreatedEdges[m]);
						}
						createdEdges.erase(createdEdges.begin() + i);
						createdEdges.erase(createdEdges.begin());
						break;
					}
				}
				else
					createdNewPolygon = false;
			}
			counter++;
			if (!createdNewPolygon || createdEdges.size() == 1) {
				tempCreatedEdges.push_back(createdEdges[0]);
				createdEdges.erase(createdEdges.begin());
			}
		}
		createdEdges = tempCreatedEdges;
		tempCreatedEdges.clear();
	}
}

void filterFittedPlanes()
{
	int dbgCounter = 0;
	double normalDistTreshold = 0.1;
	double distTreshold = 0.01;
	double newDirectionTreshold = 0.1;
	for (size_t i = 0; i < allPlanes.size(); i++) {
		for (size_t j = 0; j < allPlanes[i]->edges.size(); j++) {
			bool deletedPoint = true;
			dbgCounter = 0;
			while (deletedPoint && allPlanes[i]->edges[j]->pointsWithDir.size() > 2) {
				dbgCounter++;
				deletedPoint = false;
				auto normal = Vec3<double>::normalize(allPlanes[i]->edges[j]->pointsWithDir[allPlanes[i]->edges[j]->pointsWithDir.size() - 1].first->projected2DPosition -
					allPlanes[i]->edges[j]->pointsWithDir[allPlanes[i]->edges[j]->pointsWithDir.size() - 2].first->projected2DPosition);
				normal = { -normal.y, normal.x, 0 };
				auto linePoint = allPlanes[i]->edges[j]->pointsWithDir[allPlanes[i]->edges[j]->pointsWithDir.size() - 2].first->projected2DPosition;
				for (size_t k = 0; k < allPlanes[i]->edges[j]->pointsWithDir.size(); k++) {
					auto verticalIndex = allPlanes[i]->edges[j]->pointsWithDir[k].first->verticalIndex;
					auto previousIndex = (k + allPlanes[i]->edges[j]->pointsWithDir.size() - 1) % allPlanes[i]->edges[j]->pointsWithDir.size();
					auto previousPoint = allPlanes[i]->edges[j]->pointsWithDir[previousIndex].first;
					auto previousVerticalIndex = previousPoint->verticalIndex;
					double dist = (allPlanes[i]->edges[j]->pointsWithDir[k].first->projected2DPosition - previousPoint->projected2DPosition).length();
					if (dist < distTreshold) {
						if (previousVerticalIndex < verticalCount || verticalIndex >= previousVerticalIndex) {
							allPlanes[i]->edges[j]->pointsWithDir.erase(allPlanes[i]->edges[j]->pointsWithDir.begin() + previousIndex);
							if (previousIndex < k) k--;
						}
						else {
							allPlanes[i]->edges[j]->pointsWithDir.erase(allPlanes[i]->edges[j]->pointsWithDir.begin() + k);
							k--;
						}
						deletedPoint = true;
						continue;
					}
					double normalDist = abs(Vec3<double>::dot_product(normal, allPlanes[i]->edges[j]->pointsWithDir[k].first->projected2DPosition - linePoint));
					if (normalDist < normalDistTreshold) {
						if (previousVerticalIndex < verticalCount || verticalIndex >= previousVerticalIndex) {
							allPlanes[i]->edges[j]->pointsWithDir.erase(allPlanes[i]->edges[j]->pointsWithDir.begin() + previousIndex);
							if (previousIndex < k) k--;
						}
						else if ((Vec3<double>::normalize(allPlanes[i]->edges[j]->pointsWithDir[(k + 1) % allPlanes[i]->edges[j]->pointsWithDir.size()].first->projected2DPosition -
							previousPoint->projected2DPosition) - Vec3<double>::normalize(allPlanes[i]->edges[j]->pointsWithDir[(k
								+ 1) % allPlanes[i]->edges[j]->pointsWithDir.size()].first->projected2DPosition -
								allPlanes[i]->edges[j]->pointsWithDir[k].first->projected2DPosition)).length() < newDirectionTreshold) {
							allPlanes[i]->edges[j]->pointsWithDir.erase(allPlanes[i]->edges[j]->pointsWithDir.begin() + k);
							k--;
						}
						else
							continue;
						deletedPoint = true;
					}
					else {
						normal = Vec3<double>::normalize(allPlanes[i]->edges[j]->pointsWithDir[k].first->projected2DPosition - previousPoint->projected2DPosition);
						normal = { -normal.y, normal.x, 0 };
						linePoint = previousPoint->projected2DPosition;
					}
				}
			}
			if (allPlanes[i]->edges[j]->pointsWithDir.size() < 3) {
				allPlanes[i]->edges.erase(allPlanes[i]->edges.begin() + j);
				j--;
			}
		}
	}
}

void checkIfHolesAreContained()
{
	for (size_t i = 0; i < allPlanes.size(); i++) {
		for (size_t j = 0; j < allPlanes[i]->edges.size(); j++) {
			if (allPlanes[i]->edges[j]->isHole) {
				bool isCointained = false;
				for (size_t k = 0; k < allPlanes[i]->edges.size(); k++) {
					if (!allPlanes[i]->edges[k]->isHole) {
						for (size_t l = 0; l < allPlanes[i]->edges[j]->pointsWithDir.size(); l++) {
							if (isPointInsidePolygon(allPlanes[i]->edges[k]->getPoints(),
								allPlanes[i]->edges[j]->pointsWithDir[l].first->projected2DPosition, allPlanes[i]->edges[k]->xBounds2D,
								allPlanes[i]->edges[k]->yBounds2D)) {
								isCointained = true;
							}
							else {
								isCointained = false;
								break;
							}
						}
						if (isCointained) {
							double notInUse = 0;
							for (size_t l = 0; l < allPlanes[i]->edges[j]->pointsWithDir.size(); l++) {
								for (size_t m = 0; m < allPlanes[i]->edges[k]->pointsWithDir.size(); m++) {
									size_t onEdgeType = 0;
									intersectionOfLines(allPlanes[i]->edges[j]->pointsWithDir[l].first->projected2DPosition,
										allPlanes[i]->edges[j]->pointsWithDir[(l + 1) % allPlanes[i]->edges[j]->pointsWithDir.size()].first->projected2DPosition,
										allPlanes[i]->edges[k]->pointsWithDir[m].first->projected2DPosition,
										allPlanes[i]->edges[k]->pointsWithDir[(m + 1) % allPlanes[i]->edges[k]->pointsWithDir.size()].first->projected2DPosition, onEdgeType, notInUse);
									if (onEdgeType > 0) {
										isCointained = false;
										break;
									}
								}
								if (!isCointained)
									break;
							}
							break;
						}
					}
				}
				allPlanes[i]->edges[j]->isInUse = isCointained;
			}
		}
	}
}

bool arePlanesIntersecting(Plane* plane1, Plane* plane2)
{
	double normalTreshold = 0.1;
	double distanceTreshold = 0.1;
	if ((plane1->normal - plane2->normal).length() < normalTreshold &&
		(abs(Vec3<double>::dot_product(plane1->normal, plane2->furthestNormalPoints[0] - plane1->planePointPos)) < distanceTreshold ||
			abs(Vec3<double>::dot_product(plane1->normal, plane2->furthestNormalPoints[1] - plane1->planePointPos)) < distanceTreshold ||
			abs(Vec3<double>::dot_product(plane2->normal, plane1->furthestNormalPoints[0] - plane2->planePointPos)) < distanceTreshold ||
			abs(Vec3<double>::dot_product(plane2->normal, plane1->furthestNormalPoints[1] - plane2->planePointPos)) < distanceTreshold)) {
		for (int k = 0; k < plane1->edges.size(); k++) {
			changeBaseTo2D(plane1->edges[k], { plane1->normal, plane1->pointDirections[0] });
		}
		for (int k = 0; k < plane2->edges.size(); k++) {
			changeBaseTo2D(plane2->edges[k], { plane1->normal, plane1->pointDirections[0] });
		}
		for (int k = 0; k < plane1->edges.size(); k++) {
			for (int l = 0; l < plane2->edges.size(); l++) {
				if (plane1->edges[k]->isHole == plane2->edges[l]->isHole && plane1->edges[k]->canIntersectWithEdge(plane2->edges[l])) {
					if (areEdgesIntersect(plane2, plane1, plane2->edges[l], plane1->edges[k])) {
						return true;
					}
				}
			}
		}
		for (int k = 0; k < plane2->edges.size(); k++) {
			changeBaseTo2D(plane2->edges[k], { plane2->normal, plane2->pointDirections[0] });
		}
	}
	return false;
}

__global__
void relocateNeighbours(double* positions, bool* isInsideHole, double* polygon, size_t neighbourCount, size_t polygonSize, double xBoundsMin, double xBoundsMax,
	double yBoundsMin, double yBoundsMax, bool checkOnEdge = false, double onEdgetreshold = 0.0000001)
{
	int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index < neighbourCount) 
	{
		// 0 - outside
		// 1 - inside
		// 2 - onEdge
		size_t notInUse = 0;
		double notInUseRatio = 0;
		size_t type = 0;
		Vec3<double> point = { positions[index], positions[index + 1], positions[index + 2] };
		if (point.x > xBoundsMin && point.x < xBoundsMax &&
			point.y > yBoundsMin && point.y < yBoundsMax) {
			int rigthCounter = 0;
			for (size_t i = 0; i < polygonSize; i++) {
				Vec3<double> p1 = { polygon[i], polygon[i + 1], polygon[i + 2] };
				Vec3<double> p2 = { polygon[(i + 3) % (polygonSize * 3)], polygon[(i + 4) % (polygonSize * 3)], polygon[(i + 5) % (polygonSize * 3)] };
				if ((p1.x < point.x && p2.x < point.x) || (p1.y > point.y && p2.y > point.y) || (p1.y < point.y && p2.y < point.y) || (p1.y == p2.y))
					continue;
				else if (point == p1)
					break;
				else {
					auto intersection = intersectionOfLines(point, point + Vec3<double>({ 1,0,0 }), p1, p2, notInUse, notInUseRatio);
					if (abs(intersection.x - point.x) < onEdgetreshold && checkOnEdge)
					{
						type = 2;
						break;
					}
					if (intersection.x <= point.x)
						continue;
					else {
						rigthCounter++;
						if (p2.y == point.y)
							i++;
					}
				}
			}
			if(type == 0)
				type = rigthCounter % 2 == 1;
		}
		if (type > 0) 
		{
			isInsideHole[index] = true;
		}
	}
}

void relocateHoleNeighbours(Plane* plane)
{
	for (size_t i = 0; i < plane->edges.size(); i++) {
		if (plane->edges[i]->isHole) {
			for (size_t k = 0; k < plane->closestNeighbourPointsNonHole.size(); k++) {
				if (isPointInsidePolygon(plane->edges[i]->getPoints(), plane->closestNeighbourPointsNonHole[k].second,
					plane->edges[i]->xBounds2D, plane->edges[i]->yBounds2D, true, 0.01) > 0) {
					plane->closestNeighbourPointsHole.push_back(plane->closestNeighbourPointsNonHole[k]);
					plane->closestNeighbourPointsNonHole.erase(plane->closestNeighbourPointsNonHole.begin() + k);
					k--;
				}
			}
		}
	}
}

void relocateHoleNeighbours2(Plane* plane)
{
	double* nonHoleNeighboursCoords;
	cudaMallocManaged(&nonHoleNeighboursCoords, plane->closestNeighbourPointsNonHole.size() * sizeof(double) * 3);
	bool* isInsideHole;
	cudaMallocManaged(&isInsideHole, plane->closestNeighbourPointsNonHole.size() * sizeof(bool));
	
	for (size_t i = 0; i < plane->closestNeighbourPointsNonHole.size(); i++)
	{
		isInsideHole[i] = false;
		nonHoleNeighboursCoords[i * 3] = plane->closestNeighbourPointsNonHole[i].second.x;
		nonHoleNeighboursCoords[i * 3 + 1] = plane->closestNeighbourPointsNonHole[i].second.y;
		nonHoleNeighboursCoords[i * 3 + 2] = plane->closestNeighbourPointsNonHole[i].second.z;
	}
	
	for (size_t i = 0; i < plane->edges.size(); i++) {
		if (plane->edges[i]->isHole) {

			int blockSize = 256;
			int numBlocks = (plane->closestNeighbourPointsNonHole.size() + blockSize - 1) / blockSize;
			
			double* polygon;
			cudaMallocManaged(&polygon, plane->edges[i]->pointsWithDir.size() * sizeof(double) * 3);

			for (size_t i = 0; i < plane->edges[i]->pointsWithDir.size(); i++) 
			{
				polygon[i * 3] = plane->edges[i]->pointsWithDir[i].first->projected2DPosition.x;
				polygon[i * 3 + 1] = plane->edges[i]->pointsWithDir[i].first->projected2DPosition.y;
				polygon[i * 3 + 2] = plane->edges[i]->pointsWithDir[i].first->projected2DPosition.z;
			}
			
			relocateNeighbours <<<numBlocks, blockSize>>> (nonHoleNeighboursCoords, isInsideHole, polygon, plane->closestNeighbourPointsNonHole.size(),
				plane->edges[i]->pointsWithDir.size(), plane->edges[i]->xBounds2D.first, plane->edges[i]->xBounds2D.second, plane->edges[i]->yBounds2D.first, 
				plane->edges[i]->yBounds2D.second, true, 0.01);

			cudaDeviceSynchronize();
			cudaFree(polygon);
			/*for (size_t k = 0; k < plane->closestNeighbourPointsNonHole.size(); k++) {
				if (isPointInsidePolygon(plane->edges[i]->getPoints(), plane->closestNeighbourPointsNonHole[k].second,
					plane->edges[i]->xBounds2D, plane->edges[i]->yBounds2D, true, 0.01) > 0) {
					plane->closestNeighbourPointsHole.push_back(plane->closestNeighbourPointsNonHole[k]);
					plane->closestNeighbourPointsNonHole.erase(plane->closestNeighbourPointsNonHole.begin() + k);
					k--;
				}
			}*/
		}
	}
	for (size_t i = 0; i < plane->closestNeighbourPointsNonHole.size(); i++)
	{
		if (isInsideHole[i]) 
		{
			plane->closestNeighbourPointsHole.push_back(plane->closestNeighbourPointsNonHole[i]);
			plane->closestNeighbourPointsNonHole.erase(plane->closestNeighbourPointsNonHole.begin() + i);
			i--;
		}
	}
	cudaFree(nonHoleNeighboursCoords);
	cudaFree(isInsideHole);
}

__device__
void changeBaseTo2D_CUDA(Edge_CUDA* edge, Vec3<double> normal, Vec3<double> x)
{
	edge->xBounds2DMin = 100000;
	edge->xBounds2DMax = -100000;
	edge->yBounds2DMin = 100000;
	edge->yBounds2DMax = -100000;
	x = Vec3<double>::normalize(x - normal * Vec3<double>::dot_product(x, normal));
	auto y = Vec3<double>::crossProduct(x, normal);
	for (size_t i = 0; i < edge->pointsSize; i++) {
		edge->pointsWithDir[i]->projected2DPosition = { Vec3<double>::dot_product(edge->pointsWithDir[i]->position, x),
			Vec3<double>::dot_product(edge->pointsWithDir[i]->position, y), 0 };
		auto pos2D = edge->pointsWithDir[i]->projected2DPosition;
		if (pos2D.x < edge->xBounds2DMin) edge->xBounds2DMin = pos2D.x;
		if (pos2D.x > edge->xBounds2DMax) edge->xBounds2DMax = pos2D.x;
		if (pos2D.y < edge->yBounds2DMin) edge->yBounds2DMin = pos2D.y;
		if (pos2D.y > edge->yBounds2DMax) edge->yBounds2DMax = pos2D.y;
	}
}

__device__
size_t isPointInsidePolygon_CUDA(Point_CUDA** polygon, size_t polygonSize, Vec3<double> point, double xBoundsMin, double xBoundsMax, double yBoundsMin, double yBoundsMax,
	bool checkOnEdge = false, double onEdgetreshold = 0.0000001)
{
	// 0 - outside
	// 1 - inside
	// 2 - onEdge
	size_t notInUse = 0;
	double notInUseRatio = 0;
	if (point.x > xBoundsMin && point.x < xBoundsMax &&
		point.y > yBoundsMin && point.y < yBoundsMax) {
		int rigthCounter = 0;
		for (size_t i = 0; i < polygonSize; i++) {
			auto p1 = polygon[i]->projected2DPosition;
			auto p2 = polygon[(i + 1) % polygonSize]->projected2DPosition;
			if ((p1.x < point.x && p2.x < point.x) || (p1.y > point.y && p2.y > point.y) || (p1.y < point.y && p2.y < point.y) || (p1.y == p2.y))
				continue;
			else if (point == p1)
				return false;
			else {
				auto intersection = intersectionOfLines(point, point + Vec3<double>({ 1,0,0 }), p1, p2, notInUse, notInUseRatio);
				if (abs(intersection.x - point.x) < onEdgetreshold && checkOnEdge)
					return 2;
				if (intersection.x <= point.x)
					continue;
				else {
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

__device__
bool areEdgesIntersect_CUDA(Plane_CUDA* savedPlane, Plane_CUDA* newPlane, Edge_CUDA* savedEdge, Edge_CUDA* newEdge)
{
	bool hasFoundIntersection = false;
	auto savedEdgePoints = savedEdge->getPoints();
	auto newEdgePoints = newEdge->getPoints();
	for (size_t j = 0; j < newEdge->pointsSize; j++) {
		if (isPointInsidePolygon_CUDA(savedEdgePoints, savedEdge->pointsSize, newEdge->pointsWithDir[j]->projected2DPosition, savedEdge->xBounds2DMin, savedEdge->xBounds2DMax,
			savedEdge->yBounds2DMin, savedEdge->yBounds2DMin, true)) {
			hasFoundIntersection = true;
			break;
		}
		for (size_t i = 0; i < savedEdge->pointsSize; i++) {
			if (isPointInsidePolygon_CUDA(newEdgePoints, newEdge->pointsSize, savedEdge->pointsWithDir[i]->projected2DPosition, newEdge->xBounds2DMin, newEdge->xBounds2DMax,
				newEdge->yBounds2DMin, newEdge->yBounds2DMin, true)) {
				hasFoundIntersection = true;
				break;
			}
			size_t onEdgeType = 0;
			double distanceRatio = 0;
			auto intersectionPos = intersectionOfLines(savedEdge->pointsWithDir[i]->projected2DPosition, savedEdge->pointsWithDir[(i + 1) %
				savedEdge->pointsSize]->projected2DPosition, newEdge->pointsWithDir[j]->projected2DPosition,
				newEdge->pointsWithDir[(j + 1) % newEdge->pointsSize]->projected2DPosition, onEdgeType, distanceRatio);
			if (onEdgeType > 0) {
				hasFoundIntersection = true;
				break;
			}
		}
		if (hasFoundIntersection)
			break;
	}
	return hasFoundIntersection;
}

__device__
bool arePlanesIntersecting_CUDA(Plane_CUDA* plane1, Plane_CUDA* plane2)
{
	double normalTreshold = 0.1;
	double distanceTreshold = 0.1;
	if ((plane1->normal - plane2->normal).length() < normalTreshold &&
		(abs(Vec3<double>::dot_product(plane1->normal, plane2->furthestNormalPoints[0] - plane1->planePointPos)) < distanceTreshold ||
			abs(Vec3<double>::dot_product(plane1->normal, plane2->furthestNormalPoints[1] - plane1->planePointPos)) < distanceTreshold ||
			abs(Vec3<double>::dot_product(plane2->normal, plane1->furthestNormalPoints[0] - plane2->planePointPos)) < distanceTreshold ||
			abs(Vec3<double>::dot_product(plane2->normal, plane1->furthestNormalPoints[1] - plane2->planePointPos)) < distanceTreshold)) 
	{
		for (int k = 0; k < plane1->edgesSize; k++) {
			changeBaseTo2D_CUDA(plane1->edges[k], plane1->normal, plane1->pointDirections[0]);
		}
		for (int k = 0; k < plane2->edgesSize; k++) {
			changeBaseTo2D_CUDA(plane2->edges[k], plane1->normal, plane1->pointDirections[0]);
		}
		for (int k = 0; k < plane1->edgesSize; k++) {
			for (int l = 0; l < plane2->edgesSize; l++) {
				if (plane1->edges[k]->isHole == plane2->edges[l]->isHole && plane1->edges[k]->canIntersectWithEdge(plane2->edges[l])) {
					if (areEdgesIntersect_CUDA(plane2, plane1, plane2->edges[l], plane1->edges[k])) {
						return true;
					}
				}
			}
		}
		for (int k = 0; k < plane2->edgesSize; k++) {
			changeBaseTo2D_CUDA(plane2->edges[k], plane2->normal, plane2->pointDirections[0]);
		}
	}
	return false;
}

__global__ 
void findPlaneGroups(size_t* groupPlaneIndices, Plane_CUDA** allPlanes, size_t planeCount)
{
	int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index < (planeCount * planeCount))
	{
		size_t index1 = index / planeCount;
		size_t index2 = index % planeCount;
		if (index1 < index2 && groupPlaneIndices[index2] == planeCount && index1 != index2)
		{
			if (arePlanesIntersecting_CUDA(allPlanes[index1], allPlanes[index2]))
				groupPlaneIndices[index2] = index1;
		}
	}
}

void groupPlanes()
{
	size_t allSize = 0;
	for (size_t i = 0; i < planes.size(); i++) {
		allSize += planes[i].size();
	}
	allPlanes.reserve(allSize);
	for (size_t i = 0; i < planes.size(); i++) {
		allPlanes.insert(allPlanes.end(), planes[i].begin(), planes[i].end());
	}
	for (size_t i = 0; i < allPlanes.size(); i++) {
		if (allPlanes[i]->edges.size() == 0) {
			delete allPlanes[i];
			allPlanes.erase(allPlanes.begin() + i);
			i--;
		}
	}
	auto start = std::chrono::steady_clock::now();

	std::vector<Plane*> tempPlanes;
	bool foundNewConnectionAtAll = true;
	while (foundNewConnectionAtAll) {
		foundNewConnectionAtAll = false;
		while (allPlanes.size() > 0) {
			bool foundNewConnection = true;
			while (foundNewConnection) {
				foundNewConnection = false;
				for (size_t i = 1; i < allPlanes.size(); i++) {
					bool foundNewConnection = arePlanesIntersecting(allPlanes[0], allPlanes[i]);
					if (foundNewConnection) {
						for (size_t j = 0; j < allPlanes[i]->edges.size(); j++) {
							allPlanes[0]->edges.push_back(allPlanes[i]->edges[j]);
						}
						allPlanes[0]->presentInFrames++;
						foundNewConnectionAtAll = true;
						delete allPlanes[i];
						allPlanes.erase(allPlanes.begin() + i);
						break;
					}
				}
			}
			tempPlanes.push_back(allPlanes[0]);
			allPlanes.erase(allPlanes.begin());
		}
		allPlanes = tempPlanes;
		tempPlanes.clear();
	}

	/*size_t* groupPlaneIndices;
	cudaMallocManaged(&groupPlaneIndices, allPlanes.size() * sizeof(size_t));
	for (size_t i = 0; i < allPlanes.size(); i++) {
		groupPlaneIndices[i] = allPlanes.size();
	}

	Plane_CUDA** allPlanes_CUDA;
	cudaMallocManaged(&allPlanes_CUDA, allPlanes.size() * sizeof(Plane_CUDA*));
	for (size_t i = 0; i < allPlanes.size(); i++) {
		Plane_CUDA* newPlane_CUDA;
		cudaMallocManaged(&newPlane_CUDA, sizeof(Plane_CUDA*));
		allPlanes_CUDA[i] = newPlane_CUDA;
		newPlane_CUDA->copyFromPlane(allPlanes[i]);
	}

	int blockSize = 256;
	int numBlocks = (pow(allPlanes.size(), 2) + blockSize - 1) / blockSize;

	findPlaneGroups << <numBlocks, blockSize >> > (groupPlaneIndices, allPlanes_CUDA, allPlanes.size());
	cudaDeviceSynchronize();


	for (size_t i = 0; i < allPlanes.size(); i++)
	{
		break;
		std::cout << groupPlaneIndices[i] << std::endl;
	}

	cudaFree(groupPlaneIndices);
	cudaFree(allPlanes_CUDA);*/

	auto end = std::chrono::steady_clock::now();
	std::cout << "Planes grouped --- Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;
	start = std::chrono::steady_clock::now();
	for (size_t i = 0; i < allPlanes.size(); i++) {
		if (allPlanes[i]->presentInFrames < 2) {
			delete allPlanes[i];
			allPlanes.erase(allPlanes.begin() + i);
			i--;
			continue;
		}
		size_t neighboursSizeNonHole = 0;
		size_t neighboursSizeHole = 0;
		for (size_t j = 0; j < allPlanes[i]->edges.size(); j++) {
			if (allPlanes[i]->edges[j]->isHole)
				neighboursSizeHole += allPlanes[i]->edges[j]->closestNeighbourPoints.size();
			else
				neighboursSizeNonHole += allPlanes[i]->edges[j]->closestNeighbourPoints.size();
		}
		allPlanes[i]->closestNeighbourPointsNonHole.reserve(neighboursSizeNonHole);
		allPlanes[i]->closestNeighbourPointsHole.reserve(neighboursSizeHole);
		for (size_t j = 0; j < allPlanes[i]->edges.size(); j++) {
			if (allPlanes[i]->edges[j]->isHole)
				allPlanes[i]->closestNeighbourPointsHole.insert(allPlanes[i]->closestNeighbourPointsHole.end(), allPlanes[i]->edges[j]->closestNeighbourPoints.begin(),
					allPlanes[i]->edges[j]->closestNeighbourPoints.end());
			else
				allPlanes[i]->closestNeighbourPointsNonHole.insert(allPlanes[i]->closestNeighbourPointsNonHole.end(), allPlanes[i]->edges[j]->closestNeighbourPoints.begin(),
					allPlanes[i]->edges[j]->closestNeighbourPoints.end());
		}
		relocateHoleNeighbours(allPlanes[i]);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "Relocated neighbours: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;
}

void filterNeighbours()
{
	auto start = std::chrono::steady_clock::now();
	for (size_t l = 0; l < 2; l++) {
		for (size_t i = 0; i < allPlanes.size(); i++) {
			for (size_t j = 0; j < (l == 0 ? allPlanes[i]->closestNeighbourPointsNonHole : allPlanes[i]->closestNeighbourPointsHole).size(); j++) {
				bool hasFoundEdge = false;
				for (size_t k = 0; k < allPlanes[i]->edges.size(); k++) {
					if (allPlanes[i]->edges[k]->isHole == (l != 0) && (isPointInsidePolygon(allPlanes[i]->edges[k]->getPoints(), (l == 0 ?
						allPlanes[i]->closestNeighbourPointsNonHole : allPlanes[i]->closestNeighbourPointsHole)[j].second,
						allPlanes[i]->edges[k]->xBounds2D, allPlanes[i]->edges[k]->yBounds2D, true, 0.0) == 0) == (l != 0)) 
					{
						hasFoundEdge = true;
						break;
					}
				}
				if (!hasFoundEdge) {
					(l == 0 ? allPlanes[i]->closestNeighbourPointsNonHole : allPlanes[i]->closestNeighbourPointsHole).erase((l == 0 ?
						allPlanes[i]->closestNeighbourPointsNonHole : allPlanes[i]->closestNeighbourPointsHole).begin() + j);
					j--;
				}
			}
		}
	}
	auto end = std::chrono::steady_clock::now();
	std::cout << "Neighbours filtered: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;
}

void fitPlanes()
{
	auto start = std::chrono::steady_clock::now();
	for (size_t j = 0; j < allPlanes.size(); j++) {
		std::vector<Edge*> tempCreatedEdges;
		auto holeNeighbours = allPlanes[j]->closestNeighbourPointsHole;
		auto nonHoleNeighbours = allPlanes[j]->closestNeighbourPointsNonHole;
		bool createdNewPolygon = true;
		bool foundNewIntersection = true;
		int counter = 0;
		while (foundNewIntersection) {
			foundNewIntersection = false;
			while (allPlanes[j]->edges.size() > 0) {
				for (size_t i = 1; i < allPlanes[j]->edges.size(); i++) {
					if (allPlanes[j]->edges[0]->isHole == allPlanes[j]->edges[i]->isHole) {
						std::vector<Edge*> innerCreatedEdges;
						/*if (false && dbg && i == 24 && counter == 13) {// currentFrame == 4 && savedEdge->isHole && counter == 3) {
							savedPoints.clear();
							addedPoints.clear();
							points.clear();
							changeBaseTo2D(createdEdges[0], { savedPlane->normal, savedPlane->pointDirections.first });
							changeBaseTo2D(createdEdges[i], { savedPlane->normal, savedPlane->pointDirections.first });
							for (size_t y = 0; y < createdEdges[0]->pointsWithDir.size(); y++) {
								createdEdges[0]->pointsWithDir[y].first->cornerId = currentCornerId;
								createdEdges[0]->pointsWithDir[y].first->cornerIndex = currentCornerIndex;
								savedPoints.push_back(createdEdges[0]->pointsWithDir[y].first);
								createdEdges[0]->pointsWithDir[y].first->position = createdEdges[0]->pointsWithDir[y].first->projected2DPosition;
								currentCornerIndex++;
							}
							currentCornerId++;
							currentCornerIndex = 0;
							for (size_t y = 0; y < createdEdges[i]->pointsWithDir.size(); y++) {
								createdEdges[i]->pointsWithDir[y].first->cornerId = currentCornerId;
								createdEdges[i]->pointsWithDir[y].first->cornerIndex = currentCornerIndex;
								savedPoints.push_back(createdEdges[i]->pointsWithDir[y].first);
								createdEdges[i]->pointsWithDir[y].first->position = createdEdges[i]->pointsWithDir[y].first->projected2DPosition;
								currentCornerIndex++;
							}

							for (size_t x = 0; x < createdEdges.size(); x++) {
								for (size_t y = 0; y < createdEdges[x]->closestNeighbourPoints.size(); y++) {
									savedPoints.push_back(new Point(createdEdges[x]->closestNeighbourPoints[y].first, 0, 0, nullptr));
								}
								currentCornerIndex = 0;
								for (size_t y = 0; y < createdEdges[x]->pointsWithDir.size(); y++) {
									createdEdges[x]->pointsWithDir[y].first->cornerId = 0;
									createdEdges[x]->pointsWithDir[y].first->cornerIndex = 0;
								}
								for (size_t y = 0; y < createdEdges[x]->pointsWithDir.size(); y++) {
									if (createdEdges[x]->pointsWithDir[y].first->isCorner && createdEdges[x]->pointsWithDir[y].first->cornerId == 0) {
										createdEdges[x]->pointsWithDir[y].first->cornerId = currentCornerId;
										createdEdges[x]->pointsWithDir[y].first->cornerIndex = currentCornerIndex;
										savedPoints.push_back(createdEdges[x]->pointsWithDir[y].first);
										currentCornerIndex++;
									}
									else if (createdEdges[x]->pointsWithDir[y].first->cornerId != 0) {
										std::cout << "asd";
									}
								}
								currentCornerId++;
							}
							writeData(4);
							std::cout << "asd";
						}*/
						createdNewPolygon = true;
						mergePolygons(allPlanes[j], allPlanes[j]->edges[0], allPlanes[j]->edges[i], allPlanes[j]->edges[0]->isHole ? holeNeighbours : nonHoleNeighbours, 
							innerCreatedEdges, createdNewPolygon);

						if (createdNewPolygon) {
							foundNewIntersection = true;
							for (size_t m = 0; m < innerCreatedEdges.size(); m++) {
								if (!isEdgeTooTigth(innerCreatedEdges[m]))
									tempCreatedEdges.push_back(innerCreatedEdges[m]);
							}
							allPlanes[j]->edges.erase(allPlanes[j]->edges.begin() + i);
							allPlanes[j]->edges.erase(allPlanes[j]->edges.begin());
							break;
						}
					}
					else
						createdNewPolygon = false;
				}
				counter++;
				if (!createdNewPolygon || allPlanes[j]->edges.size() == 1) {
					tempCreatedEdges.push_back(allPlanes[j]->edges[0]);
					allPlanes[j]->edges.erase(allPlanes[j]->edges.begin());
				}
			}
			allPlanes[j]->edges = tempCreatedEdges;
			tempCreatedEdges.clear();
		}
		std::cout << "Planes processed: " << allPlanes.size() << "/" << j + 1 << std::endl;
	}
	auto end = std::chrono::steady_clock::now();
	std::cout << "Planes fit: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;
}

void convexSegmentation()
{
	double notInUseRatio = 0;
	const std::pair<double, double> acceptAngle = { 181, 359 };
	size_t currentConvexId = 1;
	for (size_t i = 0; i < allPlanes.size(); i++) {
		for (size_t j = 0; j < allPlanes[i]->edges.size(); j++) {
			for (size_t k = 0; k < allPlanes[i]->edges[j]->pointsWithDir.size(); k++) {
				allPlanes[i]->edges[j]->pointsWithDir[k].first->convexId.clear();
				allPlanes[i]->edges[j]->pointsWithDir[k].first->convexIndex.clear();
			}
		}
		allPlanes[i]->convexFaces.clear();
	}
	for (size_t i = 0; i < allPlanes.size(); i++) {
		std::vector<Edge*> holeEdges;
		for (size_t j = 0; j < allPlanes[i]->edges.size(); j++) {
			if (allPlanes[i]->edges[j]->isHole && allPlanes[i]->edges[j]->isInUse) {
				holeEdges.push_back(allPlanes[i]->edges[j]);
			}
		}
		for (size_t x = 0; x < allPlanes[i]->edges.size(); x++) {
			if (!allPlanes[i]->edges[x]->isHole) {
				std::vector<Point*> remainingPoints;
				for (size_t k = 0; k < allPlanes[i]->edges[x]->pointsWithDir.size(); k++) {
					remainingPoints.push_back(allPlanes[i]->edges[x]->pointsWithDir[k].first);
				}
				if (!isClockwise(remainingPoints)) {
					for (size_t k = 0; k < remainingPoints.size(); k++) {
						remainingPoints.push_back(remainingPoints[remainingPoints.size() - 1 - k]);
						remainingPoints.erase(remainingPoints.begin() + remainingPoints.size() - 2 - k);
					}
				}
				while (remainingPoints.size() > 3) {
					std::vector<Point*> remainingPointsHelper(remainingPoints);
					std::vector<Point*> L = { remainingPointsHelper[0], remainingPointsHelper[1] };
					std::vector<Point*> remainingPointsHelperSave;
					std::vector<Point*> LSave;

					remainingPointsHelper.erase(remainingPointsHelper.begin(), remainingPointsHelper.begin() + 2);
					std::pair<double, double> xBounds = { std::min(L[0]->projected2DPosition.x, L[1]->projected2DPosition.x),
						std::max(L[0]->projected2DPosition.x, L[1]->projected2DPosition.x) };
					std::pair<double, double> yBounds = { std::min(L[0]->projected2DPosition.y, L[1]->projected2DPosition.y),
						std::max(L[0]->projected2DPosition.y, L[1]->projected2DPosition.y) };

					for (int j = 0; j < 2; j++) {
						bool isForward = j == 0;
						while (remainingPointsHelper.size() > 0) {
							auto newPoint = isForward ? remainingPointsHelper[0] : remainingPointsHelper[remainingPointsHelper.size() - 1];
							auto v1 = isForward ? L[L.size() - 1]->projected2DPosition - L[L.size() - 2]->projected2DPosition :
								(L[0]->projected2DPosition - L[1]->projected2DPosition);
							auto v2 = isForward ? newPoint->projected2DPosition - L[L.size() - 1]->projected2DPosition :
								(newPoint->projected2DPosition - L[0]->projected2DPosition);
							auto vecToBegin = isForward ? L[0]->projected2DPosition - newPoint->projected2DPosition :
								(L[L.size() - 1]->projected2DPosition - newPoint->projected2DPosition);
							auto vecAtBegin = isForward ? L[1]->projected2DPosition - L[0]->projected2DPosition :
								(L[L.size() - 2]->projected2DPosition - L[L.size() - 1]->projected2DPosition);
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
								if (newPoint->projected2DPosition.x < xBounds.first)
									xBounds.first = newPoint->projected2DPosition.x;
								if (newPoint->projected2DPosition.x > xBounds.second)
									xBounds.second = newPoint->projected2DPosition.x;
								if (newPoint->projected2DPosition.y < yBounds.first)
									yBounds.first = newPoint->projected2DPosition.y;
								if (newPoint->projected2DPosition.y > yBounds.second)
									yBounds.second = newPoint->projected2DPosition.y;
							}
							else {
								if (L.size() > 2) {
									bool containsCorner = true;
									while (containsCorner && L.size() > 2) {
										containsCorner = false;
										for (size_t k = 0; k < remainingPointsHelper.size(); k++) {
											if (isPointInsidePolygon(L, remainingPointsHelper[k]->projected2DPosition, xBounds, yBounds)) {
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
									auto lastPointPos = L[L.size() - 1]->projected2DPosition;
									if (L.size() > 2) {
										while (isNewPointFound) {
											double minIntersectionDistance = 1000;
											int closestEdgeIndex = -1;
											size_t closestPointIndex = 0;
											auto LTemp = L;
											if (absoluteClosestEdgeIndex != -1)
												LTemp.insert(LTemp.begin(), holeEdges[absoluteClosestEdgeIndex]->pointsWithDir[absoluteClosestPointIndex].first);
											isNewPointFound = false;
											for (size_t l = 0; l < holeEdges.size(); l++) {
												std::vector<bool> isPointsInside;
												auto edgePoints = holeEdges[l];
												size_t holeType = 0;
												for (size_t m = 0; m < edgePoints->pointsWithDir.size(); m++) {
													bool isHolePointInside = isPointInsidePolygon(LTemp,
														edgePoints->pointsWithDir[m].first->projected2DPosition, xBounds, yBounds);
													isPointsInside.push_back(isHolePointInside);
													if (isHolePointInside && holeType == 0)
														holeType = 1;
													if (m > 0 && isPointsInside[m - 1] != isPointsInside[m])
														holeType = 2;
												}
												if (holeType > 0) {
													for (size_t m = 0; m < edgePoints->pointsWithDir.size(); m++) {
														if (holeType == 2 && isPointsInside[m] != isPointsInside[(m + 1) % isPointsInside.size()] &&
															LTemp[0] != edgePoints->pointsWithDir[m].first && LTemp[0] !=
															edgePoints->pointsWithDir[(m + 1) % isPointsInside.size()].first) {
															size_t onEdgeType = 0;
															auto intersection = intersectionOfLines(LTemp[0]->projected2DPosition, lastPointPos,
																edgePoints->pointsWithDir[m].first->projected2DPosition,
																edgePoints->pointsWithDir[(m + 1) % isPointsInside.size()].first->projected2DPosition,
																onEdgeType, notInUseRatio);
															if (onEdgeType > 0 && (intersection - lastPointPos).length() < minIntersectionDistance) {
																minIntersectionDistance = (intersection - lastPointPos).length();
																closestEdgeIndex = l;
																if (((edgePoints->pointsWithDir[m].first->projected2DPosition - lastPointPos).length() <
																	(edgePoints->pointsWithDir[(m + 1) % isPointsInside.size()].first->projected2DPosition
																		- lastPointPos).length() &&
																	isPointInsidePolygon(LTemp, edgePoints->pointsWithDir[m].first->projected2DPosition,
																		xBounds, yBounds)) || ((edgePoints->pointsWithDir[m].first->projected2DPosition
																			- lastPointPos).length() >= (edgePoints->pointsWithDir[(m + 1) %
																				isPointsInside.size()].first->projected2DPosition - lastPointPos).length() &&
																			!isPointInsidePolygon(LTemp, edgePoints->pointsWithDir[(m + 1) %
																				isPointsInside.size()].first->projected2DPosition,
																				xBounds, yBounds)))
																	closestPointIndex = m;
																else
																	closestPointIndex = ((m + 1) % isPointsInside.size());
															}
														}
														else if (holeType == 1 && absoluteClosestEdgeIndex == -1) {
															if ((edgePoints->pointsWithDir[m].first->projected2DPosition - lastPointPos).length() <
																minIntersectionDistance) {
																minIntersectionDistance = (edgePoints->pointsWithDir[m].first->projected2DPosition -
																	lastPointPos).length();
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
											for (size_t l = 0; l < holeEdges[absoluteClosestEdgeIndex]->pointsWithDir.size() + 1; l++) {
												remainingPointsHelper.insert(remainingPointsHelper.begin() + l,
													holeEdges[absoluteClosestEdgeIndex]->pointsWithDir[(absoluteClosestPointIndex + l) %
													holeEdges[absoluteClosestEdgeIndex]->pointsWithDir.size()].first);
											}
											for (size_t m = 0; m < remainingPoints.size(); m++) {
												if (remainingPoints[m] == L[L.size() - 1]) {
													remainingPoints.insert(remainingPoints.begin() + m + 1, L[L.size() - 1]);
													for (size_t l = 0; l < holeEdges[absoluteClosestEdgeIndex]->pointsWithDir.size() + 1; l++) {
														remainingPoints.insert(remainingPoints.begin() + m + 1 + l,
															holeEdges[absoluteClosestEdgeIndex]->pointsWithDir[(absoluteClosestPointIndex + l) %
															holeEdges[absoluteClosestEdgeIndex]->pointsWithDir.size()].first);
													}
													break;
												}
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
							for (size_t m = 0; m < edgePoints->pointsWithDir.size(); m++) {
								if (isPointInsidePolygon(L, edgePoints->pointsWithDir[m].first->projected2DPosition, xBounds, yBounds) && minDistance >
									(L[0]->projected2DPosition - edgePoints->pointsWithDir[m].first->projected2DPosition).length()) {
									minDistance = (L[0]->projected2DPosition - edgePoints->pointsWithDir[m].first->projected2DPosition).length();
									closestEdgeIndex = l;
									closestPointIndex = m;
								}
							}
						}
						if (closestEdgeIndex >= 0) {
							remainingPointsHelper.insert(remainingPointsHelper.begin(), L[0]);
							for (size_t l = 0; l < holeEdges[closestEdgeIndex]->pointsWithDir.size() + 1; l++) {
								remainingPointsHelper.insert(remainingPointsHelper.begin() + l + 1,
									holeEdges[closestEdgeIndex]->pointsWithDir[(closestPointIndex + l) % holeEdges[closestEdgeIndex]->pointsWithDir.size()].first);
							}
							for (size_t l = 0; l < L.size(); l++) {
								remainingPointsHelper.insert(remainingPointsHelper.end(), L[l]);
							}
							L.clear();
							holeEdges.erase(holeEdges.begin() + closestEdgeIndex);
							remainingPoints = remainingPointsHelper;
						}
					}
					if (L.size() > 0) {
						std::vector<Point*> convexFace;
						for (size_t j = 0; j < L.size(); j++) {
							L[j]->convexId.push_back(currentConvexId);
							L[j]->convexIndex.push_back(j);
							convexFace.push_back(L[j]);
						}
						allPlanes[i]->convexFaces.push_back(convexFace);
						/*if (currentFrame == 36 && i == 7 && allPlanes[i]->convexFaces.size() == 81)
						{
							if (false) {
								savedPoints.clear();
								addedPoints.clear();
								points.clear();
								//changeBaseTo2D(createdEdges[0], { savedPlane->normal, savedPlane->pointDirections.first });
								//changeBaseTo2D(createdEdges[i], { savedPlane->normal, savedPlane->pointDirections.first });
								for (size_t y = 0; y < allPlanes[i]->edges.size(); y++) {
									currentCornerIndex = 0;
									for (size_t j = 0; j < allPlanes[i]->edges[y]->pointsWithDir.size(); j++) {
										allPlanes[i]->edges[y]->pointsWithDir[j].first->cornerId = currentCornerId;
										allPlanes[i]->edges[y]->pointsWithDir[j].first->cornerIndex = currentCornerIndex;
										savedPoints.push_back(allPlanes[i]->edges[y]->pointsWithDir[j].first);
										allPlanes[i]->edges[y]->pointsWithDir[j].first->position = allPlanes[i]->edges[y]->pointsWithDir[j].first->projected2DPosition;
										currentCornerIndex++;
									}
									currentCornerId++;
								}
								writeData(4);
								std::cout << "asd";
							}
							//return;
						}*/
						remainingPoints = remainingPointsHelper;
						currentConvexId++;
					}
				}
				if (remainingPoints.size() == 3) {
					std::vector<Point*> convexFace;
					for (size_t j = 0; j < remainingPoints.size(); j++) {
						remainingPoints[j]->convexId.push_back(currentConvexId);
						remainingPoints[j]->convexIndex.push_back(j);
						convexFace.push_back(remainingPoints[j]);
					}
					allPlanes[i]->convexFaces.push_back(convexFace);
					currentConvexId++;
				}
			}
		}
	}
}

void fitPointsToPlane()
{
	for (size_t i = 0; i < allPlanes.size(); i++) {
		auto normal = allPlanes[i]->normal;
		auto planePointPos = allPlanes[i]->planePointPos;
		for (size_t j = 0; j < allPlanes[i]->edges.size(); j++) {
			for (size_t k = 0; k < allPlanes[i]->edges[j]->pointsWithDir.size(); k++) {
				double dist = Vec3<double>::dot_product(normal, allPlanes[i]->edges[j]->pointsWithDir[k].first->position - planePointPos);
				allPlanes[i]->edges[j]->pointsWithDir[k].first->position = allPlanes[i]->edges[j]->pointsWithDir[k].first->position - normal * dist;
			}
		}
	}
}

void exportObjects(size_t pointCloudIndex)
{
	size_t objCounter = 0;
	size_t currentCornerId = 1;
	for (size_t i = 0; i < allPlanes.size(); i++) {
		std::vector<Point*> corners;
		size_t currentCornerIndex = 0;
		for (size_t j = 0; j < allPlanes[i]->edges.size(); j++) {
			for (size_t k = 0; k < allPlanes[i]->edges[j]->pointsWithDir.size(); k++) {
				if (allPlanes[i]->edges[j]->pointsWithDir[k].first->isCorner) {
					allPlanes[i]->edges[j]->pointsWithDir[k].first->cornerId = currentCornerId;
					allPlanes[i]->edges[j]->pointsWithDir[k].first->cornerIndex = currentCornerIndex;
					corners.push_back(allPlanes[i]->edges[j]->pointsWithDir[k].first);
					currentCornerIndex++;
				}
			}
			currentCornerId++;
		}
		if (corners.size() < 3 || allPlanes[i]->convexFaces.size() == 0)
			continue;
		std::ofstream MyFile("C:/Users/ungbo/Desktop/BME/_Diplomamunka/Diplomamunka/Diplomamunka/Assets/Resources/Generated_Models_test/processed_obj_"
			+ std::to_string(objCounter) + ".obj");
		MyFile << "o Mesh" << std::endl;
		for (size_t k = 0; k < corners.size(); k++) {
			MyFile << "v " << std::to_string(-corners[k]->position.x) << " " << std::to_string(corners[k]->position.y)
				<< " " << std::to_string(corners[k]->position.z) << std::endl;
		}

		for (size_t j = 0; j < allPlanes[i]->convexFaces.size(); j++) {
			MyFile << "f ";
			for (size_t k = 0; k < allPlanes[i]->convexFaces[j].size(); k++) {
				MyFile << allPlanes[i]->convexFaces[j][k]->cornerIndex + 1 << " ";
			}
			MyFile << std::endl;
			MyFile << "f ";
			for (int k = allPlanes[i]->convexFaces[j].size() - 1; k >= 0; k--) {
				MyFile << allPlanes[i]->convexFaces[j][k]->cornerIndex + 1 << " ";
			}
			MyFile << std::endl;
		}

		for (size_t j = 0; j < allPlanes[i]->edges.size(); j++) {
			int indexShift = -1;
			for (size_t k = 0; k < allPlanes[i]->edges[j]->pointsWithDir.size(); k++) {
				if (allPlanes[i]->edges[j]->pointsWithDir[k].first->isCorner) {
					if (indexShift == -1)
						indexShift = allPlanes[i]->edges[j]->pointsWithDir[k].first->cornerIndex;
					allPlanes[i]->edges[j]->pointsWithDir[k].first->cornerIndex -= indexShift;
				}
			}
		}
		MyFile << std::endl;
		MyFile.close();
		objCounter++;
	}
}

void processData(size_t pointCloudIndex)
{
	auto start = std::chrono::steady_clock::now();
	size_t endIndex = (pointCloudTestIndex == -1 ? std::max<size_t>(1, pointCloudCount) : 1);
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) 
	{
		groundSegmentation(currentFrame);
	}
	auto end = std::chrono::steady_clock::now();
	std::cout << "groundSegmentation: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;

	start = std::chrono::steady_clock::now();
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		egoCarSegmentation(currentFrame);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "egoCarSegmentation: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;

	start = std::chrono::steady_clock::now();
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		findPlanes(currentFrame);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "findPlanes: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;


	std::cout << "dbgCounter: Elapsed time in seconds : "
		<< dbgTimeCounter
		<< " sec" << std::endl;

	start = std::chrono::steady_clock::now();
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		findEdgePoints(currentFrame);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "findEdgePoints: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;

	start = std::chrono::steady_clock::now();
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		findCorners(currentFrame);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "findCorners: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;

	start = std::chrono::steady_clock::now();
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		findPlaneConnections(currentFrame);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "findPlaneConnections: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;

	start = std::chrono::steady_clock::now();
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		connectPlanes(currentFrame);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "connectPlanes: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;

	start = std::chrono::steady_clock::now();
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		createCorners(currentFrame);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "createCorners: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;

	start = std::chrono::steady_clock::now();
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		filterEdgePoints(currentFrame);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "filterEdgePoints: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;

	start = std::chrono::steady_clock::now();
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		extract2DPolygon(currentFrame);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "extract2DPolygon: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;

	start = std::chrono::steady_clock::now();
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		deleteSelfIntersections(planes[currentFrame]);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "deleteSelfIntersections: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;

	start = std::chrono::steady_clock::now();
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		deleteTigthEdges(currentFrame);
	}
	end = std::chrono::steady_clock::now();
	std::cout << "deleteTigthEdges: Elapsed time in seconds : "
		<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
		<< " sec" << std::endl;
	


	/*groundSegmentation(pointCloudIndex);
	egoCarSegmentation(pointCloudIndex);
	findPlanes(pointCloudIndex);
	findEdgePoints(pointCloudIndex);
	findCorners(pointCloudIndex);
	findPlaneConnections(pointCloudIndex);
	connectPlanes(pointCloudIndex);
	createCorners(pointCloudIndex);
	filterEdgePoints(pointCloudIndex);
	extract2DPolygon(pointCloudIndex);
	deleteSelfIntersections(planes[pointCloudIndex]);
	deleteTigthEdges(pointCloudIndex);*/
}

void connectData()
{
	groupPlanes();
	//filterNeighbours();
	fitPlanes();
	filterFittedPlanes();
	checkIfHolesAreContained();
	deleteSelfIntersections(allPlanes);
	convexSegmentation();
	fitPointsToPlane();
	exportObjects(currentFrame);
}

int main()
{
	size_t endIndex = (pointCloudTestIndex == -1 ? std::max<size_t>(1, pointCloudCount) : 1);
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		auto start = std::chrono::steady_clock::now();
		readData(currentFrame);
		auto end = std::chrono::steady_clock::now();
		std::cout << "Read " + std::to_string(currentFrame) + " Elapsed time in seconds : "
			<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
			<< " sec" << std::endl;
	}
	processData(currentFrame);
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		break;
		auto start = std::chrono::steady_clock::now();
		processData(currentFrame);
		auto end = std::chrono::steady_clock::now();
		std::cout << "It. " + std::to_string(currentFrame) + " Elapsed time in seconds : "
			<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
			<< " sec" << std::endl;
	}
	connectData();
	return 0;
}