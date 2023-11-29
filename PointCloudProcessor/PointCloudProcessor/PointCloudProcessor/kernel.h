#pragma once

struct Point;

struct Plane;

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
	std::vector<Edge*> intersectedEdges;
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

struct Plane {
	std::vector<Point*> points;
	std::vector<Edge*> edges; //wasFirstGenerated, startpoint, points, direction
	Vec3<double> planePointPos;
	Vec3<double> normal;
	std::pair<Vec3<double>, Vec3<double>> pointDirections;
	size_t id;
	bool isNewlyCreated = true;
	std::vector<std::vector<Point*>> convexFaces;
	std::pair<Vec3<double>, Vec3<double>> furthestNormalPoints = { {0,0,0}, {0,0,0} };
	void calculateAvaragePointPos()
	{
		std::pair<double, double> normalDistances = { 1000, -1000 };
		for (size_t i = 0; i < points.size(); i++) 
		{
			double dist = Vec3<double>::dot_product(normal, points[i]->position - planePointPos);
			if (dist < normalDistances.first) 
			{
				normalDistances.first = dist;
				furthestNormalPoints.first = points[i]->position;
			}
			if (dist > normalDistances.second) {
				normalDistances.second = dist;
				furthestNormalPoints.second = points[i]->position;
			}
		}
	}
};

const double PI = 3.14159265359;
const std::pair<double, double> rayAngles = { -18, 20 };

const size_t pointCloudCount = 99;
const size_t pointCloudBeginIndex = 0;
const int pointCloudTestIndex = -1;
Vec3<double> egoCarPos;
size_t currentFrame = 0;

bool checkIfBridge(Point* p, bool onlyMarked);
void writeData(size_t pointCloudIndex);
bool isEdgeTooTigth(Edge* edge);