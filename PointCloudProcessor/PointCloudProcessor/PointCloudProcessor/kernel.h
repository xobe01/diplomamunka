#pragma once

struct Point;

struct Plane;

const size_t pointCloudBeginIndex = 35;
const int pointCloudTestIndex = 35;

bool checkIfBridge(Point* p, bool onlyMarked);