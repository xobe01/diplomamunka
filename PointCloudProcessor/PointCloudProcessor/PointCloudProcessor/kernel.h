#pragma once

struct Point;

struct Plane;

const size_t pointCloudCount = 2;
const size_t pointCloudBeginIndex = 0;
const int pointCloudTestIndex = -1;

bool checkIfBridge(Point* p, bool onlyMarked);