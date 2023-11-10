#pragma once

struct Point;

struct Plane;

const size_t pointCloudBeginIndex = 33;
const int pointCloudTestIndex = 33;

bool checkIfBridge(Point* p, bool onlyMarked);