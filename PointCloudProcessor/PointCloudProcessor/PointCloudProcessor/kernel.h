#pragma once

struct Point;

struct Plane;

const size_t pointCloudCount = 50;
const size_t pointCloudBeginIndex = 0;
const int pointCloudTestIndex = -1;
size_t currentFrame = 0;

bool checkIfBridge(Point* p, bool onlyMarked);