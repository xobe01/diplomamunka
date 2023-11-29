#include "kernel.h"

void processData()
{
	groundSegmentation();
	egoCarSegmentation();
	findPlanes();
	findEdgePoints();
	findCorners();
	findPlaneConnections();
	connectPlanes();
	createCorners();
	filterEdgePoints();
	extract2DPolygon();
	deleteSelfIntersections(planes);
	deleteTigthEdges();
	fitPlanes();
	deleteSelfIntersections(savedPlanes);
	saveSavedPoints();
}

void nonSequential()
{
	double avarageComputeTime = 0;
	size_t endIndex = (pointCloudTestIndex == -1 ? std::max<size_t>(1, pointCloudCount) : 1);
	for (currentFrame = (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
		currentFrame < endIndex; currentFrame++) {
		auto start = std::chrono::steady_clock::now();
		readData(currentFrame);
		auto end = std::chrono::steady_clock::now();
		std::cout << "Read " + std::to_string(currentFrame) + " Elapsed time in seconds : "
			<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
			<< " sec" << std::endl;
		start = std::chrono::steady_clock::now();
		processData();
		end = std::chrono::steady_clock::now();
		std::cout << "It. " + std::to_string(currentFrame) + " Elapsed time in seconds : "
			<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
			<< " sec" << std::endl;
		avarageComputeTime += (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000;
		if (currentFrame == endIndex - 1) {
			start = std::chrono::steady_clock::now();
			convexSegmentation();
			fitPointsToPlane();
			exportObjects(currentFrame);
			writeData(currentFrame);
			end = std::chrono::steady_clock::now();
			std::cout << "Write " + std::to_string(currentFrame) + " Elapsed time in seconds : "
				<< (double)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000
				<< " sec" << std::endl;
		}
		//clearMemory();
	}
	avarageComputeTime /= endIndex - (pointCloudTestIndex == -1 ? pointCloudBeginIndex : 0);
	std::cout << "Avg computation time: " << avarageComputeTime << std::endl;
}