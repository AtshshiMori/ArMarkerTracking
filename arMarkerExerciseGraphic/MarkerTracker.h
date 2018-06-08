#pragma once
class MarkerTracker {
private:
	double kMarkerSize;

public:
	MarkerTracker(const double);
	bool findMarker(cv::Mat&, float[16]);

};