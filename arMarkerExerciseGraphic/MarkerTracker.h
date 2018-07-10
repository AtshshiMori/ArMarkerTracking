#pragma once
#include "stdafx.h"
#include <math.h>
#include <opencv2/opencv.hpp>
#include "PoseEstimation.h"
#include "MarkerTracker.h"

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

using namespace std;
using namespace cv;

class MarkerTracker {
private:
	double kMarkerSize;
	cv::Mat img;
	cv::Mat img_gray;

public:
	MarkerTracker(const double);
	void findMarker(cv::Mat&, map<int, vector<float>>);
	bool correctSide(vector<cv::Point>, vector<Vec4f>&);
	void getInters(vector<Vec4f>, vector<Point2f>&);
};