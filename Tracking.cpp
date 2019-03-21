#include "Tracking.hpp"
#include "tools.hpp"
#include <iostream>
#include <string>
using namespace CVLab;
using namespace cv;
using namespace std;

Tracking::Tracking(const Calibration &c) : calib(c) {
}

Tracking::Tracking(const Tracking &other) : calib(other.calib) {
}

vector<vector<Point2f>> Tracking::operator()(const vector<Mat> &images, const vector<Point2f> &initMarkers) const {

	vector<vector<Point2f>> opticalflowoutput(images.size(),vector<Point2f>(initMarkers.size()));
	Mat status;
	Mat err;
	vector<Point2f> currentmarker = initMarkers;
	opticalflowoutput[0] = initMarkers;
	for (int i = 0; i < (images.size()-1); i++) {
		
		calcOpticalFlowPyrLK(images[i],images[i+1],currentmarker, opticalflowoutput[i+1],status,err);
		currentmarker = opticalflowoutput[i+1];

	}
       	
	return opticalflowoutput;
}
