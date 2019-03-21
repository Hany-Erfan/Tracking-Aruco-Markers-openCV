#include "Triangulation.hpp"
#include "Tools.hpp"


using namespace CVLab;
using namespace cv;
using namespace std;





Triangulation::Triangulation(const Calibration &c) : calib(c) {
}

Triangulation::Triangulation(const Triangulation &other) : calib(other.calib) {
}

vector<Point3f> Triangulation::operator()(const vector<Point2f> &markers1, const vector<Point2f> &markers2) const {
	float data[12] = { 1, 0,0,0,0,1, 0, 0, 0,0,1,0,};
	Mat extra = (Mat_<float>(1, 4) << 0, 0, 0, 1);
	Mat output;
	Mat K1 = Mat(calib.getCamera1());
	Mat K2 = Mat(calib.getCamera2());
	Mat D1 = Mat(calib.getDistortion1());
	Mat D2 = Mat(calib.getDistortion2());
	Mat P1 = K1* Mat(3, 4, CV_32F, data);
	Mat P2 = K2* Mat(calib.getTransCamera1Camera2());
	Mat toWorld = Mat(calib.getTransCamera1World());
	vector<Point3f> triangulationResult;
	vector<Point2f> correctedM1;
	vector<Point2f> correctedM2;
	vector<Point2f> undiscorrectedM1;
	vector<Point2f> undiscorrectedM2;
	correctMatches(calib.getFundamentalMat(), markers1, markers2, correctedM1, correctedM2);

	triangulatePoints(P1, P2, correctedM1, correctedM2, output);
	toWorld.push_back(extra); 
	Mat triangulateWorld =toWorld*output;
    convertPointsFromHomogeneous(triangulateWorld.t(), triangulationResult);
	return triangulationResult;
 
}

vector<vector<Point3f>> Triangulation::operator()(const vector<vector<Point2f>> &markers1, const vector<vector<Point2f>> &markers2) const {
	// do nothing if there is no data
	if (markers1.empty()) {
		return vector<vector<Point3f>>();
	}

	// check for same number of frames
	if (markers1.size() != markers2.size()) {
		throw "different number of frames";
	}

	// create result vector
	vector<vector<Point3f>> result(markers1.size());

	// trinagulate each frame for itself and store result
	for (unsigned int i = 0; i < markers1.size(); ++i) {
		result[i] = (*this)(markers1[i], markers2[i]);
	}
	//showTriangulation(result);
	// and return result
	return result;
}

vector<vector<Point3f>> Triangulation::calculateMotion(const vector<vector<Point3f>> &data) {
	vector<vector<Point3f>> motion ;
	vector<Point3f> resultmotion;

	for (int i = 0; i <data.size(); i++) {
		subtract(data[i], data[0],resultmotion);
		motion.push_back(resultmotion);
	}
	return motion;
}
