#include "Sequence.hpp"

#include "tools.hpp"
#include "Constants.hpp"

using namespace CVLab;
using namespace cv;
using namespace std;


Sequence::Sequence(const string &folder, const Calibration &c) : calib(c) {
	// read both videos
	readVideo(folder + Constants::sequence1File, images[0], calib.getCamera1(), calib.getDistortion1());
	readVideo(folder + Constants::sequence2File, images[1], calib.getCamera2(), calib.getDistortion2());

	// check if both videos have the same amount of frames
	if (images[0].size() != images[1].size()) {
		throw "both videos have different number of frames";
	}

	// load marker positions for both videos
	readMarkers(folder + Constants::markers1File, markers[0], images[0][0]);
	readMarkers(folder + Constants::markers2File, markers[1], images[1][0]);

	// check if both videos have the same amount of markers
	if (markers[0].size() != markers[1].size()) {
		throw "both videos have different number of markers";
	}
	// sort the markers so that they have the same ordering for both videos
	sortMarkers();
	
}

Sequence::Sequence(const Sequence &other) : calib(other.calib) {
	// loop over all cameras
	for (unsigned int camera = 0; camera < 2; ++camera) {
		// copy images
		images[camera].resize(other.images[camera].size());
		for (unsigned int frame = 0; frame < images[camera].size(); ++frame) {
			images[camera][frame] = other.images[camera][frame].clone();
		}

		// copy marker positions
		markers[camera] = other.markers[camera];
	}
}

unsigned int Sequence::getNumberOfFrames() const {
	return images[0].size();
}

const vector<Mat> & Sequence::operator[](unsigned int camera) const {
	// check camera index
	if (camera > 1) {
		throw "there are only two cameras";
	}

	// return sequence of images
	return images[camera];
}

vector<Point2f> Sequence::getMarkers(unsigned int camera) const {
	// check camera index
	if (camera > 1) {
		throw "there are only two cameras";
	}

	// return marker positions
	return markers[camera];
}

void Sequence::readVideo(const string &file, vector<Mat> &data, const Mat &K, const Mat &distortion) {
	// open video file
	VideoCapture vid(file);
	if (!vid.isOpened()) {
		throw "could not open video file " + file;
	}

	// get number of frames from the video file
	const unsigned int numberOfFrames = static_cast<unsigned int>(vid.get(CAP_PROP_FRAME_COUNT));
	
	// resize vector to number of frames
	data.clear();
	data.resize(numberOfFrames);

	// load images from video
	for (unsigned int i = 0; i < numberOfFrames; ++i) {
		Mat img, gray, undistorted;

		// load next frame
		vid >> img;

		// convert frame to grayscale
		cvtColor(img, gray, COLOR_BGR2GRAY);

		// undistort the image
		undistort(gray, undistorted, K, distortion);

		// save the undistorted image in the vector
		undistorted.copyTo(data[i]);
	}
}

void Sequence::readMarkers(const string &file, vector<Point2f> &data, const Mat &firstImage) {
	// read raw data from file
	Mat markerData = readMatrix(file);

	// check matrix dimension for validity
	checkMatrixDimensions(markerData, -1, 2, "marker positions");

	// resize vector to take marker positions
	data.clear();
	data.resize(markerData.rows);

	// save marker positions in the vector
	for (int i = 0; i < markerData.rows; ++i) {
		data[i].x = markerData.at<float>(i, 0);
		data[i].y = markerData.at<float>(i, 1);
	}

	// and refine the marker positions
	cornerSubPix(firstImage, data, Constants::markerRefinementWindowSize, Constants::markerRefinementZeroZone, Constants::markerRefinementCriteria);
}

void Sequence::sortMarkers() {

    
	Mat markerscam0(markers[0]);
	Rect Reccam0marker0(markerscam0.at<float>(0, 0)+10, markerscam0.at<float>(0, 1)-3, 20, 10);
	Rect Reccam0marker1(markerscam0.at<float>(1, 0)+10, markerscam0.at<float>(1, 1)-3, 20, 10);
	rectangle(images[0][0], Reccam0marker0, Scalar(255), 1, 8, 0);
	rectangle(images[0][0], Reccam0marker1, Scalar(255), 1, 8, 0); 
	Mat Roicam0marker0 = images[0][0](Reccam0marker0);
	Mat Roicam0marker1 = images[0][0](Reccam0marker1);
	Mat markerscam1(markers[1]);
	Rect Reccam1marker0(markerscam1.at<float>(0, 0), markerscam1.at<float>(0, 1)+5, 20, 10);
	Rect Reccam1marker1(markerscam1.at<float>(1, 0), markerscam1.at<float>(1, 1)-6, 20, 10);
	rectangle(images[1][0], Reccam1marker0, Scalar(255), 1, 8, 0);
	rectangle(images[1][0], Reccam1marker1, Scalar(255), 1, 8, 0);
	Mat Roicam1marker0 = images[1][0](Reccam1marker0);
	Mat Roicam1marker1 = images[1][0](Reccam1marker1);

	if(norm(Roicam0marker0, Roicam1marker0)>norm(Roicam0marker1, Roicam1marker1)){
			Point2f temp = markers[1][0];
			markers[1][0] = markers[1][1];
			markers[1][1] = temp;
					} 
}
		
	
