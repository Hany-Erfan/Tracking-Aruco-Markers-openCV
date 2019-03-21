#include <opencv2/opencv.hpp>

#include "tools.hpp"
#include "Constants.hpp"
#include "Calibration.hpp"
#include "Sequence.hpp"
#include "Tracking.hpp"
#include "Triangulation.hpp"
#include <string>
#include <iostream>

using namespace CVLab;
using namespace cv;
using namespace std;

int main(int argc, char **argv) {
	try {
		// get calibration folder, sequence folder and output file from command line
		string calibFolder, sequenceFolder, outputFile;
		if (1) {
			calibFolder = string(argv[1]) + "/";

			sequenceFolder = string(argv[2]) + "/";
			outputFile = string(argv[3]);
		} else {
			cerr << "Please specify folder with calibration data, folder with sequence and output file" << endl;
			return EXIT_FAILURE;
		}

		// load calibration data
		logMessage("load calibration data from " + calibFolder);
		Calibration calib(calibFolder);
		logMessage("loaded calibration data");

		// load sequence
		logMessage("load sequence from " + sequenceFolder);
		Sequence sequence(sequenceFolder, calib);
		logMessage("finished loading sequence with " + to_string(sequence.getNumberOfFrames()) + " frames");
		
		Tracking tracking(calib);
		Triangulation triangulate(calib);

		

		// track the markers in the sequence

		logMessage("start tracking of markers");
		vector<Point2f> mark0 = sequence.getMarkers(0);
		vector<Point2f> mark1 = sequence.getMarkers(1);
		std::vector<std::vector<cv::Point2f>>track0= tracking(sequence[0], mark0);
		std::vector<std::vector<cv::Point2f>>track1 = tracking(sequence[1], mark1);	
		logMessage("finished tracking of markers");

		// triangulate the marker positions

		logMessage("start triangulation");
		vector<vector<Point3f>> triangleResult = triangulate(tracking(sequence[0], mark0), tracking(sequence[1], mark1));
		logMessage("finished triangulation");

		// calculate the motion of the markers

		logMessage("calculate motion of markers");
		vector<vector<Point3f>> motion = triangulate.calculateMotion(triangleResult);
		logMessage("finished calculation of motion of markers");

		// write the result to the output file
		logMessage("write results to " + outputFile);
		writeResult(outputFile, triangleResult);
		logMessage("finished writing results");

		// and exit program with code for success
		return EXIT_SUCCESS;
	} catch (const string &err) {
		// print error message and exit program with code for failure
		cerr << err << endl;
		return EXIT_FAILURE;
	}
}
