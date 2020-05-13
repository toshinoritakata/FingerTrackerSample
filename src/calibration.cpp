#include "ofApp.h"

// グリッドの交点の数
// 一つのマスの大きさ(この単位は扱う座標系に依存する、メートルで入れるかミリで入れるかはアプリケーション次第
void ofApp::calcCameraCalibration(int width, int height, float squareSize)
{
	std::vector<cv::Point3f> objp;
	std::vector<cv::Point2f> corner_pts;
	std::vector<std::vector<cv::Point3f> > objpoints;
	std::vector<std::vector<cv::Point2f> > imgpoints;

	//float squareSize = 1.0;
	//const cv::Size patternsize(10, 7);
	cv::Size patternsize(width, height);
	for (int i = 0; i < patternsize.height; i++) {
		for (int j = 0; j < patternsize.width; j++) {
			objp.push_back(cv::Point3f(
				(float)j*squareSize,
				(float)i*squareSize, 0));
		}
	}

	// glob images
	std::vector<cv::String> images;
	std::string path = "C:/Work/Tracker/ofTracker/images/*.jpg";
	cv::glob(path, images);

	ofImage img;
	cv::Mat frame, gray;
	for (size_t i = 0; i < images.size(); i++)
	{
		//frame = cv::imread(images[i]);
		colorImg.load(images[i]);
		frame = ofxCv::toCv(colorImg).clone();
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

		// find checkerboard corners
		auto success = cv::findChessboardCorners(
			gray,
			patternsize,
			corner_pts,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (success)
		{
			// refine checkerboard corners
			cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
			cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
			cv::drawChessboardCorners(frame, patternsize, corner_pts, success);

			ofxCv::toOf(frame, colorImg);
			colorImg.update();

			objpoints.push_back(objp);
			imgpoints.push_back(corner_pts);
		}
	}

	/*
	 * Performing camera calibration by
	 * passing the value of known 3D points (objpoints)
	 * and corresponding pixel coordinates of the
	 * detected corners (imgpoints)
	*/
	cv::Mat cameraMatrix, distCoeffs, R, T;
	cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

	std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
	std::cout << "distCoeffs : " << distCoeffs << std::endl;
	std::cout << "Rotation vector : " << R << std::endl;
	std::cout << "Translation vector : " << T << std::endl;
}