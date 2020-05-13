#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"

//https://github.com/nlohmann/json/tree/develop/single_include/nlohmann
#include "nlohmann/json.hpp"

 // TUIO 1.1
#include "TuioServer.h"
#include "osc/OscTypes.h"

#include "mycamera.h"
#include "fingerTracker.h"

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void exit();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofImage colorImg;

	// parameters
	ofxIntSlider brightness_;
	ofxIntSlider exposure_;
	ofxIntSlider threshold_;
	ofxFloatSlider trackerThreshold_;
	ofxFloatSlider trackerMinAreaRadius_;
	ofxFloatSlider trackerMaxAreaRadius_;
	ofxPanel gui_;

	std::unique_ptr<FingerTracker> fingerTracker_;
	//FingerTracker* fingerTracker_;

	// camera calibration
    void calcCameraCalibration(int, int, float);

	// json
	void saveParam();
	void loadParam();
};
