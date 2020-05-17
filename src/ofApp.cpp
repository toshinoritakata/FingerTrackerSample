#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	gui_.setup();
	gui_.add(exposure_.setup("camera exposure", 55, 0, 480));
	gui_.add(trackerThreshold_.setup("tracker threshold", 240, 0, 255));
	gui_.add(trackerMinAreaRadius_.setup("tracker min radius", 10, 1, 100));
	gui_.add(trackerMaxAreaRadius_.setup("tracker max radius", 50, 1, 300));

	fingerTracker_ = std::make_unique<FingerTracker>();

	loadParam();

	auto inpCam = new MyOptiCam(640, 480);
	//auto inpCam = new MyWebCam(640, 480);

	fingerTracker_->StartInputCamera(inpCam);
	fingerTracker_->startThread(true);
	colorImg.allocate(640, 480, OF_IMAGE_COLOR);
}

void ofApp::update()
{
	fingerTracker_->SetCameraExposure(exposure_);
	fingerTracker_->SetFinderParam(trackerThreshold_, trackerMinAreaRadius_, trackerMaxAreaRadius_);
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofSetBackgroundColor(50, 10, 10);
	ofSetColor(255);

	fingerTracker_->GetImage(colorImg);
	colorImg.update();
	colorImg.draw(0, 0);

	fingerTracker_->draw();

	// draw FPS
	ofSetColor(0, 0, 255);
	auto msg = "fps: " + ofToString(ofGetFrameRate(), 0);
	ofDrawBitmapString(msg, 500, 20);

	// draw GUI
	gui_.draw();
}

void ofApp::exit() {
	fingerTracker_->stopThread();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == ' ') {
		if (fingerTracker_->IsCalibMode()) {
			fingerTracker_->ExitCalibMode();
		}
		else {
			fingerTracker_->EnterCalibMode();
		}
		saveParam();

	}
}

void ofApp::loadParam() {
	nlohmann::json j;
	std::ifstream ifs("data.json");
	ifs >> j;
	ifs.close();

	std::vector<ofVec2f> rect;
	for (size_t i = 0; i < 4; ++i)
	{
		rect.push_back(ofVec2f(j["rect"][i * 2], j["rect"][i * 2 + 1]));
	}
	fingerTracker_->SetPerspective(rect);

	exposure_ = j["camera"]["exposure"];
	trackerMaxAreaRadius_ = j["tracker"]["maxAreaRadius"];
	trackerMinAreaRadius_ = j["tracker"]["minAreaRadius"];
	trackerThreshold_ = j["tracker"]["threshold"];
}

void ofApp::saveParam() {
	nlohmann::json j;
	std::array<float, 8> rect;
	for (size_t i = 0; i < 4; ++i)
	{
		auto p = fingerTracker_->pts_src[i];
		rect[i*2] = p.x;
		rect[i*2 + 1] = p.y;
	}
	j["rect"] = rect;
	j["camera"]["exposure"] = (int)this->exposure_;
	j["tracker"]["maxAreaRadius"] = (float)this->trackerMaxAreaRadius_;
	j["tracker"]["minAreaRadius"] = (float)this->trackerMinAreaRadius_;
	j["tracker"]["threshold"] = (float)this->trackerThreshold_;

	std::ofstream ofs("data.json");
	ofs << j.dump(4) << std::endl;
	ofs.close();
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
	if (fingerTracker_->IsCalibMode()) {
		fingerTracker_->MoveClosestPoint(x, y);
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	if (fingerTracker_->IsCalibMode()) {
		fingerTracker_->PickClosestPoint(x, y);
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
	if (fingerTracker_->IsCalibMode()) {
		fingerTracker_->SetCalib();
	}
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
