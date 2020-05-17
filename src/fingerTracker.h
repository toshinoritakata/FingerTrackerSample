class FingerFollower : public ofxCv::RectFollower {
protected:
	ofColor color;
	float startedDying_;
	float startedNasent_;
	float dyingTime_;
	float nasentTime_;
	ofPolyline trail_;

public:
	ofVec3f cur, smooth;
	enum { NASENT, BORN, ALIVE, DEAD } state_;

	FingerFollower() :
		startedDying_(0),
		startedNasent_(0),
		dyingTime_(1),
		nasentTime_(0.5) {}

	void setup(const cv::Rect& track) {
		startedNasent_ = ofGetElapsedTimef();

		smooth = ofxCv::toOf(track).getCenter();
		state_ = NASENT;
		trail_.clear();
	}

	void update(const cv::Rect track) {
		if (state_ == BORN)
			state_ = ALIVE;

		if (state_ == ALIVE) {
			startedDying_ = 0;

			cur = ofxCv::toOf(track).getCenter();
			smooth.interpolate(cur, .5);

			trail_.addVertex(smooth);
			if (trail_.size() > 30)
				trail_.removeVertex(0);
		}
		else if (state_ == NASENT) {
			float curTime = ofGetElapsedTimef();
			if (curTime - startedNasent_ > nasentTime_) {
				color.setHsb(ofRandom(0, 255), 255, 255);
				state_ = BORN;
			}
		}
	}

	void kill() {
		float curTime = ofGetElapsedTimef();
		if (state_ == ALIVE) {
			if (startedDying_ == 0) {
				startedDying_ = curTime;
			}
			else if (curTime - startedDying_ > dyingTime_) {
				//dead = true;
				state_ = DEAD;
				//trail_.clear();
			}
		}
		else {
			//dead = true;
			state_ = DEAD;
		}
	}
	
	void terminate()
	{
		dead = true;
		state_ = DEAD;
		trail_.clear();
	}

	void draw() {
		if (state_ != ALIVE) return;

		float curTime = ofGetElapsedTimef();
		float age = curTime - nasentTime_;

		ofPushStyle();

		float size = 16;
		ofSetColor(0, 0, 255);

		if (startedDying_) {
			ofSetColor(ofColor::red);
			size = ofMap(ofGetElapsedTimef() - startedDying_, 0, dyingTime_, size, 0, true);
		}

		ofNoFill();
		auto label = this->getLabel();
		ofSeedRandom(label << 24);
		ofSetColor(ofColor::fromHsb(ofRandom(255), 255, 255));

		ofDrawCircle(cur, size);
		ofDrawBitmapString(ofToString(label), cur.x, cur.y);

		ofSetColor(0, 255, 255);
		switch (state_)
		{
		case FingerFollower::NASENT:
			ofDrawBitmapString("NASENT", cur.x, cur.y-10);
			break;
		case FingerFollower::BORN:
			ofDrawBitmapString("BORN", cur.x, cur.y-10);
			break;
		case FingerFollower::ALIVE:
			ofDrawBitmapString("ALIVE", cur.x, cur.y-10);
			break;
		case FingerFollower::DEAD:
			ofDrawBitmapString("DEAD", cur.x, cur.y-10);
			break;
		default:
			break;
		}

		trail_.draw();
		ofPopStyle();
	}
};


class FingerTracker : public ofThread
{
public:
	FingerTracker()
		: pm_(cv::Mat::eye(3, 3, CV_32F)),
		isCalibMode_(false),
		inputCamera_(NULL),
		pickOffset_(ofVec2f(0, 0))
	{
		tuioServer_ = std::make_unique<TUIO::TuioServer>();
		contourFinder_ = std::make_unique<ofxCv::ContourFinder>();
		tracker_ = std::make_unique<ofxCv::RectTrackerFollower<FingerFollower>>();

		tuioServer_->setSourceName("ofTracker");
		tuioServer_->enableObjectProfile(false);
		tuioServer_->enableBlobProfile(false);
		reset_rect();
	}

	~FingerTracker() {
	}

	void StartInputCamera(MyCamBase* cam)
	{
		inputCamera_ = cam;
		inputCamera_->Start();
	}

	void StopInputCamera()
	{
		if (inputCamera_ != NULL)
			inputCamera_->Stop();
	}

	void GetImage(ofImage& image) { 
		lock();
		ofxCv::toOf(resultImg_, image);
		unlock();
	}

	void Gamma(cv::Mat src, cv::Mat dst, double gamma)
	{
		uchar LUT[256];
		for (int i = 0; i < 256; i++) {
			LUT[i] = (int)(pow((double)i / 255.0, gamma) * 255.0);
		}

		cv::Mat lut_mat = cv::Mat(1, 256, CV_8UC1, LUT);

		cv::LUT(src, lut_mat, dst);
	}

private:
	void FingerTracker::threadedFunction()
	{
		contourFinder_->setAutoThreshold(true);

		while (isThreadRunning()) {
			inputCamera_->Grab([this](cv::Mat img) {
				cv::cvtColor(img.clone(), gray_, cv::COLOR_RGB2GRAY);
				cv::warpPerspective(gray_, gray_, pm_, gray_.size(), cv::INTER_NEAREST);
				cv::resize(gray_, gray_, cv::Size(640/2, 480/2), 0, 0, 0);
				cv::GaussianBlur(gray_, gray_, cv::Size(9, 9), 0, 0);
				Gamma(gray_, gray_, 10);

				lock();
				resultImg_ = (isCalibMode_) ? img : gray_.clone();
				contourFinder_->findContours(gray_);
				tracker_->track(contourFinder_->getBoundingRects());
				sendTUIOData();
				unlock();
				});
			Sleep(2);
		}
	}


	void sendTUIOData()
	{
		tuioServer_->initFrame(TUIO::TuioTime::getSessionTime());
		for each (auto follower in  tracker_->getFollowers())
		{
			auto label = follower.getLabel();
			auto center = follower.smooth;

			center.x /= (640/2);
			center.y /= (480/2);
			switch (follower.state_)
			{
			case FingerFollower::BORN:
				cursors_[label] = tuioServer_->addTuioCursor(center.x, center.y);
				break;

			case FingerFollower::ALIVE:
				tuioServer_->updateTuioCursor(cursors_[label], center.x, center.y);
				break;

			case FingerFollower::DEAD:
				tuioServer_->removeTuioCursor(cursors_[label]);
				follower.terminate();
				cursors_.erase(label);
				break;

			default:
				break;
			}
		}

		tuioServer_->commitFrame();
	}

public:
	void SetCameraExposure(int arg) {
		lock();
		inputCamera_->SetExposure(arg);
		unlock();
	}

	void EnterCalibMode() { 
		isCalibMode_ = true; 
		picked_ = -1;
	}

	void SetCalib() {
		SetPerspective(pts_src);
	}

	void ExitCalibMode() { 
		isCalibMode_ = false;
		picked_ = -1;
	}
	bool IsCalibMode() { return isCalibMode_; }

	void MoveClosestPoint(int x, int y) {
		if (picked_ > -1) {
			pts_src[picked_] = ofVec2f(x, y) + pickOffset_;
		}
	}

	void PickClosestPoint(int x, int y) {
		auto cls = 9999999.9;
		for (size_t i = 0; i < 4; i++)
		{
			auto v = pts_src[i] - ofVec2f(x, y);
			auto d = v.length();
			if (cls > d) {
				picked_ = i;
				cls = d;
				pickOffset_ = v;
			}
		}
	}

	void draw() {
		DrawSrcRect();

		lock();
		ofSetColor(255, 0, 0);
		contourFinder_->draw();

		for each (auto follower in  tracker_->getFollowers()) {
			follower.draw();
		}
		unlock();
	}

	void reset_rect() {
		pts_src.clear();
		pts_src.push_back(ofVec2f(0, 0));
		pts_src.push_back(ofVec2f(640 - 1, 0));
		pts_src.push_back(ofVec2f(640 - 1, 480 - 1));
		pts_src.push_back(ofVec2f(0, 480 - 1));
		SetPerspective(pts_src);
	}

	void DrawSrcRect() {
		if (isCalibMode_) {
			ofNoFill();
			ofSetColor(255, 0, 0);
			ofDrawCircle(pts_src[0], 10);
			ofDrawBitmapString("0", pts_src[0]);
			ofDrawLine(pts_src[0], pts_src[1]);

			ofSetColor(0, 255, 0);
			ofDrawCircle(pts_src[1], 10);
			ofDrawBitmapString("1", pts_src[1]);
			ofDrawLine(pts_src[1], pts_src[2]);

			ofSetColor(0, 0, 255);
			ofDrawCircle(pts_src[2], 10);
			ofDrawBitmapString("2", pts_src[2]);
			ofDrawLine(pts_src[2], pts_src[3]);

			ofSetColor(255, 255, 0);
			ofDrawCircle(pts_src[3], 10);
			ofDrawBitmapString("3", pts_src[3]);
			ofDrawLine(pts_src[3], pts_src[0]);
			ofFill();
		}
	}

	/*
	 (0,0)        (640-1,0)
	    0--------- 1
	    |          |
	    |          |
	    3----------2
	 (0,480-1)    (640-1,480-1)
	*/
	void SetPerspective(std::vector<ofVec2f> rect) {
		if (rect.size() != 4) return;

		pts_src.clear();
		std::vector<cv::Point2f> src;
		for (size_t i = 0; i < 4; i++)
		{
			pts_src.push_back(rect[i]);
			src.push_back(cv::Point2f(rect[i].x, rect[i].y));
		}

		std::vector<cv::Point2f> dst;
		dst.push_back(cv::Point2f(0, 0));
		dst.push_back(cv::Point2f(640 - 1, 0));
		dst.push_back(cv::Point2f(640 - 1, 480 - 1));
		dst.push_back(cv::Point2f(0, 480 - 1));

		lock();
		pm_ = cv::getPerspectiveTransform(src, dst);
		unlock();
	}

public:
	void SetFinderParam(int th, int minar, int maxar) {
		threshold_ = th;
		minAreaRadius_ = minar;
		maxAreaRadius_ = maxar;
		lock();
		contourFinder_->setThreshold(threshold_);
		contourFinder_->setMinAreaRadius(minAreaRadius_);
		contourFinder_->setMaxAreaRadius(maxAreaRadius_);
		tracker_->setPersistence(15);	// wait for half a second before forgetting something
		tracker_->setMaximumDistance(32);	// an object can move up to 32 pixels per frame
		unlock();
	}

	ofxCv::ContourFinder* ContourFinder() {
		return contourFinder_.get();
	}


private:
	int threshold_;
	int minAreaRadius_;
	int maxAreaRadius_;
	cv::Mat pm_;
	cv::Mat resultImg_;
	bool isCalibMode_;
	cv::Mat img_;
	cv::Mat gray_;
	cv::Mat bg_;
	ofVec2f pickOffset_;
	int picked_;

	MyCamBase* inputCamera_;
	std::map<int, TUIO::TuioCursor*> cursors_;

	std::unique_ptr<TUIO::TuioServer> tuioServer_;
	std::unique_ptr<ofxCv::ContourFinder> contourFinder_;
	std::unique_ptr<ofxCv::RectTrackerFollower<FingerFollower> > tracker_;

public:
	std::vector<ofVec2f> pts_src;
};