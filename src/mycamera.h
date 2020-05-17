class MyCamBase {
public:
	MyCamBase(int reqW, int reqH)
	{
		w = reqW;
		h = reqH;
	}

	virtual void Start() = 0;
	virtual void Stop() = 0;
	virtual void Grab(std::function<void(cv::Mat)> func) = 0;
	virtual void SetExposure(int) {}

protected:
	ofPixels pixels_;
	int w, h;
};

class MyWebCam : public MyCamBase
{
public:
	MyWebCam(int w, int h) : MyCamBase(w, h) {}

	void Start() {
		VI.setupDevice(deviceId, w, h);
		w = VI.getWidth(deviceId);
		h = VI.getHeight(deviceId);
		pixels_.allocate(w, h, 4);

		//// Range for video setting 4: Min:0 Max:6 SteppingDelta:1 Default:2 Flags:2
		//long lmin, lmax, lsd, lc, lf, ldv;
		//VI.getVideoSettingFilter(deviceId, VI.propExposure, lmin, lmax, lsd, lc, lf, ldv);
		//VI.getVideoSettingFilter(deviceId, VI.propContrast, lmin, lmax, lsd, lc, lf, ldv);
	}

	void Stop() {
		VI.stopDevice(deviceId);
	}

	void Grab(std::function<void(cv::Mat)> func) {
		if (VI.isFrameNew(deviceId))
		{
			pixels_.setFromPixels(VI.getPixels(deviceId, true, false), w, h, OF_IMAGE_COLOR);
			func(ofxCv::toCv(pixels_));
		}
	}

private:
	videoInput VI;
	int deviceId = 0;
};

// OptiCam
#include "cameralibrary.h"
using namespace CameraLibrary;
class MyOptiCam : public MyCamBase
{
public:
	MyOptiCam(int w, int h) : MyCamBase(w, h) {}

	void Start()
	{
		CameraManager::X().WaitForInitialization();
		if (CameraManager::X().AreCamerasInitialized())
			printf("complete\n\n");
		else
			printf("failed\n\n");

		CameraList list;
		printf("Cameras:\n");
		for (int i = 0; i < list.Count(); i++)
			printf("Camera %d >> %s\n", i, list[i].Name());

		if (list.Count() == 0)
			printf("None\n");

		camera = CameraManager::X().GetCamera();
		w = camera->Width();
		h = camera->Height();

		pixels_.allocate(w, h, 4);
		framebuffer = std::make_unique<Bitmap>(w, h, 0, Bitmap::ThirtyTwoBit, pixels_.getData());

		camera->SetVideoType(Core::MJPEGMode);
		camera->Start();
		camera->SetTextOverlay(false);
		camera->SetIntensity(15);
		camera->SetExposure(55);
	}

	void Stop() {
		camera->Stop();
	}

	void Grab(std::function<void(cv::Mat)> func) {
		Frame* frame = NULL;
		Frame* latestFrame = camera->GetFrame();

		while (latestFrame) {
			if (frame)
			{
				frame->Release();
			}
			frame = latestFrame;
			latestFrame = camera->GetFrame();
		}

		if (frame) {
			frame->Rasterize(framebuffer.get());
			func(ofxCv::toCv(pixels_));
			frame->Release();
		}
	}

	void SetThreshold(int threshold) {
		if (camera == NULL) return;
		camera->SetThreshold(threshold);
	}

	void SetExposure(int exposure) {
		if (camera == NULL) return;
		camera->SetExposure(exposure);
	}

private:
	CameraLibrary::Camera* camera = NULL;
	std::unique_ptr<CameraLibrary::Bitmap> framebuffer = NULL;
};