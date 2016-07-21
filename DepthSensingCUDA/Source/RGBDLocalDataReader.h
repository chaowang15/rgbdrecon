#pragma once

#include "GlobalAppState.h"
#include "RGBDSensor.h"
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"

class RGBDLocalDataReader : public RGBDSensor
{
public:

	//! Constructor; allocates CPU memory and creates handles
	RGBDLocalDataReader();

	//! Destructor; releases allocated ressources
	~RGBDLocalDataReader();

	//! Initializes the sensor
	HRESULT createFirstConnected();

	//! Processes the depth data (and color)
	HRESULT processDepth();

	//! Processes the Kinect color data
	HRESULT processColor();

	//! Toggles the Kinect to near-mode; default is far mode
	HRESULT toggleNearMode()
	{
		// PrimeSense is always in near mode
		return S_OK;
	}

	//! Toggle enable auto white balance
	HRESULT toggleAutoWhiteBalance()
	{
		return S_OK;
	}

	
private:
	void readRGBDAssociationFile(std::string filename);
	bool readColorImgFromRGBData(std::string filename);
	bool readDepthImgFromRGBData(std::string filename);
	void readCameraPoseFromFile(std::string filename);

};


