#pragma once

/************************************************************************/
/* Kinect Sensor (the old version of a Kinect)                          */
/************************************************************************/

#include "RGBDSensor.h"
#include <NuiApi.h>
#include <NuiSkeleton.h>

#include "GlobalCameraPoseOptState.h"
#include "GlobalAppState.h"

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"

class KinectSensor : public RGBDSensor
{
public:
	KinectSensor();
	//! Constructor; allocates CPU memory and creates handles

	//! Destructor; releases allocated ressources
	~KinectSensor();

	//! Initializes the sensor
	HRESULT createFirstConnected();

	//! gets the next depth frame
	HRESULT processDepth();

	//! maps the color to depth data and copies depth and color data to the GPU
	HRESULT processColor();

	//! toggles near mode if possible (only available on a windows Kinect)
	HRESULT toggleNearMode();

	//! Toggle enable auto white balance
	HRESULT toggleAutoWhiteBalance();
	

private:
	INuiSensor*		m_pNuiSensor;

	static const NUI_IMAGE_RESOLUTION   cDepthResolution = NUI_IMAGE_RESOLUTION_640x480;
	static const NUI_IMAGE_RESOLUTION   cColorResolution = NUI_IMAGE_RESOLUTION_640x480;

	HANDLE			m_hNextDepthFrameEvent;
	HANDLE			m_pDepthStreamHandle;
	HANDLE			m_hNextColorFrameEvent;
	HANDLE			m_pColorStreamHandle;

	LONG*			m_colorCoordinates;		// for mapping depth to color

	LONG			m_colorToDepthDivisor;

	bool			m_bDepthImageIsUpdated;
	bool			m_bDepthImageCameraIsUpdated;
	bool			m_bNormalImageCameraIsUpdated;

	bool			m_kinect4Windows;
	bool			m_bNearMode;

	// CHAO: newly added
	//std::string     m_strRGBDfolder;
	//USHORT*			m_depthPtr;
	//UCHAR*			m_colorPtr;
	//vector<std::string> m_strDepthImgName;
	//vector<std::string> m_strColorImgName;

//private:
//	void readRGBDAssociationFile(std::string filename);
//	bool readColorImgFromRGBData(std::string filename);
//	bool readDepthImgFromRGBData(std::string filename);
//	void readCameraPoseFromFile(std::string filename);
};
