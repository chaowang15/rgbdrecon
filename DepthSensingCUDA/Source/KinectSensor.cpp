
#include "stdafx.h"

#include "KinectSensor.h"
#include <fstream>

KinectSensor::KinectSensor()
{
	// get resolution as DWORDS, but store as LONGs to avoid casts later
	DWORD width = 0;
	DWORD height = 0;

	NuiImageResolutionToSize(cDepthResolution, width, height);
	unsigned int depthWidth = static_cast<unsigned int>(width);
	unsigned int depthHeight = static_cast<unsigned int>(height);

	NuiImageResolutionToSize(cColorResolution, width, height);
	unsigned int colorWidth = static_cast<unsigned int>(width);
	unsigned int colorHeight = static_cast<unsigned int>(height);

	RGBDSensor::init(depthWidth, depthHeight, colorWidth, colorHeight);

	m_colorToDepthDivisor = colorWidth / depthWidth;

	m_hNextDepthFrameEvent = INVALID_HANDLE_VALUE;
	m_pDepthStreamHandle = INVALID_HANDLE_VALUE;
	m_hNextColorFrameEvent = INVALID_HANDLE_VALUE;
	m_pColorStreamHandle = INVALID_HANDLE_VALUE;

	m_colorCoordinates = new LONG[depthWidth*depthHeight * 2];

	m_bDepthImageIsUpdated = false;
	m_bDepthImageCameraIsUpdated = false;
	m_bNormalImageCameraIsUpdated = false;

	initializeDepthIntrinsics(2.0f*NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240, 2.0f*NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240, 320.0f, 240.0f);
	initializeColorIntrinsics(2.0f*NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240, 2.0f*NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240, 320.0f, 240.0f);

	//if (GlobalCameraPoseOptState::getInstance().s_bReadRGBData)
	//{
	//	initializeDepthIntrinsics(GlobalCameraPoseOptState::getInstance().s_fx, GlobalCameraPoseOptState::getInstance().s_fy,
	//		GlobalCameraPoseOptState::getInstance().s_cx, GlobalCameraPoseOptState::getInstance().s_cy);
	//	initializeColorIntrinsics(GlobalCameraPoseOptState::getInstance().s_fx, GlobalCameraPoseOptState::getInstance().s_fy,
	//		GlobalCameraPoseOptState::getInstance().s_cx, GlobalCameraPoseOptState::getInstance().s_cy);
	//}
	//else
	//{
	//	initializeDepthIntrinsics(2.0f*NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240, 2.0f*NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240, 320.0f, 240.0f);
	//	initializeColorIntrinsics(2.0f*NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240, 2.0f*NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240, 320.0f, 240.0f);
	//}

	initializeDepthExtrinsics(mat4f::identity());
	//MLIB_WARNING("TODO initialize color intrs/extr");
	initializeColorExtrinsics(mat4f::identity());

	// Create RGB-D folder for saving RGB-D images scanned by Kinect
	if (GlobalAppState::getInstance().s_bSaveRGBDImages)
	{
		m_strRGBDfolder = "./rgbd/";
		//m_strRGBDfolder = util::directoryFromPath(m_strRGBDfolder);
		if (!util::directoryExists(m_strRGBDfolder)) {
			util::makeDirectory(m_strRGBDfolder);
		}
		std::string subfoldername = m_strRGBDfolder + "scan";
		int maxFolderIdx = 100;
		for (int i = 0; i != maxFolderIdx; ++i)
		{
			std::string str = subfoldername + std::to_string(i);
			if (!util::directoryExists(str)){
				m_strRGBDfolder = str;
				util::makeDirectory(m_strRGBDfolder);
				util::makeDirectory(m_strRGBDfolder + "/rgb/");
				util::makeDirectory(m_strRGBDfolder + "/depth/");
				break;
			}
		}

		m_depthPtr = new USHORT[getDepthWidth()*getDepthHeight()];
		m_colorPtr = new UCHAR[getDepthWidth()*getDepthHeight() * 3];
	}
	//if (GlobalCameraPoseOptState::getInstance().s_bReadRGBData)
	//{
	//	std::string filename = GlobalCameraPoseOptState::getInstance().s_strDataPath + GlobalCameraPoseOptState::getInstance().s_strAssociationFile;
	//	readRGBDAssociationFile(filename);
	//	GlobalCameraPoseOptState::getInstance().s_uCurrentFrameIndex = GlobalCameraPoseOptState::getInstance().s_uMinimumFrameIndex;
	//	std::cout << "Reading RGB-D data ... " << std::endl;
	//}
	//if (GlobalCameraPoseOptState::getInstance().s_bReadCameraPoseFromFile)
	//{
	//	std::string filename = GlobalCameraPoseOptState::getInstance().s_strDataPath + GlobalCameraPoseOptState::getInstance().s_strTrajFile;
	//	readCameraPoseFromFile(filename);
	//}
}

KinectSensor::~KinectSensor()
{
	if (NULL != m_pNuiSensor)
	{
		m_pNuiSensor->NuiShutdown();
		m_pNuiSensor->Release();
	}

	CloseHandle(m_hNextDepthFrameEvent);
	CloseHandle(m_hNextColorFrameEvent);

	// done with pixel data
	SAFE_DELETE_ARRAY(m_colorCoordinates);
}

HRESULT KinectSensor::createFirstConnected()
{
	INuiSensor* pNuiSensor = NULL;
	HRESULT hr = S_OK;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);
	if (FAILED(hr)) { return hr; }

	// Look at each Kinect sensor
	for (int i = 0; i < iSensorCount; ++i) {
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))	{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)	{
			m_pNuiSensor = pNuiSensor;
			break;
		}

		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}

	if (NULL == m_pNuiSensor) {
		return E_FAIL;
	}

	// Initialize the Kinect and specify that we'll be using depth
	//hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX); 
	hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);
	if (FAILED(hr)) { return hr; }

	// Create an event that will be signaled when depth data is available
	m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	// Open a depth image stream to receive depth frames
	hr = m_pNuiSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH,
		//NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
		cDepthResolution,
		(8000 << NUI_IMAGE_PLAYER_INDEX_SHIFT),
		2,
		m_hNextDepthFrameEvent,
		&m_pDepthStreamHandle);
	if (FAILED(hr)) { return hr; }

	// Create an event that will be signaled when color data is available
	m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	// Open a color image stream to receive color frames
	hr = m_pNuiSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,
		cColorResolution,
		0,
		2,
		m_hNextColorFrameEvent,
		&m_pColorStreamHandle);
	if (FAILED(hr)) { return hr; }

	INuiColorCameraSettings* colorCameraSettings;
	HRESULT hrFlag = m_pNuiSensor->NuiGetColorCameraSettings(&colorCameraSettings);

	if (hr != E_NUI_HARDWARE_FEATURE_UNAVAILABLE)
	{
		m_kinect4Windows = true;
	}

	//TODO MATTHIAS: does this function have to be called every frame?

	USHORT* test = new USHORT[getDepthWidth()*getDepthHeight()];
	// Get offset x, y coordinates for color in depth space
	// This will allow us to later compensate for the differences in location, angle, etc between the depth and color cameras
	m_pNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
		cColorResolution,
		cDepthResolution,
		getDepthWidth()*getDepthHeight(),
		test,
		getDepthWidth()*getDepthHeight() * 2,
		m_colorCoordinates
		);
	SAFE_DELETE_ARRAY(test);

	// Start with near mode on (if possible)
	m_bNearMode = false;
	if (m_kinect4Windows) {
		toggleNearMode();
	}

	//toggleAutoWhiteBalance();

	return hr;
}

HRESULT KinectSensor::processDepth()
{
	HRESULT hr = S_OK;

	//  [5/14/2016 chaowang]
	//if (GlobalCameraPoseOptState::getInstance().s_bReadRGBData)
	//{ // Read depth data from RGB-D benchmark 

	//	unsigned int frameIdx = GlobalCameraPoseOptState::getInstance().s_uCurrentFrameIndex;
	//	if (isFrameIdxInRangeOfRGBData(frameIdx))
	//	{
	//		std::string filename = GlobalCameraPoseOptState::getInstance().s_strDataPath + "depth/";
	//		filename += m_strDepthImgName[frameIdx];
	//		filename += ".png";
	//		readDepthImgFromRGBData(filename);
	//		std::cout << "  Reading depth frame " << frameIdx << ", filename = " << m_strDepthImgName[frameIdx] << std::endl;
	//	}
	//}
	//else
	//{ // Use Kinect to scan depth data
	//	
	//wait until data is available
	if (!(WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0)))	return S_FALSE;

	// This code allows to get depth up to 8m
	BOOL bNearMode = false;
	if (m_kinect4Windows)
	{
		bNearMode = true;
	}

	INuiFrameTexture * pTexture = NULL;
	NUI_IMAGE_FRAME imageFrame;

	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_pDepthStreamHandle, &imageFrame, &bNearMode, &pTexture);

	NUI_LOCKED_RECT LockedRect;
	hr = pTexture->LockRect(0, &LockedRect, NULL, 0); // the depth data captured here is x-flipped
	if (FAILED(hr)) { return hr; }

	NUI_DEPTH_IMAGE_PIXEL * pBuffer = (NUI_DEPTH_IMAGE_PIXEL *)LockedRect.pBits;

	//GlobalCameraPoseOptState::getInstance().s_uCurrentFrameIndex++;

	////#pragma omp parallel for
	//	for (int j = 0; j < (int)getDepthWidth()*(int)getDepthHeight(); j++)	{
	//		m_depthD16[j] = pBuffer[j].depth;
	//	}

	USHORT* test = new USHORT[getDepthWidth()*getDepthHeight()];
	float* depth = getDepthFloat();
	int depthFactor = GlobalCameraPoseOptState::getInstance().s_uDepthScaleFactor / 1000;
	for (unsigned int j = 0; j < getDepthHeight(); j++) {
		for (unsigned int i = 0; i < getDepthWidth(); i++) {

			unsigned int desIdx = j*getDepthWidth() + i;
			unsigned int srcIdx = j*getDepthWidth() + (getDepthWidth() - i - 1); // x-flip of the depth data

			const USHORT& d = pBuffer[srcIdx].depth;
			if (d == 0)
				depth[desIdx] = -std::numeric_limits<float>::infinity();
			else
				depth[desIdx] = (float)d * 0.001f;

			if (GlobalAppState::getInstance().s_bSaveRGBDImages)
			{
				m_depthPtr[desIdx] = d * depthFactor;
			}
			//test[srcIdx] = d * 8; // original code (don't know what does 8 mean here)
			test[srcIdx] = d * 8;
		}
	}

	// CHAO: save depth image into local folder
	if (GlobalAppState::getInstance().s_bSaveRGBDImages)
	{
		cv::Mat depthImg2(getDepthHeight(), getDepthWidth(), CV_16UC1, m_depthPtr);
		std::string strDepthImageName = m_strRGBDfolder + "/depth/" + std::to_string(GlobalCameraPoseOptState::getInstance().s_uCurrentFrameIndex) + ".png";
		cv::imwrite(strDepthImageName, depthImg2);
		//GlobalCameraPoseOptState::getInstance().s_uCurrentFrameIndex++;
	}
	hr = pTexture->UnlockRect(0);
	if (FAILED(hr)) { return hr; };

	hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

	// Get offset x, y coordinates for color in depth space
	// This will allow us to later compensate for the differences in location, angle, etc between the depth and color cameras
	m_pNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
		cColorResolution,
		cDepthResolution,
		getDepthWidth()*getDepthHeight(),
		test,
		getDepthWidth()*getDepthHeight() * 2,
		m_colorCoordinates
		);
	delete[] test;
	//}
	return hr;
}

HRESULT KinectSensor::processColor()
{
	HRESULT hr = S_OK;
	//if (GlobalCameraPoseOptState::getInstance().s_bReadRGBData)
	//{ // Read color data from RGB-D benchmark 

	//	unsigned int frameIdx = GlobalCameraPoseOptState::getInstance().s_uCurrentFrameIndex;
	//	if (isFrameIdxInRangeOfRGBData(frameIdx))
	//	{
	//		std::string filename = GlobalCameraPoseOptState::getInstance().s_strDataPath + "rgb/";
	//		filename += m_strColorImgName[frameIdx];
	//		filename += ".png";
	//		readColorImgFromRGBData(filename);
	//		std::cout << "  Reading color frame " << frameIdx << ", filename = " << m_strColorImgName[frameIdx] << std::endl;

	//		if (!isFrameIdxInRangeOfRGBData(frameIdx))
	//		{
	//			std::cout << "All RGB-D images are read." << std::endl;
	//		}
	//	}
	//}
	//else
	//{
	if (!(WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0)))	return S_FALSE;

	NUI_IMAGE_FRAME imageFrame;


	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
	if (FAILED(hr)) { return hr; }

	NUI_LOCKED_RECT LockedRect;
	hr = imageFrame.pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
	if (FAILED(hr)) { return hr; }

	// loop over each row and column of the color
#pragma omp parallel for
	for (int yi = 0; yi < (int)getColorHeight(); ++yi) {
		LONG y = yi;

		LONG* pDest = ((LONG*)m_colorRGBX) + (int)getColorWidth() * y;
		for (LONG x = 0; x < (int)getColorWidth(); ++x)
		{
			// calculate index into depth array
			//int depthIndex = x/m_colorToDepthDivisor + y/m_colorToDepthDivisor * getDepthWidth();	//TODO x flip
			int depthIndex = (getDepthWidth() - 1 - x / m_colorToDepthDivisor) + y / m_colorToDepthDivisor * getDepthWidth();

			// retrieve the depth to color mapping for the current depth pixel
			LONG colorInDepthX = m_colorCoordinates[depthIndex * 2];
			LONG colorInDepthY = m_colorCoordinates[depthIndex * 2 + 1];

			int srcIdx = (int)getColorWidth() * yi + (int)x;

			// make sure the depth pixel maps to a valid point in color space
			if (colorInDepthX >= 0 && colorInDepthX < (int)getColorWidth() && colorInDepthY >= 0 && colorInDepthY < (int)getColorHeight())
			{
				// calculate index into color array
				LONG colorIndex = colorInDepthY * (int)getColorWidth() + colorInDepthX;	//TODO x flip
				//LONG colorIndex = colorInDepthY * (int)getColorWidth() + (getColorWidth() - 1 - colorInDepthX);

				// set source for copy to the color pixel
				LONG* pSrc = ((LONG *)LockedRect.pBits) + colorIndex;
				LONG tmp = *pSrc;
				vec4uc* bgr = (vec4uc*)&tmp;
				std::swap(bgr->x, bgr->z);

				tmp |= 0xFF000000; // Flag for is valid

				*pDest = tmp;

				// CHAO: note that the color values in the opencv Mat type is BGR order instead of RGB
				if (GlobalAppState::getInstance().s_bSaveRGBDImages)
				{
					m_colorPtr[srcIdx * 3] = bgr->z;
					m_colorPtr[srcIdx * 3 + 1] = bgr->y;
					m_colorPtr[srcIdx * 3 + 2] = bgr->x;
				}
			}
			else
			{
				*pDest = 0x00000000;
				if (GlobalAppState::getInstance().s_bSaveRGBDImages)
				{
					m_colorPtr[srcIdx * 3] = m_colorPtr[srcIdx * 3 + 1] = m_colorPtr[srcIdx * 3 + 2] = 0;
				}
			}
			pDest++;
		}
	}
	// CHAO: save rgb image into local folder
	if (GlobalAppState::getInstance().s_bSaveRGBDImages)
	{
		cv::Mat colorImg(getDepthHeight(), getDepthWidth(), CV_8UC3, m_colorPtr);
		std::string strColorImageName = m_strRGBDfolder + "/rgb/" + std::to_string(GlobalCameraPoseOptState::getInstance().s_uCurrentFrameIndex) + ".png";
		cv::imwrite(strColorImageName, colorImg);
		GlobalCameraPoseOptState::getInstance().s_uCurrentFrameIndex++;
	}

	hr = imageFrame.pFrameTexture->UnlockRect(0);
	if (FAILED(hr)) { return hr; };

	hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);
	//}
	return hr;
}

HRESULT KinectSensor::toggleNearMode()
{
	HRESULT hr = E_FAIL;

	if (m_pNuiSensor)
	{
		hr = m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_pDepthStreamHandle, m_bNearMode ? 0 : NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);

		if (SUCCEEDED(hr))
		{
			m_bNearMode = !m_bNearMode;
		}
	}

	return hr;
}

HRESULT KinectSensor::toggleAutoWhiteBalance()
{
	INuiColorCameraSettings* colorCameraSettings;
	HRESULT hr = S_OK;
	hr = m_pNuiSensor->NuiGetColorCameraSettings(&colorCameraSettings);
	if (hr != E_NUI_HARDWARE_FEATURE_UNAVAILABLE) {	//feature only supported with windows Kinect

		BOOL ex;
		colorCameraSettings->GetAutoExposure(&ex);
		colorCameraSettings->SetAutoExposure(!ex);

		//double exposure;
		//colorCameraSettings->GetExposureTime(&exposure);

		//double minExp; colorCameraSettings->GetMinExposureTime(&minExp);
		//double maxExp; colorCameraSettings->GetMaxExposureTime(&maxExp);
		//std::cout << exposure << std::endl;
		//std::cout << minExp << std::endl;
		//std::cout << maxExp << std::endl;
		//colorCameraSettings->SetExposureTime(6000);

		//double fr;
		//colorCameraSettings->GetFrameInterval(&fr);
		//std::cout << fr << std::endl;

		//double gain;
		//hr = colorCameraSettings->GetGain(&gain);
		//std::cout << gain << std::endl;
		//double minG; colorCameraSettings->GetMinGain(&minG);
		//double maxG; colorCameraSettings->GetMaxGain(&maxG);
		//std::cout << minG << std::endl;
		//std::cout << maxG << std::endl;

		hr = colorCameraSettings->SetGain(4);

		BOOL ab;
		colorCameraSettings->GetAutoWhiteBalance(&ab);
		colorCameraSettings->SetAutoWhiteBalance(!ab);

		colorCameraSettings->SetWhiteBalance(4000);	//this is a wild guess; but it seems that the previously 'auto-set' value cannot be obtained
		//LONG min; colorCameraSettings->GetMinWhiteBalance(&min);
		//LONG max; colorCameraSettings->GetMaxWhiteBalance(&max);
		//std::cout << min << std::endl;
		//std::cout << max << std::endl;
	}

	return hr;
}

//
//void KinectSensor::readRGBDAssociationFile(std::string filename)
//{
//	std::ifstream readIn(filename, std::ios::in);
//	std::string str, strDepth, strColor, strTemp;
//	if (readIn.good())
//	{
//		while (readIn.good())
//		{
//			std::getline(readIn, str);
//			if (readIn.good())
//			{
//				std::stringstream sstr(str);
//				sstr >> strDepth;
//				sstr >> strTemp;
//				sstr >> strColor;
//				sstr >> strTemp;
//				m_strDepthImgName.push_back(strDepth);
//				m_strColorImgName.push_back(strColor);
//			}
//		}
//	}
//	else
//	{
//		std::cout << "CHAO WARNING: Cannot read the association file" << std::endl;
//	}
//	readIn.close();
//}
//
//void KinectSensor::readCameraPoseFromFile(std::string filename)
//{
//	vector<std::string> strDepthImgNameFromTrajFile;
//	vector<std::string> strColorImgNameFromTrajFile;
//	mat4f transformation;
//	std::string str;
//	std::ifstream readIn(filename, std::ios::in);
//	if (readIn.good())
//	{
//		while (readIn.good())
//		{
//			std::getline(readIn, str);
//			if (readIn.good())
//			{
//				std::stringstream sstr(str);
//				sstr >> str; // depth frame filename
//				// Check if the depth frame is inside the depth-color association file
//				vector<std::string>::iterator iter = std::find(m_strDepthImgName.begin(), m_strDepthImgName.end(), str);
//				if (iter != m_strDepthImgName.end())
//				{
//					int idx = (int)(iter - m_strDepthImgName.begin());
//					strDepthImgNameFromTrajFile.push_back(str);
//					strColorImgNameFromTrajFile.push_back(m_strColorImgName[idx]);
//				}
//				else
//				{
//					std::cout << "CHAO WARNING: the association file does NOT contain the depth file " << str << std::endl;
//				}
//
//				//cout << "Reading pose from depth stamp " << str << "..." << endl;
//				if (GlobalCameraPoseOptState::getInstance().s_bIsCameraPoseQuaternion)
//				{
//					float tx, ty, tz, qx, qy, qz, qw;
//					sstr >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
//					Quaternionf qua(qw, qx, qy, qz);
//					Matrix3f matTemp = qua.toRotationMatrix();
//					for (int i = 0; i != 3; ++i)
//						for (int j = 0; j != 3; ++j)
//							transformation(i, j) = matTemp(i, j);
//
//					transformation(0, 3) = tx;
//					transformation(1, 3) = ty;
//					transformation(2, 3) = tz;
//					transformation(3, 3) = 1.0;
//					transformation(3, 0) = transformation(3, 1) = transformation(3, 2) = 0;
//
//					// The groundtruth trajectories in the ICL-NUIM RGB-D data are generated with negative fy (-480.0), so
//					// we make a correction for these trajectories to ensure that the direction of the model is consistent with
//					// the scanning order.
//					if (GlobalCameraPoseOptState::getInstance().s_uRGBDataType == 1)
//					{
//						transformation(0, 1) = -transformation(0, 1);
//						transformation(1, 0) = -transformation(1, 0);
//						transformation(1, 2) = -transformation(1, 2);
//						transformation(1, 3) = -transformation(1, 3);
//						transformation(2, 1) = -transformation(2, 1);
//					}
//				}
//				else
//				{
//					for (int i = 0; i != 4; ++i)
//						for (int j = 0; j != 4; ++j)
//							sstr >> transformation(i, j);
//				}
//				recordTrajectory(transformation);
//			}
//		}
//	}
//	else
//	{
//		std::cout << "CHAO WARNING: Cannot read the association file" << std::endl;
//	}
//	readIn.close();
//
//	m_strDepthImgName.clear();
//	m_strColorImgName.clear();
//	m_strDepthImgName = strDepthImgNameFromTrajFile;
//	m_strColorImgName = strColorImgNameFromTrajFile;
//	strDepthImgNameFromTrajFile.clear();
//	strColorImgNameFromTrajFile.clear();
//}
//
//bool KinectSensor::readDepthImgFromRGBData(std::string filename)
//{
//	float* depthPtr = getDepthFloat();
//	int scaleFactor = GlobalCameraPoseOptState::getInstance().s_uDepthScaleFactor;
//	cv::Mat I = cv::imread(filename, CV_LOAD_IMAGE_ANYDEPTH);
//	if (I.depth() == CV_16U)
//	{
//		int channels = I.channels();
//
//		int nRows = I.rows;
//		int nCols = I.cols * channels;
//
//		if (I.isContinuous())
//		{
//			nCols *= nRows;
//			nRows = 1;
//		}
//
//		int i, j;
//		USHORT* p;
//		for (i = 0; i < nRows; ++i)
//		{
//			p = I.ptr<USHORT>(i);
//			for (j = 0; j < nCols; ++j)
//			{
//				depthPtr[j] = float(p[j]) / scaleFactor; // depth value is scaled by 5000 in benchmark data
//			}
//		}
//		return true;
//	}
//	else
//	{
//		std::cerr << "CHAO WARNING: cannot read depth image " << filename << std::endl;
//	}
//	return false;
//}
//
//bool KinectSensor::readColorImgFromRGBData(std::string filename)
//{
//	vec4uc*	colorPtr = m_colorRGBX;
//	cv::Mat I = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
//	if (I.data && I.depth() == CV_8U)
//	{
//		int channels = I.channels();
//
//		int nRows = I.rows;
//		int nCols = I.cols * channels;
//
//		if (I.isContinuous())
//		{
//			nCols *= nRows;
//			nRows = 1;
//		}
//
//		int i = 0, j = 0, k = 0;
//		BYTE* p;
//
//		//#pragma omp parallel for
//		for (i = 0; i < nRows; ++i)
//		{
//			p = I.ptr<BYTE>(i);
//			while (j < nCols)
//			{
//				int idx = j / 3;
//				for (k = 0; k < 3; ++k)
//				{
//					colorPtr[idx].array[2-k] = p[j++]; // OpenCV loads color image in BGR order while we need RGB here
//				}
//				colorPtr[idx].array[k] = (unsigned char)255;
//			}
//		}
//		return true;
//	}
//	else
//	{
//		std::cerr << "CHAO WARNING: cannot read color image " << filename << std::endl;
//	}
//	I.release();
//	return false;
//}