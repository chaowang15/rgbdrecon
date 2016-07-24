
#include "stdafx.h"
#include "RGBDLocalDataReader.h"

RGBDLocalDataReader::RGBDLocalDataReader()
{
	RGBDSensor::init(GlobalRGBDReaderState::getInstance().s_depthWidth, GlobalRGBDReaderState::getInstance().s_depthHeight, 
		GlobalRGBDReaderState::getInstance().s_colorWidth, GlobalRGBDReaderState::getInstance().s_colorHeight, 1);

	// Set intrinsic camera calibration matrix parameters
	initializeDepthIntrinsics(GlobalRGBDReaderState::getInstance().s_fx, GlobalRGBDReaderState::getInstance().s_fy,
		GlobalRGBDReaderState::getInstance().s_cx, GlobalRGBDReaderState::getInstance().s_cy);
	initializeColorIntrinsics(GlobalRGBDReaderState::getInstance().s_fx, GlobalRGBDReaderState::getInstance().s_fy,
		GlobalRGBDReaderState::getInstance().s_cx, GlobalRGBDReaderState::getInstance().s_cy);

}

RGBDLocalDataReader::~RGBDLocalDataReader()
{


}

HRESULT RGBDLocalDataReader::createFirstConnected()
{
	HRESULT hr = S_OK;
	std::string filename = GlobalRGBDReaderState::getInstance().s_strDataPath + GlobalRGBDReaderState::getInstance().s_strAssociationFile;
	readRGBDAssociationFile(filename);
	GlobalRGBDReaderState::getInstance().s_uCurrentFrameIndex = GlobalRGBDReaderState::getInstance().s_uMinimumFrameIndex;
	std::cout << "Reading RGB-D data ... " << std::endl;
	
	if (GlobalRGBDReaderState::getInstance().s_bReadCameraPoseFromFile)
	{
		std::string filename = GlobalRGBDReaderState::getInstance().s_strDataPath + GlobalRGBDReaderState::getInstance().s_strTrajFile;
		readCameraPoseFromFile(filename);
	}

	return hr;
}

HRESULT RGBDLocalDataReader::processDepth()
{
	HRESULT hr = S_OK;

	// Read depth frame by frame
	unsigned int frameIdx = GlobalRGBDReaderState::getInstance().s_uCurrentFrameIndex;
	if (isFrameIdxInRangeOfRGBData(frameIdx))
	{
		std::string filename = GlobalRGBDReaderState::getInstance().s_strDataPath + "depth/";
		filename += m_strDepthImgName[frameIdx];
		filename += ".png";
		readDepthImgFromRGBData(filename);
		std::cout << "  Reading depth frame " << frameIdx << ", filename = " << m_strDepthImgName[frameIdx] << std::endl;
	}

	return hr;
}

HRESULT RGBDLocalDataReader::processColor()
{
	HRESULT hr = S_OK;
	unsigned int frameIdx = GlobalRGBDReaderState::getInstance().s_uCurrentFrameIndex;
	if (isFrameIdxInRangeOfRGBData(frameIdx))
	{
		std::string filename = GlobalRGBDReaderState::getInstance().s_strDataPath + "rgb/";
		filename += m_strColorImgName[frameIdx];
		filename += ".png";
		readColorImgFromRGBData(filename);
		std::cout << "  Reading color frame " << frameIdx << ", filename = " << m_strColorImgName[frameIdx] << std::endl;

		if (!isFrameIdxInRangeOfRGBData(frameIdx))
		{
			std::cout << "All RGB-D images are read." << std::endl;
		}
	}

	return hr;
}



void RGBDLocalDataReader::readRGBDAssociationFile(std::string filename)
{
	std::ifstream readIn(filename, std::ios::in);
	std::string str, strDepth, strColor, strTemp;
	if (readIn.good())
	{
		while (readIn.good())
		{
			std::getline(readIn, str);
			if (readIn.good())
			{
				std::stringstream sstr(str);
				sstr >> strDepth;
				sstr >> strTemp;
				sstr >> strColor;
				sstr >> strTemp;
				m_strDepthImgName.push_back(strDepth);
				m_strColorImgName.push_back(strColor);
			}
		}
	}
	else
	{
		std::cout << "CHAO WARNING: Cannot read the association file" << std::endl;
	}
	readIn.close();
}

void RGBDLocalDataReader::readCameraPoseFromFile(std::string filename)
{
	vector<std::string> strDepthImgNameFromTrajFile;
	vector<std::string> strColorImgNameFromTrajFile;
	mat4f transformation;
	std::string str;
	std::ifstream readIn(filename, std::ios::in);
	if (readIn.good())
	{
		while (readIn.good())
		{
			std::getline(readIn, str);
			if (readIn.good())
			{
				std::stringstream sstr(str);
				sstr >> str; // depth frame filename
				// Check if the depth frame is inside the depth-color association file
				vector<std::string>::iterator iter = std::find(m_strDepthImgName.begin(), m_strDepthImgName.end(), str);
				if (iter != m_strDepthImgName.end())
				{
					int idx = (int)(iter - m_strDepthImgName.begin());
					strDepthImgNameFromTrajFile.push_back(str);
					strColorImgNameFromTrajFile.push_back(m_strColorImgName[idx]);
				}
				else
				{
					std::cout << "CHAO WARNING: the association file does NOT contain the depth file " << str << std::endl;
				}

				//cout << "Reading pose from depth stamp " << str << "..." << endl;
				if (GlobalRGBDReaderState::getInstance().s_bIsCameraPoseQuaternion)
				{
					float tx, ty, tz, qx, qy, qz, qw;
					sstr >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
					Quaternionf qua(qw, qx, qy, qz);
					Matrix3f matTemp = qua.toRotationMatrix();
					for (int i = 0; i != 3; ++i)
						for (int j = 0; j != 3; ++j)
							transformation(i, j) = matTemp(i, j);

					transformation(0, 3) = tx;
					transformation(1, 3) = ty;
					transformation(2, 3) = tz;
					transformation(3, 3) = 1.0;
					transformation(3, 0) = transformation(3, 1) = transformation(3, 2) = 0;

					// The groundtruth trajectories in the ICL-NUIM RGB-D data are generated with negative fy = -480.0, which
					// will results in a inverse-y-coordinate upside-down model after reconstruction (even though the rendering 
					// process is still good). To generate a normal model, we make the following corrections for the y-axis-elements 
					// for all the transformations. 
					// Just make sure use the positive fy = 480.0 this time.
					// 
					if (GlobalRGBDReaderState::getInstance().s_uRGBDataType == 1)
					{
						transformation(0, 1) = -transformation(0, 1);
						transformation(1, 0) = -transformation(1, 0);
						transformation(1, 2) = -transformation(1, 2);
						transformation(1, 3) = -transformation(1, 3);
						transformation(2, 1) = -transformation(2, 1);
					}
				}
				else
				{
					for (int i = 0; i != 4; ++i)
						for (int j = 0; j != 4; ++j)
							sstr >> transformation(i, j);
				}
				recordTrajectory(transformation);
			}
		}
	}
	else
	{
		std::cout << "CHAO WARNING: Cannot read the association file" << std::endl;
	}
	readIn.close();

	m_strDepthImgName.clear();
	m_strColorImgName.clear();
	m_strDepthImgName = strDepthImgNameFromTrajFile;
	m_strColorImgName = strColorImgNameFromTrajFile;
	strDepthImgNameFromTrajFile.clear();
	strColorImgNameFromTrajFile.clear();
}

bool RGBDLocalDataReader::readDepthImgFromRGBData(std::string filename)
{
	float* depthPtr = getDepthFloat();
	int scaleFactor = GlobalRGBDReaderState::getInstance().s_uDepthScaleFactor;
	cv::Mat I = cv::imread(filename, CV_LOAD_IMAGE_ANYDEPTH);
	if (I.depth() == CV_16U)
	{
		int channels = I.channels();

		int nRows = I.rows;
		int nCols = I.cols * channels;

		if (I.isContinuous())
		{
			nCols *= nRows;
			nRows = 1;
		}

		int i, j;
		USHORT* p; 
		for (i = 0; i < nRows; ++i)
		{
			p = I.ptr<USHORT>(i);
			for (j = 0; j < nCols; ++j)
			{
				depthPtr[j] = float(p[j]) / scaleFactor; // depth value is scaled by 5000 in benchmark data
			}
		}
		return true;
	}
	else
	{
		std::cerr << "CHAO WARNING: cannot read depth image " << filename << std::endl;
	}
	return false;
}

bool RGBDLocalDataReader::readColorImgFromRGBData(std::string filename)
{
	vec4uc*	colorPtr = m_colorRGBX;
	cv::Mat I = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
	if (I.data && I.depth() == CV_8U)
	{
		int channels = I.channels();

		int nRows = I.rows;
		int nCols = I.cols * channels;

		if (I.isContinuous())
		{
			nCols *= nRows;
			nRows = 1;
		}

		int i = 0, j = 0, k = 0;
		BYTE* p;

		//#pragma omp parallel for
		for (i = 0; i < nRows; ++i)
		{
			p = I.ptr<BYTE>(i);
			while (j < nCols)
			{
				int idx = j / 3;
				for (k = 0; k < 3; ++k)
				{
					colorPtr[idx].array[2 - k] = p[j++]; // OpenCV loads color image in BGR order while we need RGB here
				}
				colorPtr[idx].array[k] = (unsigned char)255;
			}
		}
		return true;
	}
	else
	{
		std::cerr << "CHAO WARNING: cannot read color image " << filename << std::endl;
	}
	I.release();
	return false;
}