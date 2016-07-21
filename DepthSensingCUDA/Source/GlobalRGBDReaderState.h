#pragma once

/***********************************************************************************/
/* CHAO: 
* Global App state for camera pose optimization: reads and stores all relevant parameters  */
/***********************************************************************************/

#include "DXUT.h"

#include "stdafx.h"

#include <vector>

// CHAO: The values of all these constant parameters are predefined in the "zParametersCameraPoseOpt.txt" file in the project root,
// and they are set when the TXT file is read in the main() function just after the beginning of this program runs.
#define X_GLOBAL_RGBDREADER_STATE_FIELDS \
	X(bool, s_bReadRGBData) \
	X(bool, s_bReadCameraPoseFromFile)\
	X(bool, s_bRunPoseEstimation)\
	X(bool, s_bStoreCameraPoseIntoFile)\
	X(bool, s_bIsCameraPoseQuaternion)\
	X(bool, s_bDrawCorrespondenceBetweenImages) \
	X(bool, s_bCheckDepthValInReliableRange) \
	X(bool, s_bUseWeightedOptimization)\
	X(unsigned int, s_uRGBDataType) \
	X(float, s_fx) \
	X(float, s_fy) \
	X(float, s_cx) \
	X(float, s_cy) \
	X(std::string, s_strDataPath) \
	X(std::string, s_strTrajFile) \
	X(std::string, s_strAssociationFile) \
	X(unsigned int, s_uCameraPoseEstimationType)\
	X(unsigned int, s_uOptMethodType) \
	X(unsigned int, s_uDepthScaleFactor) \
	X(unsigned int, s_uFrameCount) \
	X(unsigned int, s_uMinimumFrameIndex) \
	X(unsigned int, s_uMaximumFrameIndex) \
	X(unsigned int, s_uFrameInterval) \
	X(unsigned int, s_uCurrentFrameIndex) \
	X(float, s_depthWidth) \
	X(float, s_depthHeight) \
	X(float, s_colorWidth) \
	X(float, s_colorHeight) \
	X(float, s_maxReliableDepth) \
	X(float, s_minReliableDepth) \
	X(float, s_thresholdGrayValueDiff) \
	X(float, s_thresholdMinCrspdnsPairNumber) \
	X(float, s_thresholdCoordiff) \
	X(float, s_threshold_energy) \
	X(float, s_thresholdMinDepthValDiff) \
	X(float, s_thresholdRansacReproj)\
    X(float, s_weightScaleFactor)

#ifndef VAR_NAME
#define VAR_NAME(x) #x
#endif


class GlobalRGBDReaderState
{
public:
#define X(type, name) type name;
	X_GLOBAL_RGBDREADER_STATE_FIELDS
#undef X

		//! sets the parameter file and reads
		void readMembers(const ParameterFile& parameterFile) {
		s_ParameterFile = parameterFile;
		readMembers();
	}

	//! reads all the members from the given parameter file (could be called for reloading)
	void readMembers() {
#define X(type, name) \
	if (!s_ParameterFile.readParameter(std::string(#name), name)) {MLIB_WARNING(std::string(#name).append(" ").append("uninitialized"));	name = type();}
		X_GLOBAL_RGBDREADER_STATE_FIELDS
#undef X
	}

	//! prints all members
	void print() {
#define X(type, name) \
	std::cout << #name " = " << name << std::endl;
		X_GLOBAL_RGBDREADER_STATE_FIELDS
#undef X
	}

	static GlobalRGBDReaderState& getInstance() {
		static GlobalRGBDReaderState s;
		return s;
	}
private:
	ParameterFile s_ParameterFile;
};
