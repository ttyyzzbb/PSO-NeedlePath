#include<vector>

#pragma once
#ifndef __REFRESHBESTSTATES_H___
#define __REFRESHBESTSTATES_H___

class RefreshBestStates {
public:
	static std::vector<double> refreshBestStates(std::vector<double> (*needleFunObstacle3D2)(std::vector<double> arcs));
};

#endif // __REFRESHBESTSTATES_H___