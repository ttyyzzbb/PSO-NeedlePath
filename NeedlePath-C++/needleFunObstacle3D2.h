#include<vector>

#pragma once
#ifndef __NEEDLEFUNOBSTACLE3D2_H__
#define __NEEDLEFUNOBSTACLE3D2_H__

class NeedleFunObstacle3D2 {
public:
	// needleFunObstacle3D2·µ»Ø£ºpathLength, tipError, deviationValue, obstacleValue, nonZeroLength
	static std::vector<double> needleFunObstacle3D2(std::vector<double> arcs);
};

#endif // __NEEDLEFUNOBSTACLE3D2_H__