#include<vector>

#pragma once
#ifndef _SETOBSTACLES3D_H__
#define _SETOBSTACLES3D_H__

class SetObstacles3D {
public:
	static std::vector<std::vector<double>> setObstacles3D();
	static std::vector<double> obstacleRadius();

public:
	// 障碍物数量（修改时别忘了修改cpp里的数值）
	static int m_obstacleNumber;
};

#endif // _SETOBSTACLES3D_H__