
#include"setObstacles3D.h"

std::vector<std::vector<double>> SetObstacles3D::setObstacles3D() {
	std::vector<std::vector<double>> circleObstacles = { { -10,10,20 },
														{ 10,-10,30 },
														{ 10,10,50 },};
	return circleObstacles;
}

std::vector<double> SetObstacles3D::obstacleRadius() {
	std::vector<double> obstacleRadius = { 8,15,8 };
	return obstacleRadius;
}

// 障碍物数量（别忘了修改数值）
int SetObstacles3D::m_obstacleNumber = 3;