#include<vector>

#pragma once
#ifndef _SETOBSTACLES3D_H__
#define _SETOBSTACLES3D_H__

class SetObstacles3D {
public:
	static std::vector<std::vector<double>> setObstacles3D();
	static std::vector<double> obstacleRadius();

public:
	// �ϰ����������޸�ʱ�������޸�cpp�����ֵ��
	static int m_obstacleNumber;
};

#endif // _SETOBSTACLES3D_H__