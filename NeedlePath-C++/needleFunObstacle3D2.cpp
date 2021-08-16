#include<vector>

#include"needleFunObstacle3D2.h"
#include"globalVariable.h"
#include"myFunction.h"
#include"setObstacles3D.h"

// needleFunObstacle3D2返回：pathLength, tipError, deviationValue, obstacleValue, nonZeroLength
std::vector<double> NeedleFunObstacle3D2::needleFunObstacle3D2(std::vector<double> arcs) {

	// 初始切向量
	double p0c[] = { 0, 0, 1 };
	// 初始圆心位置
	double O0[] = { 0, (double)radius, 0 };

	double minL = 100000;

	// 各障碍坐标（二维）
	std::vector<std::vector<double>> obstacles = SetObstacles3D::setObstacles3D();
	// 各障碍半径（一维）
	std::vector<double> obstacleRadius = SetObstacles3D::obstacleRadius();
	// 障碍物个数
	int obstacleNumber = SetObstacles3D::m_obstacleNumber;
	double InObstacleValue = 100000.0;
	double obstacleValue = 0.0;
	// 起点到终点的向量
	double p0pt[3] = { 0, };
	for (int i = 0; i < 3; i++) {
		p0pt[i] = Ptarget[i] - P0[i];
	}
	// 偏差值
	double deviationValue = 0.0;
	// 系数
	double KDV = 20.0;

	// 路径段个数
	int len = arcs.size() / 2;

	std::vector<double> thetas(len);
	std::vector<std::vector<double>> points(len + 1, std::vector<double>(3, 0));
	for (int i = 0; i < 3; i++) {
		points[0][i] = P0[i];
	}

	double pic[3] = { 0, };
	double pi2c[3] = { 0, };
	for (int i = 0; i < 3; i++) {
		pic[i] = p0c[i];
	}
	for (int i = 0; i < 3; i++) {
		pi2c[i] = pic[i];
	}
	// 初始圆心坐标
	std::vector<double> Oi(3);
	for (int i = 0; i < 3; i++) {
		Oi[i] = O0[i];
	}
	// 存储将Oi旋转后的值
	std::vector<double> Oi2(3);
	for (int i = 0; i < 3; i++) {
		Oi2[i] = O0[i];
	}
	int isStop = 0;
	int nonZeroLength = 2 * len;

	// 每一段圆弧判断与障碍的距离
	for (int distanceJudge = 0; distanceJudge < len; distanceJudge++) {
		// 弧长/半径=弧度
		thetas[distanceJudge] = arcs[2 * distanceJudge + 1] / radius;
		// 从arcLength行矩阵里读取旋转角度
		double alpha = arcs[2 * distanceJudge];
		double sa = sin(alpha);
		double ca = cos(alpha);
		// 将圆心转换为向量
		for (int i = 0; i < 3; i++) {
			Oi[i] = Oi[i] - points[distanceJudge][i];
		}
		// 对向量进行旋转
		Oi2[0] = Oi[2] * (pic[1] * sa - pic[0] * pic[2] * (ca - 1)) - Oi[1] * (pic[2] * sa + pic[0] * pic[1] * (ca - 1)) + Oi[0] * (ca - pic[0] * pic[0] * (ca - 1));
		Oi2[1] = Oi[0] * (pic[2] * sa - pic[0] * pic[1] * (ca - 1)) - Oi[2] * (pic[0] * sa + pic[1] * pic[2] * (ca - 1)) + Oi[1] * (ca - pic[1] * pic[1] * (ca - 1));
		Oi2[2] = Oi[1] * (pic[0] * sa - pic[1] * pic[2] * (ca - 1)) - Oi[0] * (pic[1] * sa + pic[0] * pic[2] * (ca - 1)) + Oi[2] * (ca - pic[2] * pic[2] * (ca - 1));
		// 从向量中获取旋转后的圆心
		for (int i = 0; i < 3; i++) {
			Oi2[i] = Oi2[i] + points[distanceJudge][i];
		}
		// 计算穿刺后的端点位置
		double st = sin(thetas[distanceJudge]);
		double ct = cos(thetas[distanceJudge]);
		for (int i = 0; i < 3; i++) {
			points[distanceJudge + 1][i] = ct*points[distanceJudge][i] + (1 - ct)*Oi2[i] + radius*st*pic[i];
		}
		// 超过设定的终点z轴坐标则停止
		if (points[distanceJudge][2] > Ptarget[2]) {
			isStop = 1;
			nonZeroLength = 2 * distanceJudge;
		}
		// 计算端点处切向量
		for (int i = 0; i < 3; i++) {
			pi2c[i] = Oi2[i] * (1 - ct) + points[distanceJudge + 1][i] * ct - points[distanceJudge][i];
		}
		// 向量单位化
		double normPi2c = sqrt(pi2c[0] * pi2c[0] + pi2c[1] * pi2c[1] + pi2c[2] * pi2c[2]);
		for (int i = 0; i < 3; i++) {
			pi2c[i] = pi2c[i] / normPi2c;
		}
		// 计算偏离值
		double dotpp = pi2c[0] * p0pt[0] + pi2c[1] * p0pt[1] + pi2c[2] * p0pt[2];
		if (dotpp > 0) {
			deviationValue = deviationValue + KDV * (1 / dotpp);
		}
		else {
			deviationValue = deviationValue + 1000000;
		}

		// 计算障碍物约束 obstacle === j
		for (int obstacle = 0; obstacle < obstacleNumber; obstacle++) {
			// 判断障碍物的半径是否大于零
			if (SetObstacles3D::obstacleRadius()[obstacle] > 0) {
				// 向量，i点到障碍物圆心
				double po[3] = { 0, };
				for (int i = 0; i < 3; i++) {
					po[i] = SetObstacles3D::setObstacles3D()[obstacle][i] - points[distanceJudge][i];
				}
				// 向量，i+1点到障碍物圆心
				double p1o[3] = { 0, };
				for (int i = 0; i < 3; i++) {
					p1o[i] = SetObstacles3D::setObstacles3D()[obstacle][i] - points[distanceJudge + 1][i];
				}
				// 计算i和i+1点是否和障碍物相交，即到圆心的距离<r
				double npo = sqrt(po[0] * po[0] + po[1] * po[1] + po[2] * po[2]);
				double np1o = sqrt(p1o[0] * p1o[0] + p1o[1] * p1o[1] + p1o[2] * p1o[2]);
				// 和障碍物半径作比较
				if (npo < SetObstacles3D::obstacleRadius()[obstacle] || np1o < SetObstacles3D::obstacleRadius()[obstacle]) {
					obstacleValue = obstacleValue + InObstacleValue;
					isStop = 1;
					break;
				}
				if (npo < np1o)
					minL = npo;
				else
					minL = np1o;
				/* 计算障碍物是否在弧内部，pp1与po  p1p与p1的内积>0，否则直接跳过,保留一定余量*/
				double vecO[3] = { 0, };
				double vecOnorm = 0.0;
				double* vecN = nullptr;
				if (MyFunction::dot(pic, po) > 0 && MyFunction::dot(pi2c, p1o) < 0) {
					// 弧线圆心到障碍物圆心的向量
					for (int i = 0; i < 3; i++) {
						vecO[i] = SetObstacles3D::setObstacles3D()[obstacle][i] - Oi2[i];
					}
					vecOnorm = MyFunction::norm(vecO);
					vecN = MyFunction::cross(pic, pi2c);
					alpha = asin(abs(MyFunction::dot(vecO, vecN) / vecOnorm / MyFunction::norm(vecN)));
					minL = sqrt(vecOnorm*vecOnorm + radius*radius - 2 * vecOnorm*radius*cos(alpha)) - SetObstacles3D::obstacleRadius()[obstacle];
					if (minL < 0) {
						obstacleValue = obstacleValue + InObstacleValue;
						isStop = 1;
						break;
					}
				}

				// 将距离的倒数作为适应值函数，越靠近障碍物，适应值函数越大
				obstacleValue = obstacleValue + 1 / minL;
			}
		}

		if (isStop == 1)
			break;
		// 更新临时变量
		for (int i = 0; i < 3; i++) {
			pic[i] = pi2c[i];
		}
		for (int i = 0; i < 3; i++) {
			Oi[i] = Oi2[i];
		}
	}

	// 计算适应值
	double fx, fy, fz;
	fx = Ptarget[0] - points[len][0];
	fy = Ptarget[1] - points[len][1];
	fz = Ptarget[2] - points[len][2];


	double pathLength = 0.0, tipError = 0.0;
	for (int i = 0; i < len; i++) {
		pathLength += thetas[i] * radius;
	}
	tipError = fx*fx + fy*fy + fz*fz;

	// 返回：pathLength, tipError, deviationValue, obstacleValue, nonZeroLength
	std::vector<double> needleFunObstacle3D2Return;
	needleFunObstacle3D2Return.push_back(pathLength);
	needleFunObstacle3D2Return.push_back(tipError);
	needleFunObstacle3D2Return.push_back(deviationValue);
	needleFunObstacle3D2Return.push_back(obstacleValue);
	needleFunObstacle3D2Return.push_back(nonZeroLength);

	return needleFunObstacle3D2Return;
} // needleFunObstacle3D2