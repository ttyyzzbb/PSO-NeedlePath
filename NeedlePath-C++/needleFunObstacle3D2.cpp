#include<vector>

#include"needleFunObstacle3D2.h"
#include"globalVariable.h"
#include"myFunction.h"
#include"setObstacles3D.h"

// needleFunObstacle3D2���أ�pathLength, tipError, deviationValue, obstacleValue, nonZeroLength
std::vector<double> NeedleFunObstacle3D2::needleFunObstacle3D2(std::vector<double> arcs) {

	// ��ʼ������
	double p0c[] = { 0, 0, 1 };
	// ��ʼԲ��λ��
	double O0[] = { 0, (double)radius, 0 };

	double minL = 100000;

	// ���ϰ����꣨��ά��
	std::vector<std::vector<double>> obstacles = SetObstacles3D::setObstacles3D();
	// ���ϰ��뾶��һά��
	std::vector<double> obstacleRadius = SetObstacles3D::obstacleRadius();
	// �ϰ������
	int obstacleNumber = SetObstacles3D::m_obstacleNumber;
	double InObstacleValue = 100000.0;
	double obstacleValue = 0.0;
	// ��㵽�յ������
	double p0pt[3] = { 0, };
	for (int i = 0; i < 3; i++) {
		p0pt[i] = Ptarget[i] - P0[i];
	}
	// ƫ��ֵ
	double deviationValue = 0.0;
	// ϵ��
	double KDV = 20.0;

	// ·���θ���
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
	// ��ʼԲ������
	std::vector<double> Oi(3);
	for (int i = 0; i < 3; i++) {
		Oi[i] = O0[i];
	}
	// �洢��Oi��ת���ֵ
	std::vector<double> Oi2(3);
	for (int i = 0; i < 3; i++) {
		Oi2[i] = O0[i];
	}
	int isStop = 0;
	int nonZeroLength = 2 * len;

	// ÿһ��Բ���ж����ϰ��ľ���
	for (int distanceJudge = 0; distanceJudge < len; distanceJudge++) {
		// ����/�뾶=����
		thetas[distanceJudge] = arcs[2 * distanceJudge + 1] / radius;
		// ��arcLength�о������ȡ��ת�Ƕ�
		double alpha = arcs[2 * distanceJudge];
		double sa = sin(alpha);
		double ca = cos(alpha);
		// ��Բ��ת��Ϊ����
		for (int i = 0; i < 3; i++) {
			Oi[i] = Oi[i] - points[distanceJudge][i];
		}
		// ������������ת
		Oi2[0] = Oi[2] * (pic[1] * sa - pic[0] * pic[2] * (ca - 1)) - Oi[1] * (pic[2] * sa + pic[0] * pic[1] * (ca - 1)) + Oi[0] * (ca - pic[0] * pic[0] * (ca - 1));
		Oi2[1] = Oi[0] * (pic[2] * sa - pic[0] * pic[1] * (ca - 1)) - Oi[2] * (pic[0] * sa + pic[1] * pic[2] * (ca - 1)) + Oi[1] * (ca - pic[1] * pic[1] * (ca - 1));
		Oi2[2] = Oi[1] * (pic[0] * sa - pic[1] * pic[2] * (ca - 1)) - Oi[0] * (pic[1] * sa + pic[0] * pic[2] * (ca - 1)) + Oi[2] * (ca - pic[2] * pic[2] * (ca - 1));
		// �������л�ȡ��ת���Բ��
		for (int i = 0; i < 3; i++) {
			Oi2[i] = Oi2[i] + points[distanceJudge][i];
		}
		// ���㴩�̺�Ķ˵�λ��
		double st = sin(thetas[distanceJudge]);
		double ct = cos(thetas[distanceJudge]);
		for (int i = 0; i < 3; i++) {
			points[distanceJudge + 1][i] = ct*points[distanceJudge][i] + (1 - ct)*Oi2[i] + radius*st*pic[i];
		}
		// �����趨���յ�z��������ֹͣ
		if (points[distanceJudge][2] > Ptarget[2]) {
			isStop = 1;
			nonZeroLength = 2 * distanceJudge;
		}
		// ����˵㴦������
		for (int i = 0; i < 3; i++) {
			pi2c[i] = Oi2[i] * (1 - ct) + points[distanceJudge + 1][i] * ct - points[distanceJudge][i];
		}
		// ������λ��
		double normPi2c = sqrt(pi2c[0] * pi2c[0] + pi2c[1] * pi2c[1] + pi2c[2] * pi2c[2]);
		for (int i = 0; i < 3; i++) {
			pi2c[i] = pi2c[i] / normPi2c;
		}
		// ����ƫ��ֵ
		double dotpp = pi2c[0] * p0pt[0] + pi2c[1] * p0pt[1] + pi2c[2] * p0pt[2];
		if (dotpp > 0) {
			deviationValue = deviationValue + KDV * (1 / dotpp);
		}
		else {
			deviationValue = deviationValue + 1000000;
		}

		// �����ϰ���Լ�� obstacle === j
		for (int obstacle = 0; obstacle < obstacleNumber; obstacle++) {
			// �ж��ϰ���İ뾶�Ƿ������
			if (SetObstacles3D::obstacleRadius()[obstacle] > 0) {
				// ������i�㵽�ϰ���Բ��
				double po[3] = { 0, };
				for (int i = 0; i < 3; i++) {
					po[i] = SetObstacles3D::setObstacles3D()[obstacle][i] - points[distanceJudge][i];
				}
				// ������i+1�㵽�ϰ���Բ��
				double p1o[3] = { 0, };
				for (int i = 0; i < 3; i++) {
					p1o[i] = SetObstacles3D::setObstacles3D()[obstacle][i] - points[distanceJudge + 1][i];
				}
				// ����i��i+1���Ƿ���ϰ����ཻ������Բ�ĵľ���<r
				double npo = sqrt(po[0] * po[0] + po[1] * po[1] + po[2] * po[2]);
				double np1o = sqrt(p1o[0] * p1o[0] + p1o[1] * p1o[1] + p1o[2] * p1o[2]);
				// ���ϰ���뾶���Ƚ�
				if (npo < SetObstacles3D::obstacleRadius()[obstacle] || np1o < SetObstacles3D::obstacleRadius()[obstacle]) {
					obstacleValue = obstacleValue + InObstacleValue;
					isStop = 1;
					break;
				}
				if (npo < np1o)
					minL = npo;
				else
					minL = np1o;
				/* �����ϰ����Ƿ��ڻ��ڲ���pp1��po  p1p��p1���ڻ�>0������ֱ������,����һ������*/
				double vecO[3] = { 0, };
				double vecOnorm = 0.0;
				double* vecN = nullptr;
				if (MyFunction::dot(pic, po) > 0 && MyFunction::dot(pi2c, p1o) < 0) {
					// ����Բ�ĵ��ϰ���Բ�ĵ�����
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

				// ������ĵ�����Ϊ��Ӧֵ������Խ�����ϰ����Ӧֵ����Խ��
				obstacleValue = obstacleValue + 1 / minL;
			}
		}

		if (isStop == 1)
			break;
		// ������ʱ����
		for (int i = 0; i < 3; i++) {
			pic[i] = pi2c[i];
		}
		for (int i = 0; i < 3; i++) {
			Oi[i] = Oi2[i];
		}
	}

	// ������Ӧֵ
	double fx, fy, fz;
	fx = Ptarget[0] - points[len][0];
	fy = Ptarget[1] - points[len][1];
	fz = Ptarget[2] - points[len][2];


	double pathLength = 0.0, tipError = 0.0;
	for (int i = 0; i < len; i++) {
		pathLength += thetas[i] * radius;
	}
	tipError = fx*fx + fy*fy + fz*fz;

	// ���أ�pathLength, tipError, deviationValue, obstacleValue, nonZeroLength
	std::vector<double> needleFunObstacle3D2Return;
	needleFunObstacle3D2Return.push_back(pathLength);
	needleFunObstacle3D2Return.push_back(tipError);
	needleFunObstacle3D2Return.push_back(deviationValue);
	needleFunObstacle3D2Return.push_back(obstacleValue);
	needleFunObstacle3D2Return.push_back(nonZeroLength);

	return needleFunObstacle3D2Return;
} // needleFunObstacle3D2