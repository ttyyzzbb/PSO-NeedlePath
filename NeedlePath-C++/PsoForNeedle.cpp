#include<iostream>
#include<time.h>

#include"PsoForNeedle.h"
#include"globalVariable.h"
#include"myFunction.h"
#include"refreshBestStates.h"

void PsoForNeedle::psoForNeedle(std::vector<double>(*evaluationFun)(std::vector<double> arcs)) {

	for (int i = 0; i < arcsLengthUpperBound.size(); i++) {
		vmax.push_back((arcsLengthUpperBound[i] - arcsLengthLowerBound[i]) / 10);	// ��������ٶ�
		vmin.push_back(-vmax[i]);													// ������С�ٶ�
	}

	// ��ֹ�������е���ȷ��
	double Ptarget_P0[3] = { 0.0, };
	for (int i = 0; i < 3; i++) {
		Ptarget_P0[i] = Ptarget[i] - P0[i];
	}
	double Pstopv = MyFunction::norm(Ptarget_P0) * 1.2;

	// ��ʼ������Ⱥ״̬ �� ����Ⱥ�ٶ�
	srand((unsigned)time(NULL));
	for (int i = 0; i < len; i++) {
		for (int particleNumber = 0; particleNumber < M; particleNumber++) {
			double suiji = (rand() % (999 + 1) / (double)(999 + 1));
			Z[particleNumber][i] = suiji*(arcsLengthUpperBound[i] - arcsLengthLowerBound[i]) + arcsLengthLowerBound[i];
			V[particleNumber][i] = V[particleNumber][i] * (vmax[i] - vmin[i]) + vmin[i];
		}
	}
	std::vector<double> sump_sumPgv = RefreshBestStates::refreshBestStates(evaluationFun);

	/* ��ʼ���� N �� */
	for (int iter = 0; iter < N; iter++) {
		std::cout << "����������" << iter << std::endl;
		// ���¸������ź�ȫ������
		sump_sumPgv = RefreshBestStates::refreshBestStates(evaluationFun);
		sump_temp[iter] = sump_sumPgv[1];

		// �����ٶ�
		w = wmax - wk*iter; // ����Ӧ����Ȩ��
		srand((unsigned)time(NULL));
		for (int upV = 0; upV < M; upV++) {
			for (int upVj = 0; upVj < Plength[upV]; upVj++) {
				double r1 = rand() % (999 + 1) / (double)(999 + 1);
				double r2 = rand() % (999 + 1) / (double)(999 + 1);
				V[upV][upVj] = w * V[upV][upVj] + c1*r1*(P[upV][upVj] - Z[upV][upVj]) + c2*r2*(Pg[upVj] - Z[upV][upVj]);
				if (V[upV][upVj] < vmin[upVj]) // ��ʵ���ٶȶ��ڽ����Ӱ��ϴ󣬲��ܺ��Զ��ٶ�����
					V[upV][upVj] = vmin[upVj];
				if (V[upV][upVj] > vmax[upVj])
					V[upV][upVj] = vmax[upVj];
			}
		}

		// ����״̬
		for (int upZ = 0; upZ < M; upZ++) {
			for (int upZj = 0; upZj < Plength[upZ]; upZj++) {
				Z[upZ][upZj] = Z[upZ][upZj] + V[upZ][upZj];
				// ״̬�����߽�
				if (Z[upZ][upZj] < arcsLengthLowerBound[upZj]) {
					Z[upZ][upZj] = arcsLengthLowerBound[upZj];
					V[upZ][upZj] = -V[upZ][upZj]; // �ٶȷ���
				}
				if (Z[upZ][upZj] > arcsLengthUpperBound[upZj]) {
					Z[upZ][upZj] = arcsLengthUpperBound[upZj];
					V[upZ][upZj] = -V[upZ][upZj]; // �ٶȷ���
				}
			}
		}

		//std::cout << "��ǰ����·���� " << Pgv[1] << std::endl;
		//std::cout << "��ǰ����sump�� " << sump_sumPgv[0] << std::endl;

	}

} // end PsoForNeedle()