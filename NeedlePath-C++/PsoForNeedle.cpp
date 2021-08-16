#include<iostream>
#include<time.h>

#include"PsoForNeedle.h"
#include"globalVariable.h"
#include"myFunction.h"
#include"refreshBestStates.h"

void PsoForNeedle::psoForNeedle(std::vector<double>(*evaluationFun)(std::vector<double> arcs)) {

	for (int i = 0; i < arcsLengthUpperBound.size(); i++) {
		vmax.push_back((arcsLengthUpperBound[i] - arcsLengthLowerBound[i]) / 10);	// 粒子最大速度
		vmin.push_back(-vmax[i]);													// 粒子最小速度
	}

	// 终止条件，有点难确定
	double Ptarget_P0[3] = { 0.0, };
	for (int i = 0; i < 3; i++) {
		Ptarget_P0[i] = Ptarget[i] - P0[i];
	}
	double Pstopv = MyFunction::norm(Ptarget_P0) * 1.2;

	// 初始化粒子群状态 和 粒子群速度
	srand((unsigned)time(NULL));
	for (int i = 0; i < len; i++) {
		for (int particleNumber = 0; particleNumber < M; particleNumber++) {
			double suiji = (rand() % (999 + 1) / (double)(999 + 1));
			Z[particleNumber][i] = suiji*(arcsLengthUpperBound[i] - arcsLengthLowerBound[i]) + arcsLengthLowerBound[i];
			V[particleNumber][i] = V[particleNumber][i] * (vmax[i] - vmin[i]) + vmin[i];
		}
	}
	std::vector<double> sump_sumPgv = RefreshBestStates::refreshBestStates(evaluationFun);

	/* 开始迭代 N 次 */
	for (int iter = 0; iter < N; iter++) {
		std::cout << "迭代次数：" << iter << std::endl;
		// 更新个体最优和全局最优
		sump_sumPgv = RefreshBestStates::refreshBestStates(evaluationFun);
		sump_temp[iter] = sump_sumPgv[1];

		// 更新速度
		w = wmax - wk*iter; // 自适应惯性权重
		srand((unsigned)time(NULL));
		for (int upV = 0; upV < M; upV++) {
			for (int upVj = 0; upVj < Plength[upV]; upVj++) {
				double r1 = rand() % (999 + 1) / (double)(999 + 1);
				double r2 = rand() % (999 + 1) / (double)(999 + 1);
				V[upV][upVj] = w * V[upV][upVj] + c1*r1*(P[upV][upVj] - Z[upV][upVj]) + c2*r2*(Pg[upVj] - Z[upV][upVj]);
				if (V[upV][upVj] < vmin[upVj]) // 此实例速度对于结果的影响较大，不能忽略对速度限制
					V[upV][upVj] = vmin[upVj];
				if (V[upV][upVj] > vmax[upVj])
					V[upV][upVj] = vmax[upVj];
			}
		}

		// 更新状态
		for (int upZ = 0; upZ < M; upZ++) {
			for (int upZj = 0; upZj < Plength[upZ]; upZj++) {
				Z[upZ][upZj] = Z[upZ][upZj] + V[upZ][upZj];
				// 状态超出边界
				if (Z[upZ][upZj] < arcsLengthLowerBound[upZj]) {
					Z[upZ][upZj] = arcsLengthLowerBound[upZj];
					V[upZ][upZj] = -V[upZ][upZj]; // 速度反向
				}
				if (Z[upZ][upZj] > arcsLengthUpperBound[upZj]) {
					Z[upZ][upZj] = arcsLengthUpperBound[upZj];
					V[upZ][upZj] = -V[upZ][upZj]; // 速度反向
				}
			}
		}

		//std::cout << "当前迭代路径误差： " << Pgv[1] << std::endl;
		//std::cout << "当前迭代sump： " << sump_sumPgv[0] << std::endl;

	}

} // end PsoForNeedle()