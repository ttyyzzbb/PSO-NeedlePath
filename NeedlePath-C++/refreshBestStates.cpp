
#include<iostream>

#include"refreshBestStates.h"
#include"globalVariable.h"
#include"myFunction.h"

// refreshBestStates函数返回：sump, sumPgv
std::vector<double> RefreshBestStates::refreshBestStates(std::vector<double>(*evaluationFun)(std::vector<double> arcs)) {

	double sump = 0.0;
	double sumPvi = 0.0;
	int particalLength = Z[0].size();
	double normTemp[3] = { 0.0, };
	for (int i = 0; i < 3; i++) {
		normTemp[i] = Ptarget[i] - P0[i];
	}
	double Klen = Pgv[0] / MyFunction::norm(normTemp);
	double Kerror = 10;									// 原始值为30  这样精度还提高了
	double sumPgv = Pgv[0] * Klen + Pgv[1] * Kerror + Pgv[2] * Klen + Pgv[3];

	/* 所有粒子迭代循环 */
	for (int particalNumber = 0; particalNumber < M; particalNumber++) {
		// needleFunObstacle3D2Return[pathLen, tipError, deviationValue, obstacle, nonZeroLength]
		std::vector<double> needleFunObstacle3D2Return = evaluationFun(Z[particalNumber]);
		std::cout << needleFunObstacle3D2Return[0] << " + " << needleFunObstacle3D2Return[1] << " + " <<
			needleFunObstacle3D2Return[2] << " + " << needleFunObstacle3D2Return[3] << std::endl;
		if (needleFunObstacle3D2Return[4] < particalLength) {
			for (int i = needleFunObstacle3D2Return[4]; i < particalLength; i++) {
				Z[particalNumber][i] = 0;
			}
		}
		sump = needleFunObstacle3D2Return[0] * Klen + needleFunObstacle3D2Return[1] * Kerror + needleFunObstacle3D2Return[2] * Klen + needleFunObstacle3D2Return[3]; // 这一代粒子的适应值
		sumPvi = Pv[particalNumber][0] * Klen + Pv[particalNumber][1] * Kerror + Pv[particalNumber][2] * Klen + Pv[particalNumber][3]; // 这个粒子曾经最优状态的适应值
		//std::cout << sump << " + " << sumPvi << std::endl;
		if (sump < sumPvi) {
			for (int i = 0; i < 4; i++) {
				Pv[particalNumber][i] = needleFunObstacle3D2Return[i];
			}
			for (int i = 0; i < Z[particalNumber].size(); i++) {
				P[particalNumber][i] = Z[particalNumber][i];
			}
		}

		if (sump < sumPgv) {
			for (int i = 0; i < 4; i++) {
				Pgv[i] = needleFunObstacle3D2Return[i];
			}
			for (int i = 0; i < Z[particalNumber].size(); i++) {
				Pg[i] = Z[particalNumber][i];
			}
		}
	} // 所有粒子迭代循环

	std::vector<double> sump_sumPgv(2, 0.0);
	sump_sumPgv[0] = sump;
	sump_sumPgv[1] = sumPgv;

	return sump_sumPgv;

}