#pragma once
#include<vector>

#ifndef __GLOBALVARIABLE_H__
#define __GLOBALVARIABLE_H__

// 粒子群算法参数及矩阵
extern int radius;
extern double P0[3];
extern double Ptarget[3];
extern std::vector<double> arcsLengthLowerBound; // = [-pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0]; // , -pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0];
extern std::vector<double> arcsLengthUpperBound; // = [pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2]; // , pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2];
extern int len;

extern int N;									//迭代次数
extern int M;									// 种群数量
extern std::vector<std::vector<double>> Z;		// 种群个体   M行len列的零矩阵
extern std::vector<std::vector<double>> V;		// 个体速度
extern std::vector<std::vector<double>> P;		// 个体到达的最好状态(每个粒子的最优状态)
extern std::vector<std::vector<double>> Pv;		// 个体到达最好状态时候的适应值(每个粒子的最优适应值)
extern std::vector<double> Pg;					// 种群到达的最优状态(最优粒子)
extern std::vector<double> Pgv;					// 最优状态的适应值(也就是全局最优适应值)
extern std::vector<double> Plength;				// 一行零
extern std::vector<double> sump_temp;			// 存储每一代最优适应值
extern std::vector<double> PgvLengthbuff;
extern std::vector<double> PgvErrorbuff;

extern double c1;
extern double c2;
extern std::vector<double> vmax;				// 粒子最大速度
extern std::vector<double> vmin;				// 粒子最小速度
extern double wmax;						// 初始是1.5
extern double wmin;						// 初始是0.5
extern double wk;			// 权重系数迭代系数
extern double w;

#endif // __GLOBALVARIABLE_H__