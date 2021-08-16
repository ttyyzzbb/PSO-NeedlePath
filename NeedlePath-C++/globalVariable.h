#pragma once
#include<vector>

#ifndef __GLOBALVARIABLE_H__
#define __GLOBALVARIABLE_H__

// ����Ⱥ�㷨����������
extern int radius;
extern double P0[3];
extern double Ptarget[3];
extern std::vector<double> arcsLengthLowerBound; // = [-pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0]; // , -pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0];
extern std::vector<double> arcsLengthUpperBound; // = [pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2]; // , pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2];
extern int len;

extern int N;									//��������
extern int M;									// ��Ⱥ����
extern std::vector<std::vector<double>> Z;		// ��Ⱥ����   M��len�е������
extern std::vector<std::vector<double>> V;		// �����ٶ�
extern std::vector<std::vector<double>> P;		// ���嵽������״̬(ÿ�����ӵ�����״̬)
extern std::vector<std::vector<double>> Pv;		// ���嵽�����״̬ʱ�����Ӧֵ(ÿ�����ӵ�������Ӧֵ)
extern std::vector<double> Pg;					// ��Ⱥ���������״̬(��������)
extern std::vector<double> Pgv;					// ����״̬����Ӧֵ(Ҳ����ȫ��������Ӧֵ)
extern std::vector<double> Plength;				// һ����
extern std::vector<double> sump_temp;			// �洢ÿһ��������Ӧֵ
extern std::vector<double> PgvLengthbuff;
extern std::vector<double> PgvErrorbuff;

extern double c1;
extern double c2;
extern std::vector<double> vmax;				// ��������ٶ�
extern std::vector<double> vmin;				// ������С�ٶ�
extern double wmax;						// ��ʼ��1.5
extern double wmin;						// ��ʼ��0.5
extern double wk;			// Ȩ��ϵ������ϵ��
extern double w;

#endif // __GLOBALVARIABLE_H__