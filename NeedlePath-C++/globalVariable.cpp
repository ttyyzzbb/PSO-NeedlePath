#include<iostream>

#include"globalVariable.h"

#define pi 3.1415926

int radius = 50;
double P0[3] = { -24, -58, -70 };
double Ptarget[3] = { 50, 0, 100 };
std::vector<double> arcsLengthLowerBound = { -pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0 }; // , -pi, 0, -pi, 0, -pi, 0, -pi, 0, -pi, 0};
std::vector<double> arcsLengthUpperBound = { pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2 }; // , pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2, pi, pi*radius / 2};
int len = arcsLengthUpperBound.size();

int N = 1000;
int M = 40;	
std::vector<std::vector<double>> Z(M, std::vector<double>(len, 0.0));
std::vector<std::vector<double>> V(M, std::vector<double>(len, 0.0));
std::vector<std::vector<double>> P(M, std::vector<double>(len, 0.0));
std::vector<std::vector<double>> Pv(M, std::vector<double>(4, 0.0));
std::vector<double> Pg(len, 0.0);	
std::vector<double> Pgv(4, 0.0);
std::vector<double> Plength(M, 0.0);
std::vector<double> sump_temp(N, 0.0);
std::vector<double> PgvLengthbuff(N, 0.0);
std::vector<double> PgvErrorbuff(N, 0.0);

double c1 = 1.0;
double c2 = 2.0;

std::vector<double> vmax(arcsLengthUpperBound.size(), 0.0);
std::vector<double> vmin(arcsLengthUpperBound.size(), 0.0);
double wmax = 1.5;
double wmin = 0.5;
double wk = (wmax - wmin) / N;
double w = 0.6;