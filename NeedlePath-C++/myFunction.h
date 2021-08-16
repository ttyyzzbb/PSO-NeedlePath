#pragma once

#ifndef __MYFUNCTION_H__
#define __MYFUNCTION_H__

class MyFunction {
public:
	static double dot(double* vector1, double* vector2);
	static double norm(double* vector);
	static double* cross(double* vector1, double* vector2);
	static std::vector<std::vector<double>> randomMatrix(int row, int column);		//输入行数和列数

}; // myFunction

#endif //__MYFUNCTION_H__