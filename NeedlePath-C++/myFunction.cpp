#include<math.h>
#include<vector>
#include<time.h> 

#include"myFunction.h"

double MyFunction::dot(double* vector1, double* vector2) {
	double dotAns = vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2];
	return dotAns;
}

double MyFunction::norm(double* vector) {
	return sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

double* MyFunction::cross(double* vector1, double* vector2) {
	double ans[3] = { 0.0, };
	ans[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
	ans[1] = -(vector1[0] * vector2[2] - vector1[2] * vector2[0]);
	ans[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];
	return ans;
}

std::vector<std::vector<double>> MyFunction::randomMatrix(int row, int column) {
	
	std::vector<std::vector<double>> randomMatrix(row, std::vector<double>(column));

	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			srand((unsigned)time(NULL));
			randomMatrix[i][j] = rand() % (999 + 1) / (double)(999 + 1); // 999->小数点后三位
		}
	}

	return randomMatrix;
}