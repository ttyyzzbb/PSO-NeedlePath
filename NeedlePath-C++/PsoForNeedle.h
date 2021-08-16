#include<vector>

#pragma once
#ifndef __PSOFORNEEDLE_H__
#define __PSOFORNEEDLE_H__

class PsoForNeedle {
public:
	static void psoForNeedle(std::vector<double> (*evaluationFun)(std::vector<double> arcs));
};


#endif // __PSOFORNEEDLE_H__
