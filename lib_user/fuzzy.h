#include "stm32f4xx.h"
#include "stdbool.h"
#include "stdlib.h"

void Fuzzy_ThetaInit(void);
void Fuzzy_ThetaDotInit(void);
void Fuzzy_Fuzzification(double theta, double theta_dot);
void Fuzzy_RuleCheck(void);
float Fuzzy_Defuzzification_Z(void);
float Fuzzy_Defuzzification_Y(void);
void Fuzzy_EdgeCalc(void);
void Fuzzy_MuCalc(double theta, double theta_dot);
void Fuzzy_ClearAll(void);
