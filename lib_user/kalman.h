#include "stm32f4xx.h"
#include "math.h"
#include "stdio.h"

void Kalman(void);

double getAngle(double newAngle, double newRate, double dt);

void setAngle(double newAngle); // Used to set angle, this should be set as the starting angle
double getRate(void); // Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangle(double newQ_angle);
void setQbias(double newQ_bias); 
void setRmeasure(double newR_measure); 

double getQangle(void);
double getQbias(void);
double getRmeasure(void); 
