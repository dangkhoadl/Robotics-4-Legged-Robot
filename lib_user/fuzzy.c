#include "Fuzzy.h"

//Define fuzzy theta edge
#define NB_theta_left 	-99
#define NB_theta_mida		-98
#define NB_theta_midb		-20
#define NB_theta_right	-11


#define NS_theta_left 	-20
#define NS_theta_mida		-11
#define NS_theta_midb		-11
#define NS_theta_right 	-9

#define ZE_theta_left		-11
#define ZE_theta_mida  	-9
#define ZE_theta_midb  	 9
#define	ZE_theta_right	 11

#define PS_theta_left		 9
#define PS_theta_mida 	 11
#define PS_theta_midb 	 11
#define PS_theta_right 	 20

#define PB_theta_left 	11
#define PB_theta_mida 	20
#define PB_theta_midb		98
#define PB_theta_right 	99

//Define fuzzy theta_dot edge
#define NE_theta_dot_left		-3
#define NE_theta_dot_mida		-1
#define NE_theta_dot_midb		-1
#define	NE_theta_dot_right	-0.1


#define	ZE_theta_dot_left 	-1
#define ZE_theta_dot_mida		-0.1
#define ZE_theta_dot_midb		 0.1
#define	ZE_theta_dot_right 	 1


#define	PO_theta_dot_left 	 0.1
#define PO_theta_dot_mida		 1
#define PO_theta_dot_midb		 1
#define	PO_theta_dot_right 	 3

//Define fuzzy output_z_offset
float   NB_z_offset = -1.5,
				NS_z_offset =  -0.5,
				ZE_z_offset =  0,
				PS_z_offset =  0.5,
				PB_z_offset =  1.7;
				
//Define fuzzy output_y_offset
float   NB_y_offset = -2,
				NS_y_offset =  -1.2,
				ZE_y_offset =  0,
				PS_y_offset =  1.7,
				PB_y_offset =  3.5;

#define MAX(A,B)  ((A) > (B) ? (A) : (B))
#define MIN(A,B)  ((A) < (B) ? (A) : (B))

bool theta_state[5], theta_dot_state[3];

float theta_left_a[5], theta_left_b[5];
float theta_right_a[5], theta_right_b[5];

float theta_dot_left_a[3], theta_dot_left_b[3];
float theta_dot_right_a[3], theta_dot_right_b[3];

float theta_mu[5], theta_dot_mu[3], out_mu[5];
float out_str_z,
       sum_mu_z;
float out_str_y,
       sum_mu_y;

float theta_left[5], theta_mida[5], theta_midb[5], theta_right[5];
float theta_dot_left[3], theta_dot_mida[3], theta_dot_midb[3], theta_dot_right[3];

void Fuzzy_ThetaInit(void)
{
	//Nap gia tri cac diem cua bien trang thai theta
	theta_left[0] = NB_theta_left;
	theta_left[1] = NS_theta_left;
	theta_left[2] = ZE_theta_left;
	theta_left[3] = PS_theta_left;
	theta_left[4] = PB_theta_left;
	
	theta_mida[0] = NB_theta_mida;
	theta_mida[1] = NS_theta_mida;
	theta_mida[2] = ZE_theta_mida;
	theta_mida[3] = PS_theta_mida;
	theta_mida[4] = PB_theta_mida;
	
	theta_midb[0] = NB_theta_midb;
	theta_midb[1] = NS_theta_midb;
	theta_midb[2] = ZE_theta_midb;
	theta_midb[3] = PS_theta_midb;
	theta_midb[4] = PB_theta_midb;
	
	theta_right[0] = NB_theta_right;
	theta_right[1] = NS_theta_right;
	theta_right[2] = ZE_theta_right;
	theta_right[3] = PS_theta_right;
	theta_right[4] = PB_theta_right;
}

void Fuzzy_ThetaDotInit(void)
{
	//Nap gia tri cac diem cua bien trang thai theta_dot
	theta_dot_left[0] = NE_theta_dot_left;
	theta_dot_left[1] = ZE_theta_dot_left;
	theta_dot_left[2] = PO_theta_dot_left;
	
	theta_dot_mida[0] = NE_theta_dot_mida;
	theta_dot_mida[1] = ZE_theta_dot_mida;
	theta_dot_mida[2] = PO_theta_dot_mida;
	
	theta_dot_midb[0] = NE_theta_dot_midb;
	theta_dot_midb[1] = ZE_theta_dot_midb;
	theta_dot_midb[2] = PO_theta_dot_midb;
	
	theta_dot_right[0] = NE_theta_dot_right;
	theta_dot_right[1] = ZE_theta_dot_right;
	theta_dot_right[2] = PO_theta_dot_right;
}

void Fuzzy_Fuzzification(double theta, double theta_dot)
{
	uint8_t i;
	
	for(i=0;i<5;i++)
	{
	/* theta fuzzification */	
	theta_state[i] = (theta >= theta_left[i]) && (theta < theta_right[i]);
	}
	
	for(i=0;i<3;i++)
	{
	/* theta_dot fuzzification */	
	theta_dot_state[i] = (theta_dot >= theta_dot_left[i]) && (theta_dot < theta_dot_right[i]);
	}
}
	
void Fuzzy_RuleCheck(void)
{
	int8_t i,j;
	
	/********************************************
	Rule table
		 | j		0	    1	  2	  3	    4
		 |			NB    NS  ZE  PS   	PB
	___|________________________________________
i    |
0	NE |      PB    PB  PS  ZE    NS
1	ZE |      PB    PS  ZE  NS    NB
2	PO |      PS    ZE  NS  NB    NB
	*********************************************/
	
	//Tinh muy dau ra NB
	j=3;
	for(i=2;i>=1;i--)
	{
		if(out_mu[0] < MIN(theta_mu[j], theta_dot_mu[i]))
			out_mu[0] = MIN(theta_mu[j], theta_dot_mu[i]);
		j++;
	}		
		if(out_mu[0] < MIN(theta_mu[4], theta_dot_mu[2]))
			out_mu[0] =  MIN(theta_mu[4], theta_dot_mu[2]);
	
	//Tinh muy dua ra NS
	j=2;
	for(i=2;i>=0;i--)
	{
		if(out_mu[1] < MIN(theta_mu[j], theta_dot_mu[i]))
			out_mu[1] = MIN(theta_mu[j], theta_dot_mu[i]);
		j++;
	}		
	
	//Tinh muy dau ra ZE
	j = 1;
	for(i=2;i>=0;i--)
	{
		if(out_mu[2] < MIN(theta_mu[j], theta_dot_mu[i]))
			out_mu[2] = MIN(theta_mu[j], theta_dot_mu[i]);
		j++;
	}
	
	//Tinh muy dau ra PS
	j = 0;
	for(i=2;i>=0;i--)
	{
		if(out_mu[3] < MIN(theta_mu[j], theta_dot_mu[i]))
			 out_mu[3] = MIN(theta_mu[j], theta_dot_mu[i]);
		j++;
	}
	
	
	//Tinh muy dau ra PB
	j = 0;
	for(i=1;i>=0;i--)
	{
		if(out_mu[4] < MIN(theta_mu[j], theta_dot_mu[i]))
			 out_mu[4] = MIN(theta_mu[j], theta_dot_mu[i]);
		j++;
	}
	if(out_mu[4] < MIN(theta_mu[0], theta_dot_mu[0]))
			out_mu[4] = MIN(theta_mu[0], theta_dot_mu[0]);
}

float Fuzzy_Defuzzification_Z(void)
{
	out_str_z = NB_z_offset*out_mu[0] + NS_z_offset*out_mu[1] + PS_z_offset*out_mu[3]+ PB_z_offset*out_mu[4];
	sum_mu_z = out_mu[0]+out_mu[1]+out_mu[2]+out_mu[3]+out_mu[4];
	
	if (sum_mu_z != 0) out_str_z= out_str_z/sum_mu_z;
	else out_str_z = 0;
	
	return out_str_z;
}

float Fuzzy_Defuzzification_Y(void)
{
	out_str_y = NB_y_offset*out_mu[0] + NS_y_offset*out_mu[1] + PS_y_offset*out_mu[3]+ PB_y_offset*out_mu[4];
	sum_mu_y = out_mu[0]+out_mu[1]+out_mu[2]+out_mu[3]+out_mu[4];
	
	if (sum_mu_y != 0) out_str_y = out_str_y/sum_mu_y;
	else out_str_y = 0;
	
	return out_str_y;
}

void Fuzzy_EdgeCalc(void)
{
	uint8_t i;
	for(i=0;i<5;i++)
	{
			theta_left_a[i] = 1/(theta_mida[i] - theta_left[i]);
			theta_left_b[i] = theta_left[i]/(theta_left[i] - theta_mida[i]);
			theta_right_a[i] = 1/(theta_midb[i] - theta_right[i]);
			theta_right_b[i] = theta_right[i]/(theta_right[i] - theta_midb[i]);
	}
	
	for(i=0;i<3;i++)
	{
			theta_dot_left_a[i] = 1/(theta_dot_mida[i] - theta_dot_left[i]);
			theta_dot_left_b[i] = theta_dot_left[i]/(theta_dot_left[i] - theta_dot_mida[i]);
			theta_dot_right_a[i] = 1/(theta_dot_midb[i] - theta_dot_right[i]);
			theta_dot_right_b[i] = theta_dot_right[i]/(theta_dot_right[i] - theta_dot_midb[i]);
	}
}

void Fuzzy_MuCalc(double theta, double theta_dot)
{
	uint8_t i;
	for(i=0;i<5;i++)
	{
		if(theta_state[i] == 1)
		{
			if(theta > theta_midb[i])				theta_mu[i] = theta*theta_right_a[i] + theta_right_b[i];
			else if(theta < theta_mida[i])	theta_mu[i] = theta*theta_left_a[i] + theta_left_b[i];
			else theta_mu[i] = 1;
		}
		else
			theta_mu[i] = 0;
	}
	
	for(i=0;i<3;i++)
	{
		if(theta_dot_state[i] == 1)
		{
			if(theta_dot > theta_dot_midb[i]) 	theta_dot_mu[i] = theta_dot*theta_dot_right_a[i] + theta_dot_right_b[i];
			else if(theta_dot < theta_dot_mida[i])	theta_dot_mu[i] = theta_dot*theta_dot_left_a[i] + theta_dot_left_b[i];
			else theta_mu[i] = 1;
		}
		else
			theta_dot_mu[i] = 0;
	}
}

void Fuzzy_ClearAll(void)
{
	uint8_t i;
	for(i=0;i<5;i++)
	{
		theta_state[i] = 0;
		theta_mu[i] = 0;
		out_mu[i] = 0;
	}
	for(i=0;i<3;i++)
	{
		theta_dot_state[i] = 0;
		theta_dot_mu[i] = 0;
	}
}
