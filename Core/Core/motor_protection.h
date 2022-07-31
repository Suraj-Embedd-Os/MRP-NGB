#ifndef  __PROTECTION_H
#define  __PROTECTION_H
 
#ifdef __cplusplus
 extern "C" {
#endif
#include <stdbool.h>
#include "main.h"

/***default min and max in term of percentage*********/

typedef enum{
	MIN_UV =20,
	MAX_UV=100,
	
	MIN_OV=20, 	//over volt
	MAX_OV=100, 	//over volt
	
	MIN_UB_V=5,	//volt unbalanced
	MAX_UB_V=80,	//volt unbalanced

	MIN_UC=10,	//under current
	MAX_UC=100,	//under current

	MIN_OC=10,	//over current
	MAX_OC=100,	//over current

	
	MIN_INVT_OL=10,	//inverse time overload
	MAX_INVT_OL=100,	//inverse time overload

	MIN_UB_C=5,	//Current unbalanced
	MAX_UB_C=80,	//Current unbalanced

	MIN_RL=800,		//rotor jam
	MAX_RL=1000,		//rotor jam

	MIN_PS=400,	//PROLONG START
	MAX_PS=800,	//PROLONG START
	
	MIN_UP,	//under power
	MAX_UP,	//under power

	MIN_OP,	//over power
	MAX_OP,	//over power

	
	MIN_GR_F,	//ground fault
	MAX_GR_F,	//ground fault

	MIN_O_TEMP, //over temp
	MAX_O_TEMP, //over temp

	MIN_ER_F,	//earth fault
	MAX_ER_F,	//earth fault

	MIN_CON_F,	//contatctor failure
	MAX_CON_F,	//contatctor failure


}TRIP_SETTING_RANGE_t;

enum{
	ALARM_MIN=50,  
	ALARM_MAX=95,
};




typedef enum
{
	NONE,
	UV,   //under volt
	OV, 	//over volt
	VPH_F,	//volt phase failure 
	UB_V,	//volt unbalanced
	VPH_R, //voltage phase reversal
	
	UC,	//under current
	OC,	//over current
	INVT_OL,	//inverse time overload
	UB_C,	//Current unbalanced
	RL,		//rotor jam
	PS,	//PROLONG START
	CPH_F,	//Current phase failure
	CPH_R, //current phase reversal
	
	UP,	//under power
	OP,	//over power
	
	GR_F,	//ground fault
	O_TEMP, //over temp
	ER_F,	//earth fault
	CON_F,	//contatctor failure
	
	TOTAL_PARA_FAILURE
	
}capture_fault_t;

 enum{
	
	NONE_fault=0,
	UV_fault=1<<UV,   //under volt
	OV_fault=1<<OV, 	//over volt
	VPH_F_fault=1<<VPH_F,	//volt phase failure 
	UB_V_fault=1<<UB_V,	//volt unbalanced
	VPH_R_fault=1<<VPH_R, //voltage phase reversal
	
	UC_fault=1<<UC,	//under current
	OC_fault=1<<OC,	//over current
	INVT_OL_fault=1<<INVT_OL,	//inverse time overload
	UB_C_fault=1<<UB_C,	//Current unbalanced
	RL_fault=1<<RL,		//rotor jam
	PS_fault=1<<PS,	//PROLONG START
	CPH_F_fault=1<<CPH_F,	//Current phase failure
	CPH_R_fault=1<<CPH_R, //current phase reversal
	
	UP_fault=1<<UP,	//under power
	OP_fault=1<<OP,	//over power
	
	GR_F_fault=1<<GR_F,	//ground fault
	O_TEMP_fault=1<<O_TEMP, //over temp
	ER_F_fault=1<<ER_F,	//earth fault
	CON_F_fault=1<<CON_F,	//contatctor failure
	
};



typedef struct{
	/*
		bits (0-2) =volgate phase failure
		bits (3- ) =volgate phase unbalanced
	*/
	uint8_t volt_phase_failue; //0 bits -RY, 1-bits=YB,2 -bits =BR
	uint8_t curr_phase_failue; //0 bits -RY, 1-bits=YB,2 -bits =BR
}phase_failure_t;


typedef enum 
{
	TIM_UV,   //under volt
	TIM_OV, 	//over volt
	TIM_UB_V,	//volt unbalanced
	
	TIM_UC,	//under current
	TIM_OC,	//over current
	TIM_INVT_OL,	//inverse time overload
	TIM_UB_C,	//Current unbalanced
	TIM_RL,		//rotor jam
	TIM_PS,	//PROLONG START
	
	TIM_UP,	//under power
	TIM_OP,	//over power
	
	TIM_GR_F,	//ground fault
	TIM_O_TEMP, //over temp
	TIM_ER_F,	//earth fault
	TIM_CON_F,	//contatctor failure
	
	TIM_TOTAL_PARA,
	
	/*extra fault enmu which has no timmer */
	FAULT_VOLT_PHASE_FAILURE,
	FAULT_VOLT_PHASE_REVERSAL,
	FAULT_CURR_PHASE_FAILURE,
	FAULT_CURR_PHASE_REVERSAL,
	FAULT_NONE
	
}timmer_t;

typedef struct
{
	uint16_t tripTimer[TIM_TOTAL_PARA];
}fault_counter_t;

typedef struct
{
	uint16_t delaySetup[TIM_TOTAL_PARA];
}delaySetup_t;

/*
	All fault related paramerte 
*/
typedef struct{	
	uint32_t 	capture_fault_value; //strored capure value
	/* see capture_fault_t enum for all bit
		eg OV-0bit, UV-1 bit,.....etc*/
	uint32_t fault_status_reg;  // fault status register 
	uint32_t value_at_fault; // 0 -bit =stop,1-bit =run,2-bit =start
	//  Use to display falut text on display check range  capture_fault_t enum  
	uint16_t 	cause_of_trip;
	//
	uint16_t  cause_of_alarm;
}capture_t;


/***function defintion here***/

uint32_t capture_fault_data(timmer_t id);
int32_t FULL_LOAD_CURRENT(uint16_t percentage); //return full current value for comparing fault
int32_t NOMINAL_VOLTAGE(uint16_t percentage);
void motorFunctions(void);
void motor_default_status(void);


#ifdef __cplusplus
}
#endif
#endif


/************************ (C) COPYRIGHT NAGOBA ELECTRONICS  MAY 2022*****END OF FILE****/









