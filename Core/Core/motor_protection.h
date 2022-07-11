#ifndef  __PROTECTION_H
#define  __PROTECTION_H
 
#ifdef __cplusplus
 extern "C" {
#endif
#include <stdbool.h>
#include "main.h"

/***DEFINE*********/
#define volt_multiple     (100U)
#define Current_scal		  (10000U)

#define UC_LIMIT				  (0.5*Current_scal) // under current 
#define OC_LIMIT					(8*Current_scal)  //over current

#define UV_LIMIT				  (180*volt_multiple)  //under volt
#define OV_LIMIT					(250*volt_multiple) // over volt



/***function defintion here***/
bool IsAutoStartSet(void);
void IsMotorStartButtonPress(void);
bool OverUnderCurrFault(void);
bool OverUnderVoltFault(void);

void motorfunctions(void);
void TripRelay(void);
void reset(void);

/**** failure detect functions  here****/
bool isOverCondition(void); // return true if any volt,current,power
bool isUnderCondition(void); // return true if any volt,current,power 
bool isPhaseLoss(void); // return true if any volt,current,power
bool isPhaseReversal(void); // return true if any volt,current,power
bool isPhaseUnblance(void); // return true if any volt,current,power

void relayOnOff(bool Switch);
void clearFlag(void);
void hysteresis(void);
void Check_Motor_status(void);
uint8_t getPersentage(void);
void delayselect(void);

int32_t FULL_LOAD_CURRENT(uint16_t percentage); //return full current value for comparing fault
int32_t NOMINAL_VOLTAGE(uint16_t percentage);

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
	CPH_R, //voltage phase reversal
	
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
	CPH_R_fault=1<<CPH_R, //voltage phase reversal
	
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



typedef struct
{
	uint16_t _counter[TOTAL_PARA_FAILURE-1];
}fault_counter_t;

typedef struct
{
	uint16_t _delaySetup[TOTAL_PARA_FAILURE-1];
}delaySetup_t;

/*
	All fault related paramerte 
*/
typedef struct{
	delaySetup_t			setup_delay;		// stored setup delay
	fault_counter_t	  counter;		//Timmer running 
	uint32_t 					capture_fault_value; //strored capure value
	phase_failure_t 		phase; // capture phase for fault

	/* see capture_fault_t enum for all bit
		eg OV-0bit, UV-1 bit,.....etc*/
	uint32_t fault_status_reg;  // fault status register 
	
	//  Use to display falut text on display check range  capture_fault_t enum  
	uint8_t 	cause_of_trip_reg;

	//uint8_t value_at_fault; // 0 -bit =stop,1-bit =run,2-bit =start
	
}capture_t;




#ifdef __cplusplus
}
#endif
#endif


/************************ (C) COPYRIGHT NAGOBA ELECTRONICS  MAY 2022*****END OF FILE****/









