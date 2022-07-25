#ifndef __RMS_H
#define __RMS_H
                 // Device header
//#include "main.h"



#ifdef __cplusplus
 extern "C" {
#endif

#include"stdbool.h"
#include<stdint.h>
/***************************************************************************************/

/***********************Write preprocessor directives*************************************************/
//#define __ENABLE_CAL //enable or disable calibartions

/*
	set gain  which measeured by the ratio of register across OP_AMP value as per hardwa
*/
#define  GAIN  (16.0f)

#define VDC_OFFSET 							  1.65f // DC Offset Voltage
#define AN_VREF							      3.30f	// Analog Reference Voltage
#define ADC_BITS 								  12 // ADC bit resolutionn
#define ADC_MAXOUT 							  (1<<ADC_BITS)
/*
	As per requirement every 100ms,need to calculate RMS and takes necessay step.
	so based on that decide ACD sampling frequency.
	100ms==800smaple
	1sample =100*10^-3/800=125^10-6
	
*/
#define TIME_TO_CAL					 (100U) //100 ms
#define ENERY_PER_SEC				 (TIME_TO_CAL/10)
#define INPUT_Frequency			 (50U) 				
#define TOTAL_NO_SAMPLE      (400)  
#define SMAPLING_PERIOD			 ((1000*TIME_TO_CAL)/TOTAL_NO_SAMPLE) // sampling period time in usec
#define NO_OF_SAMPE_PER_CYCLE	 ((1000000U/SMAPLING_PERIOD)/INPUT_Frequency)

#define NO_OF_SAMPE_PER_QUATER  (NO_OF_SAMPE_PER_CYCLE/4)

#define   ENABLE                 (1)
#define   DISABLE                (0)

#define MAX_NUM_CYCLE_FREQ            (10U)
#define TOTAL_SUBMENU_PARAMETERS 			(7)

     /* first enable  CALIBARTION_EIN_DIS  then  SELECT_CALIBARTION_CHANNEL */  
		 /*****************To start calibartions  *************/
		 
#define      CALIBARTION_EIN_DIS   (0)  /* enable and disable the calibartions */
  
#define 		 NUM_CYCLE_FREQ		(10U)
#define      VOLT_Mul            (100U)
#define      CURRENT_Mul		 (10000U)	
#define      VOLT_MULTIPLIER     (230*VOLT_Mul)    
#define      CURRENT_MULTIPLIER  (5*CURRENT_Mul) 
#define      Average_Mul         ((float)1/(float)TOTAL_NO_SAMPLE)//()
	
#define volt_scal_addr			   0x15
#define current_scal_addr			 0x0

 
/**************** Adress defintion Start for Store and read data into from EEPROM ********************/
#define TOTAL_ADC_CHANNEL       (9U)
#define TOTAL_PHASE				(3U)	
#define TOTAL_LINE				(3U)
#define TOTAL_SLOT		         (2U)

/* Setup Parameters */
#define SETUP_SCALING					(1000)
#define Delay_SCALING 				(100)

//Default values for Settings
#define UNDER_VOLTAGE_DEF				(360 * SETUP_SCALING)
#define OVER_VOLTAGE_DEF				(405 * SETUP_SCALING)
#define UNDER_CURRENT_DEF				(50)
#define SHORT_CIRCUIT_DEF				(800)
#define ROTOR_LOCK_DEF					(400)
#define OVER_CURRENT_DEF				(200)
#define PROLONG_STARTING_DEF		(400)
#define FULL_LOAD_CURRENT_DEF		(4.6f * SETUP_SCALING)
#define PHASE_UNBALANCE_VOLT_DEF 	(10)
#define PHASE_UNBALANCE_CURR_DEF 	(10)
#define PHASE_LS_ED								(1)
#define PHASE_REV_E_D 							(1)
#define INV_CURR_E_D								(1)
#define MOTOR_CLASS_DEF					(3)//In index of motor class array
#define ADDRESS_DEF							(1)
#define	BAUD_RATE_DEF						(2)
#define PARITY_DEF							(0)
#define	FACTORY_RST_DEF					(0)
#define SETUP_DELAY_DEF					(5 * Delay_SCALING)

//Maximum and Minimum Values for Settings
#define UNDER_VOLTAGE_MIN				(300 * SETUP_SCALING)
#define UNDER_VOLTAGE_MAX				(390 * SETUP_SCALING)
#define OVER_VOLTAGE_MIN				(400 * SETUP_SCALING)
#define OVER_VOLTAGE_MAX				(460 * SETUP_SCALING)
#define UNDER_CURRENT_MIN				(20)
#define UNDER_CURRENT_MAX				(100)
#define SHORT_CIRCUIT_MIN				(700)
#define SHORT_CIRCUIT_MAX				(1000)
#define ROTOR_LOCK_MIN					(300)
#define ROTOR_LOCK_MAX					(700)
#define PHASE_UNBALANCE_VOLT_MIN 	(3)
#define	PHASE_UNBALANCE_VOLT_MAX	(30)
#define PHASE_UNBALANCE_CURR_MIN 	(3)
#define	PHASE_UNBALANCE_CURR_MAX	(30)
#define OVER_CURRENT_MIN				(150)
#define OVER_CURRENT_MAX				(300)
#define PROLONG_STARTING_MIN		(150)
#define PROLONG_STARTING_MAX		(800)
#define FULL_LOAD_CURRENT_MIN						(0.5 * SETUP_SCALING)
#define FULL_LOAD_CURRENT_MAX				(20 * SETUP_SCALING)
#define SETUP_DELAY_MIN							(0.5 * Delay_SCALING)
#define SETUP_DELAY_MAX							(30 * Delay_SCALING)


#define ADDRESS_MAX							(256)

/**************User variable Declaration Start***************************************/

/*
Discriptions:Measured data enum 
PARAMETERS	:
*/

enum{
	MSD_VOLT_RY=0, //voltage
	MSD_VOLT_YB,  //voltage
	MSD_VOLT_BR,  //voltage
	MSD_CURR_R,  //current
	MSD_CURR_Y, //current
	MSD_CURR_B, //current
	MSD_TL_KW, //total kw power
	MSD_TL_KVA, //total kva power
	MSD_TL_KVAR, // total kvar power
	MSD_PF,   //total pf
	MSD_MD_KW, //total measured demand KW
	MSD_MD_KVA, //total measured demand KVA
	MSD_KWH, //total energy KWH
	MSD_KVAH, //total energy KVAH
	MSD_THERMAL_PER, //Thermal pecentage
	MSD_LOAD_BARGRAPH_PER, // load bar graph pecentage
	MSD_CURR_LEAKAGE, //leakage currnet
	MSD_FREQ, //frequency
	MSD_MOTOR_STATUS, // motor status
	MSD_TRIP_COUNTER, // TRIP counter
	MSD_STOP_COUNTER, // stop counter
	MSD_START_COUNTER, // start counter
	MSD_TIME, // hh:mm:sec (0-23,0-59,0-59)
	MSD_DATE,// day:month:year (1-31,1-12,2000-2099)
	
	TOTAL_MSD_DATA
};

typedef enum
{
	R_PHASE=0,
	Y_PHASE,
	B_PHASE,
	TL_NO_PHASE
}phase_t;

typedef enum 
{
	KVA,
	KW,
	KVAR,
	TL_PARA_POWER	
}power_t;

typedef enum 
{
	KVAH,
	KWH,
	KVARH,
	TL_PARA_ENERGY	
}energy_t;

typedef enum
{
	RY_PHASE,
	YB_PHASE,
	BR_PHASE,
	
	TL_LL_NO_PHASE
}phase_ll_t;
 
typedef struct rms_data
{
		float voltage[TL_NO_PHASE],//voltage per phase
					voltage_LL[TL_NO_PHASE],// Line to Line voltage
					current_gain[TL_NO_PHASE], //current per phase with gain
					current_withoutgain[TL_NO_PHASE], //current per phase without gain
					display_currnet[TL_NO_PHASE], //current per phase to display
					power[TL_PARA_POWER][TL_NO_PHASE], //power per phase
					power_total[TL_PARA_POWER], // total power
					energy[TL_PARA_ENERGY], // enery total
					power_factor[TL_NO_PHASE],
					total_power_factor,
					frequency;	
			  
}rms_data_t;


typedef struct measured_data
{
	float all_measured_data[TOTAL_MSD_DATA];  //all measured
	
}measured_data_t;


typedef struct average_cal
{
	uint16_t volt[TL_NO_PHASE]; //volagte average
	uint16_t curr_without_gain[TL_NO_PHASE];  //current average
	uint16_t curr_gain[TL_NO_PHASE];
	
}average_cal_t;

/*
		Scaling factor 
*/
typedef struct scaling
{
	float volt_scal[TL_NO_PHASE]; 
	float curr_scal_gain[TL_NO_PHASE]; 
	float curr_scal_without_gain[TL_NO_PHASE]; 
	float power_scal_gain[TL_NO_PHASE];
	float power_scal_without_gain[TL_NO_PHASE];
}scaling_factor_t;


/***************************************************************************************/

typedef struct Common_Variable
{	
	uint32_t  Volt_sample_arr[NO_OF_SAMPE_PER_QUATER+2][TOTAL_PHASE];
	
	uint32_t Adc_volt_sample[TOTAL_PHASE],  // for volt average_cal 
	         Adc_Curr_sample_with_gain[TOTAL_PHASE], // for currnet average_ca
	         Adc_Curr_sample_without_gain[TOTAL_PHASE];  // for currnet average_ca
			 
	
	
	uint32_t Adc_SQR_Volt_Sample[TOTAL_PHASE],   // ADC volt square sample
			Adc_SQR_Curr_sample_with_gain[TOTAL_LINE], // ADC current with gain square sample
	    Adc_SQR_Curr_sample_without_gain[TOTAL_LINE];  // ADC current without gain square sample
	
 uint32_t Adc_SQR_LL_Volt_Sample[TOTAL_PHASE];
		       
		int32_t	KW_Sample[TOTAL_PHASE], // ADC KW  sample
			KVAR_Sample[TOTAL_PHASE]; // ADC KVAR  sample
	
		float freqSampleCounter_acc_curr,
					freqSampleZCDCounter_curr;
	

}Common_Var_t;

/***************************************************************************************/

typedef struct Store
{
	
	uint32_t Store_Adc_volt_sample[TOTAL_PHASE],  // for volt average_cal 
	         Store_Adc_Curr_sample_with_gain[TOTAL_PHASE], // for currnet average_ca
	         Store_Adc_Curr_sample_without_gain[TOTAL_PHASE];  // for currnet average_ca
			 
	
	
	uint32_t Store_Adc__SQR_Volt_Sample[TOTAL_PHASE],   // ADC volt square sample
			 Store_Adc__SQR_Curr_sample_with_gain[TOTAL_LINE], // ADC current with gain square sample
	         Store_Adc__SQR_Curr_sample_without_gain[TOTAL_LINE];  // ADC current without gain square sample
	
	uint32_t Store_Adc_SQR_LL_Volt_Sample[TOTAL_PHASE];
		        
			int32_t Store_KW_Sample[TOTAL_PHASE], // ADC KW  sample
			Store_KVAR_Sample[TOTAL_PHASE]; // ADC KVAR  sample
	
		float Store_freqSampleCounter_acc_curr,
					Store_freqSampleZCDCounter_curr;
		uint8_t Store_phase_shift_count[2][2];
	
}Store_Data_t;

/***************************************************************************************/

typedef enum 
{	
	/* NORMAL PARAMETERS */
	PARA_VRMS_RY = 1,
	PARA_VRMS_YB,
	PARA_VRMS_RB,
	
	PARA_IRMS_R,
	PARA_IRMS_Y,
	PARA_IRMS_B,
	
	PARA_THERMAL_CAPACITY,
	
	//MV_value
	PARA_FAULT_VALUE,
	
	//SV_Value
	PARA_SET_VALUE,
	
	/* SETUP PARAMETERS */
	PARA_UV,
	PARA_UV_DELAY,
	PARA_OV,
	PARA_OV_DELAY,
	PARA_UC,
	PARA_UC_DELAY,
	PARA_OC,
	PARA_OC_DELAY,
	PARA_RL,
	PARA_RL_DELAY,
	PARA_PS,
	PARA_PS_DELAY,
	PARA_UB_V,
	PARA_UB_V_DELAY,
	PARA_UB_C,
	PARA_UB_C_DELAY,
	PARA_SC,
	PARA_PL_E_D,
	PARA_PR_E_D,
	PARA_IC_E_D,
	PARA_FULL_LOAD_CURRENT,
	PARA_MOTOR_CLASS,
//	PARA_ADDRESS,
//	PARA_BAUD_RATE,
//	PARA_PARITY,
	PARA_FACTORY_RST,	
}RMS_READING_t;

/**************************************************************************************************/
/**************Declation of user function***************************************/

 void Rms_Calculations(void);

 void Scaling_Calculations(void);

 void Calibaration(void); 

 void Read_Eeprom_Data(void);

 void Store_Adc_Data(void);

 void  Adc_Sample_Calculations(uint32_t *Adc_Sample);

 void Hardware_Init(void);

void hard_init(void);
void meter_init(void);

//RMS_t * getRMSHandler(void);

float read_value (RMS_READING_t id);




/************************ (C) COPYRIGHT NAGOBA ELECTRONICS  MAY 2022*****END OF FILE****/

#ifdef __cplusplus
}
#endif
#endif

