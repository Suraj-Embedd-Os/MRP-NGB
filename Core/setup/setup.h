
#ifndef __SET_UP_H
#define __SET_UP_H

#include "main.h"
#include<stdint.h>
#include "rms.h"
#include "motor_protection.h"
#include "common_def.h"




/* data struture for setup menu 

0=disable
1=enable
11=NA(not available)
10,01
0%=NA(not available)

										No of bits			Bit postions		requirmnet				    range(in dec)
1	Enable/ Disable = 1 bits ,				0							0,1												0-2
	parameters
2	Auto reset			=	2 bits					1-2						00,01,10,11(NA)						0-3			
3	trip relay 1		=	2 bits,					3-4						00,01, 11(NA)						0-3
4	alarm relay 2		=	2 bits					5-6						00,01, 11(NA)						0-3
5	trip-setting		=	10 bits,				7-16					(1-1000)%of nominal val		0-1023
	range																						 	
6 Alarm range			= 7 bits 					17-23					(50-100)%of setted val		0-127
7	Time delay/trip = 7+1 bits				24-30					(0.3-100)sec 						 0-127
										 1 bits				  31						0,1
1 means less than 1 sec.
				total bits= 32(used)


31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0
for 100sec
EXAMPLE			= (0.110.0100),(00.0110.0100),(00.0110.0100),(01),(01),(01),(1)

for 0.3 sec
EXAMPLE			= (1.000.0011),(00.0110.0100),(00.0110.0100),(01),(01),(01),(1)

						
*/

typedef enum
{
	SETUP_UV,   //under volt
	SETUP_OV, 	//over volt
	SETUP_V_PF, //voltage phasae failure
	SETUP_UB_V,	//volt unbalanced
	SETUP_V_PR,	//votl reversal
	
	SETUP_UC,	//under current
	SETUP_OC,	//over current
	SETUP_INVT_OL,	//inverse time overload
	SETUP_UB_C,	//Current unbalanced
	SETUP_RL,		//rotor jam
	SETUP_PS,	//PROLONG START
	SETUP_C_PF, //current phasae failure
	SETUP_C_PR,	//current reversal


	
	SETUP_UP,	//under power
	SETUP_OP,	//over power
	
	SETUP_GR_F,	//ground fault
	SETUP_O_TEMP, //over temp
	SETUP_ER_F,	//earth fault
	SETUP_CON_F,	//contatctor failure
	SETUP_FULL_LOAD_CURR, //full load current
	SETUP_NOMINAL_VOLT, // nominal voltage
	TOTAL_SETUP_PARA,
}setup_t;

enum pos{
	
		PARA_ENABLE_POS= 					0, 			//0 pos , 1 -bits
		PARA_AUTO_RESERT_POS= 		1,		// 1 pos , 2 -bits
		PARA_TRIP_ENABLE_POS=			3,		// 3 pos , 2 -bits
		PARA_ALARM_ENABLE_POS=		5,				// 5 pos , 2 -bits
		PARA_TRIP_SETTING_POS=		7,	// 7 pos , 10 -bits
		PARA_ALARM_RANGE_POS=			17,		// 17 pos , 7 -bits
		PARA_TRIP_RANGE_POS=			24,		// 24 pos , 7 -bits
	
};

typedef enum 
{
		PARA_ENABLE_BIT= 					(1<<PARA_ENABLE_POS), 			//0 pos , 1 -bits
		PARA_AUTO_RESERT_BIT= 		(3<<PARA_AUTO_RESERT_POS),		// 1 pos , 2 -bits
		PARA_TRIP_ENABLE_BIT=			(3<<PARA_TRIP_ENABLE_POS),		// 3 pos , 2 -bits
		PARA_ALARM_ENABLE_BIT=		(3<<PARA_ALARM_ENABLE_POS),				// 5 pos , 2 -bits
		PARA_TRIP_SETTING_BIT=		(0x3ff<<PARA_TRIP_SETTING_POS),	// 7 pos , 10 -bits
		PARA_ALARM_RANGE_BIT=			(0x7f<<PARA_ALARM_RANGE_POS),		// 17 pos , 7 -bits
		PARA_TRIP_RANGE_BIT=			(0x7f<<PARA_TRIP_RANGE_POS)		// 24 pos , 7 -bits
	
}parameter_setup_bit_pos_t;

/*
		
set all bit as per default setup follow little endian;
0=disable
1=enable
11=NA(not available)
10,01
0%=NA(not available)

										No of bits			Bit postions		requirmnet				    range(in dec)
1	Enable/ Disable = 1 bits ,				0							0,1													0-2
	parameters
2	Auto reset			=	2 bits					1-2						00,01,10,11(NA)							0-3			
3	trip relay 1		=	2 bits,					3-4						00,01, 10,11(NA)						0-3
4	alarm relay 2		=	2 bits					5-6						00,01, 10,11(NA)						0-3
5	trip-setting		=	10 bits,				7-16					(1-1000)%of nominal val			0-1023
	range																						 	
6 Alarm range			= 7 bits 					17-23					(50-100)%of setted val			0-127
7	Time delay/trip = 7+1 bits				24-30					(0.3-100)sec 						 		0-127
										 1 bits				  31						0,1
1 means less than 1 sec.
				total bits= 32(used)


31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0
for 100sec
EXAMPLE			= (0.110.0100),(00.0110.0100),(00.0110.0100),(01),(01),(01),(1)

for 0.3 sec
EXAMPLE			= (1.000.0011),(00.0110.0100),(00.0110.0100),(01),(01),(01),(1)

*/
typedef enum default_value
{
	DEF_SETUP_UV=0x564197F,   //under volt
	DEF_SETUP_OV=0x564197F, 	//over volt
	DEF_SETUP_V_PF=0x6, //voltage phasae failure
	DEF_SETUP_UB_V=0x564197F,	//volt unbalanced
	DEF_SETUP_V_PR=0x6,	//votl reversal

	
	DEF_SETUP_UC=0x564197F,	//under current
	DEF_SETUP_OC=0x564197F,	//over current
	DEF_SETUP_INVT_OL=0x564197F,	//inverse time overload
	DEF_SETUP_UB_C=0x564197F,	//Current unbalanced
	DEF_SETUP_RL=0x565907F,		//rotor jam
	DEF_SETUP_PS=0x564C87F,	//PROLONG START
	DEF_SETUP_C_PF=0x6, //current phasae failure
	DEF_SETUP_C_PR=0x6,	//current reversal
	
	DEF_SETUP_UP=0x564197F,	//under power
	DEF_SETUP_OP=0x564197F,	//over power
	
	DEF_SETUP_GR_F=0x564197F,	//ground fault
	DEF_SETUP_O_TEMP=0x564197F, //over temp
	DEF_SETUP_ER_F=0x564197F,	//earth fault
	DEF_SETUP_CON_F=0x564197F,	//contatctor failure
	DEF_SETUP_FULL_LOAD_CURR=48000, //full load current 4.8 amp
	DEF_SETUP_NOMINAL_VOLT=40000, // nominal voltage 400 volt
	
}default_value_t;

typedef struct meter_setupxxxx
{
	uint32_t meter_setup_menu[TOTAL_SETUP_PARA];
	uint32_t default_setup_menu[TOTAL_SETUP_PARA];
	
}meter_setup_t;





void extact_data(uint32_t const *src,uint16_t *dest,uint8_t index,uint8_t num);
void initialised_default_setup(void);
void load_setup_parameter(void);
void seve_setup_parameter(void);


#endif


