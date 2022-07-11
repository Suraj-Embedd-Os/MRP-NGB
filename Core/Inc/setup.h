
#ifndef __SET_UP_H
#define __SET_UP_H

#include "main.h"
#include<stdint.h>
#include "rms.h"
#include "motor_protection.h"



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
	SETUP_UB_V,	//volt unbalanced
	
	SETUP_UC,	//under current
	SETUP_OC,	//over current
	SETUP_INVT_OL,	//inverse time overload
	SETUP_UB_C,	//Current unbalanced
	SETUP_RL,		//rotor jam
	SETUP_PS,	//PROLONG START
	
	SETUP_UP,	//under power
	SETUP_OP,	//over power
	
	SETUP_GR_F,	//ground fault
	SETUP_O_TEMP, //over temp
	SETUP_ER_F,	//earth fault
	SETUP_CON_F,	//contatctor failure
	SETUP_FULL_LOAD_CURR, //full load current
	SETUP_NOMINAL_VOLT, // nominal voltage
	TOTAL_SETUP_PARA
}setup_t;


typedef struct meter_setupxxxx
{
	uint32_t meter_setup_menu[TOTAL_SETUP_PARA];
	
}meter_setup_t;









#endif


