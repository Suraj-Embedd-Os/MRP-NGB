

#ifndef __COMM_H
#define __COMM_H
                 // Device header




#ifdef __cplusplus
 extern "C" {
#endif


#include<stdint.h>
#include	"rms.h"
#include "motor_protection.h"
#include "setup.h"


typedef struct msd_data{
	measured_data_t *msd_ptr; // pointer to all mesured data structure ,reference rms.h
	
}msd_data_t;

typedef struct msd_data_with_fault{
	msd_data_t *send_data;
	 	
	uint16_t  statu_register; 
	/*
		0-1 bits	= relay 1 relay 2 status
		2-9 bits	=cause of fault, per bit define the cause of fault see capture_fault_t enum
		in motor_protections.h 
	*/

}msd_data_with_fault_t;




/*
@Discriptions:	send noraml measured data to esp32
@para: Addres of send_data structure
		
*/
void send_to_esp32(msd_data_with_fault_t * send);

/*
@Discriptions:	rcv setup paramerter from esp32
@para: Address of meter_setup data structure of setting para meter
		
*/
void recv_from_esp32(meter_setup_t * setup_menu);

/**************************************************************************************************/

/************************ (C) COPYRIGHT NAGOBA ELECTRONICS  MAY 2022*****END OF FILE****/

#ifdef __cplusplus
}
#endif
#endif

