

#include"setup.h"
#include<main.h>
#include "pt24xx.h"


meter_setup_t			meter_setup;

/*
		extract data as per requirment
		@param:index = extarct from start bit 
						num = upto extact
	
*/

void extact_data(uint32_t const *src,uint16_t *dest,uint8_t index,uint8_t num)
{
		*dest=0;
    uint8_t temp=0;
    uint8_t  mask_bit=0;
		for(temp=0;temp<num;temp++)
        mask_bit |=1<<temp;

    *dest = ((*src>>index) & mask_bit);
}

void initialised_default_setup(void)
{
	
		meter_setup.default_setup_menu[SETUP_UV]=DEF_SETUP_UV;  //
		meter_setup.default_setup_menu[SETUP_OV]=DEF_SETUP_OV;
	
		meter_setup.default_setup_menu[SETUP_UB_V]=DEF_SETUP_UB_V;
		meter_setup.default_setup_menu[SETUP_UC]=DEF_SETUP_UC;
	
		meter_setup.default_setup_menu[SETUP_OC]=DEF_SETUP_OC;
		meter_setup.default_setup_menu[SETUP_INVT_OL]=DEF_SETUP_INVT_OL;
	
		meter_setup.default_setup_menu[SETUP_UB_C]=DEF_SETUP_UB_C;
		meter_setup.default_setup_menu[SETUP_RL]=DEF_SETUP_RL;
	
		meter_setup.default_setup_menu[SETUP_PS]=DEF_SETUP_PS;
		meter_setup.default_setup_menu[SETUP_UP]=DEF_SETUP_UP;
	
		meter_setup.default_setup_menu[SETUP_OP]=DEF_SETUP_OP;
		meter_setup.default_setup_menu[SETUP_GR_F]=DEF_SETUP_GR_F;
	
		meter_setup.default_setup_menu[SETUP_O_TEMP]=DEF_SETUP_O_TEMP;
		meter_setup.default_setup_menu[SETUP_ER_F]=DEF_SETUP_ER_F;
		
		meter_setup.default_setup_menu[SETUP_CON_F]=DEF_SETUP_CON_F;
		meter_setup.default_setup_menu[SETUP_FULL_LOAD_CURR]=DEF_SETUP_FULL_LOAD_CURR;
		
		meter_setup.default_setup_menu[SETUP_NOMINAL_VOLT]=DEF_SETUP_NOMINAL_VOLT;
		
		/* copy all default setup first*/
		
}


void load_setup_parameter(void)
{
		uint32_t _temp_setup_para[TOTAL_SETUP_PARA]; 
		PT24xx_read(SETUP_PARA_ADDR,&meter_setup.meter_setup_menu[0],sizeof(meter_setup.meter_setup_menu));
		
}

void save_setup_parameter(void)
{
	PT24xx_write(SETUP_PARA_ADDR,&meter_setup.meter_setup_menu[0],sizeof(meter_setup.meter_setup_menu));

}

