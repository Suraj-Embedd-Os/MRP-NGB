#include <math.h>
#include<stdio.h>
#include "pt24xx.h"
#include "rms.h"
#include "eeprom_addr.h"
#define SQR(X)  ((X)*(X))

/**global variable declare start******/
average_cal_t 		avg;
Store_Data_t  		store;
Common_Var_t  		common;
average_cal_t 		avg;
measured_data_t 	msd;
scaling_factor_t	scaling_factor;
rms_data_t          rms;

extern bool acc_data_ready;
uint8_t select_curr_line=0; // 0 bit- 1 line,1 bit- 2 line,2 bit- 3 line , default current with gain line selected.

/** local function declarations start here*************/
static void current_select_line(uint32_t *Sample);
static inline void processFrequencyCalculations(int32_t currSample);
/*
@Discriptions: function use for selecting current channel/line, gain or without gain for further operations
@parameters : sample -> gain current adc sample array,
			select_curr_line->addres of selct line variable
@Extra info:	select_curr_line-> (0-bit,for R phase current,1-bit for Y phase current, 2-bit for B phase current
 	
*/
static void current_select_line(uint32_t *Sample)
{
	static uint8_t _countSaturation[TL_NO_PHASE]={0,0,0};
	static uint8_t _tempCount=0; 
	_tempCount++; // increment every call this functions
	
	// check if it is sample start saturated.
	if(Sample[R_PHASE]>4000 || Sample[R_PHASE]<50)
	 {
		 //no of sample saturated per cycle
		 _countSaturation[R_PHASE]++;
	 }
	 // similar for 2nd phase
	if(Sample[Y_PHASE]>4000 || Sample[Y_PHASE]<50)
	 {
		_countSaturation[Y_PHASE]++;
	 }
	 // similar for 2nd phase
	 if(Sample[B_PHASE]>4000 || Sample[B_PHASE]<50)
	 {
		_countSaturation[R_PHASE]++;
	 }
	 
	 // check if one cycle has completed
	if(_tempCount>=NO_OF_SAMPE_PER_CYCLE)
	  {
			// if yes then check wheater ADC start saturation
		_tempCount=0; //make zero and start for next cycle
		
		//check is sample saturated , if greater than minimum value
		if( _countSaturation[R_PHASE]>2) // at least 2 smaple should go beyond the limit
		{
			_countSaturation[R_PHASE]=0;
			select_curr_line|=(0x07 & (1<<R_PHASE)); //set zero bit 
		}
		else
			select_curr_line&= (~(R_PHASE<<0)); //else reset zero bit
		
		if( _countSaturation[Y_PHASE]>2) //same for second
		{
			_countSaturation[Y_PHASE]=0;
			select_curr_line|=(0x07  & (1<<Y_PHASE)); //set 1 bit 
		}
		else
			select_curr_line &= (~(1<<Y_PHASE)); //else reset 1 bit
		
		if( _countSaturation[B_PHASE]>2)
		{
			_countSaturation[B_PHASE]=0;
			select_curr_line|=(0x07&(1<<B_PHASE)); //set 2 bit
		}
		else
			select_curr_line&=(~(1<<B_PHASE)); //else reset 2 bit
	}
	

}

/* To be called at sampling frequency for frequency measurement
*		Refer ti rms.h for calculation details
*///static 
	int32_t prevSample;
	//static 
		int32_t cycleCounter = 1; /* Ignore very first cycle frequency 
																			measurement */
	//static 
		float sampleCounter;
	
static inline void _ProcessFrequencyCalculation( int32_t currSample )
{
	float fracSample;
	
	// ZCD: Zero Crossing Detect check
	if( prevSample <= 0 && currSample > 0)
	{
		// Is it a Frequency Measurement Cycle (FMC) ?
		if( cycleCounter > 0) 
		{
			cycleCounter--; // Start of a new cycle
			sampleCounter += 1; // a ZCD cycle, but not a FMC
		}
		else // End of FMC, accumulate frequency of last FMC
		{
			// Calculate fractional sample: eqn.(6) d = X(n)/(X(n)-X(n+1))
			fracSample = (float)prevSample / (float)( prevSample - currSample);
			// Reload cycle counter
			cycleCounter = MFM_NUM_CYCLE_FREQ-1;
			/* Accumulate sample counter and take average while
					calculating frequency value */
			// Accumulate current cycle's sample counter value 
			common.freqSampleCounter_acc_curr += (sampleCounter+fracSample);
			// count number of accumulation cycles
			common.freqSampleZCDCounter_curr++;	
			// Assign residual sample for new cycle
			sampleCounter = 1.0f - fracSample; 
		}
	}
	else
	{
		sampleCounter += 1; // Not a ZCD cycle
	}
	
	// Retain current sample
	prevSample = currSample;
	
}	/* End of _ProcessFrequencyCalculation() */



static void phasereversal(int32_t sample[])
{
//	  tempCount++;
//	  static int32_t prev[2]={0},_curr[2]={0};
//	  _curr[0]=sample[0];
//	  _curr[1]=sample[1];
//	
//	 if( _curr[0]>0 && prev[0]<=0)
//	 {
//		 __test=1;
//		 _ZCDflasg1=true;
//		 phaseCounter[0]=tempCount;
//		
//	 }
//	
//	  if( _curr[1]>0 && prev[1]<=0)
//	 {
//		 if(__test==1)
//	 {
//		 __test=0;
//		 _ZCDflasg2=true;
//		 phaseCounter[1]=tempCount;
//	 }
//	 }
// 
//	 
//	 if(_ZCDflasg1 && _ZCDflasg2)
//	 {
//			 _ZCDflasg1=false;
//		   _ZCDflasg2=false;
//		 if(phaseCounter[0] < phaseCounter[1])
//			{
//			 _diffcount=phaseCounter[1]-phaseCounter[0];
//			}
//			tempCount=0;
//	}
//	 prev[0]=_curr[0];
//	 prev[1]=_curr[1];
}


/*
	@Discriptions: This Function is use for doing operation on ADC sample and strored into different array
	@Parameter :Adc_Sample-> arrary of collected adc sample,
				slot ->if overflow then use.
	@extra info: NA

 //TODO Verify the adc channel
0 -IR_L
1 -V_R
2 -IR_H
3 -IY_L
4 -V_Y
5 -IY_H
6 -IB_H
7 -V_B
8 -IB_L
*/

inline void  Adc_Sample_Calculations(uint32_t *Adc_Sample)
{
	static uint8_t  volt_Curr_Id = NO_OF_SAMPE_PER_QUATER,
									volt_delay90Idx=0;
	
	
	uint32_t _curr_withgain[TOTAL_PHASE];
	uint32_t _curr_withoutgain[TOTAL_PHASE];
	uint32_t _volt_sample[TOTAL_PHASE];
	
	phase_t phase;
	int32_t va[TOTAL_PHASE];
	
	//TODO select right adc channel for each  parameters
	// verify the Acd channel configuration based on circuit diagram
			_curr_withgain[R_PHASE]=Adc_Sample[2];
			_curr_withgain[Y_PHASE]=Adc_Sample[5];
			_curr_withgain[B_PHASE]=Adc_Sample[6];
	
		   _curr_withoutgain[R_PHASE]=Adc_Sample[0];
			_curr_withoutgain[Y_PHASE]=Adc_Sample[3];
			_curr_withoutgain[B_PHASE]=Adc_Sample[8];
	
			_volt_sample[R_PHASE]=Adc_Sample[1];
			_volt_sample[Y_PHASE]=Adc_Sample[4];
			_volt_sample[B_PHASE]=Adc_Sample[7];
			
	for(phase=0; phase< TOTAL_PHASE;phase++)
	 {	 
		  va[0]=(int32_t)_volt_sample[phase];
			va[1]=(int32_t)_curr_withgain[phase];
			va[2]=(int32_t)_curr_withoutgain[phase];
			
			//Accoumulated for average calculations
			common.Adc_volt_ll_sample[phase]         		+=(uint32_t)va[0];
			common.Adc_Curr_sample_with_gain[phase]  		+=(uint32_t)va[1];
			common.Adc_Curr_sample_without_gain[phase]	+=(uint32_t)va[2];
		 
		 	 //remove off set / refrence volt form the sample
			 va[0]-=avg.volt[ phase] ; // 12 bit adc =4096
																	// average=2048
			 va[1]-=avg.curr[ phase] ;
			 va[2]-=avg.curr[ phase] ;
		 
		
		  //Accoumulated sqr sample
		   common.Adc_SQR_Volt_Sample[phase] 								+=(uint32_t)SQR(va[0]);
		   common.Adc_SQR_Curr_sample_with_gain[phase] 			+=(uint32_t)SQR(va[1]);
		   common.Adc_SQR_Curr_sample_without_gain[phase] 	+=(uint32_t)SQR(va[2]);
		
			// store Sample_per_quarter for kvar calculation
		   common.Volt_sqr_sample_arr[volt_Curr_Id][phase]=va[0];
		   
		    // accumulates sample for kvr and kw calculation
		   if(select_curr_line & (1<<phase))
		   {
				 //if saturated then go for without gain current
				common.KVAR_Sample[phase]+= common.Volt_sqr_sample_arr[volt_Curr_Id][phase]*va[2];
				common.KW_Sample[phase]+= common.Volt_sqr_sample_arr[volt_delay90Idx][phase]*va[2]; 
		   }
		   else
		   {
			  common.KVAR_Sample[phase]+= common.Volt_sqr_sample_arr[volt_Curr_Id][phase]*va[1];
			  common.KW_Sample[phase]+= common.Volt_sqr_sample_arr[volt_delay90Idx][phase]*va[1]; 
		   }
		}
		
			volt_delay90Idx++;
			volt_Curr_Id++;
		if(volt_delay90Idx>NO_OF_SAMPE_PER_QUATER)
			volt_delay90Idx=0;
		if(volt_Curr_Id>NO_OF_SAMPE_PER_QUATER)
			volt_Curr_Id=0;
		/* check current adc saturations */ 
		current_select_line(&_curr_withgain[0]);

	 		
}
/*
	@Discriptions: This Function is use for store accumulated data to be used for calculation
	@Parameter :void			
	@extra info: NA
*/
inline void Store_Adc_Data(void)
{

	phase_t phase;
	for(phase=0; phase<TL_NO_PHASE;phase++)
	 {
	     //printf(" ll %d\n",common.Adc_volt_ll_sample[phase]);
		 // copy all data accumulated adc sample data
		 store.Store_Adc_volt_ll_sample[phase]=common.Adc_volt_ll_sample[phase];
		 store.Store_Adc_Curr_sample_with_gain[phase]			=common.Adc_Curr_sample_with_gain[phase];
		 store.Store_Adc_Curr_sample_without_gain[phase]		=common.Adc_Curr_sample_without_gain[phase];
		 
		 store.Store_Adc__SQR_Volt_Sample[phase]				=common.Adc_SQR_Volt_Sample[phase];
		 store.Store_Adc__SQR_Curr_sample_with_gain[phase]		=common.Adc_SQR_Curr_sample_with_gain[phase];
		 store.Store_Adc__SQR_Curr_sample_without_gain[phase]	=common.Adc_SQR_Curr_sample_without_gain[phase];
		        
		 store.Store_KW_Sample[phase]							=common.KW_Sample[phase];
		 store.Store_KVAR_Sample[phase]						=common.KVAR_Sample[phase];
			
    	//printf(" store %d\n",store.Store_Adc_volt_ll_sample[phase]);
	
	/*clear buffer  */
			common.Adc_volt_ll_sample[phase]					=0;
	    common.Adc_Curr_sample_with_gain[phase]			=0;
	    common.Adc_Curr_sample_without_gain[phase]		=0;
			 
			common.Adc_SQR_Volt_Sample[phase]					=0;
			common.Adc_SQR_Curr_sample_with_gain[phase]		=0;
	    common.Adc_SQR_Curr_sample_without_gain[phase]		=0;
		       
			common.KW_Sample[phase]							=0;
			common.KVAR_Sample[phase]							=0;
	
			//Average Calculations
		avg.volt[phase] =(uint16_t)( store.Store_Adc_volt_ll_sample[phase]*Average_Mul);
		avg.curr[phase] =(uint16_t)(store.Store_Adc_Curr_sample_with_gain[phase]*Average_Mul);	
			
	}
			/* stored sample counter and zero crossing sample counter */
					store.Store_freqSampleCounter_acc_curr=common.freqSampleCounter_acc_curr;
					store.Store_freqSampleZCDCounter_curr=common.freqSampleZCDCounter_curr;
					
	
}
/*
	@Discriptions: This Function is use to calculate scaling factors
	@Parameter :void			
	@extra info: GAIN Should be based of gain

	
*/
void Scaling_Calculations(void)
{
		phase_t phase;
	/***scaling factor calculation here***************/
	for(phase=0; phase<	TL_NO_PHASE;phase++)
	  {	
		
		// current scalling with gain
		scaling_factor.curr_scal_gain[phase]= (float)CURRENT_MULTIPLIER/\
			                      (float)sqrt(store.Store_Adc__SQR_Curr_sample_with_gain[phase]);
	  		
	// current scalling without gain
		//TODO GAIN
		scaling_factor.curr_scal_without_gain[phase]= scaling_factor.curr_scal_gain[phase]/(float)GAIN; //GAIN :check the gain fator and define
		
		// volatge scalling with gain
		scaling_factor.volt_scal[phase]= (float)VOLT_MULTIPLIER/\
													(float)sqrt(store.Store_Adc__SQR_Volt_Sample[phase]);	
		
		// power scalling with and without gain;
		scaling_factor.power_scal_gain[phase]=scaling_factor.volt_scal[phase]*scaling_factor.curr_scal_gain[phase];
		scaling_factor.power_scal_without_gain[phase]=scaling_factor.volt_scal[phase]*scaling_factor.curr_scal_without_gain[phase];
		
		
	}
		//write scalling value in eeprom
			PT24xx_write(VOLT_SCAL_ADDR,(uint32_t*)scaling_factor.volt_scal,sizeof(scaling_factor.volt_scal));
			PT24xx_write(CURR_SCAL_GAIN_ADDR,(uint32_t*)scaling_factor.curr_scal_gain,sizeof(scaling_factor.curr_scal_gain));

			PT24xx_write(CURR_SCAL_WITHOUT_GAIN_ADDR,(uint32_t*)scaling_factor.curr_scal_without_gain,sizeof(scaling_factor.curr_scal_without_gain));

}

/*
	@Discriptions: This Function is use for calculate all measured parameters
	@Parameter :void			
	@extra info: For power and energy select currnet according to selection of current line
*/
void Rms_Calculations(void)
{
	phase_t phase;
	
		energy_t energy_IDX=0;
	 float _energy[TL_PARA_POWER][TL_NO_PHASE]={0};
	 float _total_power[TL_PARA_POWER]={0};
	 float _total_energy[TL_PARA_POWER]={0};
	
	 
	for(phase=0; phase<TL_NO_PHASE;phase++)
	 {
			//voltage rms
		 rms.voltage[phase]=sqrt(store.Store_Adc__SQR_Volt_Sample[phase])*scaling_factor.volt_scal[phase];
		 
		 	//current rms with gain
		 rms.current_gain[phase]=sqrt(store.Store_Adc__SQR_Curr_sample_with_gain[phase])*scaling_factor.curr_scal_gain[phase];
		 	//voltage rms without gain
		 rms.current_withoutgain[phase]=sqrt(store.Store_Adc__SQR_Curr_sample_without_gain[phase])*scaling_factor.curr_scal_without_gain[phase];
		 
		
		 // KVA,KWA,KVAR power
		 /*
				if required convert to in Kilo by divide 1000;
		
		 */
		if(((select_curr_line & 0x01)&&phase==R_PHASE) ||  ((select_curr_line & 0x02)&&phase==Y_PHASE) 
			|| ((select_curr_line & 0x04)&&phase==B_PHASE))
		{
			/*  remove CURRENT_Mul by dividing because power scalling factor =current*vol so volt has 100 and current has 10000 */
			/* final result would be extra 100 multiples actucal supply power */ 
			
			rms.power[KVA][phase]= (rms.voltage[phase]*rms.current_withoutgain[phase])/CURRENT_Mul;
			rms.power[KW][phase]= (sqrt(store.Store_KW_Sample[phase])*scaling_factor.power_scal_without_gain[phase])/CURRENT_Mul;
			rms.power[KVAR][phase]=(sqrt(store.Store_KVAR_Sample[phase])*scaling_factor.power_scal_without_gain[phase])/CURRENT_Mul;
			
		}
		else
		{
			rms.power[KVA][phase]= (rms.voltage[phase]*rms.current_gain[phase])/CURRENT_Mul;
			rms.power[KW][phase]= (sqrt(store.Store_KW_Sample[phase])*scaling_factor.power_scal_gain[phase])/CURRENT_Mul;
			rms.power[KVAR][phase]=(sqrt(store.Store_KVAR_Sample[phase])*scaling_factor.power_scal_gain[phase])/CURRENT_Mul;
		}
		
		
		/* Accumulate total power */
		  _total_power[KVA] +=rms.power[KVA][phase];
		  _total_power[KW] +=rms.power[KW][phase];
			_total_power[KVAR] +=rms.power[KVAR][phase];
	
		
	
 }
		rms.power_total[KVA]= _total_power[KVA];
		rms.power_total[KW]= _total_power[KW];
		rms.power_total[KVAR]= _total_power[KVAR];
		
 /* make sure accumulation time 
			Enery +=power/per unit time  power/sec */
	for(energy_IDX=0;energy_IDX<TL_PARA_ENERGY;energy_IDX++)
	 {
			if(rms.power_total[energy_IDX]>0)
			{
				rms.energy[energy_IDX]+=rms.power_total[energy_IDX]/ENERY_PER_SEC;
			}
		
	 }
	 
	 
} 

/*
	TODO

*/

void Read_Eeprom_Data(void)
{	
	
		//read data from eeprom
	PT24xx_read(VOLT_SCAL_ADDR,(uint32_t*)scaling_factor.volt_scal,sizeof(scaling_factor.volt_scal));
	PT24xx_read(CURR_SCAL_GAIN_ADDR,(uint32_t*)scaling_factor.curr_scal_gain,sizeof(scaling_factor.curr_scal_gain));
	PT24xx_read(CURR_SCAL_WITHOUT_GAIN_ADDR,(uint32_t*)scaling_factor.curr_scal_without_gain,sizeof(scaling_factor.curr_scal_without_gain));
	//PT24xx_read(POW_SCAL_GAIN_ADDR,(uint32_t*)scaling_factor.power_scal_gain,sizeof(scaling_factor.power_scal_gain));
	//PT24xx_read(POW_SCAL_WITHOUT_GAIN_ADDR,(uint32_t*)scaling_factor.power_scal_without_gain,sizeof(scaling_factor.power_scal_without_gain));
			
	phase_t phase;
	for(phase=0; phase<TL_NO_PHASE;phase++)
	 {
			avg.volt[phase]=((AN_VREF)*(float)(ADC_MAXOUT))/(VDC_OFFSET);
			avg.curr[phase]=((AN_VREF)*(float)(ADC_MAXOUT))/(VDC_OFFSET);
		 
		 //POWER SCALLING FACTORS
		  scaling_factor.power_scal_gain[phase]=scaling_factor.volt_scal[phase]*scaling_factor.curr_scal_gain[phase];
			scaling_factor.power_scal_without_gain[phase]=scaling_factor.volt_scal[phase]*scaling_factor.curr_scal_without_gain[phase];
	 }	
		
	// Enable for calibarations
	  #ifdef __ENABLE_CAL
	  Calibaration();
	  #endif 
}

/*
	TODO

*/
void Calibaration(void)
{
	/* calibarte more than one time */
	while(acc_data_ready!=true);
	Scaling_Calculations();
	acc_data_ready=false;
	
	while(acc_data_ready!=true);
	Scaling_Calculations();
	acc_data_ready=false;
	
	while(acc_data_ready!=true);
	Scaling_Calculations();
	acc_data_ready=false;
	
	while(acc_data_ready!=true);
	Scaling_Calculations();
	acc_data_ready=false;
	
}

/* Displaying and other purpose used functions */	

int64_t read_value (RMS_READING_t id)
{
	/*int64_t value_return;
	switch(id)
	{	
		//normal parameters
		case PARA_VRMS_RY:
			value_return=Rms_t.voltageRms[RY_PHASE];
			break;
		case PARA_VRMS_YB: 
			value_return=Rms_t.voltageRms[YB_PHASE];
				break;
		case PARA_VRMS_RB:
			value_return=Rms_t.voltageRms[BR_PHASE];;//3rd voltage
				break;
		case PARA_IRMS_R:
			value_return=getCurrentValue(RY_PHASE);
				break;
		case PARA_IRMS_Y:
			value_return=getCurrentValue(YB_PHASE) ;///3rd current show as 2nd current
				break;
		case PARA_IRMS_B:
			value_return=getCurrentValue(BR_PHASE); 
		break;
		case PARA_THERMAL_CAPACITY:
			value_return= (presentTHCapacity*100); 
				break;
		
		case PARA_FAULT_VALUE:
			value_return= faultCap_Val;//(presentTHCapacity*100); 
				break;	
		
		case PARA_SET_VALUE:
			value_return= temp_menu_config[Index_SV];
				break;	
		
		//Setting parameters
		case PARA_UV:
			value_return= temp_menu_config[UNDER_VOLTAGE];			
			break;
		case PARA_UV_DELAY:
			value_return= temp_menu_config[UNDER_VOLTAGE_DELAY];			
			break;
		case PARA_OV:
			value_return=temp_menu_config[OVER_VOLTAGE];			
			break;
			case PARA_OV_DELAY:
			value_return=temp_menu_config[OVER_VOLTAGE_DELAY];			
			break;
		case PARA_UC:
			value_return=temp_menu_config[UNDER_CURRENT];			
			break;
		case PARA_UC_DELAY:
			value_return=temp_menu_config[UNDER_CURRENT_DELAY];			
			break;
			case PARA_SC:
			value_return=temp_menu_config[SHORT_CIRCUIT];			
			break;
			case PARA_RL:
			value_return=temp_menu_config[ROTOR_LOCK];			
			break;
			case PARA_RL_DELAY:
			value_return=temp_menu_config[ROTOR_LOCK_DELAY];			
			break;
			case PARA_UB_V:
			value_return=temp_menu_config[PHASE_UNBALANCE_VOLTAGE];			
			break;
				case PARA_UB_V_DELAY:
			value_return=temp_menu_config[PHASE_UNBALANCE_VOLTAGE_DELAY];			
			break;
				case PARA_UB_C:
			value_return=temp_menu_config[PHASE_UNBALANCE_CURRENT];			
			break;
			case PARA_UB_C_DELAY:
			value_return=temp_menu_config[PHASE_UNBALANCE_CURRENT_DELAY];			
			break;			
			case PARA_OC:
			value_return=temp_menu_config[OVER_CURRENT];			
			break;			
		case PARA_OC_DELAY:
			value_return=temp_menu_config[OVER_CURRENT_DELAY];			
			break;
			case PARA_PS:
			value_return=temp_menu_config[PROLONG_STARTING];			
			break;			
		case PARA_PS_DELAY:
			value_return=temp_menu_config[PROLONG_STARTING_DELAY];			
			break;
		case PARA_PL_E_D:
			value_return=temp_menu_config[PHASE_LOSS_ED];		
			break;			
		case PARA_PR_E_D:
			value_return=temp_menu_config[PHASE_REVERSAL_E_D];		
		break;	
		case PARA_IC_E_D:
			value_return=temp_menu_config[INVERSE_CURRENT_E_D];		
		break;	
		case PARA_FULL_LOAD_CURRENT:
			value_return=temp_menu_config[FULL_LOAD_CURRENT];			
			break;
		case PARA_MOTOR_CLASS:
			value_return= Motor_Class[temp_menu_config[MOTOR_CLASS]];
			break;
//		case PARA_ADDRESS:
//			value_return=temp_menu_config[ADDRESS];		
//		break;
//		case PARA_BAUD_RATE:
//			value_return=Menu_Baud_Rate[temp_menu_config[BAUD_RATE]];
//		break;
//		case PARA_PARITY:
//			value_return=temp_menu_config[PARITY];		
//		break;
		case PARA_FACTORY_RST:
			value_return=temp_menu_config[FACTORY_RST];
		break;
		default :			
			break;
	}
	return value_return;*/
}



