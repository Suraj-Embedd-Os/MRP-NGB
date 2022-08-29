#include "motor_protection.h"
#include "rms.h"
#include "setup.h"
#include "button.h"


#define ON  (1U)
#define OFF (0U)
#define SQR(X)	((X)*(X))
/****START GLOBAL VARIBALE ***/


bool trip_happens=false;
uint8_t motor_status=0x01; // 0 bit stop,1 bit start ,2 bit running

enum{
		MOTOR_STOP=(0x1F&(1<<0)),
		MOTOR_START=(0x1F&(1<<1)),
		MOTOR_RUNNING=(0x1F&(1<<2)),
		ALARM_RELAY=(0x1F &(1<<3)),
		TRIP_RELAY=(0x1F &(1<<4)),
	
};

extern rms_data_t   rms;  // all reading parameter
extern meter_setup_t			meter_setup;
extern Store_Data_t  		store;

fault_counter_t				 fault_trip_counter;
delaySetup_t				   delaySetupTim;
capture_t							  motor_var;

/*          helper functions               */
static bool isUnderVolt(void);
static bool isOverVolt(void);
static bool isPhaseFailueVolt(void);
static bool isPhaseUnbalanceVolt(void);
static bool isPhaseReversalVolt(void);

static bool isUnderCurr(void);
static bool isOverCurr(void);
static bool isPhaseFailueCurr(void);
static bool isPhaseUnbalanceCurr(void);
static bool isPhaseReversalCurr(void);
static bool isRotorLockCurr(void);
static bool isProlongStartCurr(void);
static bool isInvCurrOverLoad(void);

static bool isOverPower(void);
static bool isUnderPower(void);


static bool isGroundFault(void);
static bool isEarthFault(void);

static bool isContactorFault(void);
static void thermalCapacity(void);

static void voltage_related_fault(void);
static void current_related_fault(void);
static void power_related_fault(void);
static void other_related_fault(void);
static void clear_flag(void);

static void tripRelay(void);
static void alarmRelay(void);
static void clear_fault(void);

void check_min_max_limit(uint16_t max_limit,uint16_t min_limit,uint16_t *des)
{
	
	if(*des >=min_limit && *des<=max_limit)
		return;
	else
		*des=min_limit;
	return;
}

/*
	Alarm relay on /off
*/
void motor_alarm_relay(bool val)
{
	if(val){
		RL2_GPIO_Port->ODR |=RL1_Pin;
		motor_status |=ALARM_RELAY;
	}
	else
	{
		RL2_GPIO_Port->ODR &=~RL1_Pin;
		motor_status &=~ALARM_RELAY;
	}
}

/*
		Trip relay on /off
*/

void motor_trip_relay(bool val)
{
	if(val)
	{
		RL1_GPIO_Port->ODR |=RL2_Pin;
		motor_status |=TRIP_RELAY;
	}
	else
	{
		RL1_GPIO_Port->ODR &=~RL2_Pin;
		motor_status &=~TRIP_RELAY;
		
	}
}


/*
@ helper function return current value
@Arg: pass percentage
@return  :if succes return value or else -1;
*/
int32_t FULL_LOAD_CURRENT(uint16_t percentage)
{
	int32_t rated_curr=meter_setup.meter_setup_menu[SETUP_FULL_LOAD_CURR]; /// 4.8 amp //10000 multipilcations //read from menu_config
		if(percentage>0)
		{
				return (rated_curr*percentage)/100;
		}
		else
			return -1; //return no ok
}
/*
@ helper function return voltage value
@Arg: pass percentage
@return  :if succes return value or else -1;
*/
int32_t NOMINAL_VOLTAGE(uint16_t percentage)
{
		int32_t nominal_voltage=meter_setup.meter_setup_menu[SETUP_NOMINAL_VOLT]; /// 4.8 amp //100 multipilcations
		if(percentage>0)
		{
				return (nominal_voltage*percentage)/100;
		}
		else
			return -1; //return no ok
}
/********************************************/

static float find_max(float *x,float *y,float *z)
{
	return (*x>*y?*x>*z?*x:*z:*y>*z?*y:*z);
}

static float find_min(float *x,float *y,float *z)
{
	return (*x<*y?*x<*z?*x:*z:*y<*z?*y:*z);
}
/*******************************************/

/*
 @Discriptions: if voltage is less than setted voltages
	@helper function to check UnderVolt
*/

static  bool isUnderVolt()
{
		uint16_t _under_volt_per=0;
	
	//extract data from setup variable
		extact_data(&meter_setup.meter_setup_menu[SETUP_UV],&_under_volt_per,7,10);

	//check_min_max_limit
	check_min_max_limit(MAX_UV,MIN_UV,&_under_volt_per);
	
	if((rms.voltage_LL[RY_PHASE]<(float)(NOMINAL_VOLTAGE(_under_volt_per)))|| \
		(rms.voltage_LL[YB_PHASE]<(float)NOMINAL_VOLTAGE(_under_volt_per))|| \
			(rms.voltage_LL[BR_PHASE]<(float)NOMINAL_VOLTAGE(_under_volt_per)))
	{
			motor_var.fault_status_reg |=UV_fault; //set UC bit high
			fault_trip_counter.tripTimer[UV]++;
		
			return true;
	}
	else
	{
			motor_var.fault_status_reg &=(~UV_fault); //Reset UC bit high
			fault_trip_counter.tripTimer[UV]=0;
			return false;
	}
}

/*
Discriptions: if voltage is greater than setted voltages
helper function to check overvolt
*/

static bool isOverVolt()
{
			uint16_t _over_volt_per=0;//=(uint16_t)menu_config[OV];
	
	//extract data from setup variable
		extact_data(&meter_setup.meter_setup_menu[SETUP_OV],&_over_volt_per,7,10);
	
	//check_min_max_limit
	check_min_max_limit(MAX_OV,MIN_OV,&_over_volt_per);
	
	if((rms.voltage_LL[RY_PHASE]>(float)NOMINAL_VOLTAGE(_over_volt_per))|| \
		(rms.voltage_LL[YB_PHASE]>(float)NOMINAL_VOLTAGE(_over_volt_per))|| \
			(rms.voltage_LL[BR_PHASE]>(float)NOMINAL_VOLTAGE(_over_volt_per)))
	{
			motor_var.fault_status_reg |=OV_fault; //set UC bit high
		fault_trip_counter.tripTimer[OV]++;
			return true;
	}
	else
	{
			motor_var.fault_status_reg &=(~OV_fault); //Reset UC bit high
		fault_trip_counter.tripTimer[OV]=0;
			return false;
	}
}


/*
Discrption: if any one of phase is less than certain limit consider as phase loss or failure
@helper function to check voltage phase failure
*/
static bool isPhaseFailueVolt()
{
	uint16_t _phase_failure_per=80;//percent of nominal voltage;
	

	if((rms.voltage_LL[RY_PHASE]<(float)NOMINAL_VOLTAGE(_phase_failure_per))|| \
		(rms.voltage_LL[YB_PHASE]<(float)NOMINAL_VOLTAGE(_phase_failure_per))|| \
			(rms.voltage_LL[BR_PHASE]<(float)NOMINAL_VOLTAGE(_phase_failure_per)))
	{
			motor_var.fault_status_reg |=VPH_F; //set UC bit high
			fault_trip_counter.tripTimer[VPH_F]++;
			return true;
	}
	else
	{
			motor_var.fault_status_reg &=(~VPH_F_fault); //Reset UC bit high
		fault_trip_counter.tripTimer[VPH_F]=0;
			return false;
	}
}

/*
Discrption: if any one of two high and low phase is less than certain limit consider as phase unbalanced
@helper function to check voltage unbalanced
*/

static bool isPhaseUnbalanceVolt(void)
{
	uint16_t _phase_ub_volt_per=0;
	
	extact_data(&meter_setup.meter_setup_menu[SETUP_UB_V],&_phase_ub_volt_per,7,10);
	
	//check_min_max_limit
	check_min_max_limit(MAX_UB_V,MIN_UB_V,&_phase_ub_volt_per);

	float max_volt =find_max(&rms.voltage_LL[RY_PHASE],&rms.voltage_LL[YB_PHASE],&rms.voltage_LL[BR_PHASE]);
	float min_volt =find_min(&rms.voltage_LL[RY_PHASE],&rms.voltage_LL[YB_PHASE],&rms.voltage_LL[BR_PHASE]);
	
	int _per_error = (int)((100*(max_volt-min_volt))/max_volt);
	
	if(_per_error>=_phase_ub_volt_per)
	{
		motor_var.fault_status_reg |=UB_V_fault;
		fault_trip_counter.tripTimer[UB_V]++;
		return true;
		
	}
	else
	{
		motor_var.fault_status_reg &=(~UB_V_fault);
		fault_trip_counter.tripTimer[UB_V]=0;
		return false;
	}
	
	
}

/*
Discrption: if any one of two high and low phase is less than certain limit consider as phase unbalanced
@helper function to check voltage unbalanced
*/

static bool isPhaseReversalVolt(void)
{
		uint8_t _noofsample120=(120*NO_OF_SAMPE_PER_CYCLE)/360;
	
	if((store.Store_phase_shift_count[0][0] >= (_noofsample120-3)  \
		  && store.Store_phase_shift_count[0][0] <= (_noofsample120+3)) && \
			store.Store_phase_shift_count[0][1] >= (_noofsample120-3)  \
		  && store.Store_phase_shift_count[0][1] <= (_noofsample120+3)	
		)
	return false;
	else
	{
		motor_var.fault_status_reg |=VPH_R_fault;
		return true;
	}

}

/*
@helper function to check voltage under current
*/

static bool isUnderCurr(void)
{
	uint16_t _under_curr_per=0;//=(uint16_t)menu_config[UC];
	
	extact_data(&meter_setup.meter_setup_menu[SETUP_UC],&_under_curr_per,7,10);
	
	//check_min_max_limit
	check_min_max_limit(MAX_UC,MIN_UC,&_under_curr_per);

	if((rms.display_currnet[RY_PHASE]<(float)FULL_LOAD_CURRENT(_under_curr_per))|| \
		(rms.display_currnet[YB_PHASE]<(float)FULL_LOAD_CURRENT(_under_curr_per))|| \
			(rms.display_currnet[BR_PHASE]<(float)FULL_LOAD_CURRENT(_under_curr_per)))
	{
			motor_var.fault_status_reg |=UC_fault; //set UC bit high
			fault_trip_counter.tripTimer[UC]++;
			return true;
	}
	else
	{
			motor_var.fault_status_reg &=(~UC_fault); //Reset UC bit high
			fault_trip_counter.tripTimer[UC]=0;
			return false ;
	
	}
}
	
/*
@helper function to check voltage over current
*/
static bool isOverCurr(void)
{
	uint16_t _oc_per=0;//=(uint16_t)menu_config[OC];
	
		extact_data(&meter_setup.meter_setup_menu[SETUP_OC],&_oc_per,7,10);
	
		//check_min_max_limit
		check_min_max_limit(MAX_OC,MIN_OC,&_oc_per);
	
	if((rms.display_currnet[RY_PHASE]>(float)FULL_LOAD_CURRENT(_oc_per))|| \
		(rms.display_currnet[YB_PHASE]>(float)FULL_LOAD_CURRENT(_oc_per))|| \
			(rms.display_currnet[BR_PHASE]>(float)FULL_LOAD_CURRENT(_oc_per)))
	{
			motor_var.fault_status_reg |=OC_fault; //set UC bit high
			fault_trip_counter.tripTimer[OC]++;
			return true;
	}
	else
	{
			motor_var.fault_status_reg &=~(OC_fault); //Reset UC bit high
			fault_trip_counter.tripTimer[OC]=0;
			return false;
	}
	
}

/*
@helper function to check current phase failure
*/
static bool isPhaseFailueCurr(void)
{
	uint16_t _phase_failure_per= 80;//percent of nominal voltage;
	if((rms.display_currnet[RY_PHASE]<(float)FULL_LOAD_CURRENT(_phase_failure_per))|| \
		(rms.display_currnet[YB_PHASE]<(float)FULL_LOAD_CURRENT(_phase_failure_per))|| \
			(rms.display_currnet[BR_PHASE]<(float)FULL_LOAD_CURRENT(_phase_failure_per)))
	{
			motor_var.fault_status_reg |=CPH_F; //set UC bit high
			fault_trip_counter.tripTimer[CPH_F]++;
			return true;
	}
	else
	{
			motor_var.fault_status_reg &=(~CPH_F_fault); //Reset UC bit high
		fault_trip_counter.tripTimer[CPH_F]=0;
			return false;
	}
	
}

/*
@helper function to check Phase current Unbalance
*/

static bool isPhaseUnbalanceCurr(void)
{
	uint16_t _phase_ub_curr_per=10;
	extact_data(&meter_setup.meter_setup_menu[SETUP_UB_C],&_phase_ub_curr_per,7,10);
	
	//check_min_max_limit
		check_min_max_limit(MAX_UB_C,MIN_UB_C,&_phase_ub_curr_per);

	float max_curr =find_max(&rms.display_currnet[RY_PHASE],&rms.display_currnet[YB_PHASE],&rms.display_currnet[BR_PHASE]);
	float min_curr =find_min(&rms.display_currnet[RY_PHASE],&rms.display_currnet[YB_PHASE],&rms.display_currnet[BR_PHASE]);
	
	int _per_error = (int)((100*(max_curr-min_curr))/max_curr);
	
	if(_phase_ub_curr_per>_per_error)
	{
		motor_var.fault_status_reg |=UB_C_fault;
		fault_trip_counter.tripTimer[UB_C]++;
		return true;
		
	}
	else
	{
		motor_var.fault_status_reg &=(~UB_C_fault);
		fault_trip_counter.tripTimer[UB_C]=0;
		return false;
	}
	
	
}

/*
@helper function to check Phase current reversal
*/
static bool isPhaseReversalCurr(void)
{
	uint8_t _noofsample120=(120*NO_OF_SAMPE_PER_CYCLE)/360;
	
	if((store.Store_phase_shift_count[1][0] >= (_noofsample120-3)  \
		  && store.Store_phase_shift_count[1][0] <= (_noofsample120+3)) && \
			store.Store_phase_shift_count[1][1] >= (_noofsample120-3)  \
		  && store.Store_phase_shift_count[1][1] <= (_noofsample120+3)	
		)
	return false;
	else
	{
		motor_var.fault_status_reg |=CPH_R_fault;
		return true;
	}
	
}

/*
@helper function to check for rotor lock
*/

static bool isRotorLockCurr(void)
{
		uint16_t _rotor_lock_Per=0;
	
		extact_data(&meter_setup.meter_setup_menu[SETUP_RL],&_rotor_lock_Per,7,10);
	
	//check_min_max_limit
		check_min_max_limit(MAX_RL,MIN_RL,&_rotor_lock_Per);

	if((rms.display_currnet[RY_PHASE]>(float)FULL_LOAD_CURRENT(_rotor_lock_Per))|| \
		(rms.display_currnet[YB_PHASE]>(float)FULL_LOAD_CURRENT(_rotor_lock_Per))|| \
			(rms.display_currnet[BR_PHASE]>(float)FULL_LOAD_CURRENT(_rotor_lock_Per)))
			{
				motor_var.fault_status_reg |=RL_fault; //set UC bit high
				fault_trip_counter.tripTimer[RL]++;
				return true;
			}
	else
			{
				motor_var.fault_status_reg &=~(RL_fault); //Reset UC bit high
				fault_trip_counter.tripTimer[RL]=0;
				return false;
			}
}

/*
@helper function to check prolong start
*/
static bool isProlongStartCurr(void)
{
			uint16_t _prolong_lock_Per=0;
	
	extact_data(&meter_setup.meter_setup_menu[SETUP_PS],&_prolong_lock_Per,7,10);
	
	//check_min_max_limit
		check_min_max_limit(MAX_PS,MIN_PS,&_prolong_lock_Per);
	
	if((rms.display_currnet[RY_PHASE]>(float)FULL_LOAD_CURRENT(_prolong_lock_Per))|| \
		(rms.display_currnet[YB_PHASE]>(float)FULL_LOAD_CURRENT(_prolong_lock_Per))|| \
			(rms.display_currnet[BR_PHASE]>(float)FULL_LOAD_CURRENT(_prolong_lock_Per)))
			{
				motor_var.fault_status_reg |=PS_fault; //set UC bit high
				fault_trip_counter.tripTimer[PS]++;
				return true;
			}
	else
			{
				motor_var.fault_status_reg &=~(PS_fault); //Reset UC bit high
				fault_trip_counter.tripTimer[PS]=0;
				return false;
			}
	
}


static bool isOverPower(void)
{
	
	return false;
}

static bool isUnderPower(void)
{
	
	return false;
}


static bool isGroundFault(void)
{
	
	return false;
}

static bool isEarthFault(void)
{
	
	return false;
}

static bool isContactorFault(void)
{
	
	return false;
}


/*
	@presentCapacit=previousCapacity+(timeElapsed/tripTime)*100
	@ calculate thermal capacity
*/
static float presentTHCapacity=0;
static float previousTHCapacity=0;
uint32_t invTripTime=1;
static void thermalCapacity(void)
{

	float _tempCapacity;
	if((motor_status & MOTOR_STOP)!=MOTOR_STOP && invTripTime>0)
	{
		_tempCapacity=((float)10000/(invTripTime));
		presentTHCapacity +=_tempCapacity;//previousTHCapacity+((float)100000/(invTripTime));
		previousTHCapacity=presentTHCapacity;
		
	}
	
}

/*
@ helper function to calculate over current Trip delay
*/

static bool isInvCurrOverLoad(void)
{
	 float _Ir;//(float)Average_All_phase_Current/(menu_config[FULL_LOAD_CURRENT] * 10);
	if(_Ir > 1.1)
	{
	  invTripTime =(uint32_t)(1000*((float)(29.25 * 15/*Motor_Class[menu_config[MOTOR_CLASS]]*/)\
			/(_Ir*_Ir)-1));
		 thermalCapacity();
	}
		return false;
}



/*
	@presentCapacit=previousCapacity-(timeElapsed/cooloingconst)*100

*/
void coolingMotor()
{
	uint8_t runningCoolingConst=15*2;
	uint8_t stopCoolingConst=15*4;
	float __temp ;
   if(presentTHCapacity>0)  
		 {
		//if(Average_All_phase_Current<FULL_LOAD_CURRENT_PERCENTAGE(100))
		{
			__temp=(float)10/runningCoolingConst;
				presentTHCapacity =previousTHCapacity-__temp;
			if(presentTHCapacity<0)
					presentTHCapacity=0;
			    previousTHCapacity=presentTHCapacity;
		}
		//else if(stop)
		{
			__temp=(float)10/stopCoolingConst;
				presentTHCapacity =previousTHCapacity-__temp;
				if(presentTHCapacity<0)
					presentTHCapacity=0;
				previousTHCapacity=presentTHCapacity;
		}
	}
}
/*
	check the presentage
*/


/*
@ helper function to update Trip delay time accoudingly the fault conditions
*/
void delaySelect()
{	
	// extrct delay time from setup menu
		extact_data(&meter_setup.meter_setup_menu[SETUP_UV],&delaySetupTim.delaySetup[TIM_UV],24,7);
		extact_data(&meter_setup.meter_setup_menu[SETUP_OV],&delaySetupTim.delaySetup[TIM_OV],24,7);

		extact_data(&meter_setup.meter_setup_menu[SETUP_UB_V],&delaySetupTim.delaySetup[TIM_UB_V],24,7);
		extact_data(&meter_setup.meter_setup_menu[SETUP_UC],&delaySetupTim.delaySetup[TIM_UC],24,7);

		extact_data(&meter_setup.meter_setup_menu[SETUP_OC],&delaySetupTim.delaySetup[TIM_OC],24,7);
		extact_data(&meter_setup.meter_setup_menu[SETUP_INVT_OL],&delaySetupTim.delaySetup[TIM_INVT_OL],24,7);

		extact_data(&meter_setup.meter_setup_menu[SETUP_UB_C],&delaySetupTim.delaySetup[TIM_UB_C],24,7);
		extact_data(&meter_setup.meter_setup_menu[SETUP_RL],&delaySetupTim.delaySetup[TIM_RL],24,7);

		extact_data(&meter_setup.meter_setup_menu[SETUP_PS],&delaySetupTim.delaySetup[TIM_PS],24,7);
		extact_data(&meter_setup.meter_setup_menu[SETUP_UP],&delaySetupTim.delaySetup[TIM_UP],24,7);
		
		extact_data(&meter_setup.meter_setup_menu[SETUP_OP],&delaySetupTim.delaySetup[TIM_OP],24,7);
		extact_data(&meter_setup.meter_setup_menu[SETUP_GR_F],&delaySetupTim.delaySetup[TIM_GR_F],24,7);
		
		extact_data(&meter_setup.meter_setup_menu[SETUP_O_TEMP],&delaySetupTim.delaySetup[TIM_O_TEMP],24,7);
		extact_data(&meter_setup.meter_setup_menu[SETUP_ER_F],&delaySetupTim.delaySetup[TIM_ER_F],24,7);
		extact_data(&meter_setup.meter_setup_menu[SETUP_CON_F],&delaySetupTim.delaySetup[TIM_CON_F],24,7);


}
 
 /*
 
	@ checking is motor start ,stop or running 
 */
void check_Motor_status()
{
	 
	 float _avg_curr=(rms.display_currnet[RY_PHASE]+rms.display_currnet[YB_PHASE]+ \
									rms.display_currnet[BR_PHASE]);
	 _avg_curr/=3;
	
		if(_avg_curr >_avg_curr>FULL_LOAD_CURRENT(10))
		{
		 if((motor_status & MOTOR_STOP)==MOTOR_STOP)
		 {
			if((motor_status & MOTOR_RUNNING)!=MOTOR_RUNNING)
			 if(_avg_curr>FULL_LOAD_CURRENT(110))
			 {
					motor_status |=MOTOR_START; //set motor_status bit to start conditions
					motor_status &=~MOTOR_STOP; //Reset stop  motor_status bit since it is start now conditions
			 }
		 }
		 else if((motor_status & MOTOR_START)==MOTOR_START)
		 {
				if(_avg_curr<FULL_LOAD_CURRENT(100))
			 {
					motor_status |=MOTOR_RUNNING; //set running  motor_status bit to running conditions
					motor_status &=~MOTOR_STOP; //Reset stop  motor_status bit since it is start now conditions
					motor_status &=~MOTOR_START; //Reset start  motor_status bit since it is running now conditions
			 }
		 }
		 else if((motor_status & MOTOR_RUNNING)!=MOTOR_RUNNING)
		 {
			 // reset start and running bit and set stop bit since it is stop
			 motor_status &=~MOTOR_RUNNING;
			 motor_status &=~MOTOR_START;
			 motor_status |=MOTOR_STOP;
			 
		 }
	 }
		else
		{
			// reset start and running bit and set stop bit since it is stop
			 motor_status &=~MOTOR_RUNNING;
			 motor_status &=~MOTOR_START;
			 motor_status |=MOTOR_STOP; 
			
		}
	 
}
 
/*
		functions used for check all fault status before run motors,if pass then only allow run motor

*/

bool is_ok_pre_Start_Motor_Fault_Check()
{
	/* check only volatge related parameter beacuse only voltage will present if motor not running */
	
if(motor_status & MOTOR_STOP)
{
	#ifdef UV_CHECK
		if(isUnderVolt())
		{
			motor_var.cause_of_trip=TIM_UV;
			//turn off motor start relay
			motor_trip_relay(OFF);
			return false;
		}
	#endif
	
		#ifdef  OV_CHECK
		if(isOverVolt())
		{
			motor_var.cause_of_trip=TIM_OV;
			
			
			//turn off motor start relay
			motor_trip_relay(OFF);
			return false;
		}
	#endif
	
		#ifdef  VPH_F_CHECK
		if(isPhaseFailueVolt())
		{
			motor_var.cause_of_trip=FAULT_VOLT_PHASE_FAILURE;
			
			//turn off motor start relay
			motor_trip_relay(OFF);
			
			return false;
		}
	#endif
	
		#ifdef  UB_V_CHECK
		if(isPhaseUnbalanceVolt())
		{
			motor_var.cause_of_trip=TIM_UB_V;
			
			//turn off motor start relay
			motor_trip_relay(OFF);
			
			return false;
		}
	#endif
	
		#ifdef  VPH_R_CHECK
		if(isPhaseReversalVolt())
		{
			motor_var.cause_of_trip=FAULT_VOLT_PHASE_REVERSAL;
			
			//turn off motor start relay
			motor_trip_relay(OFF);
			
			return false;
		}
	
	#endif
		
		//Turn on trip relay pass relay 
		motor_trip_relay(ON);
		
		//clear timmer counter
		timmer_t d_time;
		for(d_time=TIM_UV;d_time<TIM_TOTAL_PARA;d_time++)
		fault_trip_counter.tripTimer[d_time]=0;
		
		
		motor_var.cause_of_alarm=FAULT_NONE;
		motor_var.cause_of_trip=FAULT_NONE;
		
		return true;
		
}

	return true;
	
}
/*
		functions used for check all fault status before run motors,if pass then only allow run

*/

void running_Motor_Fault_Check()
{
	if(motor_status & MOTOR_RUNNING || motor_status & MOTOR_START)
	{
			voltage_related_fault();
			current_related_fault();
			power_related_fault();
			other_related_fault();
	
	}
}

 /*
	@ motor function
 */
 
void motorFunctions(void)
{	
		/* check for motor status running,stop,start */
	check_Motor_status();
	if(is_ok_pre_Start_Motor_Fault_Check() && motor_var.cause_of_trip !=FAULT_NONE)
	{
		// CHECK ALL FAULT	
			voltage_related_fault();
			current_related_fault();
			other_related_fault();
			delaySelect();
			alarmRelay();
			tripRelay();
			
	}
	// clear fault manually
	clear_fault();

}


/* 
@Trip relay to protect motor
@update the fault status
*/



static void tripRelay(void)
{
	/* check trip relay on or off 
			if off then go to check trip or else return
	*/
	if((motor_status & TRIP_RELAY)!=TRIP_RELAY)
		return;
	
	setup_t setup_data;
	timmer_t d_time;

	for(setup_data=SETUP_UV,d_time=TIM_UV;setup_data<=SETUP_CON_F;setup_data++)
	{
		/* skip trip relay for phase failure and phase reversal for volt,current */
		if(setup_data==SETUP_V_PF || setup_data==SETUP_V_PR||setup_data==SETUP_C_PF \
			||setup_data==SETUP_C_PR)
			continue;
		
			if((meter_setup.meter_setup_menu[setup_data] & PARA_ENABLE_BIT)&& \
				(PARA_TRIP_ENABLE_BIT &meter_setup.meter_setup_menu[setup_data]))
			{
				
					if(fault_trip_counter.tripTimer[d_time]>delaySetupTim.delaySetup[d_time]*10)
					{
						//capture value at trip
								capture_fault_data(d_time);
							// trip relay 
							motor_trip_relay(OFF);
							// reset motor status 
							clear_flag();
							
							//capture cause of fault
						motor_var.cause_of_trip=d_time;
							
					}
			}
			d_time++;
			
		}
			
	
}

/*
Discriptions:check for alarm relay
	
*/
 
static void alarmRelay(void)
{
			uint16_t _alarm_per=0;
			uint16_t alarm_temp=0;
	/* check alarm relay on /off 
		if already on no need to continue
	*/
	if(motor_status & ALARM_RELAY)
		return;
	
	setup_t setup_data;
	timmer_t d_time;

	for(setup_data=SETUP_UV,d_time=TIM_UV;setup_data<=SETUP_CON_F;setup_data++)
	{
		/* skip alarm for phase failure and phase reversal for volt,current */
		if(setup_data==SETUP_V_PF || setup_data==SETUP_V_PR||setup_data==SETUP_C_PF \
			||setup_data==SETUP_C_PR)
			continue;
		
		// check from parameter and alarm  enable 
		if((PARA_ENABLE_BIT & meter_setup.meter_setup_menu[setup_data]) &&  \
			(PARA_ALARM_ENABLE_BIT &meter_setup.meter_setup_menu[setup_data]))
			{
				
					extact_data(&meter_setup.meter_setup_menu[d_time],&_alarm_per,PARA_ALARM_ENABLE_POS,2);
				/* check if alarm in between MIN and MAX else make them alarm min */	
				if(_alarm_per <ALARM_MIN && _alarm_per>ALARM_MAX)
							_alarm_per=ALARM_MIN;
					
						alarm_temp=(delaySetupTim.delaySetup[d_time]*_alarm_per)/100;
						
						//TODO
						/*
							check last bit for time in milli or in second
						*/
						alarm_temp*=10; //convert in second,warning multilple based on loop running , here is 100ms
						if(fault_trip_counter.tripTimer[d_time]>alarm_temp)
						{
							// turn on relay alarm
							motor_alarm_relay(ON);
							
							// capture cause of alarm
							motor_var.cause_of_alarm=(uint16_t)setup_data;
							break;
						}
						
					
			}
			
			d_time++;
	}
			
				
}

/*
		voltage releate fault check
*/
static void voltage_related_fault(void)
{
	
	if(motor_status & MOTOR_RUNNING)
	{
	#ifdef UV_CHECK   //under volt
		if(isUnderVolt())
		{
			
		}
	#endif
	
		#ifdef  OV_CHECK	//over volt
		if(isOverVolt())
		{
		}
	#endif
	
		#ifdef  VPH_F_CHECK		//volt phase failure 
		if(isPhaseFailueVolt())
		{
			
		}
	#endif
	
		#ifdef  UB_V_CHECK		//volt unbalanced
		if(isPhaseUnbalanceVolt())
		{
			
		}
	#endif
	
		#ifdef  VPH_R_CHECK		//voltage phase reversal
		if(isPhaseReversalVolt())
		{
			
		}
	#endif
	}
}

static void current_related_fault(void)
{
	if(motor_status & MOTOR_RUNNING)
	{
	#ifdef	UC_fCHECK  //under current
		isUnderCurr();
	#endif
	#ifdef	OC_CHECK //over current
		isOverCurr();
	#endif	
	#ifdef	INVT_OL_CHECK //inverse time overload
		isInvCurrOverLoad();
	#endif	
	#ifdef	UB_C_CHECK 	//Current unbalanced
		isPhaseUnbalanceCurr();
	#endif
	#ifdef	RL_CHECK  //rotor jam
		isRotorLockCurr();
	#endif	
		
	#ifdef	CPH_F_CHECK //Current phase failure
		isPhaseFailueCurr();
	#endif
	#ifdef	CPH_R_CHECK  //current phase reversal
	isPhaseReversalCurr();
	#endif

	}
		
	if(motor_status & MOTOR_START)
		{
			#ifdef	PS_CHECK //PROLONG START
			isProlongStartCurr();
			#endif
		}
}

static void power_related_fault(void)
{
#ifdef	UP_CHECK 	//under power
	isUnderPower();
#endif
#ifdef	OP_CHECK 	//over power
	isOverPower();
#endif
}
static void other_related_fault(void)
{
#ifdef	GR_F_CHECK 	//ground fault
	isGroundFault();
	#endif
#ifdef	O_TEMP_CHECK  //over temp
	
	#endif
#ifdef	ER_F_CHECK 	//earth fault
	isEarthFault();
	#endif
#ifdef	CON_F_CHECK 	//contatctor failure
	isContactorFault();
	#endif
}



static void clear_flag(void)
{
	motor_status &=~MOTOR_START;
	motor_status &=~MOTOR_RUNNING;
	motor_status |=MOTOR_STOP;
	
}


/*

Descriptions: read fault data based on trip 
			
*/

uint32_t capture_fault_data(timmer_t id)
{
	uint32_t retVal=0;
	 switch((int)id)
	 {
		 case TIM_UV:   // trip due to under volt
			 
		 break;
		 
		 case TIM_OV: 	//trip due to over volt
			
		 break;
		 case TIM_UB_V:	//volt unbalanced
			 
		 break;
			
			case TIM_UC:	//under current
				
			break;
			case TIM_OC:	//over current
				
			break;
			case TIM_INVT_OL:	//inverse time overload
				
			break;
			case TIM_UB_C:	//Current unbalanced
				
			break;
			case TIM_RL:		//rotor jam
				
			break;
			case TIM_PS:	//PROLONG START
				
			break;
			
			case TIM_UP:	//under power
				
			break;
			case TIM_OP:	//over power
				
			break;
			
			case TIM_GR_F:	//ground fault
				
			break;
			case TIM_O_TEMP: //over temp
				
			break;
			case TIM_ER_F:	//earth fault
				
			break;
			case TIM_CON_F:	//contatctor failure
				
			break;
			
			default:
				retVal=0;
			break;
				 
	 }
	 return retVal;
}

void motor_default_status(void)
{
		motor_var.cause_of_alarm=FAULT_NONE;
		motor_var.cause_of_trip=FAULT_NONE;
		motor_alarm_relay(OFF);
		motor_trip_relay(OFF);
}


static void clear_fault()
{
	
		// manually reset fault
	if(motor_var.cause_of_trip !=FAULT_NONE)
	{
		if(ui_isButtonNormalPressed(BUTTON_ID_B0))
		{
			motor_var.cause_of_trip=FAULT_NONE;
			motor_var.cause_of_alarm=FAULT_NONE;
			motor_alarm_relay(OFF);
			motor_trip_relay(ON);
			
		}
		
	}
}
/***END of FUNCTIONS belong to other file***/
 
 
