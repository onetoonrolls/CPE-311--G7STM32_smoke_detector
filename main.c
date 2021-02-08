#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_tim.h"
#include "math.h"

#define PIN_LedRed 1 //output gas type CO
#define PIN_LedYellow 2 //output gas type LPG
#define PIN_LedGreen 3 //output gas type smoke

#define PIN_MQ_2 4 //get value from MQ-2 sensor **ADC** PA4 

#define GAS_LPG 0
#define GAS_CO 1
#define GAS_SMOKE 2

//For playing sound
#define E_O6					(uint16_t)1318
#define MUTE					(uint16_t) 1
/*for 10ms update event*/
#define TIMx_PSC			2 
/*Macro function for ARR calculation*/
#define ARR_CALCULATE(N) ((32000000) / ((TIMx_PSC) * (N)))

//factory declare //
int RL_VALUE=5;   //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR=9.83;  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO
int CALIBARAION_SAMPLE_TIMES=50;  //define how many samples you are going to take in the calibration phase
int CALIBRATION_SAMPLE_INTERVAL=500; //define the time interal(in milisecond) between each samples in the
int READ_SAMPLE_INTERVAL=50; //define how many samples you are going to take in normal operation
int READ_SAMPLE_TIMES=5;  //define the time interal(in milisecond) between each samples in 

float LPGCurve[3] = {2.3,0.21,-0.47};
float COCurve[3] = {2.3,0.72,-0.34};
float SmokeCurve[3] = {2.3,0.53,-0.44};
float Ro = 10.0; //R value from MQ-2 sensor
//*************************************************************************//
float test_adc=0;
float rs=0;
long ippm_gas[3]={0}; //get ippm index 0 =LPG, 1 =CO, 2 =smoke

//for 7 seg
uint8_t digit_value[4] = {0};
uint32_t num_to_seg[10] = {LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14,			//0
			LL_GPIO_PIN_10 | LL_GPIO_PIN_11,											//1
			LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_15,					//2
			LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_15,					//3
			LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,													//4
			LL_GPIO_PIN_2 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,					//5
			LL_GPIO_PIN_2 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, 			//6
			LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11,																						//7
			LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, 	//8
			LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15			//9	
			};

uint32_t digit[4] = {LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3};

void GPIO_config(void);
float MQResistCal(int);
float MQCalibration(void);
long MQGetGasPercentage(float rs_ro_ratio, int gas_id);
long  MQGetPercentage(float rs_ro_ratio, float *pcurve);
float MQRead(void);
int Get_ADC();

//play note function
void TIM_BASE_Config(uint16_t);
void TIM_OC_GPIO_Config(void);
void TIM_OC_Config(uint16_t);
void TIM_BASE_DurationConfig(void);
void PlayAlarm(int);

//display 7-seg
void display_7_seg(int);

void SystemClock_Config(void);
int main()
{
	SystemClock_Config();
	GPIO_config();
	TIM_OC_Config(ARR_CALCULATE(E_O6));
	TIM_BASE_DurationConfig();
	
	Ro = MQCalibration();
	
	while(1)
	{
		ippm_gas[0] = MQGetGasPercentage(MQRead()/Ro,GAS_LPG);;
		ippm_gas[1] = MQGetGasPercentage(MQRead()/Ro,GAS_CO);
		ippm_gas[2] = MQGetGasPercentage(MQRead()/Ro,GAS_SMOKE);
		
		for(int i=0;i<3;i++)
		{
			display_7_seg(ippm_gas[i]);
			PlayAlarm(ippm_gas[i]);
			LL_mDelay(10);
		}
		
	}
}

void GPIO_config()
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	
	//7_seg config
	LL_GPIO_InitTypeDef(MQ_2);
	MQ_2.Mode = LL_GPIO_MODE_OUTPUT; 
	MQ_2.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	MQ_2.Pull = LL_GPIO_PULL_NO;
	MQ_2.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	MQ_2.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
	LL_GPIO_Init(GPIOB,&MQ_2);
	
	MQ_2.Pin =LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
	LL_GPIO_Init(GPIOC,&MQ_2);
	
	MQ_2.Pin =LL_GPIO_PIN_5; //for "."
	LL_GPIO_Init(GPIOA,&MQ_2);
	
	//ADC config
	MQ_2.Mode = LL_GPIO_MODE_ALTERNATE;
	MQ_2.Pin = LL_GPIO_PIN_4;
	LL_GPIO_Init(GPIOA,&MQ_2);
	
	RCC->APB2ENR|= (1<<9); 
	RCC->CR |= (1<<0); 
	while(((RCC->CR & 0x02)>>1) ==1);
	ADC1->CR1 &= ~(3<<24);
	ADC1->CR1 |= (1<<11); 
	ADC1->CR1 &= ~(7<<13); 
	ADC1->CR2 &= ~(1<<11);
	ADC1->SMPR3 |= (2<<12); 
	ADC1->SQR5 |=(4<<0); 
	ADC1->CR2 |=(1<<0); 
	
	//config tim
	MQ_2.Mode = LL_GPIO_MODE_ALTERNATE; //Pin B6
	MQ_2.Alternate = LL_GPIO_AF_2;
	MQ_2.Pin = LL_GPIO_PIN_6;
	LL_GPIO_Init(GPIOB, &MQ_2);
}

void display_7_seg(int gas)
{
	digit_value[0] = gas/1000;
	digit_value[1] = (gas/100) % 10;
	digit_value[2] = (gas%100) / 10;
	digit_value[3] = gas % 10;
			
	for(int i = 0; i < 4; ++i)
	{
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);
		LL_GPIO_SetOutputPin(GPIOB, num_to_seg[digit_value[i]]);
		LL_GPIO_SetOutputPin(GPIOC, digit[i]);
		LL_mDelay(1);
	}
}

int Get_ADC()
{
	ADC1->CR2|=(1<<30);
	while((ADC1->SR & (1<<1))==0);
	return(test_adc = ADC1->DR);
}


float MQResistCal(int raw_data)
{
	return ( ((float)RL_VALUE*(1023-raw_data)/raw_data));
}

float MQCalibration() //under dev
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistCal(Get_ADC());
   // LL_mDelay(500);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
  return val;                                           //according to the chart in the datasheet 

}

float MQRead() //under dev
{
	 int i;
  
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistCal(Get_ADC());
	//	LL_mDelay(50);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}

long MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
	if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
}

long  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
	
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

void TIM_BASE_DurationConfig(void)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	//Time-base configure
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = 3000 - 1;
	timbase_initstructure.Prescaler =  32000 - 1;
	LL_TIM_Init(TIM2, &timbase_initstructure);
	
	LL_TIM_EnableCounter(TIM2); 
	LL_TIM_ClearFlag_UPDATE(TIM2); //Force clear update flag
}

void TIM_BASE_Config(uint16_t ARR)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	//Time-base configure
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = ARR - 1;
	timbase_initstructure.Prescaler =  TIMx_PSC- 1;
	LL_TIM_Init(TIM4, &timbase_initstructure);
	
	LL_TIM_EnableCounter(TIM4); 
}

void TIM_OC_Config(uint16_t note)
{
	LL_TIM_OC_InitTypeDef tim_oc_initstructure;
	
	TIM_BASE_Config(note);
	
	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM4) / 2;
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &tim_oc_initstructure);
	/*Interrupt Configure*/
	NVIC_SetPriority(TIM4_IRQn, 1);
	NVIC_EnableIRQ(TIM4_IRQn);
	LL_TIM_EnableIT_CC1(TIM4);
	
	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM4);
}

void TIM4_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM4) == SET)
	{
		LL_TIM_ClearFlag_CC1(TIM4);
	}
}

void PlayAlarm(int gas){
	if(gas > 300)
	LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_SetAutoReload(TIM4, MUTE); //Change ARR of Timer PWM
	LL_TIM_SetCounter(TIM2, 0);
}

void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}