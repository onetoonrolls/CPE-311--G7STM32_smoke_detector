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

#define PIN_MQ_2 4 //get value from MQ-2 sensor **DAC** PA4 

#define GAS_LPG 0
#define GAS_CO 1
#define GAS_SMOKE 2

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

long ippm_gas[3]={0}; //get ippm index 0 =LPG, 1 =CO, 2 =smoke

void GPIO_config(void);
float MQResistCal(int);
float MQCalibration(void);
long MQGetGasPercentage(float rs_ro_ratio, int gas_id);
long  MQGetPercentage(float rs_ro_ratio, float *pcurve);
float MQRead(void);

void Show_PPM_7_seg(void); //
int main()
{
	GPIO_config();
	Ro = MQCalibration();
	
	while(1)
	{
		ippm_gas[0] = MQGetGasPercentage(MQRead()/Ro,GAS_LPG);;
		ippm_gas[1] = MQGetGasPercentage(MQRead()/Ro,GAS_CO);
		ippm_gas[2] = MQGetGasPercentage(MQRead()/Ro,GAS_SMOKE);
	}
}

void GPIO_config()
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	
	LL_GPIO_InitTypeDef(MQ_2);
	MQ_2.Mode = LL_GPIO_MODE_OUTPUT;
	MQ_2.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	MQ_2.Pull = LL_GPIO_PULL_NO;
	MQ_2.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	
	MQ_2.Mode = LL_GPIO_MODE_ANALOG;

	LL_GPIO_Init(GPIOA,&MQ_2);
}

void Show_PPM_7_seg()
{
		if(ippm_gas[2] >= 300)
		{
			//show PPM LGP & CO
		}
		else
		{
			//show PPM smoke
		}
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
    //val += MQResistCal(analogRead(mq_pin));
    LL_mDelay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
  return val;                                           //according to the chart in the datasheet 

}

float MQRead() //under dev
{
	 int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
   // rs += MQResistCal(LL_GPIO_SetPinMode(GPIOB,PIN_MQ_2));
		LL_mDelay(READ_SAMPLE_INTERVAL);
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