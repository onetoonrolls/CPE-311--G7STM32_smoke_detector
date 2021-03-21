_#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_tim.h"
#include "math.h"

#define PIN_MQ_2 4 //get value from MQ-2 sensor **ADC** PA4
#define Max_Bit_ADC 1024


#define Max_Volt 4.45

//For playing sound
#define E_O6					(uint16_t)1318
#define MUTE					(uint16_t) 1
/*for 10ms update event*/
#define TIMx_PSC			2
/*Macro function for ARR calculation*/
#define ARR_CALCULATE(N) ((32000000) / ((TIMx_PSC) * (N)))

float test_adc=0;
float VoltADC =0;
float PerVolt =0;

//for 7 seg
uint8_t digit_value[4] = {0};
uint32_t num_to_seg[10] = {LL_GPIO_PIN_3 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14,			//0
			LL_GPIO_PIN_10 | LL_GPIO_PIN_11,											//1
			LL_GPIO_PIN_3 | LL_GPIO_PIN_10 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_15,					//2
			LL_GPIO_PIN_3 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_15,					//3
			LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,													//4
			LL_GPIO_PIN_3 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,					//5
			LL_GPIO_PIN_3 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, 			//6
			LL_GPIO_PIN_3 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11,																						//7
			LL_GPIO_PIN_3 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15, 	//8
			LL_GPIO_PIN_3 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15			//9
			};

uint32_t digit[4] = {LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3};

void GPIO_config(void);

int Get_ADC();
float calVolt(int);
float calPerVolt(float);


void SystemClock_Config(void);

int main()
{
 	SystemClock_Config();
	GPIO_config();

	while(1)
	{
		test_adc = Get_ADC();
		VoltADC = calVolt(test_adc);
		PerVolt = calPerVolt(VoltADC);

		LL_mDelay(5);
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
	MQ_2.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
	LL_GPIO_Init(GPIOB,&MQ_2);

	MQ_2.Pin =LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
	LL_GPIO_Init(GPIOC,&MQ_2);

	//ADC config
	MQ_2.Mode = LL_GPIO_MODE_ALTERNATE;
	MQ_2.Pin = LL_GPIO_PIN_4;
	LL_GPIO_Init(GPIOA,&MQ_2);

	RCC->APB2ENR|= (1<<9);
	RCC->CR |= (1<<0);
	while(((RCC->CR & 0x02)>>1) ==0);
	ADC1->CR1 &= ~(3<<24);
	ADC1->CR1 |= (1<<11);
	ADC1->CR1 &= ~(7<<13);
	ADC1->CR2 &= ~(1<<11);
	ADC1->SMPR3 |= (2<<12);
	ADC1->SQR5 |=(4<<0);
	ADC1->CR2 |=(1<<0);

	//config sound
	LL_GPIO_InitTypeDef gpio_initstructure;
	gpio_initstructure.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_initstructure.Alternate = LL_GPIO_AF_2;
	gpio_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_initstructure.Pin = LL_GPIO_PIN_6;
	gpio_initstructure.Pull = LL_GPIO_PULL_NO;
	gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &gpio_initstructure);
}

int Get_ADC()
{
	ADC1->CR2|=(1<<30);
	while((ADC1->SR & (1<<1))==0);
	return(test_adc = ADC1->DR);
}


float calVolt(int rawADC)
{
	return(rawADC*Max_Volt/Max_Bit_ADC);
}

float calPerVolt(float Voltage)
{
	return(Voltage*100/Max_Volt);
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
