#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "my_printf.h"

__IO uint32_t g_timing_delay;
__IO uint32_t g_ticks = 0; // increments every millisecond

void timing_delay_decrement(void);
void delay_ms(uint32_t t);
void init_UART4();
void init_I2C1();
void init_LED();
void init_blue_push_button();
uint32_t get_ticks();

#define SA_MAG  0x1E 
#define SA_GYRO 0x35

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

    // Enable Usage Fault, Bus Fault, and MMU Fault, else it will default to HardFault handler
    //SCB->SHCSR |= 0x00070000; 

    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000); // tick every 1 ms, used by delay_ms()

    init_LED();
    init_blue_push_button();
    init_UART4();
    init_I2C1();

    I2C_SendData(I2C1, (SA_GYRO << 1) | 1); /* read */
    I2C_SendData(I2C1, 0xF);
    uint8_t val = I2C_ReceiveData(I2C1);

    my_printf("Begin ...\r\n");
    my_printf("value: %u\r\n", val);

    while(1) {
        // Light up the LEDS when the user presses the blue button
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
    		GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
    	} 
        else {
    		GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
    	}
    }
}

void init_LED()
{
    GPIO_InitTypeDef gpio; // LEDS on GPIOD

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15;
	gpio.GPIO_Mode  = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOD, &gpio);
}

void init_blue_push_button()
{
    GPIO_InitTypeDef gpio; // push button on GPIOA

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	gpio.GPIO_Pin   = GPIO_Pin_0;
	gpio.GPIO_Mode  = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOA, &gpio);
}

void init_UART4()
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);

    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
 
    /* Configure USART Tx as alternate function  */
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    /* USART configuration */
    USART_Init(UART4, &usart);

    /* Enable USART */
    USART_Cmd(UART4, ENABLE);
}

void init_I2C1()
{
    GPIO_InitTypeDef gpio; 
    I2C_InitTypeDef i2c;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_I2C1);

    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Mode  = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_PuPd  = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &gpio);

    I2C_StructInit(&i2c);
    i2c.I2C_ClockSpeed = 50000;
    i2c.I2C_Ack = I2C_Ack_Enable;

    //    I2C_DeInit(I2C1);
    //    I2C_Cmd(I2C1, DISABLE);
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
}

void delay_ms(uint32_t t)
{
    g_timing_delay = t;

    while (g_timing_delay != 0);
}

void timing_delay_decrement(void)
{
    if (g_timing_delay != 0x00) { 
        g_timing_delay--;
    }
}

uint32_t get_ticks()
{
    return g_ticks;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: my_printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

