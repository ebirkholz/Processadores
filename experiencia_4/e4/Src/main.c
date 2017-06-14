/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#define SDA (1<<9) //PB.9 - Pode alterar
#define SCL (1<<8) //PB.8 - Pode alterar
#define SDA0 GPIOB->BRR=SDA
#define SDA1 GPIOB->BSRR=SDA
#define SCL0 GPIOB->BRR=SCL
#define SCL1 GPIOB->BSRR=SCL

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int tec;
int LDR;
int x=0;
int dez_temp=0, uni_temp=0;
int temp_ajuste=0;
int temp_atual=0;
char estado[]="";
char temp_at[]="";

char temp_t[]="";

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void dados(int comando);
void lcd_comando(int comando);
void lcd_dado(int dado);
void lcd_init(void);
void lcd_string(char vetor[]);
void lcd_clear(void);
void lcd_goto(int linha, int coluna);
void rele(int num);
void modo_prog(void);
void modo_temp(void);

//I2C daqui pra BAIXO
void configura_sensor();
void start_i2c();
void stop_i2c();
void envia_1_i2c();
void envia_0_i2c();
void ack_i2c();
//void envia_byte_i2c();
float le_temp_i2c();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void configura_sensor()
{
    unsigned char conf = 0x00;
    HAL_I2C_MEM_Write(&hi2c,0x4f,1,1,&conf,1,10);
//    char x=0;
//    start_i2c();
//    envia_1_i2c();
//    envia_0_i2c();
//    envia_0_i2c();
//    envia_1_i2c();
//    envia_0_i2c();
//    envia_0_i2c();
//    envia_0_i2c();
//    envia_0_i2c(); //0 pois ´e escrita
//    x = ack_i2c();
//    envia_byte_i2c(0x00);
//    x = ack_i2c();
//    envia_byte_i2c(0x00);
//    x = ack_i2c();
//    stop_i2c();
}

void start_i2c(void) //I2C START
{
    SDA1;
    HAL_Delay(1);//1 milisegundo
    SCL1;
    HAL_Delay(1);//1 milisegundo
    SDA0;
    HAL_Delay(1);//1 milisegundo
    SCL0;
    HAL_Delay(1);//1 milisegundo
}

void stop_i2c(void) //I2C STOP
{
    SDA0;
    HAL_Delay(1);//1 milisegundo
    SCL0;
    HAL_Delay(1);//1 milisegundo
    SCL1;
    HAL_Delay(1);//1 milisegundo
    SDA1;
    HAL_Delay(1);//1 milisegundo
}

void envia_1_i2c(void) //Envia 1 pelo I2C
{
    SDA1;
    HAL_Delay(1);//1 milisegundo
    SCL1;
    HAL_Delay(1);//1 milisegundo
    SCL0;
    HAL_Delay(1);//1 milisegundo
}

void envia_0_i2c(void) //Envia 0 pelo I2C
{
    SDA0;
    HAL_Delay(1);//1 milisegundo
    SCL1;
    HAL_Delay(1);//1 milisegundo
    SCL0;
    HAL_Delay(1);//1 milisegundo
}

int ack_i2c(void)
{
    int x;
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_7; // SDA => PB.7
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; //FAZ SDA COMO ENTRADA
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    SCL1;
    HAL_Delay(1);//1 milisegundo
    x = HAL_GPIO_ReadPin(GPIOB,SDA); //L^E O PINO
    SCL0;
    HAL_Delay(1);//1 milisegundo
    GPIO_InitStruct.Pin = GPIO_PIN_7; // SDA => PB.7
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //FAZ SDA COMO SA´IDA
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    return x; //se 0 ok, se 1 erro
}

float le_temp_i2c()
{
    float temperatura;
    char temp[2];
    
    HAL_I2C_MeM_Read(&hi2c,0x4f,1,1,&temp,2,20);
    temperatura=temp[1];
    if(temp[0]&(1<<7)!=0) temperatura+=0.5;
    
    return temperatura;
}


void dados(int comando)
{
	if((comando & 0x80)==0)
		GPIOA->BRR=D7_Pin; //D7=0
	else
		GPIOA->BSRR=D7_Pin; //D7=1
	
	if((comando & 0x40)==0)
		GPIOB->BRR=D6_Pin; //D6=0
	else
		GPIOB->BSRR=D6_Pin; //D6=0
	
	if((comando & 0x20)==0)
		GPIOB->BRR=D5_Pin; //D5=0
	else
		GPIOB->BSRR=D5_Pin; //D5=1
	
	if((comando & 0x10)==0)
		GPIOB->BRR=D4_Pin; //D4=0
	else
		GPIOB->BSRR=D4_Pin; //D4=1
	
	GPIOC->BSRR=EN_Pin; //EN=1
	HAL_Delay(1);
	GPIOC->BRR=EN_Pin; //EN=0
	HAL_Delay(1);
	
	//////
	
	if((comando & 0x8)==0)
		GPIOA->BRR=D7_Pin; //D7=0
	else
		GPIOA->BSRR=D7_Pin; //D7=1
	
	if((comando & 0x4)==0)
		GPIOB->BRR=D6_Pin; //D6=0
	else
		GPIOB->BSRR=D6_Pin; //D6=0
	
	if((comando & 0x2)==0)
		GPIOB->BRR=D5_Pin; //D5=0
	else
		GPIOB->BSRR=D5_Pin; //D5=1
	
	if((comando & 0x1)==0)
		GPIOB->BRR=D4_Pin; //D4=0
	else
		GPIOB->BSRR=D4_Pin; //D4=1
	
	GPIOC->BSRR=EN_Pin; //EN=1
	HAL_Delay(1);
	GPIOC->BRR=EN_Pin; //EN=0
	HAL_Delay(1);
}

void lcd_comando(int comando)
{
	GPIOA->BRR=RS_Pin;
	dados(comando);	
}

void lcd_dado(int dado)
{
	GPIOA->BSRR=RS_Pin;
	dados(dado);	
}

void lcd_init(void)
{
	lcd_comando(0x33);
	lcd_comando(0x32);
	lcd_comando(0x28);
	lcd_comando(0x0e);
	lcd_comando(0x06);
	lcd_comando(0x01);
	HAL_Delay(1);
}

void lcd_string(char vetor[])
{
	for(int k=0;vetor[k]!=0;k++)
		lcd_dado(vetor[k]);
}

void lcd_clear(void)
{
	lcd_comando(0x01);
}

void lcd_goto(int linha, int coluna)
{
	int v;
	if(linha==1)
		v = 0x80 + (coluna-1)*0x01;
	else if(linha==2)
		v = 0xc0 + (coluna-1)*0x01;
	lcd_comando(v);
}

void rele(int num)//1- LAMPADA, 2- COOLER
{
	if(num==1)
	{
		GPIOC->BSRR=1<<12;
		GPIOC->BRR=1<<10;		
	}
	if(num==2)
	{
		GPIOC->BSRR=1<<10;
		GPIOC->BRR=1<<12;		
	}
	if(num==0)
	{
		GPIOC->BRR=1<<10;
		GPIOC->BRR=1<<12;		
	}
}
int le_teclas(void)
{
	int tecla=0;
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc,ADC_CHANNEL_10);
	LDR=HAL_ADC_GetValue(&hadc);
	if((LDR>650)&&(LDR<800))tecla=1; //CONFIGURA (LEFT)
	else if((LDR>100)&&(LDR<300))tecla=2; //DEZ_MAIS (UP)
	else if((LDR>400)&&(LDR<600))tecla=3; //UNI_MAIS (DOWN)
	else if(LDR<50)tecla=4; //CONFIRMA (RIGTH)
	else tecla=0;
	return tecla;
}

void teclas_livres(void)
{
	while(le_teclas()!=0);	
}
int aguarda_tecla(void)
{
	int tecla=0;
	while(tecla==0) tecla=le_teclas();
	teclas_livres();
	return tecla;	
}
void modo_prog(void)
{
	lcd_clear();
	lcd_goto(1,1);
	lcd_string("MODO PROGRAMACAO");
	lcd_goto(2,1);
	lcd_string("TEMPERATURA:");
	HAL_Delay(10);
	
	while(le_teclas()!=4)
	{
		
		lcd_goto(2,14);
		sprintf(temp_t,"%d%d",dez_temp,uni_temp);
		lcd_string(temp_t);
		HAL_Delay(10);
		tec= aguarda_tecla();
		if(tec==4) modo_temp(); //SE CONFIRMAR VAI PARA MODO ESTUFA
		if(tec==2) //INCREMENTA DEZENA TEMPERATURA
		{
			if(dez_temp<9) dez_temp++;
			else dez_temp=0;
		}
		if(tec==3) //INCREMENTA UNIDADE_TEMPERATURA
		{
			if(uni_temp<9) uni_temp++;
			else uni_temp=0;
		}		
	}	
}
void modo_temp(void)
{
	lcd_clear();
	lcd_goto(1,1);
	lcd_string("ESTUFA:");
	lcd_goto(2,1);
	lcd_string("TEMP:");
	HAL_Delay(10);
	
	while(le_teclas()!=4)
	{
		//LE TEMPERATURA
		//TESTA TEMPERATURA
		//LIGA COOLER OU LAMPADA
		temp_ajuste=(dez_temp*10) + uni_temp;
		temp_atual=13;
		if(temp_atual==temp_ajuste)
		{
			rele(0);
			sprintf(estado,"%s","DESLIG");
		}			
		if(temp_atual>temp_ajuste)
		{
			rele(2); //LIGA COOLER
			sprintf(estado,"%s","RESFR");
		}
		if(temp_atual<temp_ajuste)
		{
			rele(1); //LIGA LAMPADA
			sprintf(estado,"%s","AQUEC");
		}
		lcd_goto(2,7);
		sprintf(temp_at,"%d",temp_atual);
		lcd_string(temp_at);
		lcd_goto(2,10);
		lcd_string(estado);
		
		tec=aguarda_tecla();
		if(tec==4) modo_prog(); //SE CONFIRMAR VAI PARA MODO PROGRAMACAO
		teclas_livres();
	}	
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_ADC_Init();

  /* USER CODE BEGIN 2 */
		lcd_init();
		GPIOB->BSRR=BL_Pin;
	
	//BOTOES
		ADC_ChannelConfTypeDef sConfig;
		HAL_ADC_Start(&hadc);
		HAL_ADCEx_Calibration_Start(&hadc);
		sConfig.Channel=ADC_CHANNEL_0;
		HAL_ADC_ConfigChannel(&hadc,&sConfig);
		
		lcd_goto(1,6);
		lcd_string("ESTUFA");
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		tec=aguarda_tecla();
		
		if(tec==1) modo_prog();
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
