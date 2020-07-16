
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "usb_host.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "uart.h"
#include "mp3_player.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
ApplicationTypeDef applicationState = APPLICATION_IDLE;
FATFS fs;
LCD_TypeDef *lcd;
char clearVar[] = "                                ";
char prevName[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void loadFileListing();
void mountVolume();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_USB_HOST_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  uart_init();
  ApplicationTypeDef localState = APPLICATION_IDLE;

  //LCD1602_Begin4BIT(RS_GPIO_Port, RS_Pin, E_Pin, D4_GPIO_Port, D4_Pin, D5_Pin, D6_Pin, D7_Pin);
  PortPin_Map LCD_RS = {.GPIOx = RS_GPIO_Port, .GPIO_Pin_x = RS_Pin};
  PortPin_Map LCD_RW = {.GPIOx = RW_GPIO_Port, .GPIO_Pin_x = RW_Pin};
  PortPin_Map LCD_E = {.GPIOx = E_GPIO_Port, .GPIO_Pin_x = E_Pin};
  PortPin_Map LCD_D4 = {.GPIOx = D4_GPIO_Port, .GPIO_Pin_x = D4_Pin};
  PortPin_Map LCD_D5 = {.GPIOx = D5_GPIO_Port, .GPIO_Pin_x = D5_Pin};
  PortPin_Map LCD_D6 = {.GPIOx = D6_GPIO_Port, .GPIO_Pin_x = D6_Pin};
  PortPin_Map LCD_D7 = {.GPIOx = D7_GPIO_Port, .GPIO_Pin_x = D7_Pin};

  lcd = LCD_init(&LCD_RS, &LCD_RW, &LCD_E, &LCD_D4, &LCD_D5, &LCD_D6, &LCD_D7);
  if(!lcd)
      return HAL_ERROR;
  LCD_begin(lcd);
  HAL_Delay(300);
  LCD_clearScreen(lcd);
  LCD_putstr(lcd, (uint8_t*)clearVar);
  HAL_Delay(300);
  LCD_home(lcd);
  LCD_putstr(lcd, (uint8_t*)"STM32 Mp3 Reader");
  HAL_Delay(3000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */
    MX_USB_HOST_Process();

  /* USER CODE BEGIN 3 */
    if (localState != applicationState) {
           switch (applicationState) {
           case APPLICATION_IDLE:
       		printf("Idle state\r\n");
           	break;
           case APPLICATION_START:
       		printf("Start state\r\n");
           	break;
           case APPLICATION_READY:
       		printf("Ready state\r\n");
       		mountVolume();
       		// lettura e riproduzione delle canzoni
       		readSongs();
           	break;
           case APPLICATION_DISCONNECT:
           	stopMP3Song();
       		printf("Disconnect state\r\n");
           	break;
           default:
       		printf("Unknown state\r\n");
           	break;
           }
           localState = applicationState;
    }

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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



void MX_USB_HOST_StateChanged(ApplicationTypeDef appState) {
	applicationState = appState;
}

int _write (int fd, const void *buf, size_t count) {
	// Write to UART Here
	uart_write(buf, count);
	return count;
}
void mountVolume() {
	FRESULT res = f_mount(&fs, "0:/", 0);
	if (res != FR_OK) {
		printf("Error mounting filesystem %d\r\n", res);
	}
}

void driveDisplay(char *filename){			// Scrittura brano in riproduzione + crop string
	LCD_setCursor(lcd, 0, 0);
	LCD_putstr(lcd, (uint8_t*)clearVar);
	HAL_Delay(500);
	LCD_home(lcd);
	HAL_Delay(300);
	if(strlen(filename) > 16){
		char tmp[16];
		for(int i=0; i<15; i++)
			tmp[i] = filename[i];
		tmp[15]='\0';
		LCD_putstr(lcd, (uint8_t*)tmp);
	}
	else
		LCD_putstr(lcd, (uint8_t*)filename);
	LCD_setCursor(lcd, 1, 0);
	LCD_putstr(lcd, (uint8_t*)"In riproduzione");
}

void readSongs(){
	DIR rootdir;
	FILINFO finfo;
	FRESULT res = FR_OK;
	if ((res = f_opendir(&rootdir, "/")) != FR_OK) {
		printf("Error opening root directory %d\r\n", res);
		return;
	}
	while (f_readdir(&rootdir, &finfo) == FR_OK){
		if (finfo.fname[0] == '\0'){
			// tentativo di rewind della dir
			f_readdir(&rootdir, NULL);
			continue;
			//break;
		}
		if (finfo.fname[0] == '.') continue;	// salto curdir
		if (finfo.fattrib & AM_DIR) continue;	// salto directory
		if (strstr(finfo.fname, ".mp3")){
			// se la canzone è appena stata riprodotta la salto
//			if(strcmp(finfo.fname, prevName) == 0) continue;
//			strcpy(prevName, finfo.fname);
			driveDisplay(finfo.fname);			// chiamata a funzione per pilotare LCD
			playMP3Song(finfo.fname);
			while(f_eof(&currentSong)==0){		// controllo dello User Button
				if(BUTTON){
					while(BUTTON);
					break;
				}
				playMP3Task();					// Riproduzione dello stream MP3
			}
			stopMP3Song();
		}
	}
}
void loadFileListing() {
	DIR rootdir;
	FILINFO finfo;
    FRESULT res = FR_OK;

	if ((res = f_opendir(&rootdir, "/")) != FR_OK) {
		printf("Error opening root directory %d\r\n", res);
		return;
	}

	while (f_readdir(&rootdir, &finfo) == FR_OK) {
		if (finfo.fname[0] == '\0') break;
		if (finfo.fname[0] == '.') continue;

		if (finfo.fattrib & AM_DIR) {
			printf("found directory %s\r\n", finfo.fname);
		} else {
			printf("found file %s\r\n", finfo.fname);
		}


		if (strstr(finfo.fname, ".mp3")) {
			if(strcmp(finfo.fname, prevName) == 0) break;
			printf("found mp3 file! trying to play!\r\n");
			// prova scrittura LCD
			//LCD_clearScreen(lcd);
			strcpy(prevName, finfo.fname);
			LCD_putstr(lcd, (uint8_t*)clearVar);
			HAL_Delay(500);
			LCD_home(lcd);
			HAL_Delay(300);
			LCD_putstr(lcd, (uint8_t*)finfo.fname);
			playMP3Song(finfo.fname);
			break;
		}
	}
	f_closedir(&rootdir);
	printf("done reading rootdir\r\n");
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	UNUSED(file);
	UNUSED(line);
	while(1);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
