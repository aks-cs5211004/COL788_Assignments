/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
#include "max30102_for_stm32_hal.h"
#include "max_processing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

osThreadId SensorHandle;
osThreadId ProcessorHandle;
osThreadId LoggerHandle;
osThreadId HandlerHandle;
osMessageQId queue1Handle;
osMessageQId queue2Handle;
SemaphoreHandle_t sem1Handle;
/* USER CODE BEGIN PV */

// printf() function
int __io_putchar(int ch)
{
  uint8_t temp = ch;
  HAL_UART_Transmit(&huart2, &temp, 1, HAL_MAX_DELAY);
  return ch;
}

// MAX30102 object
max30102_t max30102;
FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations
typedef struct{
	uint32_t red;
	uint32_t ir;
} Data1;

typedef struct{
	uint32_t inte;
	uint32_t frac;
} Data2;

void float_to_data2(float value, Data2 *data) {
    data->inte = (int32_t)value;  // Extract integer part
    data->frac = (int32_t)((value - data->inte) * 1000);  // Extract fractional part in thousandths
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
void sSensor(void const * argument);
void pProcessor(void const * argument);
void sLogger(void const * argument);
void sHandler(void const * argument);

/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

// HAL_Delay(1000);
//
//	//Open the file system
//  	FRESULT fres1;
//  	fres1 = f_mount(NULL, "", 0);
//  	if(fres1==FR_OK){
//  		myprintf("Successful Unmont\r\n");
//  	}
//  	else{
//  		myprintf("UNNSuccessful Unmont\r\n");
//  	}
//  	HAL_Delay(3000);
//	fres1 = f_mount(&FatFs, "", 1); //1=mount now
//	if (fres1 != FR_OK) {
//	myprintf("f_mount error (%i)\r\n", fres1);
//	while(1);
//	}
//
//    //Let's get some statistics from the SD card
//    DWORD free_clusters, free_sectors, total_sectors;
//
//    FATFS* getFreeFs;
//
//    fres = f_getfree("", &free_clusters, &getFreeFs);
//    if (fres != FR_OK) {
//  	myprintf("f_getfree error (%i)\r\n", fres);
//  	while(1);
//    }
//
//    //Formula comes from ChaN's documentation
//    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
//    free_sectors = free_clusters * getFreeFs->csize;
//
//    myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);
//
//
//
//    //Now let's try and write a file "write.txt"
//    fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
//    if(fres == FR_OK) {
//  	myprintf("I was able to open 'write.txt' for writing\r\n");
//    } else {
//  	myprintf("f_open error (%i)\r\n", fres);
//    }
//
//    char readBuf[100];
//
//    //Copy in a string
//    strncpy((char*)readBuf, "a new file is made!", 19);
//    UINT bytesWrote;
//    fres = f_write(&fil, readBuf, 19, &bytesWrote);
//    if(fres == FR_OK) {
//  	myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
//    } else {
//  	myprintf("f_write error (%i)\r\n");
//    }
//
//
//    //Be a tidy kiwi - don't forget to close your file!
//    f_close(&fil);


    // Initiation Sensor
    max30102_init(&max30102, &hi2c1);
  	max30102_reset(&max30102);
  	max30102_clear_fifo(&max30102);
  	max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);

  	// Sensor settings
  	max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
  	max30102_set_adc_resolution(&max30102, max30102_adc_2048);
  	max30102_set_sampling_rate(&max30102, max30102_sr_800);
  	max30102_set_led_current_1(&max30102, 6.2);
  	max30102_set_led_current_2(&max30102, 6.2);

  	// Enter SpO2 mode
  	max30102_set_mode(&max30102, max30102_spo2);
  	max30102_set_a_full(&max30102, 1);

  	// Initiate 1 temperature measurement
  	max30102_set_die_temp_en(&max30102, 1);
  	max30102_set_die_temp_rdy(&max30102, 1);

  	uint8_t en_reg[2] = {0};
  	max30102_read(&max30102, 0x00, en_reg, 1);
    initialize_filters_1();
    initialize_filters_2();
	// Initiation Sensor End




  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of sem1 */
    sem1Handle = xSemaphoreCreateBinary();

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of queue1 */
  osMessageQDef(queue1, 1, Data1);
  queue1Handle = osMessageCreate(osMessageQ(queue1), NULL);

  /* definition and creation of queue2 */
  osMessageQDef(queue2, 1, Data2);
  queue2Handle = osMessageCreate(osMessageQ(queue2), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Sensor */
  osThreadDef(Sensor, sSensor, osPriorityNormal, 0, 128);
  SensorHandle = osThreadCreate(osThread(Sensor), NULL);

  /* definition and creation of Processor */
  osThreadDef(Processor, pProcessor, osPriorityAboveNormal, 0, 128);
  ProcessorHandle = osThreadCreate(osThread(Processor), NULL);

  /* definition and creation of Logger */
  osThreadDef(Logger, sLogger, osPriorityHigh, 0, 128);
  LoggerHandle = osThreadCreate(osThread(Logger), NULL);

  /* definition and creation of Handler */
//  osThreadDef(Handler, sHandler, osPriorityNormal, 0, 128);
//  HandlerHandle = osThreadCreate(osThread(Handler), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Max30102Samples samples = max30102_read_fifo(&max30102);
	  uint32_t latest_red_value = samples.red_sample;
	  uint32_t latest_ir_value = samples.ir_sample;
	  myprintf("Red Sample1: %d\r\n", latest_red_value);
	 // Part 1: Measure Heart Rate, Write and Read
//		int heart_rate = calculate_heart_rate(latest_red_value);
//		if(heart_rate!=-1)
//			printf("Heart Rate= %d\r\n",heart_rate);
//		if (heart_rate != -1) {
//		  fres = f_open(&fil, "test.txt", FA_WRITE | FA_OPEN_APPEND);
//		  if (fres == FR_OK) {
//			len = sprintf(buffer1, "Heart Rate (bpm): %d\r\n", heart_rate);
//			f_write(&fil, buffer1, len, &bytesWrote);
//			f_close(&fil);
//		  }
//		  fres = f_open(&fil, "test.txt", FA_READ);
//		  if (fres == FR_OK) {
//			TCHAR readBuf[30];
//			f_gets(readBuf, 30, &fil);
//			myprintf("Read Heart Rate: %s\r\n", readBuf);
//			f_close(&fil);
//		  }
//		}

	 // Part 2: Measure Advanced Heartbeat Detection, Write and Read
//	  int adv_heart_rate = advanced_heartbeat_detection(latest_red_value);
//	  if (adv_heart_rate != -1) {
//		fres = f_open(&fil, "test.txt", FA_WRITE | FA_OPEN_APPEND);
//		if (fres == FR_OK) {
//		  len = sprintf(buffer1, "Advanced Heart Rate (avg, bpm): %d\r\n", adv_heart_rate);
//		  f_write(&fil, buffer1, len, &bytesWrote);
//		  f_close(&fil);
//		}
//		fres = f_open(&fil, "test.txt", FA_READ);
//		if (fres == FR_OK) {
//		  TCHAR readBuf[30];
//		  f_gets(readBuf, 30, &fil);
//		  myprintf("Read Advanced Heart Rate: %s\r\n", readBuf);
//		  f_close(&fil);
//		}
//	  }


	  // Part 3: Measure SPO2, Write and Read
//	    float spo2 = calculate_spo2(latest_red_value, latest_ir_value);

//	    if (spo2 != -1) {
//	      fres = f_open(&fil, "test.txt", FA_WRITE | FA_OPEN_APPEND);
//	      if (fres == FR_OK) {
//	        len = sprintf(buffer1, "SpO2 (avg, %%): %.2f\r\n", spo2);
//	        f_write(&fil, buffer1, len, &bytesWrote);
//	        f_close(&fil);
//	      }
//	      fres = f_open(&fil, "test.txt", FA_READ);
//	      if (fres == FR_OK) {
//	        TCHAR readBuf[30];
//	        f_gets(readBuf, 30, &fil);
//	        myprintf("Read SPO2: %s\r\n", readBuf);
//	        f_close(&fil);
//	      }
//	    }


	// Part 4: Temperature Interrupt, Write and Read
//	  if (max30102_has_interrupt(&max30102)){
	 		  // Run interrupt handler to read FIFO
//	 		  printf("%.2f\r\n",max30102_interrupt_handler(&max30102));
//		if (temperature != -1) {
//		  fres = f_open(&fil, "test.txt", FA_WRITE | FA_OPEN_APPEND);
//		  if (fres == FR_OK) {
//			len = sprintf(buffer1, "Temperature: %.2f °C\r\n", temperature);
//			f_write(&fil, buffer1, len, &bytesWrote);
//			f_close(&fil);
//		  }
//		  fres = f_open(&fil, "test.txt", FA_READ);
//		  if (fres == FR_OK) {
//			TCHAR readBuf[30];
//			f_gets(readBuf, 30, &fil);
//			myprintf("Read Temperature: %s\r\n", readBuf);
//			f_close(&fil);
//		  }
//		}


  }
  //We're done, so de-mount the drive
//  f_mount(NULL, "", 0);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000608;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin PC0 */
  GPIO_InitStruct.Pin = B1_Pin|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_sSensor */
/**
  * @brief  Function implementing the Sensor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_sSensor */
void sSensor(void const * argument)
{
  /* USER CODE BEGIN 5 */
	 Data1 sensorData;
	 BaseType_t xStatus;
  /* Infinite loop */
  for(;;)
  {
	  Max30102Samples samples = max30102_read_fifo(&max30102);
	  sensorData.red= samples.red_sample;
	  sensorData.ir = samples.ir_sample;
	  xStatus =  xQueueSend(queue1Handle, &sensorData, 0 );
	  if( xStatus != pdPASS )
	  {
		  printf( "Could not send to the queue.\r\n" );
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_pProcessor */
/**
* @brief Function implementing the Processor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pProcessor */
void pProcessor(void const * argument)
{
  /* USER CODE BEGIN pProcessor */
	Data1 receivedData;
	Data2 processedData;
	BaseType_t xStatus;
  /* Infinite loop */
  for(;;)
  {
	  xStatus = xQueueReceive(queue1Handle, &receivedData, portMAX_DELAY);

	  if( xStatus == pdPASS )
	  {
			  float answer= calculate_heart_rate(receivedData.red);
//			  float answer= advanced_heartbeat_detection(receivedData.red);
//		  	  float answer= calculate_spo2(receivedData.red, receivedData.ir);
			  float_to_data2(answer, &processedData);
			  if(processedData.inte!=-1){
				  xQueueSend(queue2Handle, &processedData, 0 );
			  }
	  }
  }
  /* USER CODE END pProcessor */
}

/* USER CODE BEGIN Header_sLogger */
/**
* @brief Function implementing the Logger thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sLogger */
void sLogger(void const * argument)
{
  /* USER CODE BEGIN sLogger */
  /* Infinite loop */
	Data2 processedData;
	BaseType_t xStatus;
  for(;;)
  {
	  xStatus = xQueueReceive(queue2Handle, &processedData, portMAX_DELAY);
	  if( xStatus == pdPASS )
	  	{
		  myprintf("LOGGER: Printing Processed Data: %u.%02u\r\n",processedData.inte, processedData.frac);
//		  // Open the file in append mode
//		            fres = f_open(&fil, "log.txt", FA_WRITE | FA_OPEN_APPEND | FA_CREATE_ALWAYS);
//		            if(fres == FR_OK)
//		            {
//		                // Get the system tick count as a basic timestamp
//		                uint32_t timestamp = osKernelSysTick();
//
//		                // Format the log entry to include heart rate data and timestamp
//		                snprintf(writeBuf, sizeof(writeBuf), "Timestamp: %lu, Heart rate Data: %d\r\n", timestamp, processedData.inte);
//
//		                // Write the data to the file
//		                fres = f_write(&fil, writeBuf, strlen(writeBuf), &bytesWritten);
//		                if(fres == FR_OK && bytesWritten == strlen(writeBuf))
//		                {
//		                    myprintf("Successfully wrote %u bytes to 'log.txt'\r\n", bytesWritten);
//		                }
//		                else
//		                {
//		                    myprintf("Error writing data to 'log.txt' (%i)\r\n", fres);
//		                }
//
//		                // Close the file after writing
//		                f_close(&fil);
//		            }
//		            else
//		            {
//		                myprintf("Error opening 'log.txt' for writing (%i)\r\n", fres);
//		            }

		}
  }
  /* USER CODE END sLogger */
}

/* USER CODE BEGIN Header_sHandler */
/**
* @brief Function implementing the Handler thread.
* @param argument: Not used
* @retval None
*/

void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */
	 HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	max30102_on_interrupt(&max30102);
	static signed long xHigherPriorityTaskWoken;
	 xHigherPriorityTaskWoken = pdFALSE;
	 if (sem1Handle != NULL){
		 if(xSemaphoreGiveFromISR(sem1Handle,&xHigherPriorityTaskWoken)==pdTRUE){
			 myprintf("Semaphore released\r\n");
		 }
	 }
	 else{
		 myprintf("Semaphore handle is null.\r\n");
	 }
	 portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	 portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  /* USER CODE END EXTI0_1_IRQn 0 */
}


/* USER CODE END Header_sHandler */
void sHandler(void const * argument)
{
  /* USER CODE BEGIN sHandler */
  Data2 answer;
  for(;;)
  {
	  if (sem1Handle != NULL){
		  xSemaphoreTake( sem1Handle, portMAX_DELAY );
		  float_to_data2(max30102_interrupt_handler(&max30102), &answer);
		  myprintf("Semaphore caught: Temperature: %u.%02u\r\n",answer.inte, answer.frac);
	  }
	  else{
		  myprintf("Semaphore handle is null.\r\n");
	  }
  }
  /* USER CODE END sHandler */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
