/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sx126x.h"
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
/* USER CODE BEGIN Variables */

uint8_t device_EUI [] =
{ 0x9E, 0xD6, 0xE4, 0xCD, 0xDE, 0x71, 0x10, 0x2A } ;
uint8_t app_EUI [] =
{ 0xDA, 0x83, 0x0C, 0x72, 0x31, 0x01, 0xE6, 0x92 } ;
uint8_t app_key [] =
{ 0xE9, 0x49, 0x8D, 0x25, 0xC7, 0x6F, 0xDD, 0x43, 0x04, 0x84, 0x5C, 0x3B, 0xEB,
		0x71, 0xFF, 0xD6 } ;

extern SPI_HandleTypeDef hspi1 ;

sx126x_t sx1262 ;
RadioIOCallbacks_t sx1262_IO_callbacks ;
RadioCallbacks_t sx1262_callbacks ;

/* USER CODE END Variables */
osThreadId defaultTaskHandle ;
uint32_t defaultTaskBuffer [128] ;
osStaticThreadDef_t defaultTaskControlBlock ;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void setRadioNSS ( uint8_t value ) ;
void setRadioRESET ( uint8_t value ) ;
uint8_t getRadioBUSY ( void ) ;
void setRadioAntSwitchPower ( uint8_t value ) ;

void txDone ( void ) ;
void rxDone ( void ) ;
void rxPreambleDetect ( void ) ;
void rxSyncWordDone ( void ) ;
void rxHeaderDone ( void ) ;
void txTimeout ( void ) ;
void rxTimeout ( void ) ;
void rxError ( IrqErrorCode_t errCode ) ;
void cadDone ( uint8_t cadFlag ) ;

void wait_ms ( uint32_t ms ) ;
void set_interrupts ( uint8_t enable ) ;
void spi_TxRx ( uint8_t* buf_tx, uint8_t* buf_rx, uint16_t size ) ;
void spi_Tx ( uint8_t* buf, uint16_t size ) ;
void spi_Rx ( uint8_t* buf, uint16_t size ) ;

/* USER CODE END FunctionPrototypes */

void StartDefaultTask ( void const * argument ) ;

void MX_FREERTOS_Init ( void ) ; /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory ( StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize ) ;

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory ( StaticTask_t **ppxTimerTaskTCBBuffer,
		StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize ) ;

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer ;
static StackType_t xIdleStack [configMINIMAL_STACK_SIZE] ;

void vApplicationGetIdleTaskMemory ( StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer ;
	*ppxIdleTaskStackBuffer = &xIdleStack [0] ;
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE ;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer ;
static StackType_t xTimerStack [configTIMER_TASK_STACK_DEPTH] ;

void vApplicationGetTimerTaskMemory ( StaticTask_t **ppxTimerTaskTCBBuffer,
		StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer ;
	*ppxTimerTaskStackBuffer = &xTimerStack [0] ;
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH ;
	/* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init ( void )
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadStaticDef( defaultTask, StartDefaultTask, osPriorityNormal, 0, 128,
			defaultTaskBuffer, &defaultTaskControlBlock ) ;
	defaultTaskHandle = osThreadCreate ( osThread( defaultTask ), NULL ) ;

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask ( void const * argument )
{
	/* USER CODE BEGIN StartDefaultTask */
	RadioStatus_t status ;
	RadioError_t errors ;

	sx1262_IO_callbacks.setRadioNSS = &setRadioNSS ;
	sx1262_IO_callbacks.setRadioRESET = &setRadioRESET ;
	sx1262_IO_callbacks.getRadioBUSY = &getRadioBUSY ;
	sx1262_IO_callbacks.setRadioAntSwitchPower = &setRadioAntSwitchPower ;

	sx1262_callbacks.txDone = &txDone ;
	sx1262_callbacks.rxDone = &rxDone ;
	sx1262_callbacks.rxPreambleDetect = &rxPreambleDetect ;
	sx1262_callbacks.rxSyncWordDone = &rxSyncWordDone ;
	sx1262_callbacks.rxHeaderDone = &rxHeaderDone ;
	sx1262_callbacks.txTimeout = &txTimeout ;
	sx1262_callbacks.rxTimeout = &rxTimeout ;
	sx1262_callbacks.rxError = &rxError ;
	sx1262_callbacks.cadDone = &cadDone ;

	sx1262.wait_ms = &wait_ms ;
	sx1262.set_interrupts = &set_interrupts ;
	sx1262.spi_TxRx = &spi_TxRx ;
	sx1262.spi_Tx = &spi_Tx ;
	sx1262.spi_Rx = &spi_Rx ;

	sx1262.IO_callbacks = &sx1262_IO_callbacks ;
	sx1262.callbacks = &sx1262_callbacks ;

	Init ( &sx1262 ) ;
	SetTxParams ( &sx1262, 0, RADIO_RAMP_200_US ) ;

	/* Infinite loop */
	for ( ; ; )
	{
		osDelay ( 1 ) ;
		status = GetStatus ( &sx1262 ) ;
		osDelay ( 1 ) ;
		errors = GetDeviceErrors ( &sx1262 ) ;
	}
	/* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void setRadioNSS ( uint8_t value )
{
	HAL_GPIO_WritePin ( SPI1_NSS_GPIO_Port, SPI1_NSS_Pin,
			value ? GPIO_PIN_SET : GPIO_PIN_RESET ) ;
}

void setRadioRESET ( uint8_t value )
{
	HAL_GPIO_WritePin ( SPI1_NSS_GPIO_Port, SPI1_NSS_Pin,
			value ? GPIO_PIN_SET : GPIO_PIN_RESET ) ;
}

uint8_t getRadioBUSY ( void )
{
	return HAL_GPIO_ReadPin ( BUSY_GPIO_Port, BUSY_Pin ) == GPIO_PIN_SET ;
}

void setRadioAntSwitchPower ( uint8_t value )
{
	HAL_GPIO_WritePin ( DIO2_GPIO_Port, DIO2_Pin,
			value ? GPIO_PIN_SET : GPIO_PIN_RESET ) ;
}

void txDone ( void )
{

}

void rxDone ( void )
{

}

void rxPreambleDetect ( void )
{

}

void rxSyncWordDone ( void )
{

}

void rxHeaderDone ( void )
{

}

void txTimeout ( void )
{

}

void rxTimeout ( void )
{

}

void rxError ( IrqErrorCode_t errCode )
{

}

void cadDone ( uint8_t cadFlag )
{

}

void wait_ms ( uint32_t ms )
{
	osDelay ( ms ) ;
}

void set_interrupts ( uint8_t enable )
{

}

void spi_TxRx ( uint8_t* buf_tx, uint8_t* buf_rx, uint16_t size )
{
	HAL_SPI_TransmitReceive ( &hspi1, buf_tx, buf_rx, size, 5 ) ;
}

void spi_Tx ( uint8_t* buf, uint16_t size )
{
	HAL_SPI_Transmit ( &hspi1, buf, size, 5 ) ;
}

void spi_Rx ( uint8_t* buf, uint16_t size )
{
	HAL_SPI_Receive ( &hspi1, buf, size, 5 ) ;
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
