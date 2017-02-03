/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "i2c.h"
//#include "lwip.h"
//#include "lwip/debug.h"
//#include "lwip/stats.h"
//#include "lwip/tcp.h"
#include "SEGGER_RTT.h"
#include "trcUser.h"
#include "l3g4200d_driver.h"

#include "FreeRTOS_sockets.h"
#include "FreeRTOSIPConfigDefaults.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId vEthernet_TaskHandle;
osThreadId TempSensor_TaskHandle;
osThreadId GyroSensor_TaskHandle;
osThreadId LCD_TaskHandle;
osThreadId Controller_TaskHandle;
osMessageQId TempSensorQueueHandle;
osMessageQId PingQueueHandle;
osMessageQId ErrorMessageQueueHandle;
osMutexId LCDRefreshMutexHandle;
osSemaphoreId TemperatureSemaphoreHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartEthernetTask(void const * argument);
void StartTempSensor(void const * argument);
void StartGyroSensor(void const * argument);
void StartLCDTask(void const * argument);
void StartControllerTask(void const * argument);

void GLCD_Initialize(void);
void GLCD_ClearScreen(void);
void GLCD_GoTo(unsigned char, unsigned char);
void GLCD_WriteString(char *);

uint32_t uiTraceStart(void);
uint32_t uiTraceStop(void);

BaseType_t FreeRTOS_IPInit( const uint8_t *, const uint8_t *, const uint8_t *, const uint8_t *, const uint8_t * );

/*TCP IP*/
extern struct netif gnetif;
extern BaseType_t xNetworkInterfaceInitialise( void );
BaseType_t vSendPing( const char * pcIPAddress );


extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/*MCP Registers*/
#define TEMPERATURE_REG_TEMP      0x00  // Temperatur register
#define TEMPERATURE_REG_CONFIG      0x01  // Configuration register
#define TEMPERATURE_REG_HYSTERISIS    0x02  // Temperature Hysteresis register
#define TEMPERATURE_REG_LIMIT     0x03  // Temperature Limit-set register

/*Config bits*/
#define TEMPERATURE_CONFIG_ONE_SHOT   0x07  // One shot enabled/disabled. Disabled by default
#define TEMPERATURE_CONFIG_ADC_RES    0x05  // ADC resolution: 00 = 9bit (0.5c), 01 = 10bit (0.25c), 10 = 11bit (0.125c), 11 = 12bit (0.0625c)
#define TEMPERATURE_CONFIG_FAULT_QUEUE  0x03  // Fault queue bits, 00 = 1 (default), 01 = 2, 10 = 4, 11 = 6
#define TEMPERATURE_CONFIG_ALERT_POL  0x04  // Alert polarity (high/low). Default low
#define TEMPERATURE_CONFIG_COMP_INT   0x03  // 1 = Interrupt mode, 0 = Comparator mode (default)
#define TEMPERATURE_CONFIG_SHUTDOWN 0x02 // 1 = Enable shutdown, 0 = Disable shutdown (default)

#define TEMPERATURE_DEVICE_ADDRESS ( 0x48 << 1 )

#define TEMPERATURE_SENSOR_ERROR 1001




/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Create the mutex(es) */
	/* definition and creation of LCDRefreshMutex */
	//In xQueueGenericCreate was commented out configASSERT
	osMutexDef(LCDRefreshMutex);
	LCDRefreshMutexHandle = osMutexCreate(osMutex(LCDRefreshMutex));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of TemperatureSemaphore */
	//osSemaphoreDef(TemperatureSemaphore);
	//TemperatureSemaphoreHandle = osSemaphoreCreate(osSemaphore(TemperatureSemaphore), 1);
	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	//osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128); //osPriorityNormal
	//defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of vEthernet_Task */
	//osThreadDef(vEthernet_Task, StartEthernetTask, osPriorityNormal, 0, 128);
	//vEthernet_TaskHandle = osThreadCreate(osThread(vEthernet_Task), NULL);

	/* definition and creation of TempSensor_Task */
	osThreadDef(TempSensor_Task, StartTempSensor, osPriorityLow, 0, 128); //osPriorityLow
	TempSensor_TaskHandle = osThreadCreate(osThread(TempSensor_Task), NULL);

	/* definition and creation of Gyro */
	osThreadDef(GyroSensor_Task, StartGyroSensor, osPriorityLow, 0, 128); //osPriorityLow
	GyroSensor_TaskHandle = osThreadCreate(osThread(GyroSensor_Task), NULL);

	/* definition and creation of LCD_Task */
	osThreadDef(LCD_Task, StartLCDTask, osPriorityNormal, 0, 128); //osPriorityBelowNormal
	LCD_TaskHandle = osThreadCreate(osThread(LCD_Task), NULL);

	/* definition and creation of Controller_Task */
	//osThreadDef(Controller_Task, StartControllerTask, osPriorityNormal, 0,
	//		128); //osPriorityAboveNormal
	//Controller_TaskHandle = osThreadCreate(osThread(Controller_Task), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s) */
	/* definition and creation of TempSensorQueue */
	osMessageQDef(TempSensorQueue, 16, uint16_t);
	TempSensorQueueHandle = osMessageCreate(osMessageQ(TempSensorQueue), NULL);


	osMessageQDef(ErrorMessageQueue, 16, uint16_t);
	ErrorMessageQueueHandle = osMessageCreate(osMessageQ(ErrorMessageQueue),
			NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */
}


/* The default IP and MAC address used by the demo.  The address configuration
defined here will be used if ipconfigUSE_DHCP is 0, or if ipconfigUSE_DHCP is
1 but a DHCP server could not be contacted.  See the online documentation for
more information.  http://www.FreeRTOS.org/tcp */
const uint8_t ucIPAddress[ 4 ] = { 192, 168, 76, 230 };
const uint8_t ucNetMask[ 4 ] = { 255, 255, 255, 0 };
const uint8_t ucGatewayAddress[ 4 ] = { 192, 168, 76, 1 };
const uint8_t ucDNSServerAddress[ 4 ] = {192, 168, 76, 1};

/* Default MAC address configuration. */
const uint8_t ucMACAddress[ 6 ] = { 0, 0, 0x81, 0xe1, 0, 0 };


/* StartDefaultTask function */
void StartDefaultTask(void const * argument) {

	/* USER CODE BEGIN StartDefaultTask */
	/* init code for LWIP */

	uiTraceStart();
	 //FreeRTOS_IPInit( ucIPAddress, ucNetMask, ucGatewayAddress, ucDNSServerAddress, ucMACAddress );
	 //MX_LWIP_Init();

	SEGGER_RTT_WriteString(0,"LWIP INIT\r\n");



	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartDefaultTask */
}

/* StartEthernetTask function */
void StartEthernetTask(void const * argument) {
	/* USER CODE BEGIN StartEthernetTask */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartEthernetTask */
}

uint8_t temperatureSensorExecute(uint8_t config, uint32_t timeout,
		osMutexId LCDRefreshMutexHandle, osMessageQId ErrorMessageQueueHandle,
		osMessageQId TempSensorQueueHandle, uint8_t* temperature) {

	/* LOCK writing */
	osMutexWait(LCDRefreshMutexHandle, osWaitForever);

	HAL_GPIO_WritePin(LED_1_Pin_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

	HAL_I2C_Mem_Write(&hi2c1, TEMPERATURE_DEVICE_ADDRESS, TEMPERATURE_REG_CONFIG,
			sizeof(uint8_t), &config, sizeof(config), timeout);
	HAL_I2C_Mem_Read(&hi2c1, TEMPERATURE_DEVICE_ADDRESS, TEMPERATURE_REG_TEMP,
			sizeof(uint8_t), &config, sizeof(config), timeout);
	config &= ~(1 << TEMPERATURE_CONFIG_SHUTDOWN);
	config |= (1 << TEMPERATURE_CONFIG_SHUTDOWN);
	HAL_I2C_Mem_Write(&hi2c1, TEMPERATURE_DEVICE_ADDRESS, TEMPERATURE_REG_CONFIG,
			sizeof(uint8_t), &config, sizeof(config), timeout);
	config = 0;
	HAL_I2C_Mem_Write(&hi2c1, TEMPERATURE_DEVICE_ADDRESS, TEMPERATURE_REG_CONFIG,
			sizeof(uint8_t), &config, sizeof(config), timeout);
	HAL_I2C_Mem_Read(&hi2c1, TEMPERATURE_DEVICE_ADDRESS, TEMPERATURE_REG_TEMP,
			sizeof(uint8_t), &config, sizeof(config), timeout);
	config &= ~(1 << TEMPERATURE_CONFIG_ONE_SHOT);
	config |= (1 << TEMPERATURE_CONFIG_ONE_SHOT);
	HAL_I2C_Mem_Write(&hi2c1, TEMPERATURE_DEVICE_ADDRESS, TEMPERATURE_REG_CONFIG,
			sizeof(uint8_t), &config, sizeof(config), timeout);
	HAL_I2C_Mem_Read(&hi2c1, TEMPERATURE_DEVICE_ADDRESS, TEMPERATURE_REG_TEMP,
			sizeof(uint8_t), &*temperature, sizeof(*temperature), timeout);



		if (hi2c1.ErrorCode != 0) {
			osMessagePut(ErrorMessageQueueHandle, TEMPERATURE_SENSOR_ERROR, 0);
		} else {
			osMessagePut(TempSensorQueueHandle, 0, 0);
		}

		/* Sent temperature to queue */
		osMessagePut(TempSensorQueueHandle, *temperature, 0);

	HAL_GPIO_WritePin(LED_1_Pin_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);

	/* UNLOCK writing */
	osMutexRelease(LCDRefreshMutexHandle);


	return config;

}

/* StartTempSensor function */
void StartTempSensor(void const * argument) {
	/* USER CODE BEGIN StartTempSensor */
	uint8_t config = 0, temperature = 0;
	uint32_t timeout = 1000;
    //char text_buffer[30];
	/* Infinite loop */
	for (;;) {


		config = temperatureSensorExecute(config, timeout,
				LCDRefreshMutexHandle, ErrorMessageQueueHandle,
				TempSensorQueueHandle, &temperature);

		SEGGER_RTT_WriteString(0, "TEMP\n\r");


		osDelay(5000);

	}
	/* USER CODE END StartTempSensor */
}


/* StartGyroSensor function */
void StartGyroSensor(void const * argument) {
	/* USER CODE BEGIN StartGyroSensor */

	  	  char text_buffer[20];
		  AxesRaw_t data;
		  status_t response;

		  osMutexWait(LCDRefreshMutexHandle, osWaitForever);

		  //set ODR (turn ON device)
		 response = L3G4200D_SetODR(L3G4200D_ODR_95Hz_BW_25);
		 if(response==1){  //debug print response for MKI109V1 board
		        SEGGER_RTT_WriteString(0, "SET_ODR_OK    \n\r");
		   }
		 //set PowerMode
		 response = L3G4200D_SetMode(L3G4200D_NORMAL);
		 if(response==1){  //debug print response for MKI109V1 board
		        SEGGER_RTT_WriteString(0, "SET_MODE_OK    \n\r");
		  }
		 //set Fullscale
		 response = L3G4200D_SetFullScale(L3G4200D_FULLSCALE_250);
		 if(response==1){  //debug print response for MKI109V1 board
		        SEGGER_RTT_WriteString(0, "SET_FULLSCALE_OK    \n\r");
		  }
		 //set axis Enable
		 response = L3G4200D_SetAxis(L3G4200D_X_ENABLE | L3G4200D_Y_ENABLE | L3G4200D_Z_ENABLE);
		 if(response==1){     //debug print response for MKI109V1 board
			 	 SEGGER_RTT_WriteString(0, "SET_AXIS_OK    \n\r");
		 }

		/* UNLOCK writing */
		osMutexRelease(LCDRefreshMutexHandle);


	/* Infinite loop */
	for (;;) {

		/* LOCK writing */
		osMutexWait(LCDRefreshMutexHandle, osWaitForever);

		//get Acceleration Raw data
		  response = L3G4200D_GetAngRateRaw(&data);
		  if(response==1){    //debug print axies value for MKI109V1 board
		    //len = sprintf((char*)buffer, "X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);

		    SEGGER_RTT_WriteString(0, "X:  ");
		    snprintf(text_buffer, 10, "%d", data.AXIS_X);
		    SEGGER_RTT_WriteString(0, text_buffer);
		    SEGGER_RTT_WriteString(0, "   ");

		    SEGGER_RTT_WriteString(0, "Y:  ");
		    snprintf(text_buffer, 10, "%d", data.AXIS_Y);
		   	SEGGER_RTT_WriteString(0, text_buffer);
		   	SEGGER_RTT_WriteString(0, "   ");

		    SEGGER_RTT_WriteString(0, "Z:  ");
		    snprintf(text_buffer, 10, "%d", data.AXIS_Z);
		   	SEGGER_RTT_WriteString(0, text_buffer);
		   	SEGGER_RTT_WriteString(0, "\n\r");

		  }

		  /* UNLOCK writing */
		  osMutexRelease(LCDRefreshMutexHandle);

			osDelay(5000);

	}
	/* USER CODE END StartTempSensor */
}


/* StartLCDTask function */
void StartLCDTask(void const * argument) {
	/* USER CODE BEGIN StartLCDTask */

	//LCD lines 0-7
	char temperature_text[21] = "TEMPERATURE:";
	osEvent message, error_message;
	uint32_t temperature_value;
	GLCD_Initialize();
	GLCD_ClearScreen();

	temperature_text[12] = (char) ' '; // [
	temperature_text[15] = (char) 'C'; // ]

	for (;;) {

		osMutexWait(LCDRefreshMutexHandle, osWaitForever);

		error_message = osMessageGet(ErrorMessageQueueHandle, 0);

		if (error_message.value.v == TEMPERATURE_SENSOR_ERROR) {
			GLCD_GoTo(0, 2);
			GLCD_WriteString("SENSOR ERROR");
		} else {
			GLCD_GoTo(0, 2);
			GLCD_WriteString("SENSOR OK   ");

		}

		message = osMessageGet(TempSensorQueueHandle, 0);
		temperature_value = message.value.v;

		if (temperature_value != 0) {
			temperature_text[13] = (temperature_value / 10) + 0x30;
			temperature_text[14] = (temperature_value % 10) + 0x30;
		}

		osMutexRelease(LCDRefreshMutexHandle);

		GLCD_GoTo(0, 1);
		GLCD_WriteString(temperature_text);

		osDelay(1000);
	}

	/* USER CODE END StartLCDTask */
}

/* StartControllerTask function */
void StartControllerTask(void const * argument) {
	/* USER CODE BEGIN StartControllerTask */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartControllerTask */
}



	/* USER CODE END Application */

	/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
