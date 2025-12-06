/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "semphr.h"
#include "app_states.h"
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
DeviceHSM_t device;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readBatteryTask */
osThreadId_t readBatteryTaskHandle;
const osThreadAttr_t readBatteryTask_attributes = {
  .name = "readBatteryTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for binsemaModbusMaster */
osSemaphoreId_t binsemaModbusMasterHandle;
const osSemaphoreAttr_t binsemaModbusMaster_attributes = {
  .name = "binsemaModbusMaster"
};
/* Definitions for binsemaModbusSlave */
osSemaphoreId_t binsemaModbusSlaveHandle;
const osSemaphoreAttr_t binsemaModbusSlave_attributes = {
  .name = "binsemaModbusSlave"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint8_t ReadModbusAddress(void)
{
    uint8_t addr = 0;

    if (!HAL_GPIO_ReadPin(ADDR4_GPIO_Port, ADDR4_Pin)) addr |= (1<<0);
    if (!HAL_GPIO_ReadPin(ADDR3_GPIO_Port, ADDR3_Pin)) addr |= (1<<1);
    if (!HAL_GPIO_ReadPin(ADDR2_GPIO_Port, ADDR2_Pin)) addr |= (1<<2);
    if (!HAL_GPIO_ReadPin(ADDR1_GPIO_Port, ADDR1_Pin)) addr |= (1<<3);
    if (!HAL_GPIO_ReadPin(ADDR0_GPIO_Port, ADDR0_Pin)) addr |= (1<<4);

    return (addr == 0) ? 1 : addr;  // Nếu = 0 thì dùng địa chỉ 1
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartReadBatteryTask(void *argument);
void StartCommTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of binsemaModbusMaster */
  binsemaModbusMasterHandle = osSemaphoreNew(1, 1, &binsemaModbusMaster_attributes);

  /* creation of binsemaModbusSlave */
  binsemaModbusSlaveHandle = osSemaphoreNew(1, 1, &binsemaModbusSlave_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of readBatteryTask */
  readBatteryTaskHandle = osThreadNew(StartReadBatteryTask, NULL, &readBatteryTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	/* Master initialization */
	osSemaphoreAcquire(binsemaModbusMasterHandle, 0);
	osSemaphoreAcquire(binsemaModbusSlaveHandle, 0);

	device.modbus_address = ReadModbusAddress();
	hsmTimerInit();
	default_parameter_init(&device);
	app_states_hsm_init(&device);
	if(!device.modbus_address) {
		uint8_t old_modbus_address = device.modbus_address;
		while(1) {
			device.modbus_address = ReadModbusAddress();
			if(device.modbus_address == 0) {
				HSM_Run((HSM *)&device, HSME_SET_INVALID_BAUD, 0);
			} else {
				if(device.modbus_address != old_modbus_address) {
					old_modbus_address = device.modbus_address;
					HSM_Run((HSM *)&device, HSME_SET_BAUD_CHANGE_VALUE, 0);
				}
			}
			osDelay(10);
		}
	} else {

		/* Master initialization */
		device.handlerModbusMaster.uModbusType = MB_MASTER;
		device.handlerModbusMaster.port =  &MB_BAT_UART;
		device.handlerModbusMaster.u8id = 0; // For master it must be 0
		device.handlerModbusMaster.u16timeOut = 1000;
		device.handlerModbusMaster.EN_Port = MB_BAT_PORT;
		device.handlerModbusMaster.EN_Pin = MB_BAT_PIN;
		device.handlerModbusMaster.u16regs = device.dataModbusMaster;
		device.handlerModbusMaster.u16regsize= sizeof(device.dataModbusMaster)/sizeof(device.dataModbusMaster[0]);
		device.handlerModbusMaster.xTypeHW = USART_HW_DMA;

		/* Slave initialization */
		UART_ReconfigureByCode(&MB_COM_UART, device.storage.baudrate);
		device.handlerModbusSlave.uModbusType = MB_SLAVE;
		device.handlerModbusSlave.port =  &MB_COM_UART;
		device.handlerModbusSlave.u8id = device.modbus_address;
		device.handlerModbusSlave.u16timeOut = 1000;
		device.handlerModbusSlave.EN_Port = MB_COM_PORT;
		device.handlerModbusSlave.EN_Pin = MB_COM_PIN;
		device.handlerModbusSlave.u16regs = device.dataModbusSlave;
		device.handlerModbusSlave.u16regsize= sizeof(device.dataModbusSlave)/sizeof(device.dataModbusSlave[0]);

		//Initialize Modbus library
		device.handlerModbusSlave.xTypeHW = USART_HW_DMA;

		ModbusInit(&device.handlerModbusMaster);
		ModbusStart(&device.handlerModbusMaster);

		ModbusInit(&device.handlerModbusSlave);
		ModbusStart(&device.handlerModbusSlave);

		osSemaphoreRelease(binsemaModbusMasterHandle);
		osSemaphoreRelease(binsemaModbusSlaveHandle);
	}

  for(;;)
  {
	  // Check Limit Switch
	  if (HAL_GPIO_ReadPin(LIMIT_SWITCH0_GPIO_Port, LIMIT_SWITCH0_Pin) == (GPIO_PinState)LS_ACTIVE
			  || HAL_GPIO_ReadPin(LIMIT_SWITCH1_GPIO_Port, LIMIT_SWITCH1_Pin) == (GPIO_PinState)LS_ACTIVE ) {

		  HSM_Run((HSM *)&device, HSME_SWITCH_LIMIT_ACTIVE, 0);
	  } else {
		  device.dataModbusSlave[REG_STA_IS_PIN_IN_SLOT] = SLOT_EMPTY;
		  HSM_Run((HSM *)&device, HSME_SWITCH_LIMIT_PASSTIVE, 0);
	  }
	  osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartReadBatteryTask */
/**
* @brief Function implementing the readBatteryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadBatteryTask */
void StartReadBatteryTask(void *argument)
{
  /* USER CODE BEGIN StartReadBatteryTask */
  /* Infinite loop */
  uint32_t u32NotificationValue;
  static uint16_t timeout_count = 0;
  osSemaphoreAcquire(binsemaModbusMasterHandle, osWaitForever);

  device.telegramMaster[TELE_MASTER_READ_FULL_BAT_DATA].u8id = 0x10; // slave address
  device.telegramMaster[TELE_MASTER_READ_FULL_BAT_DATA].u8fct = 0x03; // function code (this one is registers read)
  device.telegramMaster[TELE_MASTER_READ_FULL_BAT_DATA].u16RegAdd = 0x00; // start address in slave
  device.telegramMaster[TELE_MASTER_READ_FULL_BAT_DATA].u16CoilsNo = TOTAL_BAT_REGISTERS; // number of elements (coils or registers) to read
  device.telegramMaster[TELE_MASTER_READ_FULL_BAT_DATA].u16reg = device.dataModbusMaster; // pointer to a memory array
  for(;;)
  {
	if(device.dataModbusSlave[REG_STA_IS_PIN_IN_SLOT] == SLOT_FULL) {
		ModbusQuery(&device.handlerModbusMaster, device.telegramMaster[TELE_MASTER_READ_FULL_BAT_DATA]);
		u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(u32NotificationValue != OP_OK_QUERY) {
			timeout_count = 0;
			HSM_Run((HSM *)&device, HSME_BAT_RECEIVED_OK, 0);
		} else {
			if(++timeout_count >= 5) {
				timeout_count = 5;
				HSM_Run((HSM *)&device, HSME_BAT_RECEIVED_TIMEOUT, 0);
			}
		}
	}

    osDelay(100);
  }
  /* USER CODE END StartReadBatteryTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  /* Infinite loop */
  osSemaphoreAcquire(binsemaModbusSlaveHandle, osWaitForever);
  for(;;)
  {
	xSemaphoreTake(device.handlerModbusSlave.ModBusSphrHandle , portMAX_DELAY);
	xSemaphoreGive(device.handlerModbusSlave.ModBusSphrHandle);
	osDelay(100);
  }
  /* USER CODE END StartCommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ModbusSlave_TxCompleteCallback(modbusHandler_t *modH)
{
	if(modH == &device.handlerModbusSlave) {
    HSM_Run((HSM *)&device, HSME_COMM_RECEIVED_OK, 0);
	}
}
/* USER CODE END Application */

