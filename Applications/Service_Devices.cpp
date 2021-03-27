/**
  ******************************************************************************
  * @file   Service_Devices.c
  * @brief  Devices service running file.
  ******************************************************************************
  * @note
  *  - Before running your devices, just do what you want ~ !
  *  - More devices or using other classification is decided by yourself ~ !
  ===============================================================================
                                    Task List
  ===============================================================================
  * <table>
  * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
  * <tr><td>              <td>                  <td>                <td>    
  * </table>
  *
 */
/* Includes ------------------------------------------------------------------*/
#include "Service_Devices.h"
#include "inertial_navigation.h"
#include "Data_Dealer.h"
/* Private define ------------------------------------------------------------*/
TaskHandle_t DeviceIMU_Handle;
TaskHandle_t DeviceBMP_Handle;
TaskHandle_t DeviceSDcard_Handle;
TaskHandle_t DeviceFLASH_Handle;
TaskHandle_t DeviceCamera_Handle;

/* Private variables ---------------------------------------------------------*/
char data_path[36];		// �洢���ݵ��ļ�·��
char photo_path[36];	// �洢ͼƬ���ļ�·��
			
/* Private function declarations ---------------------------------------------*/
void Device_IMU(void *arg);
void Device_BMx(void *arg);
void Device_SDcard(void *arg);
void Device_FLASH(void *arg);
void Device_Camera(void *arg);

/* Exported devices ----------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
/**
* @brief  Initialization of device management service
* @param  None.
* @return None.
*/
void Service_Devices_Init(void)
{
	xTaskCreate(Device_SDcard,    "Device.SDcard",   	Normal_Stack_Size, 	NULL, 	PriorityRealtime, 			&DeviceSDcard_Handle);
	xTaskCreate(Device_IMU,       "Device.IMU",       Normal_Stack_Size, 	NULL, 	PriorityHigh, 			&DeviceIMU_Handle		);
	xTaskCreate(Device_Camera,    "Device.Cemara",   	Normal_Stack_Size, 	NULL, 	PriorityHigh, 				&DeviceCamera_Handle);
//	xTaskCreate(Device_BMx,       "Device.BMx",       Normal_Stack_Size, NULL, PrioritySuperHigh, &DeviceBMP_Handle);
//	xTaskCreate(Device_FLASH,     "Device.FLASH",     Normal_Stack_Size, NULL, PriorityHigh, &DeviceFLASH_Handle);	//��������FLASH��д
}

/**
* @brief  Task for IMU.
*/
void Device_IMU(void *arg)
{
  /* Cache for Task */
  TickType_t xLastWakeTime_t;	
	
	/* IMU  Init */
	if(!imu.Init(IMU_GPIO_Port, IMU_SCL_Pin, IMU_SDA_Pin))
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		
  /* Pre-Load for task */
  xLastWakeTime_t = xTaskGetTickCount();
	
  /* Infinite loop */
  for(;;)
  {
		/* IMU�����ݸ��� */
		imu.Update();
		
		/* ���Ͷ�ֵ�ź���,����IMU���ݸ��� */
		xSemaphoreGive(IMU_SemaphoreHandle);
			
    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t,UPDATE_TIME);
  }
}

/**
* @brief  Task for Cemara.
*/
void Device_Camera(void *arg)
{
	/* MT9V032 Init */
	if(!mt9v032.Init(&huart2, &hdcmi))
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);	
	
  /* Infinite loop */
  for(;;)
  {
		/* �ȴ�֪ͨ */
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		
		/* ���Ͷ�ֵ�ź���,����������ݸ��� */
		xSemaphoreGive(Camera_SemaphoreHandle);
  }
}

/**
* @brief  Task for SDcard.
*/
void Device_SDcard(void *arg)
{
	/* SDcard Init */
	if(!SDcard.Init())
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	
	/* ���ݴ洢����ʼ�� */
	data_dealer.Init();
	
	/* test */
	
	for(;;)
	{
//		data_dealer.View_Condition(25);
//		/* Delete task */ 
//		vTaskDelay(10);
		vTaskDelete(DeviceSDcard_Handle);	
	}	
}

/**
* @brief  Task for FLASH.
*/
void Device_FLASH(void *arg)
{
	 /* Cache for Task */
  TickType_t xLastWakeTime_t;	
	
	/* Pre-Load for task */
  xLastWakeTime_t = xTaskGetTickCount();
	for(;;)
	{
	
    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t,100);		
	}
}

/**
* @brief  Task for BMx.
*/
void Device_BMx(void *arg)
{
	double vel;
	
	/* BME280 Init */
	bme280.Init(BMP_GPIO_Port, BMP_SCL_Pin, BMP_SDA_Pin);	
	
	for(;;)
	{
		/* �ȴ�IMU״̬���� */
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY); 
		
		/* ���������ٶ� */
		vel = sqrt(pow(nav.status.speed[0], 2) + pow(nav.status.speed[1], 2) + pow(nav.status.speed[2], 2));
		
		/* ����BME280��״̬ */
		bme280.Update_Data();
		bme280.height = bme280.Bernoulli_Height(vel, bme280.data.pressure);
	}
}

/*=======================================================================================================*/
/** @brief      ����ͷ֡�ж��¼��ص�����
	* @param[in]  hdcmi DCMI���
 	* @return     void
	*/
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
	
	mt9v032.DCMI_Frame_Callback(hdcmi);
	
	/* ����֪ͨ */
	vTaskNotifyGiveFromISR(DeviceCamera_Handle, &xHigherPriorityTaskWoken);
	
	/* ���Ϊ pdTRUE�������һ���������л� */ 
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); 
}
/*=======================================================================================================*/

/* User Code End Here ---------------------------------*/

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
