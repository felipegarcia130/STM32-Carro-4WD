/* USER CODE BEGIN Header */
/**
 *IMPLEMENTACION
  **********
  * @file           : main.c
  * @brief          : Main program body
  **********
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **********
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "my_nrf24.h"
#include <stdbool.h>
#include <mpu6050.h>
#include "nRF24L01.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0 */
#define Nrf_CE GPIO_PIN_6  // Pin PC6 para NRF_CE
#define Nrf_CSn GPIO_PIN_7 // Pin PC7 para NRF_CSN
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// Periféricos
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef hi2c4;
SPI_HandleTypeDef hspi5;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// NRF24L01
const uint64_t pipe_address = 0x11223344AALL;
char myRxDataNRF[50];
uint8_t bufferNRF[6];
char myAckPayload[32] = "Ack by STMF7!";
uint16_t coordX = 0;
uint16_t coordY = 0;
int16_t angle = 0;
uint16_t targetX = 100;
uint16_t targetY = 75;

// CAN
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxDataCAN[8] = {0};
uint8_t RxDataCAN[8];
uint8_t buzzer = 0; // Estado del buzzer
uint8_t Vuel = 0;
float vueltas = 0.0f;
bool direccion = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_FDCAN1_Init(void);

/* USER CODE BEGIN PFP */

// Configuración y manejo del MPU6050
//void SetupMPU(void);

// Configuración y manejo del NRF24L01
void SetupNRF24(void);
void ProcessNRF24Data(void);

// Configuración y manejo del protocolo CAN
void ProcessCANMessage(void);
void SendCANMessage(uint8_t state);

// Control del motor y dirección
void SetupPWM(void);
void SetMotorSpeed(uint16_t pulseWidth);
void BrakeMotor(void);
void Turning_SetAngle(uint16_t angle);

// Rutinas de control
void RutaHardCoding(void);
void NRF24Motor(void);
void NRFMotorCAN(void);
void RPMCAN(void);
void RutaRPM(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
 * @brief Configuración inicial del MPU6050
 */
void SetupMPU(void) {
    mpu6050_init(&hi2c4, AD0_LOW, AFSR_2G, GFSR_250DPS, 0.98, 0.004);
//    MPU_calibrateAccel(&hi2c4, 1000); // Calibrar acelerómetro
//    MPU_calibrateGyro(&hi2c4, 1000);  // Calibrar giroscopio
}

/**
 * @brief Configuración inicial del NRF24L01
 */
void SetupNRF24(){
	NRF24_begin(GPIOC, Nrf_CSn_Pin, Nrf_CE_Pin, hspi5);
	nrf24_DebugUART_Init(huart3);
	NRF24_openReadingPipe(1, pipe_address);
	NRF24_setAutoAck(false);
	NRF24_setChannel(60);
	NRF24_setPayloadSize(32);
	NRF24_setDataRate(RF24_2MBPS);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();
	printRadioSettings();
	NRF24_startListening();
	printf("Starting NRF24\r\n");
}


/**
 * @brief Procesa los datos recibidos por NRF24L01
 */
void ProcessNRF24Data() {
    char myRxData[50] = "";
    if (NRF24_available()) {
        NRF24_read(bufferNRF, 6);
        if (bufferNRF[0] == 0xFF && bufferNRF[1] == 0xFF) {
            snprintf(myRxData, 32, "Coordinates not found\r\n");
        }
        else {
        	coordX = (bufferNRF[0]<<8|bufferNRF[1]);
        	coordY = (bufferNRF[2]<<8|bufferNRF[3]);
        	angle  = (int16_t)(bufferNRF[4]<<8|bufferNRF[5]);
        	if (angle > 180) angle -= 360;
        	snprintf(myRxData, 32, "%u %u %d \r\n", coordX, coordY, angle);
        }
        NRF24_writeAckPayload(1, myAckPayload, 32);
        HAL_UART_Transmit(&huart3, (uint8_t *)myRxData, strlen(myRxData), 10);
    }
}

/**
 * @brief Procesa los mensajes recibidos por el protocolo CAN
 */
void ProcessCANMessage() {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxDataCAN) == HAL_OK) {
        //printf("\n\rSTM32 - Mensaje CAN recibido:\n\r");
        //printf("ID: 0x%lx, Longitud: %ld bytes\n\r", RxHeader.Identifier, RxHeader.DataLength >> 16);
        //printf("Datos: ");

        if (RxHeader.Identifier == 0x100) {
            //printf("\n\rMensaje recibido de Arduino: ");
        }
        for (uint8_t i = 0; i < 8; i++) {
            //printf("%02X ", RxDataCAN[i]);
        }

        if (RxDataCAN[4]==0x01){
        	buzzer = 1;
        }else{
        	buzzer = 0;
        }

        // Interpretar las vueltas como un float
		union {
			float valueFloat;
			uint8_t valueInt[4];
		} conversion;

		conversion.valueInt[0] = RxDataCAN[0];
		conversion.valueInt[1] = RxDataCAN[1];
		conversion.valueInt[2] = RxDataCAN[2];
		conversion.valueInt[3] = RxDataCAN[3];

		vueltas = conversion.valueFloat;

        //printf("\n\rSTM32 - Mensaje recibido correctamente.\n\r");

        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin); // Indicar recepción
        //printf("Vueltas: %.4f\n\r", vueltas);
    } else {
        //printf("Esperando mensajes CAN...\n\r");
    }
}

/**
 * @brief Envia los mensajes por protocolo CAN
 */
void SendCANMessage(uint8_t state){
	TxDataCAN[0] = state;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxDataCAN) != HAL_OK) {
	        //printf("Error enviando mensaje CAN\r\n");
	    } else {

	    	//printf("Mensaje CAN enviado: Buzzer %s\r\n", state ? "Activado" : "Apagado");
	    }

    //printf("Contenido de TxDataCAN: ");
    for (int i = 0; i < 8; i++) { //  Asumir que el tamaño del mensaje es 8 bytes
        //printf("0x%02X ", TxDataCAN[i]);
    }
    //printf("\n\r");

}

/**
 * @brief Configuración inicial del PWM
 */
void SetupPWM(void) {
    // Iniciar el PWM en el canal 1 de TIM13
    if (HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler(); // Error al iniciar PWM en TIM13
    }

    // Iniciar el PWM en el canal 1 de TIM14
    if (HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler(); // Error al iniciar PWM en TIM14
    }
}

/**
 * @brief Ajusta la velocidad del motor
 */
void SetMotorSpeed(uint16_t pulseWidth) {
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pulseWidth);
}

/**
 * @brief Frena el motor
 */
void BrakeMotor(void) {
    SetMotorSpeed(1500);
}

/**
 * @brief Ajusta el ángulo de dirección
 * Limites:
 *  	Derecha:170
 *  	Izquierda: 10
 *  	Centro: 90
 */
void Turning_SetAngle(uint16_t angle) {
    uint32_t pulseWidth = 1000 + (angle*1000) / 180;

    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, pulseWidth);
}

/**
 * @brief Rutina codificada para probar segmentos de ruta
 */
void RutaHardCoding(){
	BrakeMotor();
	printf("primer segmento\n");
	Turning_SetAngle(90);
	SetMotorSpeed(1220); 		 // Velocidad primer segmento
	HAL_Delay(2000);	  		 // Tiempo primer segmento

	BrakeMotor();
	for (int i = 0; i < 100; i++){
		ProcessCANMessage();
	}
	HAL_Delay(5000);

	printf("primera vuelta\n");
	Turning_SetAngle(160);

	// Tiempo total: 10.5 segundos
	const int totalTime = 2000;   // Tiempo total en milisegundos
	const int sectionTime = 1000; // Duración de cada sección (2 segundos)
	const int numSections = totalTime / sectionTime; // Número de secciones

	for (int i = 0; i < numSections; i++) {
		SetMotorSpeed(1000);
		HAL_Delay(sectionTime); // Ejecutar sección
		SetMotorSpeed(1500);       // Detener motor durante el tiempo de espera
		for (int i = 0; i < 100; i++){
			ProcessCANMessage();
		}
		HAL_Delay(5000);    // Espera
	}

	printf("segundo segmento\n");
	Turning_SetAngle(90);
	SetMotorSpeed(1220);		 // Velocidad segundo segmento
	HAL_Delay(1500);	// Tiempo segundo segmento

	for (int i = 0; i < 100; i++){
		ProcessCANMessage();
	}

	SetMotorSpeed(1500);
	HAL_Delay(10000);
}


/**
 * @brief Rutina codificada para probar segmentos de ruta
 */
void NRFMotorCAN(){

	ProcessNRF24Data();
	ProcessCANMessage();
//	RutaRPM();

	if((abs(coordX - targetX) <= 10 && abs(coordY - targetY) <= 10) || vueltas > 4){
		BrakeMotor();
		SendCANMessage(1);
		while(buzzer == 0){
			ProcessCANMessage();
			ProcessNRF24Data();
			//printf("ciclo while");
		}
		SendCANMessage(0);
	}else{
		SendCANMessage(0);
		double targetAngle = atan2(targetY - coordY, targetX - coordX);
		targetAngle = round(targetAngle * 180.0 / M_PI); // Convertimos radianes a grados

		//printf("Target Angle: %lf degrees\n", targetAngle);

		double angleError = targetAngle - (double)(angle);

		//printf("Angle Error: %lf degrees\n", angleError);

		targetAngle = 90 - angleError;
		if(targetAngle > 150) targetAngle = 150;
		if(targetAngle < 30) targetAngle = 30;

		// Ajusta el ángulo del carro

		Turning_SetAngle(targetAngle);

		SetMotorSpeed(1220);

		//printf("Servo Angle: %lf degrees\n", targetAngle);
		//MPU_readProcessedData(&hi2c4);

	}
}
//RUTA FINAL
void NRFMotorEncoderCAN(){
	uint8_t segmento = 0;
	uint8_t state = 0;
	float posiciones[] = {
	        1,   // Primer avance
	        5,   // Primera vuelta
	        8.5,   // Segunda vuelta
	        11  // Segundo avance
	};

	uint8_t targetsX[] = {230,155,125,125};
	uint8_t targetsY[] = {75,25,75,120};

	while (segmento < 5){
		ProcessNRF24Data();
		ProcessCANMessage();
		if((abs(coordX - targetsX[segmento]) <= 10 && abs(coordY - targetsY[segmento]) <= 10) || vueltas >= posiciones[segmento]){
			BrakeMotor();
			if (state == 0){
				state = 1;
				SendCANMessage(state);
			}
			while(buzzer == 0){
				ProcessCANMessage();
			}
			HAL_Delay(1000);
			ProcessNRF24Data();
			HAL_Delay(1000);
			segmento++;
			if (state == 1){
				state = 0;
				SendCANMessage(state);
			}
		}else{
			if (state == 1){
				state = 0;
				SendCANMessage(state);
			}
			double targetAngle = atan2(targetsY[segmento] - coordY, targetsX[segmento] - coordX);
			targetAngle = round(targetAngle * 180.0 / M_PI); // Convertimos radianes a grados

			double angleError = targetAngle - (double)(angle);

			targetAngle = 90 - angleError;
			if(targetAngle > 160) targetAngle = 160;
			if(targetAngle < 20) targetAngle = 20;

			Turning_SetAngle(targetAngle);
			if (targetAngle <= 60 || targetAngle >= 120 ){
				SetMotorSpeed(1100);
			}else{
				SetMotorSpeed(1300);
			}

		}
	}
}

void MotorEncoderCAN(){
	uint8_t segmento = 0;
	float posiciones[] = {
	        1,   // Primer avance
	        5,   // Primera vuelta
	        8.5,   // Segunda vuelta
	        11,
			15,
			18.5,
			21
	};

	uint8_t state = 0;

	while (segmento < 4){
		ProcessCANMessage();
		printf("Segmento: %u\r\n", segmento);
		if(vueltas >= posiciones[segmento]){
			BrakeMotor();
			if (state == 0){
				state = 1;
				SendCANMessage(state);
			}
			while(buzzer == 0){
				ProcessCANMessage();
			}
			HAL_Delay(3000);
			segmento++;
			if (state == 1){
				state = 0;
				SendCANMessage(state);
			}
		}else{
			if (state == 1){
				state = 0;
				SendCANMessage(state);
			}
			if(segmento == 1 || segmento == 2){
				Turning_SetAngle(160);
				SetMotorSpeed(1100);
				printf("Vuelta\r\n");
			} else {
				Turning_SetAngle(90);
				SetMotorSpeed(1300);
				printf("Recto\r\n");
			}

		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
//HW semaphore Clock enable/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
//Release HSEM in order to notify the CPU2(CM4)/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_I2C4_Init();
  MX_SPI5_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_FDCAN1_Init();

  FDCAN_FilterTypeDef filterConfig;
  filterConfig.IdType = FDCAN_STANDARD_ID;
  filterConfig.FilterIndex = 0;
  filterConfig.FilterType = FDCAN_FILTER_MASK;
  filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filterConfig.FilterID1 = 0x100;   // ID exacto que queremos aceptar
  filterConfig.FilterID2 = 0x7FF;   // Mascara para aceptar solo el ID exacto
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filterConfig) != HAL_OK) {
      Error_Handler();
  }

  printf("Welcome to the CANbus and Motor Control System!!!\n\r");
  SetupNRF24();
  SetupPWM();

//  NRFMotorCAN();
//  SetupMPU();
  //RutaHardCoding();
  //MotorEncoderCAN();
  HAL_Delay(5000);
  NRFMotorEncoderCAN();

  while (1){
	  // Probar las rutinas

	  //NRFMotorCAN();
	  //RutaHardCoding();

//	  HAL_Delay(100);
//	  ProcessNRF24Data();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 24;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 0x1F;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  // PARTE DE CAN ---------------------------------------------------------------------------

  //AAO+/
   /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;

    /* Configure global filter to reject all non-matching frames */
      HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

      if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
        {
           /* Filter configuration Error */
           Error_Handler();
        }
       /* Start the FDCAN module */
      if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        }
           /* Start Error */
      if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        }
           /* Notification Error */

       /* Configure Tx buffer message */
      TxHeader.Identifier = 0x111;
      TxHeader.IdType = FDCAN_STANDARD_ID;
      TxHeader.TxFrameType = FDCAN_DATA_FRAME;
      TxHeader.DataLength = FDCAN_DLC_BYTES_12;
      TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
      TxHeader.BitRateSwitch = FDCAN_BRS_ON;
      TxHeader.FDFormat = FDCAN_FD_CAN;
      TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
      TxHeader.MessageMarker = 0x00;
     //AAO-/
// PARTE DE CAN -----------------------------------------------------------------------------

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00404C74;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x0;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 239;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 19999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 239;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 19999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Nrf_CE_Pin|Nrf_CSn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Nrf_CE_Pin Nrf_CSn_Pin */
  GPIO_InitStruct.Pin = Nrf_CE_Pin|Nrf_CSn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
