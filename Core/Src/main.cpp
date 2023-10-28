/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <iostream>
#include "string.h"
#include "motor_controller.h"
#include "pid_controller.h"
#include "encoder.h"
#include "SimulatedEncoder.h"
#include "constants.h"
#include "timer_initialize.h"
#include <sstream>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
using namespace std;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

HAL_StatusTypeDef ret;

//instance of timers
Timer_initialize timINIT;


/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void delay(uint32_t ms);
void checkmsg(const uint8_t* data);
void parsemsg(const uint8_t* data);
void overfull_err();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Create an instance of the Encoder class
Encoder encoder(&timINIT.htim4, ENCODER_M1_A_PORT, ENCODER_M1_A_PIN, ENCODER_M1_B_PORT, ENCODER_M1_B_PIN);

// Create an instance of the Motor Controller
MotorController motorController(&timINIT.htim4, MOTOR_TIM_A,
MOTOR_ENA1_PORT, MOTOR_ENA1_PIN, // Replace X with the actual GPIO port and pin for PWM
MAX_OUTPUT, DEFAULT_MAX_INTEGRAL, DEFAULT_TARGET_START,
true);

// PID controller instance
PIDController pid_controller(POS_KP, POS_KI, POS_KD, MAX_OUT, DEFAULT_MAX_INTEGRAL, DEFAULT_TARGET_START);


// Simulated encoder instance for testing
SimulatedEncoder SimulatedEncoder(0);
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
// constants for PID_controller and buffer
bool bFreewheel = true, bMotorstop=true;
float dKp = 0.0000f, dKi = 0.0000f, dKd = 0.0000f, dSetpoint = 0.0f, dSetpoint_new = 0.0f;

//Needed for GPIO ENCODER
//int iEncCount = 4;
//float fEncpos = 5;

int uartRxIndex = 0;

//debug variable
bool debug = false;

float currentPosition =0;
float error = -1;

const char* exampleMsg = "X1, 12.12, 13.13, 14.14, 20.0,\n"; // which will be recieved
//const char* exampleMsg2 = "0, 0.0, 0.0, 0.0, 0.0, \n"; // which will be recieved
//const char* rxSeperator = "-----------------------\n";

//TX-RX Buffer, the size is the same
uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
uint8_t uartTxBuffer[UART_RX_BUFFER_SIZE];

//Flag for new Data
volatile bool newDataReceived = false;
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // Check if data has been received
    if (huart == &huart3) {
        // Data has been received, store it in the buffer
        uartRxBuffer[uartRxIndex++] = huart3.Instance->RDR;
        newDataReceived = true;
        // Restart UART receive
        HAL_UART_Receive_IT(&huart3, uartRxBuffer, 1);
    }
}

/* USER CODE END 2 */

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

  /* USER CODE BEGIN Init */
    HAL_Init();

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */
    SystemClock_Config();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
    MX_GPIO_Init();
    MX_ETH_Init();
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_PCD_Init();
    MX_I2C1_Init();
    MX_TIM4_Init();
    HAL_NVIC_EnableIRQ(USART3_IRQn); // Enable USART3 global interrupt
    encoder.init();
    //encoder_simulation = true;
    //encoder_simulation = false;

    /* Bootup  check */
    if (debug) {
        HAL_UART_Transmit(&huart3, (uint8_t *) "\n**************************************\n", 40, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart3, (uint8_t *) "BOOTING procedure successfull\n", 30, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart3, (uint8_t *) "**************************************\n", 40, HAL_MAX_DELAY);

        // Enable the UART receive interrupt
        const char *msgTest = " Testing parser \n";
        HAL_UART_Transmit(&huart3, (uint8_t *) msgTest, strlen(msgTest), HAL_MAX_DELAY);
        checkmsg((const uint8_t *) exampleMsg);
        //checkmsg((const uint8_t *) exampleMsg2);

        const char *msgTest2 = " Parser initialization done \n";
        HAL_UART_Transmit(&huart3, (uint8_t *) msgTest2, strlen(msgTest2), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart3, (uint8_t *) "starting pwm setting up timer\n", 31, HAL_MAX_DELAY);
    }
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    HAL_UART_Receive_IT(&huart3, uartRxBuffer, 1);

    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
    //HAL_UART_Transmit(&huart3, (uint8_t *) "BOOTING procedure successfull\n", 30, HAL_MAX_DELAY);
    //HAL_UART_Transmit(&huart3, (uint8_t *) "X\n", 30, HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        //check for new data arrived from GUI
        if (newDataReceived) {
            //checkmsg(uartRxBuffer);
            parsemsg(uartRxBuffer);
            //HAL_UART_Transmit(&huart3,uartRxBuffer, UART_RX_BUFFER_SIZE, HAL_MAX_DELAY);
            newDataReceived = false;
        }

        //check if a new Setpoint is set
        if (dSetpoint != dSetpoint_new) {
            dSetpoint_new = dSetpoint;
            //set the Target
            motorController.set_target(dSetpoint);
        }

        //BEGIN UPDATE
        //Update actual position via encoder
        currentPosition = encoder.get_positionI2C();
        // update motorcontroller with new values
        if (!bMotorstop){
            motorController.update(EXP_DT, currentPosition, bFreewheel, dKp, dKi, dKd);
        }
        else{
            motorController.set_direction(true, true);
        }
        //END UPDATE

        //BEGIN SEND
        //Send data to GUI
        stringstream ss;
        ss << bFreewheel << "," << dKp << "," << dKi << "," << dKd << "," << dSetpoint << "," << bMotorstop << "," << currentPosition  << "," << "\n";
        // Copy the formatted string to the uartTxBuffer
        string formattedString = ss.str();
        memcpy(uartTxBuffer, formattedString.c_str(), formattedString.length());
        HAL_UART_Transmit(&huart3, uartTxBuffer, UART_RX_BUFFER_SIZE, HAL_MAX_DELAY);
        HAL_Delay(20);
        // END SEND
    }
  /* USER CODE END WHILE */
}
/**
 * @brief parser
 * @param data = recieved data
 */
void parsemsg(const uint8_t* data) {
    char *endptr;
    char dataCopy[UART_RX_BUFFER_SIZE]; // Make a copy of the data to avoid modifying the original

    // Copy the received data to a mutable buffer
    strcpy(dataCopy, reinterpret_cast<const char *>(data));

    // Use strtok to split the string into individual substrings
    //bNewData
    char *token = strtok(dataCopy, ",");
    //token = strtok(dataCopy, ",");
    if (token != nullptr) {
        if (token[0] == 'X' || token[0] == ' ') {
            // Handle the case where the token is 'X' or a space
            token++;
        }
        int temp = strtol(token, &endptr, 10);
        // Check if the value is either 0 or 1
        if (temp == 1) {
            bFreewheel = true;
        } else {
            bFreewheel = false;
        }
    }

    //dkp
    token = strtok(nullptr, ",");
    if (token != nullptr) {
        dKp = strtof(token, &endptr);
    }

    //dki
    token = strtok(nullptr, ",");
    if (token != nullptr) {
        dKi = strtof(token, &endptr);
    }

    //dkd
    token = strtok(nullptr, ",");
    if (token != nullptr) {
        dKd = strtof(token, &endptr);
    }

    //Setpoint
    token = strtok(nullptr, ",");
    if (token != nullptr) {
        dSetpoint = strtof(token, &endptr);
    }
    //Motor start
    token = strtok(nullptr, ",");
    if (token != nullptr) {
        bMotorstop = strtof(token, &endptr);
    }

    memset(uartRxBuffer,0,1);
    uartRxIndex=0;
}

void checkmsg(const uint8_t* data)
{
    const char *c_data = (const char*)data;
    // Parse the data into variables
    uint8_t tx_buffer[UART_RX_BUFFER_SIZE];
    int rawCount = snprintf(reinterpret_cast<char *>(tx_buffer), sizeof(tx_buffer),"raw recieved: %s\n",c_data);
    {HAL_UART_Transmit(&huart3,tx_buffer , rawCount, HAL_MAX_DELAY);}

    int count = snprintf(reinterpret_cast<char *>(tx_buffer), sizeof(tx_buffer), "before calling parser data:\n"
                                                                                 "bFreewheel: %d\n"
                                                                                 "dKp: %.2f\n"
                                                                                 "dKi: %.2f\n"
                                                                                 "dKd: %.2f\n"
                                                                                 "dSetpoint: %.2f\n\n\n",
                         bFreewheel, dKp, dKi, dKd, dSetpoint);

    {HAL_UART_Transmit(&huart3, tx_buffer, count, HAL_MAX_DELAY);}

    parsemsg(data);

    count = snprintf(reinterpret_cast<char *>(tx_buffer), sizeof(tx_buffer), "parsed data:\n"
                                                                             "bFreewheel: %d\n"
                                                                             "dKp: %.2f\n"
                                                                             "dKi: %.2f\n"
                                                                             "dKd: %.2f\n"
                                                                             "dSetpoint: %.2f\n\n\n",
                     bFreewheel, dKp, dKi, dKd, dSetpoint);

    {HAL_UART_Transmit(&huart3, tx_buffer, count, HAL_MAX_DELAY);}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.Timing = 0x20404768;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_2|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin PB2 LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|GPIO_PIN_2|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : PB2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PD12 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
