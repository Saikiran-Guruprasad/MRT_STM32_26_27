/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CANSPI.h"
#include "MCP2515.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO_AD (0x28 << 1)
#define AS5600_AD (0x36 << 1)

#define ANGLE_REG 0x0E
#define ANGLE_REG_2 0x0F
#define AS_STATUS_REG 0x0B
#define ANG_VEL 0x14
#define BN_STATUS_REG 0x39

#define RPI 0
#define FR 1
#define FL 2
#define BR 3
#define BL 4

//#define EXPLICIT_QUAD  0b00000
//#define EXPLICIT_MAGN  0b00001
//#define EXPLICIT_IMU   0b00010
//#define EXPLICIT_ACS   0b00011
//#define DRIVE_QUAD     0b00100
//#define DRIVE_ACS      0b00101
//#define EXPLICIT_PWM   0b00110
//#define DRIVE_PWM      0b00111
//#define EXPLICIT_DIRN  0b01000
//#define DRIVE_DIRN     0b01001
//#define STATUS         0b01010
//#define HEARTBEAT      0b11111

#define MOTOR_DATA 0b00010
#define SENSOR_DATA 0b00011
#define SENSORS_CHECK 0b00001

#define HEARTBEAT 0b00000

#define EXPLICIT_PWM 0b10001
#define DRIVE_PWM 0b10010
#define START_NODE 0b10000

#define MAKE_ARBITRATION_ID(node, msg) (((node) << 5) | (msg))

#define CAN_FAIL 10000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
typedef enum {
    I2C_IDLE,
    I2C_AS_STATUS,
    I2C_AS_ANGLE,
    I2C_BNO_STATUS,
    I2C_BNO_ANGVEL,
    I2C_DONE
} I2C_SeqState;

volatile I2C_SeqState i2c_state = I2C_IDLE;
uint8_t status_as, status_bn;
uint8_t angle[2], angvel[2];

uint8_t status_as_buf, status_bn_buf;
uint8_t angle_buf[2], angvel_buf[2];
volatile HAL_StatusTypeDef s1, s2, s3, s4;

uint16_t adc_buf[2];

uint16_t enc1_count;
uint16_t enc2_count;

uCAN_MSG tx_motor;
uCAN_MSG tx_sensor;
uCAN_MSG tx_sensor_check;

uCAN_MSG tx_heartbeat;

uCAN_MSG rx;

uint8_t timer = 0;
static volatile uint32_t counter = 0;
static volatile uint8_t heartbeater = 0;
static volatile uint32_t CAN_failed_counter = 0;
volatile bool beat_pls = false;
volatile bool start_node = false;
volatile bool CAN_checker = false;

uint16_t nm[2];

//void decode_id(uint16_t arb_id){
//	uint16_t node = (arb_id >> 5) & 0b00111111;
//	uint16_t msg = arb_id & 0b00011111;
//	nm[0] = node;
//	nm[1] = msg;
//}
//
//uint16_t cast_to_arbid(uint16_t id){
// return (id & 0b11111111111);
//}

uint16_t exp_pwm_msg = MAKE_ARBITRATION_ID(FR,EXPLICIT_PWM);
uint16_t dr_pwm_msg = MAKE_ARBITRATION_ID(FR,DRIVE_PWM);
uint16_t start_node_msg = MAKE_ARBITRATION_ID(FR,START_NODE);
//
//uint16_t exp_dirn_msg = MAKE_ARBITRATION_ID(FR, EXPLICIT_DIRN);
//uint16_t dr_dirn_msg = MAKE_ARBITRATION_ID(FR, DRIVE_DIRN);
//
//uint16_t exp_acs_msg = MAKE_ARBITRATION_ID(FR,EXPLICIT_ACS);
//uint16_t dr_acs_msg = MAKE_ARBITRATION_ID(FR, DRIVE_ACS);
//
//uint16_t exp_quad_msg = MAKE_ARBITRATION_ID(FR,EXPLICIT_QUAD);
//uint16_t dr_quad_msg = MAKE_ARBITRATION_ID(FR, DRIVE_QUAD);
//
//uint16_t imu_msg = MAKE_ARBITRATION_ID(FR,EXPLICIT_IMU);
//uint16_t mag_msg = MAKE_ARBITRATION_ID(FR,EXPLICIT_MAGN);
//
//uint16_t status_msg = MAKE_ARBITRATION_ID(FR, STATUS);

uint16_t motor_msg = MAKE_ARBITRATION_ID(FR, MOTOR_DATA);
uint16_t sensor_msg = MAKE_ARBITRATION_ID(FR, SENSOR_DATA);
uint16_t sensor_check_msg = MAKE_ARBITRATION_ID(FR, SENSORS_CHECK);

uint16_t heartbeat_msg = MAKE_ARBITRATION_ID(FR, HEARTBEAT);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void I2C1_Sequence_Start(void) {
    i2c_state = I2C_AS_STATUS;
    s1 = HAL_I2C_Mem_Read_DMA(&hi2c1, AS5600_AD, AS_STATUS_REG, I2C_MEMADD_SIZE_8BIT, &status_as_buf, 1);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C1) return;

    switch(i2c_state){
        case I2C_AS_STATUS:
            i2c_state = I2C_AS_ANGLE;
            s3 = HAL_I2C_Mem_Read_DMA(&hi2c1, AS5600_AD, ANGLE_REG, I2C_MEMADD_SIZE_8BIT, angle_buf, 2);
            break;
        case I2C_AS_ANGLE:
            i2c_state = I2C_BNO_STATUS;
            s2 = HAL_I2C_Mem_Read_DMA(&hi2c1, BNO_AD, BN_STATUS_REG, I2C_MEMADD_SIZE_8BIT, &status_bn_buf, 1);
            break;
        case I2C_BNO_STATUS:
            i2c_state = I2C_BNO_ANGVEL;
            s4 = HAL_I2C_Mem_Read_DMA(&hi2c1, BNO_AD, ANG_VEL, I2C_MEMADD_SIZE_8BIT, angvel_buf, 2);
            break;
        case I2C_BNO_ANGVEL:
            i2c_state = I2C_DONE;
            status_as = status_as_buf;
            status_bn = status_bn_buf;
            angle[0] = angle_buf[0];
            angle[1] = angle_buf[1];
            angvel[0] = angvel_buf[0];
            angvel[1] = angvel_buf[1];
            break;
        default: break;
    }
}

void CAN_Send_MotorFrame(void){
	enc1_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
	enc2_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

    tx_motor.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    tx_motor.frame.id = motor_msg;
    tx_motor.frame.dlc = 8;
    tx_motor.frame.data0 = enc1_count & 0xFF;
    tx_motor.frame.data1 = (enc1_count >> 8) & 0xFF; //explicit encoder

    tx_motor.frame.data2 = enc2_count & 0xFF;
    tx_motor.frame.data3 = (enc2_count >> 8) & 0xFF; //drive encoder

    tx_motor.frame.data4 = adc_buf[0] & 0xFF;
    tx_motor.frame.data5 = (adc_buf[0] >> 8) & 0xFF; //explicit current

    tx_motor.frame.data6 = adc_buf[1] & 0xFF;
    tx_motor.frame.data7 = (adc_buf[1] >> 8) & 0xFF; //drive current

    CANSPI_Transmit(&tx_motor);

    tx_sensor.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    tx_sensor.frame.id = sensor_msg;
    tx_sensor.frame.dlc = 4;
	tx_sensor.frame.data0 = angle[0];  //LSB TO MSB
	tx_sensor.frame.data1 = angle[1];

	tx_sensor.frame.data2 = angvel[0]; //LSB TO MSB
	tx_sensor.frame.data3 = angvel[1];

	//tx_sensor.frame.data2 = status_as; //LSB TO MSG
    CANSPI_Transmit(&tx_sensor);

    if (beat_pls){
		tx_heartbeat.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		tx_heartbeat.frame.id = heartbeat_msg;
		tx_heartbeat.frame.dlc = 1;
		tx_heartbeat.frame.data0 = heartbeater;
		beat_pls = false;
		CANSPI_Transmit(&tx_heartbeat);
    }

//    tx_imu.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
//    tx_imu.frame.id = imu_msg;
//    tx_imu.frame.dlc = 2;
//	tx_imu.frame.data0 = angvel[0]; //LSB TO MSB
//	tx_imu.frame.data1 = angvel[1];
//	//tx_imu.frame.data2 = status_bn;
//    CANSPI_Transmit(&tx_imu);

//    tx_exp_acs.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
//    tx_exp_acs.frame.id = exp_acs_msg;
//    tx_exp_acs.frame.dlc = 2;
//    tx_exp_acs.frame.data0 = adc_buf[0] & 0xFF;
//    tx_exp_acs.frame.data1 = (adc_buf[0] >> 8) & 0xFF;
//    CANSPI_Transmit(&tx_exp_acs);

//    tx_dr_acs.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
//    tx_dr_acs.frame.id = dr_acs_msg;
//    tx_dr_acs.frame.dlc = 2;
//    tx_dr_acs.frame.data0 = adc_buf[1] & 0xFF;
//    tx_dr_acs.frame.data1 = (adc_buf[1] >> 8) & 0xFF;
//    CANSPI_Transmit(&tx_dr_acs);

//    tx_exp_quad.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
//    tx_exp_quad.frame.id = exp_quad_msg;
//    tx_exp_quad.frame.dlc = 2;
//    tx_exp_quad.frame.data0 = enc1_count & 0xFF;
//    tx_exp_quad.frame.data1 = (enc1_count >> 8) & 0xFF;
//    CANSPI_Transmit(&tx_exp_quad);

}

void CAN_Send_SensorCheck(void){
	tx_sensor_check.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	tx_sensor_check.frame.id = sensor_check_msg;
	tx_sensor_check.frame.dlc = 2;
	tx_sensor_check.frame.data0 = status_as;
	tx_sensor_check.frame.data1 = status_bn;
	CANSPI_Transmit(&tx_sensor_check);
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C1) return;
    i2c_state = I2C_DONE;   // force the main loop to retry from scratch next pass
}

//void status_check(){
//	tx_stat.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
//	tx_stat.frame.id = status_msg;
//	tx_stat.frame.dlc = HAL_I2C_Mem_Read_DMA(&hi2c1, AS5600_AD, ANGLE_REG, I2C_MEMADD_SIZE_8BIT, angle, 2);
//	//to check what kinds of status check we need to implement, as of now its hardwired into the code
//}

//int heartbeat(int a, int b){
//	return (a+b);
//}
void decode_id(uint16_t arb_id){
	uint16_t node = (arb_id >> 5) & 0b00111111;
	uint16_t msg = arb_id & 0b00011111;
	nm[0] = node;
	nm[1] = msg;
}

uint16_t cast_to_arbid(uint16_t id){
 return (id & 0b11111111111);
}

void delay_us (uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
  HAL_Delay(100);

  HAL_Delay(1000);
  CAN_checker = CANSPI_Initialize();
  HAL_Delay(1000);

  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
  HAL_Delay(100);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);

  uint8_t opr_mode = 0x0C;
  HAL_I2C_Mem_Write(&hi2c1, BNO_AD, 0x3D, I2C_MEMADD_SIZE_8BIT, &opr_mode, 1, 100);
  HAL_Delay(20);

  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
  HAL_Delay(100);

  HAL_TIM_Base_Start(&htim1);

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);

  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 150);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 150);

  HAL_Delay(2000);

  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);

  HAL_Delay(1000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      if (i2c_state == I2C_IDLE || i2c_state == I2C_DONE) {
    	  I2C1_Sequence_Start();
      }

      if (CAN_failed_counter>=10000){
    	  start_node = false;
      }

      if (!start_node){
    	  while(!CANSPI_Receive(&rx)){}
    	  if (rx.frame.id == start_node_msg){
    		  start_node = true;
    	  }
    	  else if (rx.frame.id == sensor_check_msg){
    		  CAN_Send_SensorCheck();
    	  }
//    	  else{
//    		  start_node = true;
//    	  }
      }

  	  if(timer==10){
  		  timer = 0;
  		  CAN_Send_MotorFrame();
  	  }
  	  if (counter==1000){
  		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
  		  counter = 0;
  	  }

  	  if (s1 == HAL_OK && s2 == HAL_OK && s3 == HAL_OK && s4 == HAL_OK){
  	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1); // I2C_debug
  	  }
  	  if (CAN_checker){
  	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1); // SPI_debug
  	  }

  	  if (CANSPI_Receive(&rx)){
  		  CAN_failed_counter=0;
  	      if (rx.frame.id == exp_pwm_msg){
  	          uint16_t pwm_exp = rx.frame.data1;
  	          int dir = rx.frame.data0;
  	          if (pwm_exp <= 255 && pwm_exp>=0){
  	        	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,dir);
  	        	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pwm_exp);
  	          }
  	      }
  	  	  else if (rx.frame.id == dr_pwm_msg){
  	          uint16_t pwm_dr = rx.frame.data1;
  	          int dir = rx.frame.data0;
  	          if (pwm_dr <= 255 && pwm_dr >= 0){
  	        	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,dir);
  	        	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pwm_dr);
  	          }
  	      }
  	  	  else if (rx.frame.id == heartbeat_msg){
  	  		  beat_pls = true;
  	  		  heartbeater = rx.frame.data0 + rx.frame.data1;
  	  	  }
  	  }

  	  timer++;
  	  counter++;
  	  CAN_failed_counter++;
  	  delay_us(100);
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 71;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|SPI_debug_Pin|I2C_debug_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR1_Pin|DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAN_CS_Pin|IMU_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 SPI_debug_Pin I2C_debug_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|SPI_debug_Pin|I2C_debug_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR1_Pin DIR2_Pin */
  GPIO_InitStruct.Pin = DIR1_Pin|DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_CS_Pin IMU_RST_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin|IMU_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

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
