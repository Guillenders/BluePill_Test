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
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
FATFS fs;
FIL fil;
FRESULT fres;
UINT bw;
char archivo_nombre[32];
uint8_t archivo_abierto = 0;  // Indica si el archivo está abierto
RTC_HandleTypeDef hrtc;

void crear_archivo();
void guardar_datos(uint8_t*);
void cerrar_archivo();
void Get_RTC_DateTime(char *datetime_str);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDR                    0x52    // I²C address of VL6180X shifted by 1 bit (0x29 << 1) so the R/W command can be added.
#define MIN_RANGE_MEAS 			14 		//Min measure = 50mm (para freq)
#define MAX_RANGE_MEAS 			25		//Max measure = 60mm (para freq)
#define MIN_FSR_PRESS_OK		1500	//min value of FSR for hands position
#define duty_sound				7.5     //duty cycle for buzzer 666Hz --> 7.5 = 50% DC.
#define min_battery_OK			4000    //valor minimo de ADC para lectura de bateria y dar un OK para realizar el ensayo.

///////////////////////////////////////////////////////////////////
// register addresses VL6180x(por ahora no los uso)
///////////////////////////////////////////////////////////////////
enum regAddr
    {
      IDENTIFICATION__MODEL_ID              = 0x000,
      IDENTIFICATION__MODEL_REV_MAJOR       = 0x001,
      IDENTIFICATION__MODEL_REV_MINOR       = 0x002,
      IDENTIFICATION__MODULE_REV_MAJOR      = 0x003,
      IDENTIFICATION__MODULE_REV_MINOR      = 0x004,
      IDENTIFICATION__DATE_HI               = 0x006,
      IDENTIFICATION__DATE_LO               = 0x007,
      IDENTIFICATION__TIME                  = 0x008, // 16-bit

      SYSTEM__MODE_GPIO0                    = 0x010,
      SYSTEM__MODE_GPIO1                    = 0x011,
      SYSTEM__HISTORY_CTRL                  = 0x012,
      SYSTEM__INTERRUPT_CONFIG_GPIO         = 0x014,
      SYSTEM__INTERRUPT_CLEAR               = 0x015,
      SYSTEM__FRESH_OUT_OF_RESET            = 0x016,
      SYSTEM__GROUPED_PARAMETER_HOLD        = 0x017,

      SYSRANGE__START                       = 0x018,
      SYSRANGE__THRESH_HIGH                 = 0x019,
      SYSRANGE__THRESH_LOW                  = 0x01A,
      SYSRANGE__INTERMEASUREMENT_PERIOD     = 0x01B,
      SYSRANGE__MAX_CONVERGENCE_TIME        = 0x01C,
      SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x01E, // 16-bit
      SYSRANGE__CROSSTALK_VALID_HEIGHT      = 0x021,
      SYSRANGE__EARLY_CONVERGENCE_ESTIMATE  = 0x022, // 16-bit
      SYSRANGE__PART_TO_PART_RANGE_OFFSET   = 0x024,
      SYSRANGE__RANGE_IGNORE_VALID_HEIGHT   = 0x025,
      SYSRANGE__RANGE_IGNORE_THRESHOLD      = 0x026, // 16-bit
      SYSRANGE__MAX_AMBIENT_LEVEL_MULT      = 0x02C,
      SYSRANGE__RANGE_CHECK_ENABLES         = 0x02D,
      SYSRANGE__VHV_RECALIBRATE             = 0x02E,
      SYSRANGE__VHV_REPEAT_RATE             = 0x031,

      SYSALS__START                         = 0x038,
      SYSALS__THRESH_HIGH                   = 0x03A,
      SYSALS__THRESH_LOW                    = 0x03C,
      SYSALS__INTERMEASUREMENT_PERIOD       = 0x03E,
      SYSALS__ANALOGUE_GAIN                 = 0x03F,
      SYSALS__INTEGRATION_PERIOD            = 0x040,

      RESULT__RANGE_STATUS                  = 0x04D,
      RESULT__ALS_STATUS                    = 0x04E,
      RESULT__INTERRUPT_STATUS_GPIO         = 0x04F,
      RESULT__ALS_VAL                       = 0x050, // 16-bit
      RESULT__HISTORY_BUFFER_0              = 0x052, // 16-bit
      RESULT__HISTORY_BUFFER_1              = 0x054, // 16-bit
      RESULT__HISTORY_BUFFER_2              = 0x056, // 16-bit
      RESULT__HISTORY_BUFFER_3              = 0x058, // 16-bit
      RESULT__HISTORY_BUFFER_4              = 0x05A, // 16-bit
      RESULT__HISTORY_BUFFER_5              = 0x05C, // 16-bit
      RESULT__HISTORY_BUFFER_6              = 0x05E, // 16-bit
      RESULT__HISTORY_BUFFER_7              = 0x060, // 16-bit
      RESULT__RANGE_VAL                     = 0x062,
      RESULT__RANGE_RAW                     = 0x064,
      RESULT__RANGE_RETURN_RATE             = 0x066, // 16-bit
      RESULT__RANGE_REFERENCE_RATE          = 0x068, // 16-bit
      RESULT__RANGE_RETURN_SIGNAL_COUNT     = 0x06C, // 32-bit
      RESULT__RANGE_REFERENCE_SIGNAL_COUNT  = 0x070, // 32-bit
      RESULT__RANGE_RETURN_AMB_COUNT        = 0x074, // 32-bit
      RESULT__RANGE_REFERENCE_AMB_COUNT     = 0x078, // 32-bit
      RESULT__RANGE_RETURN_CONV_TIME        = 0x07C, // 32-bit
      RESULT__RANGE_REFERENCE_CONV_TIME     = 0x080, // 32-bit

      RANGE_SCALER                          = 0x096, // 16-bit

      READOUT__AVERAGING_SAMPLE_PERIOD      = 0x10A,
      FIRMWARE__BOOTUP                      = 0x119,
      FIRMWARE__RESULT_SCALER               = 0x120,
      I2C_SLAVE__DEVICE_ADDRESS             = 0x212,
      INTERLEAVED_MODE__ENABLE              = 0x2A3,
    };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

int init = 1;
extern int volatile timer_1m; //flag para el timer de 1 minuto
extern int volatile timer_buzzer; //timer para activar el pwm del buzzer.
extern int volatile timer_startsound;
uint32_t last_compression_times[10] = {0}; // Ventana de tiempo de los últimos 2 segundos (10 mediciones de 200 ms)
uint8_t compression_index = 0;
int max_depth = 0;
void WriteByte(wchar_t, char);
char ReadByte(wchar_t);
int  VL6180X_Init();
int  VL6180X_Start_Range();
int  VL6180X_Poll_Range();
int  VL6180X_Read_Range();
int  VL6180X_Clear_Interrupts();
void startsound();
void buzzer();
int VL6180x_offset();
int FSR_offset();
int calculate_frequency();
status_TypeDef  FSR_Check(uint16_t);

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
	uint8_t MSG[100] = {'\0'};
	int DISTANCE_value = 0, distance_offsetok = 0, FSR_offsetok = 0, FSR_value = 0, battery_lvl = 0;
	int compression_count = 0, compression_active = 0;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  VL6180X_Init(); //load settings on to VL6180X.
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);// Calibrate The ADC On Power-Up For Better Accuracy
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //AGREGAR LECTURA DE BATERIA, SI ES OK INICIAR ENSAYO, SINO INDICAR QUE LA BATERIA EST�? BAJA (LUZ O SONIDO?)
	// if(timer_1m >= 0.001 && timer_1m <= 750){startsound();	 }
	//  if(timer_1m >= 2200 && timer_1m <= 62200){ //ensayo de 1 minuto

		  //start sound buzzer
		  buzzer();

		  // start single range measurement
		  VL6180X_Start_Range();

		  // poll the VL6180X till new sample ready
		  VL6180X_Poll_Range();

		  // Start ADC Conversion FOR FSR & Read batery lvl
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_Start(&hadc2);

		  // Poll ADC1&ADC2 Perihperal & TimeOut = 1mSec
		  HAL_ADC_PollForConversion(&hadc1, 1);
		  HAL_ADC_PollForConversion(&hadc2, 1);
		  battery_lvl = HAL_ADC_GetValue(&hadc1);



		  //funcion para obtener offset del sensor de distancia y FSR
		  if(init==1){
			  distance_offsetok = VL6180x_offset();
			  FSR_offsetok = FSR_offset();
		  }
		  init = 0;

		  //if (!archivo_abierto) {
		  	//			crear_archivo();
		  		//	}

		  // read range result in [mm] (AVG)
		  DISTANCE_value = 0, FSR_value = 0;
		  for(int a = 0; a < 10; a++){
			  VL6180X_Start_Range();
			  VL6180X_Poll_Range();
			  DISTANCE_value += VL6180X_Read_Range();
			  FSR_value += HAL_ADC_GetValue(&hadc2);
			 // HAL_Delay(1); //estaba en 2, probar y despues probar sacandolo.
		  }


		  DISTANCE_value = distance_offsetok - (DISTANCE_value/10); //calculo de distancia restando el offset.
		  FSR_value = (FSR_value/10) - FSR_offsetok; //calculo FSR menos el offset
		  if(DISTANCE_value < 0 ){DISTANCE_value = 0;} //esto es por si tira algun valor negativo (-2 x ej), pongo q directamente sea 0
		  if(FSR_value < 0 ){FSR_value = 0;}

		  // Detección de compresión
		            if (DISTANCE_value >= MIN_RANGE_MEAS && DISTANCE_value <= MAX_RANGE_MEAS) {
		                if (!compression_active) {
		                    compression_active = 1;
		                }
		            } else {
		                if (compression_active) {
		                    compression_active = 0; // Final de la compresión
		                    last_compression_times[compression_index] = HAL_GetTick();
		                    compression_index = (compression_index + 1) % 10;
		                    if (DISTANCE_value > max_depth) {
		                  	  max_depth = DISTANCE_value; // Actualizar máxima profundidad alcanzada
		                    }
		                    int frequency = calculate_frequency(); // Calcular frecuencia

		                    // Enviar datos al finalizar la compresión
		                    const char* hand_position = (FSR_Check(FSR_value) == PASS_OK) ? "OK" : "NOK";
		                    sprintf(MSG, "\nUBICACION= %s MAX PROFUNDIDAD= %d FRECUENCIA= %d Hz", hand_position, max_depth, frequency);
		                    HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);

		                    max_depth = 0; // Reiniciar profundidad máxima para la siguiente compresión
		                }
		            }
		            //guardar_datos(MSG);
		            VL6180X_Clear_Interrupts();
		        }

		        if (timer_1m > 62200) { // Termina ensayo
		            HAL_TIM_Base_Stop_IT(&htim3);
		            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		            cerrar_archivo();
		            timer_1m = 0;
		            init = 1;
		            compression_count = 0;
		        }
		    }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void crear_archivo() {


	fres = f_mount(&fs, "", 1);

	if (fres != FR_OK) {
		return;
	}

	char archivo_nombre[50];
	Get_RTC_DateTime(archivo_nombre);

	if (!archivo_abierto) {

		fres = f_open(&fil, archivo_nombre, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
		archivo_abierto= 1;

		if (fres != FR_OK) {
			// Si hay un error al abrir el archivo, salir del ciclo
			return;
		}

	}

}


void guardar_datos(uint8_t* datos) {
	// Verifica si el archivo está abierto
	if (archivo_abierto) {
		f_write(&fil, datos, sizeof(uint8_t) * 100, &bw);
	}
}

void cerrar_archivo() {
	if (archivo_abierto) {
		f_close(&fil);  // Cerrar el archivo
		archivo_abierto = 0;  // Marcamos que el archivo está cerrado

	}
}


void Get_RTC_DateTime(char *datetime_str) {
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	// Obtener la hora actual
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	// Obtener la fecha actual
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	// Formatear la fecha y la hora en una cadena
	sprintf(datetime_str, "Ensayo_%02d-%02d-%02d_%02d-%02d-%02d.txt",
			sDate.Year + 2000,  // Año
			sDate.Month,        // Mes
			sDate.Date,         // Día
			sTime.Hours,        // Hora
			sTime.Minutes,      // Minuto
			sTime.Seconds);     // Segundo
}










///////////////////////////////////////////////////////////////////
// ISR PIN_B1 PULL UP
// DESC.: START BUTTON.
///////////////////////////////////////////////////////////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_1){
		HAL_TIM_Base_Start_IT(&htim3); //START TIMER 1mS
	}
}

///////////////////////////////////////////////////////////////////
// Split 16-bit register address into two bytes and write
// the address + data via I²C
///////////////////////////////////////////////////////////////////

void WriteByte(wchar_t reg_addr, char data2wr) {
	uint8_t data_write[3];

	data_write[0] = (reg_addr >> 8) & 0xFF; // MSB of register address
	data_write[1] = reg_addr & 0xFF;       // LSB of register address
	data_write[2] = data2wr & 0xFF;
	HAL_I2C_Master_Transmit(&hi2c1, ADDR, data_write, 3, 1000);
}


///////////////////////////////////////////////////////////////////
// Split 16-bit register address into two bytes and write
// required register address to VL6180X and read the data back
///////////////////////////////////////////////////////////////////
char ReadByte(wchar_t reg) {
	uint8_t data_write[2];
	uint8_t data_read[1];

	data_write[0] = (reg >> 8) & 0xFF; // MSB of register address
	data_write[1] = reg & 0xFF;       // LSB of register address

	HAL_I2C_Master_Transmit(&hi2c1, ADDR, data_write, 2, 1000);
	HAL_I2C_Master_Receive(&hi2c1, ADDR, data_read, 1, 1000);
	return data_read[0];
}


///////////////////////////////////////////////////////////////////
//DESC.: load VL6180X settings
///////////////////////////////////////////////////////////////////

int VL6180X_Init() {

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET); //GPIO0 to logic ‘1’ allows the device to come out of reset.
	HAL_Delay(1);								//Necesita ese ms.
	uint8_t reset;

	reset = ReadByte(0x0016);
	if(reset != 1){
		WriteByte(0x0016, 0x01);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
	}
	WriteByte(0x0207, 0x01);
	WriteByte(0x0208, 0x01);
	WriteByte(0x0133, 0x01);
	WriteByte(0x0096, 0x00);
	WriteByte(0x0097, 0xFD); // RANGE_SCALER = 253 FD
	WriteByte(0x00e3, 0x00);
	WriteByte(0x00e4, 0x04);
	WriteByte(0x00e5, 0x02);
	WriteByte(0x00e6, 0x01);
	WriteByte(0x00e7, 0x03);
	WriteByte(0x00f5, 0x02);
	WriteByte(0x00D9, 0x05);
	WriteByte(0x00DB, 0xCE);
	WriteByte(0x00DC, 0x03);
	WriteByte(0x00DD, 0xF8);
	WriteByte(0x009f, 0x00);
	WriteByte(0x00a3, 0x3c);
	WriteByte(0x00b7, 0x00);
	WriteByte(0x00bb, 0x3c);
	WriteByte(0x00b2, 0x09);
	WriteByte(0x00ca, 0x09);
	WriteByte(0x0198, 0x01);
	WriteByte(0x01b0, 0x17);
    WriteByte(0x01ad, 0x00);
	WriteByte(0x00FF, 0x05);
	WriteByte(0x0100, 0x05);
	WriteByte(0x0199, 0x05);
	WriteByte(0x0109, 0x07);
	WriteByte(0x010a, 0x30);
	WriteByte(0x003f, 0x46);
	WriteByte(0x01a6, 0x1b);
	WriteByte(0x01ac, 0x3e);
	WriteByte(0x01a7, 0x1f);
	WriteByte(0x0103, 0x01);
	WriteByte(0x0030, 0x00);
	WriteByte(0x001b, 0x0A);
	WriteByte(0x003e, 0x0A);
	WriteByte(0x0131, 0x04);
	WriteByte(0x0011, 0x10);
	WriteByte(0x0014, 0x24);
	WriteByte(0x0031, 0xFF);
	WriteByte(0x00d2, 0x01);
	WriteByte(0x00f2, 0x01);

	return 0;
}

///////////////////////////////////////////////////////////////////
// Start a range measurement in single shot mode
///////////////////////////////////////////////////////////////////
int VL6180X_Start_Range() {
	WriteByte(0x0018,0x01);
	return 0;
}


///////////////////////////////////////////////////////////////////
// DESC.:poll for new sample ready ready
// When the measurement of VL6180X_Start_Range() is completed, bit 2
// of RESULT__INTERRUPT_STATUS_GPIO{0x4F} will be set.
///////////////////////////////////////////////////////////////////
int VL6180X_Poll_Range() {
	char status;
	char range_status;

	// check the status
	status = ReadByte(0x004f);
	range_status = status & 0x07;

	// wait for new measurement ready status
	while (range_status != 0x04) {
		status = ReadByte(0x004f);
		range_status = status & 0x07;
		HAL_Delay(1);
	}
	return 0;
}

///////////////////////////////////////////////////////////////////
// Read range result (mm)
///////////////////////////////////////////////////////////////////
int VL6180X_Read_Range() {
	int range;
	range=ReadByte(0x0062);
	return range;
}

///////////////////////////////////////////////////////////////////
// clear interrupts
///////////////////////////////////////////////////////////////////
int VL6180X_Clear_Interrupts() {
	 WriteByte(0x0015,0x07);
	 return 0;
}

///////////////////////////////////////////////////////////////////
//FSR check
//DESC.: Check the hands position
///////////////////////////////////////////////////////////////////
status_TypeDef FSR_Check(uint16_t FSR_VALUE){
	if(MIN_FSR_PRESS_OK < FSR_VALUE) return PASS_OK;
	else return FAIL;
}

///////////////////////////////////////////////////////////////////////////////
// start sound
// DESC.: 3 tonos diferentes para el inicio modificando el ARR del mismo timer
//////////////////////////////////////////////////////////////////////////////
void startsound(){
	if(timer_startsound>0.1 && timer_startsound<=250){
		TIM2->ARR = 17-1; //600hz
		TIM2->CCR1 = 8.5;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	}

	if(timer_startsound>250 && timer_startsound<=500){
		TIM2->ARR = 12-1; //800hz
		TIM2->CCR1 = 6;
	}

	if(timer_startsound>500 && timer_startsound<=750){
		TIM2->ARR = 10-1;//1000hz
		TIM2->CCR1 = 5;
	}

	if(timer_startsound >= 750){HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);timer_startsound=0;}


}

///////////////////////////////////////////////////////////////////
// Buzzer sound
// DESC.: 0,5s turn on PWM for tone, 0,5s turn off PWM
///////////////////////////////////////////////////////////////////
void buzzer(){
	if(timer_buzzer>0.001 && timer_buzzer<=250){ //durante 0,25s PWM BUZZER ON
		TIM2->ARR = 15-1;
		TIM2->CCR1 = duty_sound;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//START PWM FOR BUZZER SOUND
	}
	if(timer_buzzer>=250 && timer_buzzer<=500){ //apago el timer despues de 250 ms y cumplo el periodo de 500ms para una maniobra
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	}
	if(timer_buzzer>=500){timer_buzzer=0;} //reinicio la cuenta del timer
}

///////////////////////////////////////////////////////////////////
// func.:zero_distance
// DESC.: returns offset distance
///////////////////////////////////////////////////////////////////
int VL6180x_offset(){
	int offset_measure = 0;
	for(int i = 0; i<15; i++){
		offset_measure += VL6180X_Read_Range();
		//HAL_Delay(2);
	}
	return offset_measure = offset_measure/15; //promedio de lecturas para obtener el offset
}

///////////////////////////////////////////////////////////////////
// func.:FSR value
// DESC.: calculates FSR offset.
///////////////////////////////////////////////////////////////////
int FSR_offset(){
	int offset = 0;
	// Start ADC Conversion FOR FSR
	HAL_ADC_Start(&hadc2);

	// Poll ADC1 Perihperal & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc2, 1);

	for(int j = 0; j < 15; j++){
		offset += HAL_ADC_GetValue(&hadc2);
	}
	return offset = offset / 15;
}


///////////////////////////////////////////////////////////////////
// func.:calculate freq
// DESC.:Función para calcular frecuencia en ventana de 2 segundos
///////////////////////////////////////////////////////////////////
int calculate_frequency() {
    uint32_t current_time = HAL_GetTick(); // Tiempo actual en ms
    int recent_compressions = 0;
    for (int i = 0; i < 10; i++) {
        if ((current_time - last_compression_times[i]) <= 2000) {
            recent_compressions++;
        }
    }
    return recent_compressions / 2; // Frecuencia en Hz (cada 2 segundos)
}




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
