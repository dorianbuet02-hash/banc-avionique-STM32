/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    : main.c
  * @brief   : Banc de test avionique – STM32L432KC
  *            FP1: Affichage OLED + terminal (5 Hz)
  *            FP2: Enregistrement microSD (FatFs + EXTI)
  *            FP3: Trame ARINC 429 (bit-banging)
  *            FP4: Lacet -> servomoteur (PWM registres directs)
  *            FP5: Alarmes LED + ADC temperature interne
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

#include "main.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "servo_pwm.h"
#include "oled.h"
#include "bmp280.h"
#include "icm20948.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef  hadc1;
I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t enregistrement_actif = 1;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* --------------------------------------------------------------------------
 * Utilitaire – printf serie
 * -------------------------------------------------------------------------- */
void myprintf(const char *fmt, ...) {
    static char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), -1);
}

/* --------------------------------------------------------------------------
 * FP3 – Construction de la trame ARINC 429 (32 bits, parite impaire)
 * -------------------------------------------------------------------------- */
uint32_t Construire_Trame_ARINC(float pression_hPa) {
    uint32_t trame = 0;
    uint8_t  label = 0256;
    uint32_t data  = (uint32_t)pression_hPa;

    trame |= label;
    trame |= (data << 10);

    int nombre_de_1 = 0;
    for (int i = 0; i < 31; i++)
        if ((trame >> i) & 1) nombre_de_1++;

    if (nombre_de_1 % 2 == 0)
        trame |= (1U << 31);

    return trame;
}

/* --------------------------------------------------------------------------
 * FP3 – Transmission physique BPRZ par bit-banging
 * -------------------------------------------------------------------------- */
void delay_us(uint32_t us) {
    uint32_t count = us * (32 / 4);
    while (count--) __ASM("nop");
}

void Envoyer_ARINC_Physique(uint32_t trame) {
    for (int i = 0; i < 32; i++) {
        uint8_t bit = (trame >> i) & 1;

        if (bit == 1) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        }
        delay_us(40);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        delay_us(40);
    }
    delay_us(40 * 8);
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */

  /* --- FP1 : Initialisation des capteurs I2C ----------------------------- */
  OLED_Init();
  BMP280_Init();

  char msg[64];
  sprintf(msg, "\r\n--- Scanner I2C ---\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
  for (uint8_t i = 1; i < 128; i++) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 5) == HAL_OK) {
          sprintf(msg, "-> 0x%02X\r\n", i);
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
      }
  }
  sprintf(msg, "--- Fin du Scan ---\r\n\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  /* --- FP4 : Initialisation PWM par registres directs -------------------- */
  PWM_Servo_Init();

  /* --- FP1 : Calibration gyroscopique (ICM-20948) ------------------------ */
  OLED_WriteString("--- BANC AVIONIQUE ---", 0, 0);
  OLED_WriteString("  Ne pas bouger !    ", 0, 4);
  OLED_WriteString("  Calibration...     ", 0, 5);
  ICM_Init();
  OLED_Fill(0x00);
  OLED_WriteString("--- BANC AVIONIQUE ---", 0, 0);

  /* --- FP5 : Calibration et demarrage de l'ADC --------------------------- */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    char ligne[30];

    /* -----------------------------------------------------------------------
     * FP1 – Lecture de tous les capteurs
     * --------------------------------------------------------------------- */
    float temp = BMP280_ReadTemperature();
    float pres = BMP280_ReadPressure();
    float roulis, tangage, lacet;
    ICM_ReadAngles(&roulis, &tangage, &lacet);

    /* -----------------------------------------------------------------------
     * FP5 – Alarmes LED (roulis / tangage hors de +-40°)
     * --------------------------------------------------------------------- */
    HAL_GPIO_WritePin(LED_ROULIS_GPIO_Port, LED_ROULIS_Pin,
        (roulis  >  40.0f || roulis  < -40.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_TANGAGE_GPIO_Port, LED_TANGAGE_Pin,
        (tangage >  40.0f || tangage < -40.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* -----------------------------------------------------------------------
     * FP5 – Acquisition ADC : temperature interne du STM32
     * --------------------------------------------------------------------- */
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t adc_brut  = HAL_ADC_GetValue(&hadc1);
    float tension_puce = ((float)adc_brut * 3.3f) / 4095.0f;
    float temp_puce    = ((tension_puce - 0.76f) / 0.0025f) + 25.0f + 18.0f;
    int tp_e = (int)temp_puce;
    int tp_d = (int)((temp_puce - tp_e) * 100);
    if (tp_d < 0) tp_d = -tp_d;

    /* -----------------------------------------------------------------------
     * FP4 – Mise a jour du servomoteur (lacet)
     * --------------------------------------------------------------------- */
    Mettre_A_Jour_Servomoteur(lacet);

    /* -----------------------------------------------------------------------
     * FP1 – Conversion float -> int et affichage OLED
     * --------------------------------------------------------------------- */
    int pres_e = (int)pres; int pres_d = (int)((pres - pres_e) * 100);
    int temp_e = (int)temp; int temp_d = (int)((temp - temp_e) * 100); if (temp_d < 0) temp_d = -temp_d;
    int rou_e  = (int)roulis;  int rou_d  = (int)((roulis  - rou_e)  * 100); if (rou_d  < 0) rou_d  = -rou_d;
    int tan_e  = (int)tangage; int tan_d  = (int)((tangage - tan_e)  * 100); if (tan_d  < 0) tan_d  = -tan_d;
    int lac_e  = (int)lacet;   int lac_d  = (int)((lacet   - lac_e)  * 100); if (lac_d  < 0) lac_d  = -lac_d;

    sprintf(ligne, "Press:   %d.%02d hPa",  pres_e, pres_d); OLED_WriteString(ligne, 0, 2);
    sprintf(ligne, "Temp:    %d.%02d C",    temp_e, temp_d); OLED_WriteString(ligne, 0, 3);
    sprintf(ligne, "Roulis:  %d.%02d deg ", rou_e,  rou_d);  OLED_WriteString(ligne, 0, 5);
    sprintf(ligne, "Tangage: %d.%02d deg ", tan_e,  tan_d);  OLED_WriteString(ligne, 0, 6);
    sprintf(ligne, "Lacet:   %d.%02d deg ", lac_e,  lac_d);  OLED_WriteString(ligne, 0, 7);

    HAL_Delay(200);

    /* -----------------------------------------------------------------------
     * FP2 – Enregistrement CSV sur microSD (si enregistrement actif)
     * --------------------------------------------------------------------- */
    char ligne_etat[30];
    if (enregistrement_actif == 1) {
        sprintf(ligne_etat, "ETAT: REC  [%d.%02d C] ", tp_e, tp_d);
        OLED_WriteString(ligne_etat, 0, 1);

        FATFS fs; FIL fil; UINT bw;
        char buffer_sd[100];

        if (f_mount(&fs, "", 1) == FR_OK) {
            if (f_open(&fil, "donnees.csv",
                       FA_OPEN_ALWAYS | FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
                sprintf(buffer_sd, "%d.%02d;%d.%02d;%d.%02d;%d.%02d;%d.%02d\n",
                        pres_e, pres_d, temp_e, temp_d,
                        rou_e,  rou_d,  tan_e,  tan_d, lac_e, lac_d);
                f_write(&fil, buffer_sd, strlen(buffer_sd), &bw);
                f_close(&fil);
            }
            f_mount(NULL, "", 0);
        }
    } else {
        sprintf(ligne_etat, "ETAT: PAUSE[%d.%02d C] ", tp_e, tp_d);
        OLED_WriteString(ligne_etat, 0, 1);
    }

    /* -----------------------------------------------------------------------
     * FP3 – Construction et envoi de la trame ARINC 429
     * --------------------------------------------------------------------- */
    uint32_t mot_arinc = Construire_Trame_ARINC(pres);
    Envoyer_ARINC_Physique(mot_arinc);

  /* USER CODE END 3 */
  }  /* <-- accolade fermante du while(1) */
}    /* <-- accolade fermante du main()   */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    Error_Handler();

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState            = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 1;
  RCC_OscInitStruct.PLL.PLLN            = 16;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    Error_Handler();

  HAL_RCCEx_EnableMSIPLLMode();
}

static void MX_ADC1_Init(void)
{
  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait      = DISABLE;
  hadc1.Init.ContinuousConvMode    = ENABLE;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode      = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    Error_Handler();

  sConfig.Channel      = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();

  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */
}

static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */
  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */

  hi2c1.Instance              = I2C1;
  hi2c1.Init.Timing           = 0x00B07CB4;
  hi2c1.Init.OwnAddress1      = 0;
  hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2      = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    Error_Handler();

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    Error_Handler();

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    Error_Handler();

  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */
}

static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */
  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */

  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 7;
  hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
    Error_Handler();

  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */
}

static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */
  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */

  huart2.Instance                    = USART2;
  huart2.Init.BaudRate               = 115200;
  huart2.Init.WordLength             = UART_WORDLENGTH_8B;
  huart2.Init.StopBits               = UART_STOPBITS_1;
  huart2.Init.Parity                 = UART_PARITY_NONE;
  huart2.Init.Mode                   = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
    Error_Handler();

  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, ARINC_A_Pin|ARINC_B_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LED_ROULIS_Pin|LED_TANGAGE_Pin|LD3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin   = ARINC_A_Pin|ARINC_B_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = BOUTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOUTON_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LED_ROULIS_Pin|LED_TANGAGE_Pin|LD3_Pin|SD_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* --------------------------------------------------------------------------
 * FP2 – Callback EXTI : bascule enregistrement/pause avec anti-rebond 250 ms
 * -------------------------------------------------------------------------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BOUTON_Pin) {
        static uint32_t dernier_appui = 0;
        uint32_t temps_actuel = HAL_GetTick();
        if ((temps_actuel - dernier_appui) > 250) {
            enregistrement_actif = !enregistrement_actif;
            dernier_appui = temps_actuel;
        }
    }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
