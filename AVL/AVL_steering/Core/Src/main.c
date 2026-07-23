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
#include "eth.h"
#include "i2c.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU9250_ADDR        (0x68 << 1)   // HAL uses 8-bit address

#define MPU_REG_WHOAMI       0x75
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_PWR_MGMT_2   0x6C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_CONFIG2 0x1D
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_SMPLRT_DIV   0x19

#define ACCEL_SCALE          16384.0f   // LSB/g  at +-2g
#define GYRO_SCALE           131.0f     // LSB/(deg/s) at +-250 dps
#define STREAM_PERIOD_MS     10U        // 100 Hz UART stream
#define ZERO_SAMPLE_WARN_COUNT 25U

// Software low-pass (exponential moving average) on the scaled outputs.
// filtered = ALPHA*new + (1-ALPHA)*previous.
// Smaller ALPHA -> smoother / less jitter but more lag. Range (0,1].
// 1.0 disables the filter. Tune this to taste.
#define FILTER_ALPHA         0.15f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t mpu_addr = MPU9250_ADDR;
static uint32_t zero_sample_count = 0;

// Exponential-moving-average filter state for the scaled outputs.
static float flt_ax = 0, flt_ay = 0, flt_az = 0;
static float flt_gx = 0, flt_gy = 0, flt_gz = 0;
static uint8_t filter_primed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Redirect printf to UART3 (ST-Link virtual COM port)
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

HAL_StatusTypeDef MPU_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return HAL_I2C_Master_Transmit(&hi2c1, mpu_addr, buf, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&hi2c1, mpu_addr, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(&hi2c1, mpu_addr, data, len, HAL_MAX_DELAY);
}

int16_t CombineBytes(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

HAL_StatusTypeDef MPU_Detect(uint8_t *whoami) {
    const uint16_t addrs[] = {(0x68 << 1), (0x69 << 1)};

    for (uint32_t i = 0; i < (sizeof(addrs) / sizeof(addrs[0])); i++) {
        mpu_addr = addrs[i];
        if (MPU_ReadRegs(MPU_REG_WHOAMI, whoami, 1) == HAL_OK) {
            return HAL_OK;
        }
    }

    mpu_addr = MPU9250_ADDR;
    return HAL_ERROR;
}

HAL_StatusTypeDef MPU_WriteRegChecked(uint8_t reg, uint8_t data, const char *name) {
    HAL_StatusTypeDef status = MPU_WriteReg(reg, data);
    if (status != HAL_OK) {
        printf("# ERROR: write %s failed, HAL=%d\r\n", name, status);
        return status;
    }

    uint8_t readback = 0;
    status = MPU_ReadRegs(reg, &readback, 1);
    if (status != HAL_OK) {
        printf("# ERROR: readback %s failed, HAL=%d\r\n", name, status);
        return status;
    }

    if (readback != data) {
        printf("# WARN: %s readback 0x%02X, expected 0x%02X\r\n", name, readback, data);
    }

    return HAL_OK;
}

HAL_StatusTypeDef MPU_Init(void) {
    HAL_StatusTypeDef status;
    uint8_t whoami = 0;

    printf("\r\n# MPU-9250/MPU-6500 CSV streamer\r\n");

    HAL_Delay(100);

    status = MPU_Detect(&whoami);
    if (status != HAL_OK) {
        printf("# ERROR: no MPU found at I2C address 0x68 or 0x69\r\n");
        return status;
    }
    printf("# I2C addr=0x%02X WHO_AM_I=0x%02X\r\n", (unsigned int)(mpu_addr >> 1), whoami);
    if (whoami == 0x71)      printf("# detected=MPU-9250\r\n");
    else if (whoami == 0x70) printf("# detected=MPU-6500\r\n");
    else                     printf("# WARN: unexpected WHO_AM_I; wiring/address may be wrong\r\n");

    // Reset, wake, and enable all accel/gyro axes.
    if (MPU_WriteReg(MPU_REG_PWR_MGMT_1, 0x80) != HAL_OK) {
        printf("# ERROR: failed to reset sensor\r\n");
        return HAL_ERROR;
    }
    HAL_Delay(100);

    if (MPU_WriteRegChecked(MPU_REG_PWR_MGMT_1, 0x01, "PWR_MGMT_1") != HAL_OK) return HAL_ERROR;
    if (MPU_WriteRegChecked(MPU_REG_PWR_MGMT_2, 0x00, "PWR_MGMT_2") != HAL_OK) return HAL_ERROR;
    if (MPU_WriteRegChecked(MPU_REG_SMPLRT_DIV, 0x09, "SMPLRT_DIV") != HAL_OK) return HAL_ERROR;   // 100 Hz
    if (MPU_WriteRegChecked(MPU_REG_CONFIG, 0x05, "CONFIG") != HAL_OK) return HAL_ERROR;           // gyro DLPF ~10 Hz
    if (MPU_WriteRegChecked(MPU_REG_GYRO_CONFIG, 0x00, "GYRO_CONFIG") != HAL_OK) return HAL_ERROR; // +-250 deg/s, DLPF enabled
    if (MPU_WriteRegChecked(MPU_REG_ACCEL_CONFIG, 0x00, "ACCEL_CONFIG") != HAL_OK) return HAL_ERROR; // +-2g
    if (MPU_WriteRegChecked(MPU_REG_ACCEL_CONFIG2, 0x05, "ACCEL_CONFIG2") != HAL_OK) return HAL_ERROR; // accel DLPF ~10 Hz (was wide-open)

    printf("# Streaming at %lu Hz over USART3 115200 baud\r\n", 1000UL / STREAM_PERIOD_MS);
    printf("# Accel in g, Gyro in deg/s\r\n");
    return HAL_OK;
}

void MPU_ReadAndPrint(void) {
    uint8_t raw[14];
    if (MPU_ReadRegs(MPU_REG_ACCEL_XOUT_H, raw, 14) != HAL_OK) {
        printf("[ERROR] Read failed\r\n");
        return;
    }

    int16_t ax_raw = CombineBytes(raw[0],  raw[1]);
    int16_t ay_raw = CombineBytes(raw[2],  raw[3]);
    int16_t az_raw = CombineBytes(raw[4],  raw[5]);
    int16_t temp_raw = CombineBytes(raw[6], raw[7]);
    int16_t gx_raw = CombineBytes(raw[8],  raw[9]);
    int16_t gy_raw = CombineBytes(raw[10], raw[11]);
    int16_t gz_raw = CombineBytes(raw[12], raw[13]);

    if ((ax_raw == 0) && (ay_raw == 0) && (az_raw == 0) &&
        (gx_raw == 0) && (gy_raw == 0) && (gz_raw == 0)) {
        zero_sample_count++;
        if ((zero_sample_count % ZERO_SAMPLE_WARN_COUNT) == 0U) {
            printf("# WARN: accel and gyro have been zero for %lu samples; check IMU power, SDA/SCL pullups, address, and that this firmware is flashed\r\n",
                   zero_sample_count);
        }
    } else {
        zero_sample_count = 0;
    }

    float ax = ax_raw / ACCEL_SCALE;
    float ay = ay_raw / ACCEL_SCALE;
    float az = az_raw / ACCEL_SCALE;
    float gx = gx_raw / GYRO_SCALE;
    float gy = gy_raw / GYRO_SCALE;
    float gz = gz_raw / GYRO_SCALE;
    float temp_c = ((float)temp_raw / 333.87f) + 21.0f;

    // Exponential moving average to smooth jitter. First sample seeds the state.
    if (!filter_primed) {
        flt_ax = ax; flt_ay = ay; flt_az = az;
        flt_gx = gx; flt_gy = gy; flt_gz = gz;
        filter_primed = 1;
    } else {
        flt_ax += FILTER_ALPHA * (ax - flt_ax);
        flt_ay += FILTER_ALPHA * (ay - flt_ay);
        flt_az += FILTER_ALPHA * (az - flt_az);
        flt_gx += FILTER_ALPHA * (gx - flt_gx);
        flt_gy += FILTER_ALPHA * (gy - flt_gy);
        flt_gz += FILTER_ALPHA * (gz - flt_gz);
    }

    printf("ACCEL[g]  x=%+8.3f  y=%+8.3f  z=%+8.3f   |   GYRO[dps]  x=%+8.2f  y=%+8.2f  z=%+8.2f   |   T=%5.1fC\r\n",
           flt_ax, flt_ay, flt_az, flt_gx, flt_gy, flt_gz, temp_c);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  MPU_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    MPU_ReadAndPrint();
    HAL_Delay(STREAM_PERIOD_MS);

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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
#ifdef USE_FULL_ASSERT
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
