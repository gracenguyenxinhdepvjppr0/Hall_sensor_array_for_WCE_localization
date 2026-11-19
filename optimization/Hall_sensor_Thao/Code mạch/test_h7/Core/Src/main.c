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
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define VREF_ADC    3.3f     // điện áp tham chiếu AD7689 (V)
#define ADC_RES     65535.0f   // 16-bit resolution
#define VCC_DRV     3.3f
#define VREF_DRV    (VCC_DRV/2.0f)  // offset điện áp DRV5055
#define SENSITIVITY 0.0075f    // V/mT (DRV5055A4 ~ 7.5mV/mT)


#define CNV_PORT   GPIOE
#define CNV_PIN    LL_GPIO_PIN_2
#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

//---------------- H7 series----------------------//
static inline void DWT_Init(void)
{
	 /* Mở khóa DWT */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* Reset bộ đếm về 0 */
    DWT->CYCCNT = 0;
    /* Bật CYCCNT */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

#define DELAY_US(us) \
do { \
    uint32_t start = DWT->CYCCNT; \
    uint32_t ticks = us * (SystemCoreClock / 1000000); \
    while ((DWT->CYCCNT - start) < ticks); \
} while(0)

/*
#define DELAY_US(us) \
    do { \
         uint32_t start = SysTick->VAL; \
         uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  \
         while((start - SysTick->VAL) < ticks); \
    } while (0)
*/


#define DELAY_MS(ms) \
    do { \
        for (uint32_t i = 0; i < ms; ++i) { \
            DELAY_US(1000); \
        } \
    } while (0)

#define GPIO_SET_PIN(port, pin)     ((port)->BSRR = (pin))
#define GPIO_CLEAR_PIN(port, pin)     ((port)->BSRR = (pin << 16u))
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
uint16_t adc_raw=0;
uint16_t adc_val[100];
float V_sensor;
float V_sample[100];
float sum;
char uart_tx_buf[128];

uint16_t sensor[8][8];
uint16_t adc_value[8][8];
int a=0;

/*------------External ref 3.3V------------------*/
uint16_t cfg[8] = {
    0x3C71, 0x3CF1, 0x3D71, 0x3DF1,
    0x3E71, 0x3EF1, 0x3F71, 0x3FF1
};

uint16_t ADC_LDO = 0;
float B[8][8];
float V[8][8];
char uart_buf[64];
uint8_t rx_buf[64];
uint32_t sent_count = 0;   // số lượng giá trị B đã gửi thành công
#define ROWS 8
#define COLS 8

/* ============================================================
 * Scalar Kalman Filter – Persistent State
 * Model: x[k] = x[k-1] + w,  w ~ N(0, Q)   (random walk)
 *        z[k] = x[k]   + v,  v ~ N(0, R)   (ADC noise)
 * ============================================================ */

/* --- Tuning parameters (đơn vị V²) ---
 * Q  nhỏ  → filter smooth hơn, response chậm hơn (tín hiệu ít thay đổi)
 * R  lớn  → ADC/sensor noise cao → filter tin tưởng dự đoán hơn đo lường
 *
 * Gợi ý ban đầu cho DRV5055 + AD7689 16-bit:
 *   σ_ADC ≈ 3 LSB → σ_V ≈ 150 µV → R ≈ 2.3e-8
 *   Tuy nhiên, thực tế board noise thường cao hơn → R = 1e-4 an toàn hơn
 *   Q = 1e-5 phù hợp với từ trường thay đổi chậm (capsule endoscopy)
 */
#define KF_Q    1.4e-9f    /* Process noise variance  [V²] */
#define KF_R    4.0e-8f    /* Measurement noise variance [V²] */

typedef struct {
    float x;        /* State estimate: V_filtered     */
    float P;        /* Error covariance               */
    uint8_t init;   /* 0 = chưa khởi tạo             */
} KF1D_t;

/* Persistent state – tồn tại suốt vòng đời chương trình */
static KF1D_t kf_one = { .x = 0.0f, .P = 1.0f, .init = 0U };

static KF1D_t kf_sensor[ROWS][COLS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int16_t AD7689_Read(uint16_t cfg)
{
    uint16_t adc_value = 0;

    __NOP();
    __NOP();
    __NOP();

    DELAY_US(1);

    // CNV HIGH: Bắt đầu quá trình lấy mẫu
    GPIO_SET_PIN(GPIOE, LL_GPIO_PIN_2);

    // SỬA ĐỔI: Tăng thời gian chờ lên tối thiểu 4us (t_conv max = 3.2us)
    DELAY_US(4);

    // CNV LOW: Bắt đầu quá trình đọc dữ liệu
    GPIO_CLEAR_PIN(GPIOE, LL_GPIO_PIN_2);

    __NOP();
    __NOP();

    // Enable SPI if disabled
    if ((SPI2->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
    {
        SET_BIT(SPI2->CR1, SPI_CR1_SPE);
    }

    // SỬA ĐỔI: Chờ buffer TX trống và nạp cấu hình (cfg) TRƯỚC
    while (!(SPI2->SR & SPI_SR_TXP));
    *((__IO uint16_t*)&SPI2->TXDR) = cfg;

    // SỬA ĐỔI: Sau khi đã có data trong buffer, mới bắt đầu tạo SCK
    SET_BIT(SPI2->CR1, SPI_CR1_CSTART);

    // Wait RX ready
    while (!(SPI2->SR & SPI_SR_RXP));

    // Read ADC value
    adc_value = *((__IO uint16_t*)&SPI2->RXDR);

    // Wait SPI idle
    while (!LL_SPI_IsActiveFlag_TXC(SPI2));

    __NOP();

    return adc_value;
}

void MUX_SetColumn(uint8_t col) {
    // Bit 0 của col -> S0 = PB3
    if (col & 0x01) GPIO_SET_PIN(GPIOB, LL_GPIO_PIN_3);
    else            GPIO_CLEAR_PIN(GPIOB, LL_GPIO_PIN_3);

    // Bit 1 của col -> S1 = PB4
    if (col & 0x02) GPIO_SET_PIN(GPIOB, LL_GPIO_PIN_4);
    else            GPIO_CLEAR_PIN(GPIOB, LL_GPIO_PIN_4);

    // Bit 2 của col -> S2 = PB5
    if (col & 0x04) GPIO_SET_PIN(GPIOB, LL_GPIO_PIN_5);
    else            GPIO_CLEAR_PIN(GPIOB, LL_GPIO_PIN_5);
}

// ---- Hàm chuyển đổi ADC -> V (V) ----
static inline float ADC_To_V(uint16_t adc_raw)
{
	return ((float)adc_raw / ADC_RES) * VREF_ADC;
}

void Read_Hall_array_sensor()
{
	for (uint8_t col = 0; col < COLS; col++)
	{
		MUX_SetColumn(col); // chọn cột MUX
		AD7689_Read(cfg[0] << 2);  // dummy
		AD7689_Read(cfg[1] << 2);  // dummy
		for (uint8_t row = 2; row < 8; row++)
		{
			sensor[row - 2][col] = AD7689_Read(cfg[row] << 2);
			V[row - 2][col] = ADC_To_V(sensor[row - 2][col]);
			int len = snprintf(uart_buf, sizeof(uart_buf), "%.3f\r\n",  V[row - 2][col]);
			HAL_UART_Transmit(&huart4, (uint8_t*) uart_buf, len, HAL_MAX_DELAY);
		}
		// Đọc hàng 6 và 7 (dummy channels 0 và 1)
		sensor[6][col] = AD7689_Read(cfg[0] << 2); //dummy
		sensor[7][col] = AD7689_Read(cfg[1] << 2); //dummy
		V[6][col] = ADC_To_V(sensor[6][col]);
		V[7][col] = ADC_To_V(sensor[7][col]);
		int len6 = snprintf(uart_buf, sizeof(uart_buf), "%.3f\r\n", V[6][col]);
		HAL_UART_Transmit(&huart4, (uint8_t*) uart_buf, len6, HAL_MAX_DELAY);
		int len7 = snprintf(uart_buf, sizeof(uart_buf), "%.3f\r\n", V[7][col]);
		HAL_UART_Transmit(&huart4, (uint8_t*) uart_buf, len7, HAL_MAX_DELAY);
	}
}

void Collect_Data()
{
	// Chờ lệnh START từ PC
	memset(rx_buf, 0, sizeof(rx_buf));
	HAL_UART_Receive(&huart4, rx_buf, 5, HAL_MAX_DELAY);
	if (strncmp((char*) rx_buf, "START", 5) == 0)
	{
		// Bắt đầu thu tín hiệu 5 lần
		a = 1;
		for (uint8_t repeat = 0; repeat < 5; repeat++)
		{
			Read_Hall_array_sensor();
		}
		// Gửi chuỗi END sau khi xong 3 lần
		const char *end_msg = "END\r\n";
		HAL_UART_Transmit(&huart4, (uint8_t*) end_msg, strlen(end_msg),
		HAL_MAX_DELAY);
	}
}

/**
 * @brief  Scalar Kalman filter update (1 measurement per call)
 * @param  kf  Con trỏ đến persistent state
 * @param  z   Measurement tại bước hiện tại (V)
 * @return     Giá trị ước lượng x̂[k|k]
 */
static inline float KF1D_Update(KF1D_t *kf, float z)
{
    /* --- Khởi tạo lần đầu: seed state bằng measurement đầu tiên --- */
    if (!kf->init)
    {
        kf->x    = z;
        kf->P    = KF_R;   /* P0 = R: không chắc chắn bằng mức noise đo */
        kf->init = 1U;
        return z;
    }

    /* --- Prediction step --- */
    float P_pred = kf->P + KF_Q;            /* P⁻[k] = P[k-1] + Q */

    /* --- Update step --- */
    float K    = P_pred / (P_pred + KF_R);  /* Kalman gain         */
    kf->x     += K * (z - kf->x);           /* x̂[k] = x̂⁻ + K·ε   */
    kf->P      = (1.0f - K) * P_pred;       /* P[k] = (1-K)·P⁻    */

    return kf->x;
}


/*----------------FILTER--------------------*/
void Read_one_sensor(void)
{
    GPIO_CLEAR_PIN(GPIOB, LL_GPIO_PIN_3);
    GPIO_CLEAR_PIN(GPIOB, LL_GPIO_PIN_4);
    GPIO_CLEAR_PIN(GPIOB, LL_GPIO_PIN_5);

    /* AD7689: 2-cycle latency – cần 2 dummy reads trước */
    AD7689_Read(cfg[0] << 2);   /* dummy cycle 1 – configure channel 1 */
    AD7689_Read(cfg[1] << 2);   /* dummy cycle 2 – configure channel 1, nhận kq ch0 (bỏ) */

    /* Lần đọc thứ 3 mới cho kết quả hợp lệ của channel 1 (cfg[1] = 0x3CF1) */
    uint16_t raw = (uint16_t)AD7689_Read(cfg[1] << 2);
    float z = ADC_To_V(raw);

    V_sensor = KF1D_Update(&kf_one, z);

    int len = snprintf(uart_buf, sizeof(uart_buf), "%.4f\r\n", V_sensor);
    HAL_UART_Transmit(&huart4, (uint8_t *)uart_buf, len, HAL_MAX_DELAY);
}

void Read_Hall_array_sensor_Filter(void)
{
    for (uint8_t col = 0; col < COLS; col++)
    {
        // Chọn cột MUX
        MUX_SetColumn(col);

        // =========================================================
        // Dummy conversion
        // Sau khi đổi MUX cần bỏ các mẫu đầu tiên để ADC ổn định
        // =========================================================
        AD7689_Read(cfg[0] << 2);
        AD7689_Read(cfg[1] << 2);

        // =========================================================
        // Đọc sensor row 0 -> row 5
        // =========================================================
        for (uint8_t row = 2; row < 8; row++)
        {
            uint8_t sensor_row = row - 2;

            // Đọc ADC
            sensor[sensor_row][col] =
                AD7689_Read(cfg[row] << 2);

            // ADC -> Voltage
            float voltage_raw =
                ADC_To_V(sensor[sensor_row][col]);

            // Kalman filter
            V[sensor_row][col] =
                KF1D_Update(&kf_sensor[sensor_row][col],
                            voltage_raw);

            // Gửi dữ liệu đã lọc qua UART
            int len = snprintf(
                uart_buf,
                sizeof(uart_buf),
                "%.4f\r\n",
                V[sensor_row][col]
            );

            HAL_UART_Transmit(
                &huart4,
                (uint8_t *)uart_buf,
                len,
                HAL_MAX_DELAY
            );
        }

        // =========================================================
        // Đọc sensor row 6
        // =========================================================
        sensor[6][col] =
            AD7689_Read(cfg[0] << 2);

        float voltage_raw_6 =
            ADC_To_V(sensor[6][col]);

        V[6][col] =
            KF1D_Update(&kf_sensor[6][col],
                        voltage_raw_6);

        int len6 = snprintf(
            uart_buf,
            sizeof(uart_buf),
            "%.4f\r\n",
            V[6][col]
        );

        HAL_UART_Transmit(
            &huart4,
            (uint8_t *)uart_buf,
            len6,
            HAL_MAX_DELAY
        );

        // =========================================================
        // Đọc sensor row 7
        // =========================================================
        sensor[7][col] =
            AD7689_Read(cfg[1] << 2);

        float voltage_raw_7 =
            ADC_To_V(sensor[7][col]);

        V[7][col] =
            KF1D_Update(&kf_sensor[7][col],
                        voltage_raw_7);

        int len7 = snprintf(
            uart_buf,
            sizeof(uart_buf),
            "%.4f\r\n",
            V[7][col]
        );

        HAL_UART_Transmit(
            &huart4,
            (uint8_t *)uart_buf,
            len7,
            HAL_MAX_DELAY
        );
    }
}

void Collect_Data_Filter(void)
{
    // Chờ lệnh START từ PC
    memset(rx_buf, 0, sizeof(rx_buf));

    HAL_UART_Receive(
        &huart4,
        rx_buf,
        5,
        HAL_MAX_DELAY
    );

    if (strncmp((char *)rx_buf, "START", 5) == 0)
    {
        // =========================================================
        // Reset trạng thái Kalman cho 64 sensor
        // =========================================================
        memset(kf_sensor, 0, sizeof(kf_sensor));

        a = 1;

        // =========================================================
        // Thu tín hiệu 5 lần
        // =========================================================
        for (uint8_t repeat = 0; repeat < 5; repeat++)
        {
            Read_Hall_array_sensor_Filter();
        }

        // =========================================================
        // Báo PC đã kết thúc
        // =========================================================
        const char *end_msg = "END\r\n";

        HAL_UART_Transmit(
            &huart4,
            (uint8_t *)end_msg,
            strlen(end_msg),
            HAL_MAX_DELAY
        );
    }
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
  DWT_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 // Thu data
	 //Collect_Data();
	  Collect_Data_Filter();

	//Read_Hall_array_sensor();
		//DELAY_MS(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1Q_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_4_8);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(5);
  LL_RCC_PLL1_SetN(192);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(5);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

   /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
   LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
  {

  }
  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);
  LL_SetSystemCoreClock(480000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL1Q);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PC1   ------> SPI2_MOSI
  PC2_C   ------> SPI2_MISO
  PB10   ------> SPI2_SCK
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI2, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_EnableNSSPulseMgt(SPI2);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 921600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOH);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_2);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
