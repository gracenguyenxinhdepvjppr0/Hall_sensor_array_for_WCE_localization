/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <TMAG5170.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct { err_t err; uint32_t tx, rx; } fail_info_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_BUS        2                  /* SPI1, SPI2 */
#define N_PER_BUS    8                  /* mỗi decoder 3->8 chọn 8 cảm biến */
#define N_SENSOR     (N_BUS * N_PER_BUS)
#define TX_BUF_SIZE  512                /* dòng dài nhất ~403 byte */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_tx;

/* USER CODE BEGIN PV */
static const struct { GPIO_TypeDef *port; uint16_t pin; } cs_pin[2][3] = {
    { { A1_GPIO_Port, A1_Pin }, { B1_GPIO_Port, B1_Pin }, { C1_GPIO_Port, C1_Pin } },   /* decoder 1: A1, B1, C1 */
    { { A2_GPIO_Port, A2_Pin }, { B2_GPIO_Port, B2_Pin }, { C2_GPIO_Port, C2_Pin } },   /* decoder 2: A2, B2, C2 */
};
static uint8_t cs_addr[2];
static TMAG5170_t tmag[2];          /* 1 struct / bus SPI (16 cảm biến cùng cấu hình) */
static uint8_t    want_addr[2];     /* cảm biến đang nhắm tới trên mỗi bus */
static bool       cs_active[2];     /* true = đang giữ CS của want_addr ở mức thấp */
static int16_t    raw_code[N_SENSOR][3];
float Bx[4][4], By[4][4], Bz[4][4];     /* từ trường (mT); cảm biến i -> [i/4][i%4], i = bus*8 + addr */
static uint16_t err_mask;               /* bit i = cảm biến i đọc lỗi ở lần quét gần nhất */
static uint32_t frame_no;

static fail_info_t init_info[N_SENSOR], scan_info[N_SENSOR];
static fail_info_t *cur_info = init_info;

uint32_t t0, scan_us;

static char     tx_buf[2][TX_BUF_SIZE] __attribute__((aligned(32)));
static uint8_t  tx_sel;                 /* buffer sẽ được định dạng ở lần gọi kế tiếp */
static volatile uint8_t tx_busy;        /* 1 = DMA đang gửi */
static uint32_t tx_timeouts, tx_drops, tx_polled;  /* tx_timeouts tăng => ngắt UART7 chưa bật (xem ghi chú) */
volatile uint32_t prof_scan_us, prof_send_us, prof_cycle_us;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void Write_addr(uint8_t bus, uint8_t addr);
uint8_t step(uint8_t bus);
uint8_t Sensor_select(uint8_t bus, uint8_t addr);
void Decoder_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint8_t gray_of_step(uint8_t s) { return (uint8_t)((s ^ (s >> 1)) & 7u); }
static inline uint8_t step_of_gray(uint8_t g) { return (uint8_t)((g ^ (g >> 1) ^ (g >> 2)) & 7u); }

void Write_addr(uint8_t bus, uint8_t addr)
{
    for (int k = 0; k < 3; k++)         /* chân không đổi được ghi lại cùng mức -> không có chuyển mạch */
        HAL_GPIO_WritePin(cs_pin[bus][k].port, cs_pin[bus][k].pin,
                          (addr & (1u << k)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    cs_addr[bus] = (uint8_t)(addr & 7u);
}

void Decoder_Init(void)
{
	for (uint8_t b = 0; b < 2; b++) Write_addr(b, 0);
}

uint8_t step(uint8_t bus)
{
    uint8_t s = step_of_gray(cs_addr[bus]);
    uint8_t a = gray_of_step((uint8_t)((s + 1u) & 7u));
    Write_addr(bus, a);
    return a;
}

uint8_t Sensor_select(uint8_t bus, uint8_t addr)
{
	   addr &= 7u;
	   uint8_t steps = 0;
	   while (cs_addr[bus] != addr)
	   {
		   step(bus);
		   steps++;
	   }
	   return steps;
}

static void cs_ring(uint8_t bus, bool select)
{
	if (select) {
		Sensor_select(bus, want_addr[bus]);
		cs_active[bus] = true;
	} else if (cs_active[bus]) {
		step(bus);
		cs_active[bus] = false;
	}
}

static void cs_bus0(bool select) { cs_ring(0, select); }
static void cs_bus1(bool select) { cs_ring(1, select); }

static void us_timer_init(void)
{
    uint32_t clk = HAL_RCC_GetPCLK2Freq();                         /* APB2 */
    if (LL_RCC_GetAPB2Prescaler() != LL_RCC_APB2_DIV_1) clk *= 2u; /* clock timer = 2 x APB2 (TIMPRE = 0, mặc định) */
    __HAL_TIM_SET_PRESCALER(&htim1, clk / 1000000u - 1u);
    htim1.Instance->EGR = TIM_EGR_UG;                              /* nạp PSC ngay (đồng thời reset bộ đếm) */
    HAL_TIM_Base_Start(&htim1);                                    /* ARR = 65535 -> tràn mỗi 65.5 ms */
}

static void delay_us(uint16_t us)                                  /* us <= 60000 */
{
    uint32_t tick0 = HAL_GetTick();
    uint16_t t0 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim1) - t0) < us) {
        if ((HAL_GetTick() - tick0) > 5u) break;                   /* TIM1 không chạy (chưa bật clock) -> không treo */
    }
}

static void uart_puts(const char *s)
{
    HAL_UART_Transmit(&huart7, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

/* ---------- Quét theo vòng ----------
 * Mỗi cảm biến nhận ĐÚNG 1 khung SPI cho mỗi vòng: sau khung, cs(false) bước sang cảm biến kế và
 * chính cảm biến đó được chọn cho khung kế tiếp -> không có cảm biến nào bị kéo CS mà không có clock
 * (tránh cờ FRAME_STAT của TMAG5170). Một vòng luôn đủ 8 khung nên con trỏ decoder quay về chỗ cũ. */
typedef err_t (*lap_job_t)(TMAG5170_t *d, uint8_t idx, uint8_t arg);

static uint16_t ring_lap(uint8_t bus, lap_job_t job, uint8_t arg)
{
    uint16_t fail = 0;                                        /* bit i = cảm biến i lỗi */
    uint8_t  s0   = step_of_gray(cs_addr[bus]);               /* bắt đầu từ chỗ con trỏ đang đứng */

    for (uint8_t k = 0; k < N_PER_BUS; k++) {
        uint8_t addr = gray_of_step((uint8_t)((s0 + k) & 7u));
        uint8_t idx  = (uint8_t)(bus * N_PER_BUS + addr);     /* idx = bus*8 + địa chỉ decoder */
        want_addr[bus] = addr;
        if (job(&tmag[bus], idx, arg) != TMAG_OK) fail |= (uint16_t)(1u << idx);
    }
    return fail;
}

static err_t job_init   (TMAG5170_t *d, uint8_t idx, uint8_t step) { (void)idx; return TMAG5170_InitStep(d, step); }
static err_t job_trigger(TMAG5170_t *d, uint8_t idx, uint8_t arg)  { (void)idx; (void)arg; return TMAG5170_Trigger(d); }
static err_t job_axis   (TMAG5170_t *d, uint8_t idx, uint8_t axis) { return TMAG5170_ReadAxis(d, axis, &raw_code[idx][axis]); }

/* Khởi tạo 16 cảm biến: từng bước cấu hình chạy hết một vòng trên bus 0 rồi bus 1 */
static void TMAG5170_InitAll(void)
{
    uint16_t fail = 0;
    char msg[32];

    HAL_Delay(2);                                             /* > tstart_power_up (tối đa 500 us) */
    for (uint8_t b = 0; b < N_BUS; b++)
        TMAG5170_InitBegin(&tmag[b], b ? &hspi2 : &hspi1, b ? cs_bus1 : cs_bus0, TRIGGER);

    for (uint8_t step = 0; step < TMAG5170_INIT_STEPS; step++)
        for (uint8_t b = 0; b < N_BUS; b++)
            fail |= ring_lap(b, job_init, step);

    if (fail) {
        snprintf(msg, sizeof msg, "INIT_FAIL,%04X\r\n", (unsigned)fail);   /* bit i = cảm biến i */
        uart_puts(msg);
    }
}

/* Quét 16 cảm biến -> Bx, By, Bz (mT). Cảm biến lỗi: giữ giá trị cũ, bật bit trong err_mask.
 * Vòng 1: trigger cả 16 cảm biến; chờ chuyển đổi; vòng 2..4: đọc X, Y, Z. */
static void Scan16(void)
{
    uint16_t fail = 0;

    cur_info = scan_info;
     memset(scan_info, 0, sizeof scan_info);                   /* chỉ giữ lỗi của lần quét này */
     for (uint8_t b = 0; b < N_BUS; b++) fail |= ring_lap(b, job_trigger, 0);
     delay_us(3000);                                  /* ~2.44 ms với 32x, 3 trục */
     for (uint8_t axis = 0; axis < 3; axis++)
         for (uint8_t b = 0; b < N_BUS; b++) fail |= ring_lap(b, job_axis, axis);

    for (uint8_t i = 0; i < N_SENSOR; i++) {
        if (fail & (1u << i)) continue;
        const TMAG5170_t *d = &tmag[i / N_PER_BUS];
        Bx[i / 4][i % 4] = TMAG5170_CodeToMT(d, raw_code[i][0]);
        By[i / 4][i % 4] = TMAG5170_CodeToMT(d, raw_code[i][1]);
        Bz[i / 4][i % 4] = TMAG5170_CodeToMT(d, raw_code[i][2]);
    }
    err_mask = fail;
}

static int append_array(char* buf, int n, float a[4][4])
{
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (n >= TX_BUF_SIZE) return n;
            float  x  = a[r][c] * 1000.0f;                       /* mT -> uT */
            long   ut = (long)(x + (x >= 0.0f ? 0.5f : -0.5f));  /* làm tròn, không cần printf float */
            n += snprintf(buf + n, TX_BUF_SIZE - n, ",%ld", ut);
        }
    }
    return n;
}

static void uart_wait_idle(void)
{
    uint32_t t0 = HAL_GetTick();
    while (tx_busy) {
        if (((DMA_Stream_TypeDef *)hdma_uart7_tx.Instance)->NDTR == 0u && __HAL_UART_GET_FLAG(&huart7, UART_FLAG_TC)) {
            if (tx_busy) {                                    /* callback chưa tới */
                HAL_UART_AbortTransmit(&huart7);
                tx_polled++;
                tx_busy = 0;
            }
            break;
        }
        if ((HAL_GetTick() - t0) > 100u) {
            HAL_UART_AbortTransmit(&huart7);
            tx_busy = 0;
            tx_timeouts++;
            break;
        }
    }
}

/* 1 dòng / lần quét:  B,<số khung>,<err_mask hex>,Bx[0][0..3],Bx[1][..],...,By...,Bz...   (đơn vị uT) */
static void Send_UART(void)
{
    char *buf = tx_buf[tx_sel];
    int n = snprintf(buf, TX_BUF_SIZE, "B,%lu,%04X", (unsigned long)frame_no++, (unsigned)err_mask);
    n = append_array(buf, n, Bx);
    n = append_array(buf, n, By);
    n = append_array(buf, n, Bz);
    if (n > TX_BUF_SIZE - 3) n = TX_BUF_SIZE - 3;
    buf[n++] = '\r';
    buf[n++] = '\n';

    uart_wait_idle();
    tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart7, (uint8_t *)buf, (uint16_t)n) != HAL_OK) {
        tx_busy = 0;
        tx_drops++;
        return;
    }
    tx_sel ^= 1u;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART7_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Decoder_Init();
  us_timer_init();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     /* bộ đếm chu kỳ để đo thời gian */
  DWT->LAR = 0xC5ACCE55;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  HAL_NVIC_SetPriority(UART7_IRQn, 1, 0);             /* đảm bảo ngắt UART7 được bật (báo xong TX DMA) */
  HAL_NVIC_EnableIRQ(UART7_IRQn);
  TMAG5170_InitAll();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
      static uint32_t c_prev;
      uint32_t k  = SystemCoreClock / 1000000u;       /* chu kỳ / us */
      uint32_t c0 = DWT->CYCCNT;
      Scan16();
      uint32_t c1 = DWT->CYCCNT;
      Send_UART();
      uint32_t c2 = DWT->CYCCNT;
      prof_scan_us  = (c1 - c0) / k;                  /* chỉ Scan16() */
      prof_send_us  = (c2 - c1) / k;                  /* định dạng + chờ khung trước + bật DMA */
      prof_cycle_us = (c0 - c_prev) / k;              /* từ đầu vòng trước tới đầu vòng này */
      c_prev = c0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_HSI_SetDivider(LL_RCC_HSI_DIV1);
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSI);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1Q_Enable();
  LL_RCC_PLL1R_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(4);
  LL_RCC_PLL1_SetN(60);
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
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
  htim1.Init.Prescaler = 119;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */
  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */
  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 921600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */
  /* USER CODE END UART7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOH);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, A1_Pin|B1_Pin|C1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_8|A2_Pin|B2_Pin|C2_Pin);

  /**/
  GPIO_InitStruct.Pin = A1_Pin|B1_Pin|C1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = A2_Pin|B2_Pin|C2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{

  /* Disables the MPU */
  LL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER0, 0x87, 0x0, LL_MPU_REGION_SIZE_4GB|LL_MPU_TEX_LEVEL0|LL_MPU_REGION_NO_ACCESS|LL_MPU_INSTRUCTION_ACCESS_DISABLE|LL_MPU_ACCESS_SHAREABLE|LL_MPU_ACCESS_NOT_CACHEABLE|LL_MPU_ACCESS_NOT_BUFFERABLE);
  /* Enables the MPU */
  LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
