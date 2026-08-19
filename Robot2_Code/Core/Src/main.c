/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
 ***************************************c***************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "blue.h"
#include "general.h"
#include "lpms.h"
#include "red.h"
#include "sequence.h"
#include "test.h"
#include <stdio.h>
#include <string.h> // memcpy() — PS5 багцыг PCB2 руу дамжуулахад

// #include "lpms.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM13_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

int control_data[5][4] = {0};

void send_DMA(uint8_t command) {
  HAL_UART_Transmit(&huart4, &command, 1, HAL_MAX_DELAY);
}
uint8_t information[23]; // шинэ урт
uint8_t rx_idx = 0;
uint8_t data;

/* ---- USART2 (PA2) → 2 дахь PCB руу PS5 багц дамжуулах ------------------- */
uint8_t link_tx[23]; // Transmit_IT-ийн буфер (information-оос хуулна)
volatile uint32_t link_fwd_n = 0;    // амжилттай дамжуулсан багц
volatile uint32_t link_fwd_skip = 0; // BUSY-аас болж алгассан багц

/* ---- USART2 (PA3 RX) ← PCB2-оос ирэх ЗАМ (route) -----------------------
 *  PCB2 тактик setup-д хамгийн сайн баганыг тооцоод [START][route][0x0A]
 *  багцаар илгээнэ:  START=0xB2 preview (харуулна),  START=0xB3 SET (эхэлнэ).
 *  g_route (1/2/3) -ыг auto_climb climb_1/2/3-д, g_route_set-ийг ЭХЛЭХ гэрэлд.
 *  USART2 нь full-duplex — TX (дамжуулах) ба RX (route) зэрэг ажиллана.     */
volatile uint8_t g_route = 0; // 1/2/3 = сонгосон багана (preview/set), 0=алга
volatile uint8_t g_route_set =
    0; // 1 = тактик БАТАЛГААЖСАН (0xB3 ирсэн) → эхэлж болно
volatile uint8_t g_grab_done =
    0; // 1 = PCB2 grab дуусгасан (0xB5 ирсэн) — handshake ack
volatile uint8_t g_grab_ans =
    0; // PCB2-ийн "энд grab уу?" хариу (0xB6): 1=grab 0=skip
volatile uint8_t g_grab_ans_rdy = 0; // 1 = дээрх хариу шинээр ирсэн
volatile uint8_t g_strafe_cmd =
    0; // PCB2-ийн strafe команд (0xB7): 0=зогс 1=зүүн 2=баруун
volatile uint32_t g_strafe_ms =
    0; // сүүлд strafe команд ирсэн үе (шинэлэг эсэхийг шалгах)
volatile uint32_t route_rx_n = 0;  // USART2 RX-д ирсэн НИЙТ байт (оношилгоо)
volatile uint32_t route_pkt_n = 0; // бүрэн задарсан route/ack багц (оношилгоо)
static uint8_t route_rx = 0;       // HAL RX буфер (1 байт)
static uint8_t route_state = 0;    // 0=START хүлээх, 1=route, 2=END
static uint8_t route_start = 0;    // энэ багцын START (0xB2/0xB3)
static uint8_t route_tmp = 0;      // задарч буй route

/* -----------------------------------------------------------------------------
 *  HAL_UART_RxCpltCallback — ESP32-оос ирсэн PS5 багцыг задлах (huart3, 1
 * байт/удаа)
 *
 *  Багц: 23 байт,  [0]=0xAA (START) ... [22]=0x0A (END),  [21]=холболтын флаг.
 *  START байт хүлээж, 23 байт бүрдэхэд задалж control_data[5][4]-д хийнэ.
 *
 *  ─ Джойстик (information[1..4] − 100 → control_data[0][0..3], утга −100..100)
 * ─ control_data[0][0] = Зүүн стик X       control_data[0][2] = Баруун стик X
 *      control_data[0][1] = Зүүн стик Y       control_data[0][3] = Баруун стик
 * Y
 *
 *  ─ Товч (information[5..20] → control_data[1..4][0..3], утга 0/1) ─
 *      control_data[1][0]=Cross    [1][1]=Square   [1][2]=Triangle
 * [1][3]=Circle control_data[2][0]=D-Down   [2][1]=D-Right  [2][2]=D-Up
 * [2][3]=D-Left control_data[3][0]=L1       [3][1]=R1       [3][2]=L2 [3][3]=R2
 *      control_data[4][0]=Share    [4][1]=Options  [4][2]=L3       [4][3]=R3
 *
 *  Холбогдоогүй (флаг = 0) бол бүх утгыг 0 болгож цэвэрлэнэ.
 * -----------------------------------------------------------------------------
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart3) {
    if (rx_idx == 0 && data != 0xAA) {
      HAL_UART_Receive_IT(&huart3, &data, 1);
      return;
    }

    information[rx_idx++] = data;

    if (rx_idx == 23 && information[22] == 0x0A) {
      uint8_t isConnected = information[21];

      if (isConnected == 1) {
        // --- Джойстик: information[1..4] → control_data[0][0..3], төв 100-г
        // хасаж -100..100 ---
        //   [0][0]=LStickX  [0][1]=LStickY  [0][2]=RStickX  [0][3]=RStickY
        for (int i = 0; i < 4; i++) {
          control_data[0][i] = information[1 + i] - 100;
        }

        // --- Товч: information[5..20] → control_data[1..4][0..3] (0/1) ---
        //   мөр 1 = Cross/Square/Triangle/Circle
        //   мөр 2 = D-pad Down/Right/Up/Left
        //   мөр 3 = L1/R1/L2/R2
        //   мөр 4 = Share/Options/L3/R3
        for (int i = 0; i < 16; i++) {
          control_data[1 + (i / 4)][i % 4] = information[5 + i];
        }

      } else {
        // --- Холбогдоогүй: бүх джойстик/товчийг тэглэх ---
        for (int i = 0; i < 5; i++) {
          for (int j = 0; j < 4; j++) {
            control_data[i][j] = 0;
          }
        }
      }

      /* ===== 2 дахь PCB руу ДАМЖУУЛАХ (USART2, PA2) =====
       *  PS5 нэг ширхэг — түүнийг ЭНЭ самбарт залгаад PCB2 руу дамжуулна.
       *
       *  Түүхий 23 байтыг ЯГ ТЭР ХЭВЭЭР нь дамжуулна (дахин задлаж угсрахгүй).
       *  Тиймээс PCB2 нь ЯГ ижил задлагч ашиглана — формат хэзээ ч зөрөхгүй.
       *
       *  Transmit_IT — blocking БИШ. HAL_UART_Transmit (blocking) бол 23 байт
       *  @115200 = ~2мс ISR дотор гацна, тэр нь бусад тасалдлыг хойшлуулна.
       *
       *  Өмнөх дамжуулалт дуусаагүй бол HAL_BUSY буцаана → энэ багцыг чимээгүй
       *  алгасана. ESP32 нь ~10-20мс тутам илгээдэг, дамжуулалт 2мс тул
       *  давхцах нь ховор; давхцлаа ч дараагийн багц шууд ирнэ.
       */
      memcpy(link_tx, information, 23);
      if (HAL_UART_Transmit_IT(&huart2, link_tx, 23) == HAL_OK) {
        link_fwd_n++;
      } else {
        link_fwd_skip++;
      }

      rx_idx = 0;
    } else if (rx_idx >= 23) {
      rx_idx = 0; // invalid
    }

    HAL_UART_Receive_IT(&huart3, &data, 1);
  } else if (huart == &huart2) {
    /* --- PCB2-оос ирэх багц [START][data][0x0A] ---
     *   0xB2 preview · 0xB3 SET(route) · 0xB5 GRAB-DONE · 0xB6 GRAB-ANS(1/0)
     *   0xB7 STRAFE(0=зогс/1=зүүн/2=баруун) — grab дундах strafe-ийг PCB2
     * удирдана */
    route_rx_n++; // оношилгоо: нийт байт
    switch (route_state) {
    case 0:
      if (route_rx == 0xB2 || route_rx == 0xB3 || route_rx == 0xB5 ||
          route_rx == 0xB6 || route_rx == 0xB7) {
        route_start = route_rx;
        route_state = 1;
      }
      break;
    case 1:
      route_tmp = route_rx;
      route_state = 2;
      break; // data
    default: // END
      if (route_rx == 0x0A) {
        if (route_start == 0xB5) { // PCB2 grab дуусгасан ack
          g_grab_done = 1;
          route_pkt_n++;
        } else if (route_start == 0xB6) { // "энд grab уу?" хариу
          g_grab_ans = (route_tmp != 0);
          g_grab_ans_rdy = 1;
          route_pkt_n++;
        } else if (route_start == 0xB7) { // strafe команд (grab дунд)
          g_strafe_cmd = route_tmp;       // 0=зогс 1=зүүн 2=баруун
          g_strafe_ms = HAL_GetTick();    // шинэлэг тэмдэг
          route_pkt_n++;
        } else if (route_tmp >= 1 && route_tmp <= 3) { // route (0xB2/0xB3)
          g_route = route_tmp;
          route_pkt_n++;
          if (route_start == 0xB3)
            g_route_set = 1; // SET → LOCK
        }
      }
      route_state = 0;
      break;
    }
    HAL_UART_Receive_IT(&huart2, &route_rx, 1);
  }
}

/* -----------------------------------------------------------------------------
 *  Link_Send_Grab — PCB2 руу "GRAB эхлүүл" команд (USART2 TX, PS5-тэй нэг
 * шугам)
 *
 *  Багц: [0xC1][0xC2][type][0x0A] — давхар magic (0xC1,0xC2) нь PS5 урсгалтай
 *  санамсаргүй давхцахаас сэргийлнэ.  type: 0 = down_20, 1 = up_20 (платформ).
 *  PCB2 үүнийг ялгаж, төрөл + f/b тоологчоор аль грабыг ажиллуулахаа сонгоно.
 *  Forwarding идэвхтэй (gState=BUSY_TX) бол HAL_BUSY → чимээгүй алгасана;
 *  auto_climb grab-ийн хүлээх төлөвт давтан илгээдэг тул нэг нь хүрнэ. */
void Link_Send_Grab(uint8_t type) {
  uint8_t pkt[4] = {0xC1, 0xC2, type, 0x0A};
  HAL_UART_Transmit(&huart2, pkt, 4, 5);
}

/* -----------------------------------------------------------------------------
 *  Link_Query_Block — PCB2-оос "энэ блок дээр grab хийх үү?" асуух
 *    Багц: [0xC3][block][0x0A]  (block = 1..12). PCB2 тактикаа шалгаад
 *    [0xB6][1/0][0x0A] хариулна → g_grab_ans / g_grab_ans_rdy.               */
void Link_Query_Block(uint8_t block) {
  uint8_t pkt[3] = {0xC3, block, 0x0A};
  HAL_UART_Transmit(&huart2, pkt, 3, 5);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern int timer;
extern int counter[4];

uint8_t usb_received_value = 0;
uint8_t usb_new_data_flag = 0;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the  Flash interface and the Systick.
   */

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
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USART2 (PA3 RX) — PCB2-оос ирэх route-ыг ЭРТ зэвсэглэнэ. generalInit нь
     huart3-ыг зэвсэглэдэг ба түүний ISR huart2 TX-д Transmit_IT хийж болзошгүй
     тул huart2 RX-ийг ТҮҮНЭЭС ӨМНӨ армлавал HAL_LOCK-ийн race гарахгүй. */
  HAL_UART_Receive_IT(&huart2, &route_rx, 1);

  generalInit();

  /* Серво эхлэлийн байрлал (180°). generalInit нь PWM сувгийг асаадаг ч
     өнцөг бичдэггүй — энэгүйгээр серво асахдаа хаана байснаа тэндээ үлдэнэ. */
  Servo_Home();

  /* ===== RACK HOMING — encoder-ийн 0 цэгийг доод limit switch дээр тогтоох
   * ===== Тохируулга ХЭМЖИГДСЭН (Rack_Joystick_Test): FRONT = M6 / counter[1] /
   * S2 ,  BACK = M5 / counter[0] / S1 +PWM → counter ӨСНӨ  ⇒  −PWM нь ракыг
   * доод switch рүү БУУЛГАНА (homing зөв).
   *
   *  Preset түвшнүүд 0 цэгээс тоологддог тул homing ЗААВАЛ эхэлж хийгдэнэ.
   *  Switch олдохгүй бол (timeout) байрлал тодорхойгүй — preset рүү ОРОХГҮЙ.
   */
  uint8_t home_ok = Rack_SetHome(&frontRack);
  if (!Rack_SetHome(&backRack))
    home_ok = 0;

  if (!home_ok) {
    colorFill(Black);
    setCursor(4, 16);
    printStr("HOMING FAILED");
    setCursor(4, 36);
    printStr("check limit sw");
    setScreen();
    while (1) {
    } // байрлал тодорхойгүй — rack-ийг БҮҮ хөдөлгө
  }

  /* ===== LPMS GYRO =====
   *  UART4 дээр LPMS-ийг асааж, урсгалыг цэвэрлээд, yaw-ийн 0 цэгийг тогтооно.
   *  ⚠ Rack_Telemetry_Serial МӨН UART4 тул түүнийг ЗЭРЭГ БҮҮ ДУУД. */
  /* auto_climb (мод 2) нь Drive_Straight (gyro) ашиглах тул LPMS идэвхтэй байх
   * ёстой. ⚠ UART4-ийг LPMS эзэлнэ → мод 15/21-ийн Rack_Telemetry_Serial
   * ажиллахгүй болно. Рак serial тааруулга хийх бол ДООРХ 3 мөрийг түр comment
   * болгоно. */
  LPMS_Init();
  for (int i = 0; i < 50; i++) {
    LPMS_Read();
  } // stream-ийг цэвэрлэх (эхний хог)
  Gyro_ZeroYaw(); // одоогийн чиглэлийг 0 болгох

  /* ===== ГОРИМ СОНГОХ =====
   *  Share = дараах горим,  Options = сонгох.  БУЦАХГҮЙ — сонгосон горим
   *  өөрийн while(true)-д үлдэнэ. Мөн энэ нь эхлүүлэх хамгаалалт болно:
   *  robot тэжээл өгмөгц дүүлэхгүй, товч дартал хүлээнэ.                    */
  selectMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* selectMode() БУЦАХГҮЙ — горим бүр өөрийн while(true)-д үлддэг.
     Энд хүрэх нь зөвхөн ямар нэг зүйл буруудсан гэсэн үг.               */
  while (1) {
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 60000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 42 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 42 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000 - 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */
}

/**
 * @brief TIM13 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM13_Init(void) {

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 83;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 19999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

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
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M1InB_Pin | M2InA_Pin | Buzzer_Pin | M1InA_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(
      GPIOC, M2InB_Pin | M3InA_Pin | M3InB_Pin | OP6_Pin | OP5_Pin | OP1_Pin,
      GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD,
                    OP3_Pin | BnoReset_Pin | OP8_Pin | OP7_Pin | M4InA_Pin |
                        M4InB_Pin | M5InA_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OP4_Pin | OP2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M5InB_Pin | M6InA_Pin | M6InB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Sen2_Pin Sen3_Pin Sen4_Pin Sen1_Pin */
  GPIO_InitStruct.Pin = Sen2_Pin | Sen3_Pin | Sen4_Pin | Sen1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : M1InB_Pin M2InA_Pin Buzzer_Pin M1InA_Pin */
  GPIO_InitStruct.Pin = M1InB_Pin | M2InA_Pin | Buzzer_Pin | M1InA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : M2InB_Pin M3InA_Pin M3InB_Pin OP6_Pin
                           OP5_Pin OP1_Pin */
  GPIO_InitStruct.Pin =
      M2InB_Pin | M3InA_Pin | M3InB_Pin | OP6_Pin | OP5_Pin | OP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Sen5_Pin Sen6_Pin Sen7_Pin En4_In_Pin */
  GPIO_InitStruct.Pin = Sen5_Pin | Sen6_Pin | Sen7_Pin | En4_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Sen8_Pin */
  GPIO_InitStruct.Pin = Sen8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Sen8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OP3_Pin BnoReset_Pin OP8_Pin OP7_Pin
                           M4InA_Pin M4InB_Pin M5InA_Pin */
  GPIO_InitStruct.Pin = OP3_Pin | BnoReset_Pin | OP8_Pin | OP7_Pin | M4InA_Pin |
                        M4InB_Pin | M5InA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : OP4_Pin OP2_Pin */
  GPIO_InitStruct.Pin = OP4_Pin | OP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : En3_In_Pin En2_In_Pin */
  GPIO_InitStruct.Pin = En3_In_Pin | En2_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : En4_Ext_Pin En3_Ext_Pin */
  GPIO_InitStruct.Pin = En4_Ext_Pin | En3_Ext_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : M5InB_Pin M6InA_Pin M6InB_Pin */
  GPIO_InitStruct.Pin = M5InB_Pin | M6InA_Pin | M6InB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : En1_In_Pin */
  GPIO_InitStruct.Pin = En1_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(En1_In_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : En2_Ext_Pin En1_Ext_Pin */
  GPIO_InitStruct.Pin = En2_Ext_Pin | En1_Ext_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
