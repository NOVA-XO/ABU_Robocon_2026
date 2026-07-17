/* =============================================================================
 *  lpms.c — LPMS-BE2 IMU драйвер (UART stream) ба gyro-д суурилсан PID
 *           (yaw эргэлт, roll налуу гарах, anchor чиглэл)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "lpms.h"

extern UART_HandleTypeDef huart4;

// =============================================================
//  LPMS protocol тогтмолууд
// =============================================================
#define ID            0x01
#define START         0x3A
#define END1          0x0D
#define END2          0x0A
#define COMMAND_MODE  0x06
#define STREAM_MODE   0x07

// =============================================================
//  LPMS-BE2 packet-ууд
// =============================================================
static uint8_t Packet_Stream[11] = {
    START, ID, 0x00, STREAM_MODE,  0x00, 0x00, 0x00, 0x08, 0x00, END1, END2
};

// =============================================================
//  Эулерийн өнцөг (бусад файлаас extern-ээр хэрэглэгдэнэ)
// =============================================================
float roll    = 0.0f;
float pitch   = 0.0f;
float yaw     = 0.0f;       // raw yaw (0..360)
float yaw_rel = 0.0f;       // тасралтгүй yaw (-360..+360)


// =============================================================
//  Sensor-ыг RST pin-ээр reset хийх
// =============================================================
void LPMS_Reset(void) {
    HAL_GPIO_WritePin(GYRO_RST_PORT, GYRO_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GYRO_RST_PORT, GYRO_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(1000);
}


// =============================================================
//  yaw_rel-ыг шинэчлэх (тасралтгүй -360..+360)
// =============================================================
// update_yaw_rel-ийн төлөв (Gyro_ZeroYaw эндээс дахин суурьшуулна)
static float   yr_prev_yaw = 0.0f;   // өмнөх raw yaw
static uint8_t yr_init     = 0;      // 0 бол дараагийн уншилтыг шинэ суурь болгоно

static void update_yaw_rel(void) {
    if (!yr_init) {
        yr_prev_yaw = yaw;
        yr_init     = 1;
        yaw_rel     = 0.0f;
        return;
    }

    // yaw delta-г wrap-аас цэвэрлэх
    float delta = yaw - yr_prev_yaw;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    yaw_rel += delta;

    if (yaw_rel >  360.0f) yaw_rel =  360.0f;
    if (yaw_rel < -360.0f) yaw_rel = -360.0f;

    yr_prev_yaw = yaw;
}


// =============================================================
//  Одоогийн yaw-г 0° болгон тэмдэглэх
//  ЗАСВАР: yr_init = 0 болгосноор update_yaw_rel дараагийн уншилтад
//  одоогийн yaw-г ШИНЭ суурь болгоно — өмнө нь yaw_rel үсэрдэг байсан.
// =============================================================
void Gyro_ZeroYaw(void) {
    yaw_rel = 0.0f;
    yr_init = 0;
}


// =============================================================
//  UART4 RX — DMA CIRCULAR + ring buffer
//
//  ЯАГААД: LPMS 70 Hz-ээр 27 байтын packet илгээнэ → байт бүр ~87µs тутам.
//  UART-д ердөө 1 БАЙТЫН регистр байдаг тул полингоор уншихад гогцоо 87µs-ээс
//  удаан блоклох бүрд (ж: OLED_Display ~25мс) байт АЛДАГДАЖ packet бүрддэггүй.
//  DMA нь байт бүрийг ТОНОГ ТӨХӨӨРӨМЖИЙН түвшинд буферт бичих тул гогцооны
//  хурднаас ҮЛ ХАМААРАН юу ч алдагдахгүй.
//
//  256 байт ≈ 135мс-ийн өгөгдөл (70Hz × 27B = 1890 B/s) — тайван нөөц.
//  DMA1 Stream2 Channel4 = UART4_RX (STM32F407). Stream4 = UART4_TX (аль хэдийн).
// =============================================================
#define LPMS_RX_BUF  256
static uint8_t  lpms_rx_buf[LPMS_RX_BUF];
static uint16_t lpms_rd = 0;     // унших байрлал (DMA-ийн бичих байрлалыг NDTR-ээс авна)
DMA_HandleTypeDef hdma_uart4_rx;

static void LPMS_DMA_Start(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_uart4_rx.Instance                 = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel             = DMA_CHANNEL_4;      // UART4_RX
    hdma_uart4_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode                = DMA_CIRCULAR;       // тасралтгүй эргэнэ
    hdma_uart4_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_uart4_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_uart4_rx);

    __HAL_LINKDMA(&huart4, hdmarx, hdma_uart4_rx);

    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

    lpms_rd = 0;
    HAL_UART_Receive_DMA(&huart4, lpms_rx_buf, LPMS_RX_BUF);
}

/* DMA1 Stream2 тасалдал — HAL-ийн төлөвийг хөтлөхөд шаардлагатай */
void DMA1_Stream2_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_uart4_rx);
}

/* Ring buffer-ээс НЭГ байт авах. return: 1 = байт байсан, 0 = хоосон (non-blocking) */
static uint8_t lpms_get(uint8_t *b) {
    uint16_t wr = (uint16_t)(LPMS_RX_BUF - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx));
    if (wr == lpms_rd) return 0;                       // хоосон
    *b = lpms_rx_buf[lpms_rd];
    lpms_rd = (uint16_t)((lpms_rd + 1) % LPMS_RX_BUF);
    return 1;
}


// =============================================================
//  LPMS-ийг тохируулж stream горимд оруулах
// =============================================================
void LPMS_Init(void) {
    //HAL_GPIO_WritePin(GYRO_RST_PORT, GYRO_RST_PIN, GPIO_PIN_SET); // <- tur comment bolgoson
    //HAL_Delay(100);

    //LPMS_Reset(); // <- tur comment bolgoson

    // UART буферийг цэвэрлэх
    uint8_t dummy;
    while (HAL_UART_Receive(&huart4, &dummy, 1, 5) == HAL_OK) { }

    // Stream горимд оруулах
    HAL_UART_Transmit(&huart4, Packet_Stream, sizeof(Packet_Stream), 100);

    while (HAL_UART_Receive(&huart4, &dummy, 1, 5) == HAL_OK) { }

    // ЭНЭ ЦЭГЭЭС хойш RX-ийг DMA хийнэ — байт алдагдахгүй.
    // (Дээрх полинг уншилтууд DMA асахаас ӨМНӨ дуусах ёстой.)
    LPMS_DMA_Start();

    yaw_rel = 0.0f;


    Set_Yaw_Anchor();

    // OLED дээр нэг удаа "LPMS Ready" харуулах
    colorFill(Black);
    setCursor(4, 2);
    printStr("LPMS Ready");
    setScreen();
}

// =============================================================
//  Stream-ээс packet уншиж roll/pitch/yaw-г шинэчлэх
//
//  Packet-ийн Euler хэсэг:
//      [17..22]  Euler X, Y, Z (3 × int16, * 100)
// =============================================================
/* -----------------------------------------------------------------------------
 *  lpms_feed — НЭГ байтыг frame-ийн state machine-д оруулах
 *    return: 1 = БҮРЭН packet задлагдаж roll/pitch/yaw шинэчлэгдэв
 * -----------------------------------------------------------------------------
 */
static uint8_t lpms_feed(uint8_t rx_byte) {
    static uint8_t rx_buf[50];
    static uint8_t rx_idx = 0;
    static uint8_t ready  = 0;

    // START байт хүлээх
    if (!ready) {
        if (rx_byte == START) {
            ready  = 1;
            rx_idx = 0;
            rx_buf[rx_idx++] = rx_byte;
        }
        return 0;
    }

    // Буферт нэмэх
    if (rx_idx < 50) {
        rx_buf[rx_idx++] = rx_byte;
    } else {
        ready  = 0;
        rx_idx = 0;
        return 0;
    }

    // ID шалгах
    if (rx_idx == 2 && rx_buf[1] != ID) {
        ready  = 0;
        rx_idx = 0;
        return 0;
    }

    // Packet төгсгөл (END1, END2) илрүүлэх
    if (rx_idx >= 7 &&
        rx_buf[rx_idx - 2] == END1 &&
        rx_buf[rx_idx - 1] == END2) {

        uint8_t packet_len = rx_idx;
        ready  = 0;
        rx_idx = 0;

        if (packet_len < 23) {
            return 0;
        }

        // Euler X, Y, Z задлах
        int16_t raw_x = (int16_t)(rx_buf[17] | (rx_buf[18] << 8));
        int16_t raw_y = (int16_t)(rx_buf[19] | (rx_buf[20] << 8));
        int16_t raw_z = (int16_t)(rx_buf[21] | (rx_buf[22] << 8));

        float fx = (float)raw_x / 100.0f;
        float fy = (float)raw_y / 100.0f;
        float fz = (float)raw_z / 100.0f;

        roll  = fx;
        pitch = fy;

        // Yaw-г 0..360 болгох
        if (fz < 0.0f) {
            yaw = fz + 360.0f;
        } else {
            yaw = fz;
        }

        // yaw_rel-ыг тасралтгүй болгож шинэчлэх
        update_yaw_rel();

        return 1;
    }

    return 0;
}

/* -----------------------------------------------------------------------------
 *  LPMS_Read — DMA буферт хуримтлагдсан БҮХ байтыг залгиж, packet задална
 *    return: 1 = энэ дуудалтад дор хаяж НЭГ packet шинэчлэгдэв, 0 = үгүй
 *
 *  Блоклохгүй. Нэг дуудалт = буфер бүрэн цэвэрлэгдэнэ. Тиймээс гогцоо удаан
 *  (ж: OLED 25мс) байсан ч байт алдагдахгүй — DMA буферлэсээр байна.
 *  Гогцоо 135мс-ээс удаан бол л буфер дүүрч эргэнэ (256 байт нөөц).
 * -----------------------------------------------------------------------------
 */
uint8_t LPMS_Read(void) {
    uint8_t got = 0;
    uint8_t b;

    while (lpms_get(&b)) {          // буфер хоосортол залгина
        if (lpms_feed(b)) got = 1;
    }
    return got;
}


// =============================================================
//  OLED дээр өнцгийг харуулах
// =============================================================
void OLED_Display(void) {
    colorFill(Black);
    setCursor(4, 2);
    printStr("== LPMS ==");

    setCursor(4, 14);
    printStr("R:%d", (int)roll);

    setCursor(4, 26);
    printStr("P:%d", (int)pitch);

    setCursor(4, 38);
    printStr("Y:%d", (int)yaw_rel);   // тасралтгүй yaw

    setCursor(4, 50);
    printStr("Yr:%d", (int)yaw);      // raw yaw (0..360)

    setScreen();
}


// =============================================================
//  Gyro PID — Yaw тэнхлэгээр эргэх
// =============================================================
#define TURN_KP        80.0f
#define TURN_KI        2.5f
#define TURN_KD        110.0f
#define TURN_TOLERANCE 0.5f
#define TURN_MAX_PWM   700
#define TURN_MIN_PWM   0
#define TURN_HOLD_MS   700
#define TURN_I_MAX     200.0f

// Эргэлтийн төлөв (бусад файлаас extern-ээр харагдах шаардлагагүй,
// зөвхөн доорх 3 функц дотор хэрэглэгдэнэ)
static float    turn_target       = 0.0f;
static float    turn_prev_err     = 0.0f;
static float    turn_integral     = 0.0f;
static int      turn_last_pwm     = 0;
static uint8_t  turn_done         = 1;
static uint32_t turn_in_zone_time = 0;


// Эргэлт эхлүүлэх
//   angle: + зүүн, - баруун
void Gyro_TurnInit(float angle) {
    turn_target       = yaw_rel + angle;   // тасралтгүй утга, wrap БИШ
    turn_prev_err     = 0.0f;
    turn_integral     = 0.0f;
    turn_last_pwm     = 0;
    turn_done         = 0;
    turn_in_zone_time = 0;
}


// PID гаралт — мотор руу өгөх PWM
int Gyro_TurnPID(void) {
    if (turn_done) {
        turn_last_pwm = 0;
        return 0;
    }

    // Error — wrap БАЙХГҮЙ, шууд хасах (yaw_rel тасралтгүй учраас)
    float error = turn_target - yaw_rel;

    // Integral (anti-windup-тай)
    turn_integral += error;
    if (turn_integral >  TURN_I_MAX) turn_integral =  TURN_I_MAX;
    if (turn_integral < -TURN_I_MAX) turn_integral = -TURN_I_MAX;

    // Derivative
    float derivative = error - turn_prev_err;
    turn_prev_err    = error;

    // PID гаралт
    float output = (TURN_KP * error)
                 + (TURN_KI * turn_integral)
                 + (TURN_KD * derivative);

    // PWM хязгаар
    if (output >  TURN_MAX_PWM) output =  TURN_MAX_PWM;
    if (output < -TURN_MAX_PWM) output = -TURN_MAX_PWM;

    // Зорилт хүрсэн эсэх (tolerance дотор тогтвортой барих)
    if (error > -TURN_TOLERANCE && error < TURN_TOLERANCE) {
        if (turn_in_zone_time == 0) {
            turn_in_zone_time = HAL_GetTick();
        }
        if (HAL_GetTick() - turn_in_zone_time >= TURN_HOLD_MS) {
            turn_done     = 1;
            turn_last_pwm = 0;
            return 0;
        }
    } else {
        turn_in_zone_time = 0;
    }

    // Min PWM (мотор хөдлөх босго)
    int pwm = (int)output;
    if (pwm > 0 && pwm <  TURN_MIN_PWM) pwm =  TURN_MIN_PWM;
    if (pwm < 0 && pwm > -TURN_MIN_PWM) pwm = -TURN_MIN_PWM;

    turn_last_pwm = pwm;
    return pwm;
}


uint8_t Gyro_TurnDone(void) {
    return turn_done;
}


// =============================================================
//  Үндсэн чиглэл хадгалах (0..360, raw yaw дээр суурилсан)
// =============================================================
float yaw_anchor = 0.0f;   // үндсэн чиглэл (0..360)
uint8_t yaw_anchor_set = 0;


// Одоогийн чиглэлийг үндсэн болгож тэмдэглэх
void Set_Yaw_Anchor(void) {
    // sensor шинэчлэх
    for (int i = 0; i < 3; i++) {
        LPMS_Read();
    }
    yaw_anchor = yaw;
    yaw_anchor_set = 1;
}


// Үндсэн чиглэлээс хэр зөрсөн (signed, -180..+180)
//   + бол зүүн зөрөөтэй, - бол баруун зөрөөтэй
//   (эсвэл чиний gyro-ын ороомгоос хамаарч эсрэгээрээ)
float Get_Yaw_Offset_From_Anchor(void) {
    float diff = yaw - yaw_anchor;

    // -180..+180 болгох (wrap)
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    return diff;
}


// =============================================================
//  Gyro PID — Налуу дээр гарах (Roll тэнхлэгээр)
// =============================================================
#define UP_KP        75.0f
#define UP_KI        2.5f
#define UP_KD        50.0f
#define UP_MAX_PWM   450
#define UP_MIN_PWM   300
#define UP_I_MAX     200.0f

static float   up_target   = 0.0f;
static float   up_prev_err = 0.0f;
static float   up_integral = 0.0f;
static int     up_last_pwm = 0;
static uint8_t up_done     = 1;


// target_roll: өөд гарах зорилтот өнцөг (жишээ нь -28)
void Gyro_UpInit(float target_roll) {
    up_target   = target_roll;
    up_prev_err = 0.0f;
    up_integral = 0.0f;
    up_last_pwm = 0;
    up_done     = 0;
}

void Gyro_UpReset(void) {
    up_target   = 0.0f;
    up_prev_err = 0.0f;
    up_integral = 0.0f;
    up_last_pwm = 0;
    up_done     = 1;
}

int Gyro_UpPID(void) {
    if (up_done) {
        up_last_pwm = 0;
        return 0;
    }

    // Зорилт хүрсэн эсэх
    if (roll <= up_target) {
        up_done     = 1;
        up_last_pwm = 0;
        return 0;
    }

    // Error: roll - target (эерэг утга = урагш явах)
    float error = roll - up_target;

    // Integral
    up_integral += error;
    if (up_integral >  UP_I_MAX) up_integral =  UP_I_MAX;
    if (up_integral < -UP_I_MAX) up_integral = -UP_I_MAX;

    // Derivative
    float derivative = error - up_prev_err;
    up_prev_err      = error;

    // PID гаралт
    float output = UP_KP * error + UP_KI * up_integral + UP_KD * derivative;

    // PWM хязгаар (зөвхөн эерэг)
    if (output > UP_MAX_PWM) output = UP_MAX_PWM;
    if (output < 0)          output = 0;

    // Min PWM (мотор хөдлөх босго)
    int pwm = (int)output;
    if (pwm > 0 && pwm < UP_MIN_PWM) pwm = UP_MIN_PWM;

    up_last_pwm = pwm;
    return pwm;
}


uint8_t Gyro_UpDone(void) {
    return up_done;
}
