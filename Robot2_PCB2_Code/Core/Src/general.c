/* =============================================================================
 *  general.c — Хөдөлгөөний өндөр түвшний удирдлага
 *              (mecanum жолоо, серво, соленоид, ADC, USB)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "general.h"
#include <stdlib.h>   // abs() — runner() дахь mecanum нормчлолд хэрэглэнэ
#include <stdio.h>    // sprintf()

/* ---- main.c дахь periphery handle-ууд ---------------------------------- */
extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;   // PA2/PA3 — 1-р PCB-тэй холбоо

extern ADC_HandleTypeDef hadc1;

/* ---- Глобал төлөв ------------------------------------------------------- */
extern int counter[4];            // encoder тоолуурууд
extern int control_data[5][4];    // джойстик/товчлуурын өгөгдөл
extern int timer;                 // TIM7-оос нэмэгддэг зөөлөн тоолуур

extern uint8_t usb_received_value;
extern uint8_t usb_new_data_flag;

/* ---- USART2 холбоо (main.c-д тодорхойлогдсон, RX ISR бичнэ) ------------- */
extern volatile uint32_t link_byte_n;   // ирсэн байт
extern volatile uint32_t link_pkt;      // бүрэн задарсан багц
extern volatile uint32_t link_ms;       // сүүлийн багцын агшин

/* ---- Серво: одоогийн өнцөг (Servo_Preset_Control бичнэ, OLED уншина) ------
 *  SERVO_DEG_MAX нь general.h-д (Servo_SetDeg-ийн дуудагчид хэрэгтэй).       */
static int g_servo_deg = SERVO_DEG_MAX;     // АСААХАД 180°-оос эхэлнэ (0..SERVO_DEG_MAX)


/* =============================================================================
 *  MECANUM УДИРДЛАГА
 * =============================================================================
 */
#define DEADZONE    10     // joystick үхмэл бүс
#define SPEED_GAIN   8     // -100..100 утгыг PWM болгох коэффициент

/* -----------------------------------------------------------------------------
 *  applyDeadzone — joystick утгыг deadzone-оор шүүх (жижиг утгыг 0 болгоно)
 * -----------------------------------------------------------------------------
 */
int applyDeadzone(int v) {
    if (v > -DEADZONE && v < DEADZONE) return 0;
    return v;
}

/* -----------------------------------------------------------------------------
 *  runner — Джойстикоор mecanum хөдөлгөөн (inverse kinematics + нормчлол)
 * -----------------------------------------------------------------------------
 */
void runner(void) {
    // --- joystick унших (-100..100) ---
    int Vy = applyDeadzone(control_data[0][1]);   // зүүн стик Y → урагш/хойш
    int Vx = applyDeadzone(control_data[0][0]);   // зүүн стик X → хажуу (strafe)
    int W  = applyDeadzone(control_data[0][2]);   // баруун стик X → эргэлт

    // joystick Y нь дээш түлхэхэд сөрөг тул урагш = эерэг болгож эргүүлнэ
    Vy = -Vy;

    // --- Mecanum inverse kinematics ---
    int fl = Vy - Vx - W;   // Урд-Зүүн    (мотор 1)
    int fr = Vy + Vx + W;   // Урд-Баруун  (мотор 2)
    int rl = Vy + Vx - W;   // Хойд-Зүүн   (мотор 3)
    int rr = Vy - Vx + W;   // Хойд-Баруун (мотор 4)

    // --- Нормчлол: хамгийн их утга 100 хэтэрвэл бүгдийг хувь тэнцүүлж багасгана ---
    int m = abs(fl);
    if (abs(fr) > m) m = abs(fr);
    if (abs(rl) > m) m = abs(rl);
    if (abs(rr) > m) m = abs(rr);
    if (m > 100) {
        fl = fl * 100 / m;
        fr = fr * 100 / m;
        rl = rl * 100 / m;
        rr = rr * 100 / m;
    }

    // --- PWM болгож моторт өгөх ---
    motor_control(1, fl * SPEED_GAIN);   // Урд-Зүүн
    motor_control(2, fr * SPEED_GAIN);   // Урд-Баруун
    motor_control(3, rl * SPEED_GAIN);   // Хойд-Зүүн
    motor_control(4, rr * SPEED_GAIN);   // Хойд-Баруун
}


/* -----------------------------------------------------------------------------
 *  Товчны DEBOUNCE + edge/repeat таних туслах
 *    - Түүхий 1/0 хэлбэлзлийг BTN_DEBOUNCE_MS тогтвортой болтол шүүнэ.
 *    - Цэвэр даралт (rising edge) бүрт НЭГ алхам.
 *    - Дараад барьж байвал BTN_REPEAT_MS тутам давтана.
 * -----------------------------------------------------------------------------
 */
#define BTN_DEBOUNCE_MS  25    // тогтвортой гэж үзэх хугацаа
#define BTN_REPEAT_MS   120    // барьж байхад давтах интервал

typedef struct {
    uint8_t  stable;      // debounced төлөв
    uint8_t  raw_last;    // сүүлийн түүхий уншилт
    uint32_t change_ms;   // түүхий төлөв өөрчлөгдсөн хугацаа
    uint32_t repeat_ms;   // сүүлийн repeat алхмын хугацаа
} Btn_t;

// return: энэ дуудалтад "нэг алхам хий" гэвэл 1 (rising edge эсвэл repeat)
//   unused: одоогоор зөвхөн btn_rising ашиглагдаж байна (давталттай хувилбар нь
//   утга алхмаар тааруулахад хэрэг болно — тиймээс хасалгүй үлдээв).
__attribute__((unused))
static uint8_t btn_step(Btn_t *b, uint8_t raw) {
    uint32_t now = HAL_GetTick();

    if (raw != b->raw_last) {          // түүхий төлөв өөрчлөгдвөл цаг эхлүүлнэ
        b->raw_last  = raw;
        b->change_ms = now;
    }

    // тогтвортой болтол хүлээж байж л debounced төлвийг шинэчилнэ
    if ((now - b->change_ms) >= BTN_DEBOUNCE_MS && b->stable != raw) {
        b->stable = raw;
        if (raw) {                     // цэвэр шинэ даралт → эхний алхам
            b->repeat_ms = now;
            return 1;
        }
    }

    if (b->stable && (now - b->repeat_ms) >= BTN_REPEAT_MS) {  // барьж байвал давтах
        b->repeat_ms = now;
        return 1;
    }

    return 0;
}


/* -----------------------------------------------------------------------------
 *  btn_rising — Debounce хийсэн ЦЭВЭР ШИНЭ даралт (0→1), ДАВТАЛТГҮЙ
 *
 *  btn_step нь товчийг барьж байхад BTN_REPEAT_MS тутам давтдаг. Preset-д
 *  давталт хэрэггүй тул давталтгүй хувилбар.
 * -----------------------------------------------------------------------------
 */
static uint8_t btn_rising(Btn_t *b, uint8_t raw) {
    uint32_t now = HAL_GetTick();

    if (raw != b->raw_last) {          // түүхий төлөв өөрчлөгдвөл цаг эхлүүлнэ
        b->raw_last  = raw;
        b->change_ms = now;
    }

    if ((now - b->change_ms) >= BTN_DEBOUNCE_MS && b->stable != raw) {
        b->stable = raw;
        if (raw) return 1;             // зөвхөн 0 → 1 шилжилтэд
    }

    return 0;
}

/* =============================================================================
 *  СЕРВО УДИРДЛАГА — ГРАДУСААР (1 серво, htim1 CH1)
 *
 *    Triangle △  →  0° ↔ 180° СЭЛГЭХ (debounce, давталтгүй)
 *
 *  Өөр товч ашиглахгүй — 0/180-аас өөр байрлал хэрэггүй тул алхмаар тааруулах
 *  (✕ +5° / ○ 0° / ▭ 180°) хувилбарыг хассан.
 *
 *  Одоогийн өнцөг g_servo_deg-д хадгалагдаж OLED дээр харагдана.
 *  АСААХАД 180°-оос эхэлнэ (эхлэлийн байрлал).
 *
 *  ⚠ controlServo нь 0..SERVO_UNIT_MAX-аас гарсан утга авбал errorFunction()
 *    руу орж роботыг БҮРЭН ГАЦААНА (buzzer-ийн хязгааргүй гогцоо). Тиймээс
 *    градусыг хааж, дараа нь л unit болгож хөрвүүлнэ.
 * =============================================================================
 */
#define SERVO_UNIT_MAX  24000   // controlServo-ийн дээд утга (= SERVO_DEG_MAX градус)

/* -----------------------------------------------------------------------------
 *  servo_deg_to_units — градус (0..180) → controlServo-ийн unit (0..24000)
 *  Хязгаарыг ЭНД хаана — errorFunction()-аас сэргийлнэ.
 * -----------------------------------------------------------------------------
 */
static int servo_deg_to_units(int deg) {
    if (deg < 0)             deg = 0;
    if (deg > SERVO_DEG_MAX) deg = SERVO_DEG_MAX;
    return (deg * SERVO_UNIT_MAX) / SERVO_DEG_MAX;   // 180° → 24000
}

/* -----------------------------------------------------------------------------
 *  Servo_SetDeg — сервог өнцгөөр тавих (автомат дараалалд ашиглана)
 *
 *  g_servo_deg-ийг МӨН шинэчилнэ — эс тэгвээс △ toggle нь одоогийн байрлалаас
 *  шийддэг тул автомат дараалал сервог зөөсний дараа буруу тал руу үсэрнэ.
 * -----------------------------------------------------------------------------
 */
void Servo_SetDeg(int deg) {
    if (deg < 0)             deg = 0;
    if (deg > SERVO_DEG_MAX) deg = SERVO_DEG_MAX;
    g_servo_deg = deg;
    controlServo(true, servo_deg_to_units(deg));   // htim1 CH1
}

/* -----------------------------------------------------------------------------
 *  Servo_Home — эхлэлийн байрлалд (180°) тавих.  Init-д ЗААВАЛ дуудна.
 *
 *  g_servo_deg нь 180-аар эхэлдэг ч энэ нь зөвхөн ПРОГРАММЫН утга — hardware
 *  руу бичигдэхгүй. Өмнө нь зөвхөн Servo_Preset_Buttons-ийн inited блок бичдэг
 *  байсан тул автомат дараалал (тэр функцийг дуудахгүй) үед серво асахдаа
 *  хаана байснаа тэндээ үлдэж, g_servo_deg-тэй зөрж байв.
 * -----------------------------------------------------------------------------
 */
void Servo_Home(void) {
    Servo_SetDeg(SERVO_HOME_DEG);
}

/*  Товчны логик — OLED-ГҮЙ (дэлгэцийг дуудагч нь өөрөө зурна).
 *  Буцаах: одоогийн өнцөг.                                                 */
int Servo_Preset_Buttons(void) {
    static Btn_t   bTri   = {0};
    static uint8_t inited = 0;

    if (!inited) {                 // асаахад мэдэгдэж буй байрлалд тавина
        controlServo(true, servo_deg_to_units(g_servo_deg));
        inited = 1;
    }

    /* --- △ : нэг даралт = 0° ↔ 180° сэлгэх (debounce, давталтгүй) --- */
    if (btn_rising(&bTri, (uint8_t)control_data[1][2])) {
        g_servo_deg = (g_servo_deg >= SERVO_DEG_MAX / 2) ? 0 : SERVO_DEG_MAX;
        controlServo(true, servo_deg_to_units(g_servo_deg));   // htim1 CH1
    }

    return g_servo_deg;
}

void Servo_Preset_Control(void) {
    int deg = Servo_Preset_Buttons();

    /* --- OLED (100мс тутам): өнцөг + timer-т бичигдсэн бодит unit --- */
    static uint32_t t = 0;
    if (HAL_GetTick() - t >= 100) {
        t = HAL_GetTick();
        colorFill(Black);
        setCursor(2, 2);
        printStr("SERVO");
        setCursor(2, 22);
        printStr("%d deg", deg);
        setCursor(2, 42);
        printStr("u:%d", servo_deg_to_units(deg));
        setScreen();
    }
}


/* =============================================================================
 *  СОЛЕНОИД 1 — D-Up товчоор TOGGLE (нэг даралт = нэг сэлгэлт)
 *
 *    control_data[2][2] = D-Up  →  соленоид 1
 *
 *  Товч дарах бүрд соленоид ON ↔ OFF сэлгэнэ. Дараад БАРЬЖ байхад давтахгүй
 *  (зөвхөн 0→1 ирмэг дээр), механик чичиргээг BTN_DEBOUNCE_MS-ээр шүүнэ.
 *  Асаахад мэдэгдэж буй OFF төлөвт тавина.
 * =============================================================================
 */
#define SOLENOID1_BTN   2   // control_data[2][SOLENOID1_BTN] = D-Up
#define SOLENOID1_NUM   1   // controlSolenoid-ийн дугаар

void Solenoid_Control(void) {
    static Btn_t   btn    = {0};
    static bool    state  = false;
    static uint8_t inited = 0;

    if (!inited) {                                    // программын төлөвтэй нийцүүлнэ
        controlSolenoid(SOLENOID1_NUM, false);
        inited = 1;
    }

    if (btn_rising(&btn, (uint8_t)control_data[2][SOLENOID1_BTN])) {
        state = !state;                               // ON ↔ OFF
        controlSolenoid(SOLENOID1_NUM, state);
    }
}


/* =============================================================================
 *  ADC (PC3) унших
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 *  Read_PC3 — ADC1-ээс нэг утга унших (12-bit)
 * -----------------------------------------------------------------------------
 */
uint16_t Read_PC3(void)
{
    uint16_t value = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return value;
}

/* -----------------------------------------------------------------------------
 *  Read_PC3_OLED — ADC утгыг OLED дэлгэц дээр харуулах
 * -----------------------------------------------------------------------------
 */
void Read_PC3_OLED(void)
{
    uint16_t adc_value = Read_PC3();

    colorFill(Black);
    setCursor(10, 10);
    printStr("ADC: %d", adc_value);
    setScreen();
}


/* =============================================================================
 *  USB CDC ФУНКЦУУД
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 *  USB_GetValue — Хүлээн авсан утгыг буцаах (хэзээ ч дуудаж болно)
 * -----------------------------------------------------------------------------
 */
uint8_t USB_GetValue(void)
{
    return usb_received_value;
}

/* -----------------------------------------------------------------------------
 *  USB_HasNewData — Шинэ өгөгдөл ирсэн эсэх (1 = бий, 0 = үгүй)
 * -----------------------------------------------------------------------------
 */
uint8_t USB_HasNewData(void)
{
    return usb_new_data_flag;
}

/* -----------------------------------------------------------------------------
 *  USB_ClearFlag — flag-ыг цэвэрлэх (өгөгдлийг уншиж дууссаны дараа дуудна)
 * -----------------------------------------------------------------------------
 */
void USB_ClearFlag(void)
{
    usb_new_data_flag = 0;
}

/* -----------------------------------------------------------------------------
 *  USB_Show_OLED — USB утгыг OLED дэлгэц дээр төлөвийн текстээр харуулах
 * -----------------------------------------------------------------------------
 */
void USB_Show_OLED(void)
{
    colorFill(Black);

    setCursor(10, 10);
    printStr("USB: %d", usb_received_value);

    setCursor(10, 40);

    // Утганд тохирох текст харуулах
    if (usb_received_value == '0' || usb_received_value == 0)
    {
        printStr("Status: hooson");
    }
    else if (usb_received_value == '1' || usb_received_value == 1)
    {
        printStr("Status: fake");
    }
    else if (usb_received_value == '2' || usb_received_value == 2)
    {
        printStr("Status: real");
    }
    else
    {
        printStr("Status: ???");
    }

    setScreen();
}


/* =============================================================================
 *  Link_Recv_Test — 1-р PCB-ээс ирэх PS5 удирдлагыг шалгах (USART2, PA3 = RX)
 *
 *  PS5 нэг ширхэг, PCB1-д залгаастай. PCB1 нь ESP32-оос ирсэн 23 байтыг яг тэр
 *  хэвээр дамжуулж, энд ps5_feed() задлаад control_data[5][4]-г дүүргэнэ.
 *  Тиймээс энэ самбарын БҮХ код (runner, серво, соленоид) ӨӨРЧЛӨЛТГҮЙ ажиллана.
 *
 *  Утас: PCB1 PA2 (TX) → PCB2 PA3 (RX),  БАС GND ↔ GND (заавал).
 *
 *  Уншиж дүгнэх — b (байт) ба p (багц) хоёрыг ЗААВАЛ хамт харна:
 *    b өсөж, p өсөж    → бүх зүйл зөв
 *    b өсөж, p ЗОГССОН → байт ирж байгаа ч багц бүрдэхгүй: baud зөрөх юм уу
 *                        байт алдагдаж байна (framing хэзээ ч бүтэхгүй)
 *    b ч өсөхгүй       → утас/чиглэл буруу, GND алга, эсвэл PCB1 асаагүй
 *    STALE             → өмнө ирж байсан, одоо тасарсан
 * =============================================================================
 */
void Link_Recv_Test(void) {
    static uint32_t t = 0;
    if (HAL_GetTick() - t < 100) return;
    t = HAL_GetTick();

    /* volatile-ыг НЭГ л удаа уншина — ISR дунд нь өөрчилвөл дэлгэц зөрнө */
    uint32_t b   = link_byte_n;
    uint32_t p   = link_pkt;
    uint32_t age = HAL_GetTick() - link_ms;

    colorFill(Black);
    setCursor(2, 2);
    printStr("LINK RX");
    setCursor(2, 18);
    printStr("b:%lu p:%lu", (unsigned long)b, (unsigned long)p);
    setCursor(2, 34);
    printStr("LX:%d LY:%d", control_data[0][0], control_data[0][1]);
    setCursor(2, 50);
    if      (p == 0 && b == 0) printStr("NO DATA");
    else if (p == 0)           printStr("BYTES, NO PKT");
    else if (age > 500)        printStr("STALE %lums", (unsigned long)age);
    else                       printStr("OK");
    setScreen();
}


/* =============================================================================
 *  PCB2_Manual — 2 дахь PCB-ийн ҮНДСЭН ГАРЫН УДИРДЛАГА
 *
 *  Энэ самбар ӨӨРИЙН PS5-тай (ESP32 → USART3). Нэг гарын товч хүрэлцэхгүй
 *  болсон тул хоёр робот ТУСДАА удирдагдана — PCB1-ээс USART2-оор ирэх
 *  урсгалыг энд ХЭРЭГЛЭХГҮЙ (main.c-ийн тайлбарыг үз).
 *
 *  МОТОР (дарж байхад эргэнэ, тавихад зогсоно):
 *    L1 / L2       → мотор 6,  500 PWM   (L1 = +, L2 = −)
 *    R1 / R2       → мотор 4,  500 PWM   (R1 = +, R2 = −)
 *    D-Up / D-Down → мотор 5, 1000 PWM   (D-Up = +, D-Down = −)
 *
 *  СОЛЕНОИД (нэг даралт = toggle, debounce):
 *    ✕ → 1    ▭ → 2    △ → 3    ○ → 4    D-Left → 5    D-Right → 6
 *
 *  ⚠ Чиглэл буруу бол доорх #define-ийн тэмдгийг сольж болно, эсвэл товчны
 *    хосыг соль (ж: L1/L2-г солих). Мотор эргэх чиглэлийг би мэдэхгүй.
 * =============================================================================
 */
#define PCB2_M6_PWM   500   // L1 / L2
#define PCB2_M4_PWM   500   // R1 / R2
#define PCB2_M5_PWM  1000   // D-Up / D-Down

/* -----------------------------------------------------------------------------
 *  hold_axis — хоёр товчийг нэг тэнхлэг болгох (дарж байхад эргэнэ)
 *    Хоёуланг нь ЗЭРЭГ дарвал 0 — эс тэгвээс аль нэг нь "хожиж" гэнэт хөдөлнө.
 *    Debounce хэрэггүй: түүхий хэлбэлзэл нэг л loop-д мотор бага зэрэг цохилно.
 * -----------------------------------------------------------------------------
 */
static int hold_axis(uint8_t pos, uint8_t neg, int pwm) {
    if (pos && !neg) return  pwm;
    if (neg && !pos) return -pwm;
    return 0;
}

void PCB2_Manual(void) {
    static Btn_t   bs[6]   = {0};
    static bool    st[6]   = {false};
    static uint8_t inited  = 0;

    if (!inited) {                       // программын төлөвтэй нийцүүлнэ
        for (int i = 0; i < 6; i++) controlSolenoid(i + 1, false);
        inited = 1;
    }

    /* --- Мотор: hold --- */
    /*   control_data[3]: [0]=L1 [1]=R1 [2]=L2 [3]=R2
     *   control_data[2]: [0]=D-Down [1]=D-Right [2]=D-Up [3]=D-Left      */
    motor_control(6, hold_axis((uint8_t)control_data[3][0],
                               (uint8_t)control_data[3][2], PCB2_M6_PWM));  // L1 / L2
    motor_control(4, hold_axis((uint8_t)control_data[3][1],
                               (uint8_t)control_data[3][3], PCB2_M4_PWM));  // R1 / R2
    motor_control(5, hold_axis((uint8_t)control_data[2][2],
                               (uint8_t)control_data[2][0], PCB2_M5_PWM));  // D-Up / D-Down

    /* --- Соленоид 1..4: ✕ ▭ △ ○  (control_data[1][0..3] ижил дараалалтай) --- */
    for (int i = 0; i < 4; i++) {
        if (btn_rising(&bs[i], (uint8_t)control_data[1][i])) {
            st[i] = !st[i];
            controlSolenoid(i + 1, st[i]);
        }
    }
    /* --- Соленоид 5, 6: D-Left / D-Right --- */
    if (btn_rising(&bs[4], (uint8_t)control_data[2][3])) {   // D-Left
        st[4] = !st[4];
        controlSolenoid(5, st[4]);
    }
    if (btn_rising(&bs[5], (uint8_t)control_data[2][1])) {   // D-Right
        st[5] = !st[5];
        controlSolenoid(6, st[5]);
    }

    /* --- OLED (100мс тутам) --- */
    static uint32_t t = 0;
    if (HAL_GetTick() - t >= 100) {
        t = HAL_GetTick();

        char sol[7];                       // соленоидын төлөв: "123..6" / "-"
        for (int i = 0; i < 6; i++) sol[i] = st[i] ? ('1' + i) : '-';
        sol[6] = '\0';

        colorFill(Black);
        setCursor(2, 2);
        printStr("PCB2 MANUAL");
        setCursor(2, 18);
        printStr("M6:%d M4:%d",
                 hold_axis((uint8_t)control_data[3][0], (uint8_t)control_data[3][2], PCB2_M6_PWM),
                 hold_axis((uint8_t)control_data[3][1], (uint8_t)control_data[3][3], PCB2_M4_PWM));
        setCursor(2, 34);
        printStr("M5:%d",
                 hold_axis((uint8_t)control_data[2][2], (uint8_t)control_data[2][0], PCB2_M5_PWM));
        setCursor(2, 50);
        printStr("SOL:%s", sol);
        setScreen();
    }
}
