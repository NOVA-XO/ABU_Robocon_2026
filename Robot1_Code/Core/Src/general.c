/* =============================================================================
 *  general.c — Хөдөлгөөний өндөр түвшний удирдлага
 *              (differential (tank) жолоодлого, ADC, USB CDC)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "general.h"
#include <stdlib.h>   // abs() — runner() дахь differential нормчлолд хэрэглэнэ

/* ---- main.c дахь periphery handle-ууд ---------------------------------- */
extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;

extern ADC_HandleTypeDef hadc1;

/* ---- Глобал төлөв ------------------------------------------------------- */
extern int counter[4];            // encoder тоолуурууд
extern int control_data[5][4];    // джойстик/товчлуурын өгөгдөл
extern int timer;                 // TIM7-оос нэмэгддэг зөөлөн тоолуур

extern uint8_t usb_received_value;
extern uint8_t usb_new_data_flag;


/* =============================================================================
 *  ЖОЛООДЛОГО — DIFFERENTIAL (tank / skid-steer)
 *
 *  Энэ робот MECANUM БИШ. Хажуу тийш (strafe) хөдөлгөөн БАЙХГҮЙ.
 *  Зүүн тал (мотор 1, 3) ба баруун тал (мотор 2, 4) тус бүр нэг хурдаар эргэнэ:
 *    урагш = хоёр тал ижил чиглэл,  эргэлт = хоёр тал эсрэг чиглэл.
 *
 *  ТЭМДЭГЛЭЛ: 4 хөдөлгүүрт (тал бүрт 2) гэж үзсэн — mecanum-ийн үеийн адил
 *  1..4 суваг жолоодлогынх. Хэрэв энэ робот ЗӨВХӨН 2 хөдөлгүүртэй бол
 *  мотор 3, 4 руу өгсөн нь хоосон явна (хор хөнөөлгүй); эсвэл доорх
 *  motor_control(3/4, ...) мөрүүдийг устгаж болно.
 * =============================================================================
 */
#define DEADZONE    10     // joystick үхмэл бүс
#define SPEED_GAIN  10     // урагш/хойш (V): стик 100 → 1000 PWM (бүрэн хурд)
#define TURN_GAIN    8     // эргэлт (W):    стик 100 → 800 PWM  ← эргэлтийг ИХЭСГЭХэд энэ утгыг нэмэгдүүл
#define PWM_MAX   1000     // моторын PWM дээд хязгаар (нормчлол энэ руу хийнэ)

/* -----------------------------------------------------------------------------
 *  applyDeadzone — joystick утгыг deadzone-оор шүүх (жижиг утгыг 0 болгоно)
 * -----------------------------------------------------------------------------
 */
int applyDeadzone(int v) {
    if (v > -DEADZONE && v < DEADZONE) return 0;
    return v;
}

/* -----------------------------------------------------------------------------
 *  runner — Джойстикоор differential (tank) жолоодлого
 *    Зүүн стик Y   → урагш/хойш (V)
 *    Баруун стик X → эргэлт (W)
 *    Зүүн тал  = V - W   (мотор 1, 3)
 *    Баруун тал = V + W  (мотор 2, 4)
 *  (Mecanum-ийн strafe гишүүн Vx-ийг л хассан тул шулуун явах/эргэх тэмдэг
 *   өмнөх кодтой ижил хэвээр.)
 * -----------------------------------------------------------------------------
 */
void runner(void) {
    // --- joystick унших (-100..100) ---
    int V = applyDeadzone(control_data[0][1]);   // зүүн стик Y → урагш/хойш
    int W = applyDeadzone(control_data[0][2]);   // баруун стик X → эргэлт

    // joystick Y нь дээш түлхэхэд сөрөг тул урагш = эерэг болгож эргүүлнэ
    V = -V;

    // --- Differential mixing (шууд PWM орон зайд; V ба W тус тусын gain-тай) ---
    int v_pwm = V * SPEED_GAIN;   // урагш/хойш:  ±800
    int w_pwm = W * TURN_GAIN;    // эргэлт:      ±800 (TURN_GAIN ихэсгэвэл эргэлт хүчтэй)
    int left  = v_pwm - w_pwm;    // Зүүн тал
    int right = v_pwm + w_pwm;    // Баруун тал

    // --- Нормчлол: аль нэг тал ±PWM_MAX хэтэрвэл харьцаагаа хадгалж багасгана ---
    //     (100 нэгжид биш, PWM орон зайд хийснээр явж байхад эргэлтийн хүч хадгалагдана)
    int m = abs(left);
    if (abs(right) > m) m = abs(right);
    if (m > PWM_MAX) {
        left  = left  * PWM_MAX / m;
        right = right * PWM_MAX / m;
    }

    // --- Моторт өгөх (утга аль хэдийн PWM) ---
    motor_control(1, right);   // Урд-Зүүн
    motor_control(2, right);   // Хойд-Зүүн
    motor_control(3, left);    // Урд-Баруун
    motor_control(4, left);    // Хойд-Баруун
}


/* =============================================================================
 *  СОЛЕНОИД УДИРДЛАГА — L1/R1/L2/R2 товчоор TOGGLE (нэг даралт = нэг сэлгэлт)
 *
 *    control_data[3][0] = L1  →  соленоид 1
 *    control_data[3][1] = R1  →  соленоид 5
 *    control_data[3][2] = L2  →  соленоид 2
 *    control_data[3][3] = R2  →  соленоид 4
 *
 *  Товч дарах бүрд тухайн соленоид HIGH ↔ LOW сэлгэнэ. Товчийг ДАРААД БАРЬЖ
 *  байхад давтахгүй (зөвхөн 0→1 ирмэг дээр нэг удаа). Механик чичиргээг
 *  (bounce) BTN_DEBOUNCE_MS-ээр шүүнэ.
 *
 *  Холболт өөрчлөгдвөл зөвхөн SOLENOID_MAP-ыг засна.
 * =============================================================================
 */
#define BTN_DEBOUNCE_MS  25   // товчийг тогтвортой гэж үзэх хугацаа (мс)
#define SOLENOID_COUNT    4   // control_data[3] дэх товчны тоо

/* control_data[3][i] товч  →  соленоидын дугаар (бодит холболтын дагуу) */
static const uint8_t SOLENOID_MAP[SOLENOID_COUNT] = { 1, 3, 2, 4 };
/*                                                    L1 R1 L2 R2 */

typedef struct {
    uint8_t  stable;      // debounce хийсэн төлөв
    uint8_t  raw_last;    // сүүлийн түүхий уншилт
    uint32_t change_ms;   // түүхий төлөв сүүлд өөрчлөгдсөн агшин
} Btn_t;

/* -----------------------------------------------------------------------------
 *  btn_pressed — Debounce хийсэн ЦЭВЭР ШИНЭ даралт (rising edge) таних
 *    return: 1 = дөнгөж дарагдлаа (0→1). Дараад барьж байхад дахин 1 БУЦААХГҮЙ.
 * -----------------------------------------------------------------------------
 */
static uint8_t btn_pressed(Btn_t *b, uint8_t raw) {
    uint32_t now = HAL_GetTick();

    if (raw != b->raw_last) {          // түүхий төлөв өөрчлөгдвөл цагийг эхлүүлнэ
        b->raw_last  = raw;
        b->change_ms = now;
    }

    // BTN_DEBOUNCE_MS турш тогтвортой байж байж л debounced төлвийг шинэчилнэ
    if ((now - b->change_ms) >= BTN_DEBOUNCE_MS && b->stable != raw) {
        b->stable = raw;
        if (raw) return 1;             // 0 → 1 шилжилт = шинэ даралт
    }

    return 0;
}

/* -----------------------------------------------------------------------------
 *  solenoidControl — L1/R1/L2/R2 тус бүрээр соленоид 1..4-ийг toggle хийх
 *  Main loop-оос давталт бүрд дуудна (блоклохгүй).
 * -----------------------------------------------------------------------------
 */
void solenoidControl(void) {
    static Btn_t btn[SOLENOID_COUNT]   = {0};
    static bool  state[SOLENOID_COUNT] = {false};
    static bool  inited = false;

    // Эхний дуудалтад бүх соленоидыг OFF болгож, программын төлөвтэй нийцүүлнэ
    if (!inited) {
        for (int i = 0; i < SOLENOID_COUNT; i++) controlSolenoid(SOLENOID_MAP[i], false);
        inited = true;
    }

    for (int i = 0; i < SOLENOID_COUNT; i++) {
        if (btn_pressed(&btn[i], (uint8_t)control_data[3][i])) {
            state[i] = !state[i];                          // HIGH ↔ LOW сэлгэх
            controlSolenoid(SOLENOID_MAP[i], state[i]);
        }
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
