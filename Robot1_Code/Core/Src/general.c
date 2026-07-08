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
