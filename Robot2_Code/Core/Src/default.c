/* =============================================================================
 *  default.c — Суурь драйверууд (мотор, соленоид, серво, UART, горим сонголт)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "default.h"
#include "test.h"
#include "blue.h"       // weapon_blue
#include "sequence.h"   // climb_1/2/3
#include <stdlib.h>
#include <stdio.h>
#include "string.h"
#include <stdbool.h>
#include "pca9685.h"

/* ---- main.c дахь periphery handle-ууд ---------------------------------- */
extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;

/* ---- Глобал төлөв (бусад файлд тодорхойлогдсон) ------------------------ */
extern int     counter[];            // encoder тоолуурууд
extern int     control_data[5][4];   // джойстик/товчлууруудын өгөгдөл
extern int     timer;                // TIM7-оос нэмэгддэг зөөлөн тоолуур
extern uint8_t data;                 // UART3 хүлээн авах 1 байт

/* -----------------------------------------------------------------------------
 *  generalInit — бүх periphery-г асааж, эхлүүлэх (main-ээс 1 удаа дуудна)
 * -----------------------------------------------------------------------------
 */
void generalInit(void)
{
    oledInit();
    PCA9685_Init(&hi2c2);

    HAL_UART_Receive_IT(&huart3, &data, 1);   // джойстик UART interrupt
    HAL_TIM_Base_Start_IT(&htim7);            // timer тоолуур

    /* Мотор PWM сувгууд */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // M1
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // M2
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // M3
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // M4
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // M5
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // M6

    /* Brush / серво PWM сувгууд */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // brush1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // brush2
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // brush direction
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // brush direction
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
}

/* -----------------------------------------------------------------------------
 *  controlSolenoid — 1..8 дугаартай соленоидыг асаах/унтраах
 *    solenoidNumber: 1..8 (хязгаараас гарвал errorFunction)
 *    state         : true = ON, false = OFF
 * -----------------------------------------------------------------------------
 */
void controlSolenoid(uint8_t solenoidNumber, bool state)
{
    if (solenoidNumber < 1 || solenoidNumber > 8) errorFunction();

    GPIO_TypeDef *ports[] = {OP1_GPIO_Port, OP2_GPIO_Port, OP3_GPIO_Port, OP4_GPIO_Port,
                             OP5_GPIO_Port, OP6_GPIO_Port, OP7_GPIO_Port, OP8_GPIO_Port};

    uint16_t pins[] = {OP1_Pin, OP2_Pin, OP3_Pin, OP4_Pin,
                       OP5_Pin, OP6_Pin, OP7_Pin, OP8_Pin};

    GPIO_PinState pinState = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(ports[solenoidNumber - 1], pins[solenoidNumber - 1], pinState);
}

/* -----------------------------------------------------------------------------
 *  errorFunction — сэргэхгүй алдааны төлөв: buzzer тасралтгүй дуугарна
 * -----------------------------------------------------------------------------
 */
void errorFunction(void)
{
    while (1) {
        buzzer;
        HAL_Delay(100);
        silent;
        HAL_Delay(100);
    }
}

/* -----------------------------------------------------------------------------
 *  controlServo — сервог PWM compare утгаар удирдах
 *    isServo: true = htim1 CH1, false = htim1 CH3
 *    angle  : 0..24000 (хязгаараас гарвал errorFunction)
 * -----------------------------------------------------------------------------
 */
void controlServo(bool isServo, int angle)
{
    if (angle > 24000 || angle < 0) errorFunction();

    if (isServo) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 24000 + angle);
    else         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 24000 + angle);
}

/* -----------------------------------------------------------------------------
 *  selectMode — OLED дээр горим сонгуулж, тухайн горимын тестийг ажиллуулах
 *    control_data[4][0] = "дараах" товч, control_data[4][1] = "сонгох" товч
 * -----------------------------------------------------------------------------
 */
/* -----------------------------------------------------------------------------
 *  Горимын нэрс — тоо ба нэр ХАМТ энд. MODE_COUNT нь массиваас бодогдоно,
 *  тиймээс горим нэмэхэд нэг л мөр засна (өмнө нь "if (mode == 14)" гэж
 *  тоог гараар бичдэг байсан — жагсаалттай зөрөх эрсдэлтэй).
 * -----------------------------------------------------------------------------
 */
static const char *const mode_name[] = {
    "RUN blue",         //  0  автомат дараалал
    "Joystick",         //  1
    "Encoder",          //  2
    "Sensor v1-v8",     //  3
    "Motor",            //  4
    "Solenoid",         //  5
    "Servo",            //  6
    "Rack preset",      //  7
    "Rack climb",       //  8
    "Drive straight",   //  9
    "Turn 90 + all",    // 10
    "Runner",           // 11
    "Climb 1",          // 12
    "Climb 2",          // 13
    "Climb 3",          // 14
    "Rack serial",      // 15  ⚠ UART4 — LPMS-тэй зөрчилдөнө
};
#define MODE_COUNT  ((int)(sizeof(mode_name) / sizeof(mode_name[0])))

/* Ракийн алдааг шалгаж, гарвал аюулгүй төлөвт (Robot_Error буцахгүй) */
static void mode_check_rack(void)
{
    uint8_t f = Rack_Fault();
    if (f) Robot_Error(f == 1 ? "RACK TIMEOUT" : "RACK SYNC");
}

/* -----------------------------------------------------------------------------
 *  selectMode — OLED дээр горим сонгуулж, тухайн горимыг ажиллуулах
 *    control_data[4][0] = "дараах" товч (Share),  control_data[4][1] = "сонгох" (Options)
 *
 *  ⚠ Горим бүр while(true) — БУЦАХГҮЙ. Сонгосны дараа зөвхөн reset.
 *  ⚠ Дуудахаас ӨМНӨ main.c-д generalInit / Servo_Home / rack homing / LPMS_Init
 *    хийгдсэн байх ёстой — gyro болон ракийн горимууд түүнээс хамаарна.
 * -----------------------------------------------------------------------------
 */
void selectMode(void)
{
    static bool flag = false;
    int mode = 0;

    /* ---- Горим гүйлгэх (сонгох товч дарагдтал) ---- */
    while (control_data[4][1] == 0) {

        LPMS_Read();   // сонголтын үед ч DMA буферээ хоослох (хуучирсан өгөгдөл хурааахгүй)

        if (control_data[4][0] == 1) {
            if (flag) {
                flag = false;
                mode++;
                if (mode >= MODE_COUNT) mode = 0;
            }
        } else {
            flag = true;
        }

        colorFill(Black);
        setCursor(10, 10);
        printStr("MODE %d/%d", mode, MODE_COUNT - 1);
        setCursor(10, 34);
        printStr("%s", mode_name[mode]);
        setScreen();
    }

    colorFill(Black);
    setCursor(10, 10);
    printStr("THIS MODE: %d ", mode);
    setCursor(10, 34);
    printStr("%s", mode_name[mode]);
    setScreen();

    /* ---- Сонгосон горимыг ажиллуулах (бүгд БУЦАХГҮЙ) ---- */
    switch (mode) {

    case 0:                                  // автомат: цэнхэр тал
        while (true) { weapon_blue(); }      // дотроо Rack_Fault шалгана

    case 1:
        while (true) { test_joyStick(); }

    case 2:
        while (true) { showEncoderStatus(); }

    case 3:
        while (true) { test_sensor(); }

    case 4:
        while (true) { test_motor(); }

    case 5:
        while (true) { testOptocoupler(); }

    case 6:                                  // △ → серво 0° ↔ 180°
        while (true) { Servo_Preset_Control(); }

    case 7:                                  // L1/L2/R1/R2 → 0/900/1350/1950
        while (true) { mode_check_rack(); Rack_Preset_Control(); }

    case 8:                                  // L1/R1 → хоёул,  L2/R2 → тус тусад нь
        while (true) { mode_check_rack(); Rack_Climb_Test(); }

    case 9:                                  // D-Up сэлгэх, L1/R1 хурд
        while (true) { Drive_Straight_Test(); }

    case 10:                                 // жолоо + 90° эргэлт + рак + серво
        while (true) { mode_check_rack(); Gyro_Turn_Test(); }

    case 11:                                 // зөвхөн гарын жолоо
        while (true) { LPMS_Read(); runner(); }

    case 12:
        while (true) { mode_check_rack(); climb_1_function(); }

    case 13:
        while (true) { mode_check_rack(); climb_2_function(); }

    case 14:
        while (true) { mode_check_rack(); climb_3_function(); }

    case 15:
        /* ⚠ UART4-ийг LPMS эзэлсэн. Энэ горимыг ашиглах бол main.c-д
           LPMS_Init()-ийг тайлбар болгож унтраах ёстой — эс тэгвээс хоёр
           протокол нэг шугам дээр холилдож хоёул ажиллахгүй.            */
        while (true) { Rack_Telemetry_Serial(); }

    default:
        while (true) { }
    }
}

/* -----------------------------------------------------------------------------
 *  motor_control — 1..6 дугаартай моторыг PWM + чиглэлээр удирдах
 *    type: мотор дугаар 1..6
 *    PWM : -1000..1000  (сөрөг = ухрах, эерэг = урагш; хэмжээ 1000-аар хязгаарлагдана)
 *
 *  PWM == 0 ирвэл тухайн моторыг ТООРМОСЛОНО (хоёр талын пин SET → богиносгол),
 *  зүгээр чөлөөтэй эргүүлэхгүй.
 * -----------------------------------------------------------------------------
 */
void motor_control(uint8_t type, int PWM)
{
    uint8_t direction = (PWM < 0) ? 0 : 1;   // 0 = ухрах, 1 = урагш
    uint8_t do_brake  = (PWM == 0);          // 0 → тоормослох
    if (PWM < 0)    PWM = -PWM;               // хэмжээ (абсолют утга) авах
    if (PWM > 1000) PWM = 1000;               // PWM дээд хязгаар

    switch (type)
    {
    case 1:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM); //M1
        if (do_brake) {
            HAL_GPIO_WritePin(M1InA_GPIO_Port, M1InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M1InB_GPIO_Port, M1InB_Pin, GPIO_PIN_SET);
        } else if (direction == 0) {
            HAL_GPIO_WritePin(M1InA_GPIO_Port, M1InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M1InB_GPIO_Port, M1InB_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(M1InA_GPIO_Port, M1InA_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M1InB_GPIO_Port, M1InB_Pin, GPIO_PIN_SET);
        }
        break;

    case 2:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM); //M2
        if (do_brake) {
            HAL_GPIO_WritePin(M2InA_GPIO_Port, M2InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M2InB_GPIO_Port, M2InB_Pin, GPIO_PIN_SET);
        } else if (direction == 0) {
            HAL_GPIO_WritePin(M2InA_GPIO_Port, M2InA_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M2InB_GPIO_Port, M2InB_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(M2InA_GPIO_Port, M2InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M2InB_GPIO_Port, M2InB_Pin, GPIO_PIN_RESET);
        }
        break;

    case 3:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM); //M3
        if (do_brake) {
            HAL_GPIO_WritePin(M3InA_GPIO_Port, M3InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M3InB_GPIO_Port, M3InB_Pin, GPIO_PIN_SET);
        } else if (direction == 1) {
            HAL_GPIO_WritePin(M3InA_GPIO_Port, M3InA_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M3InB_GPIO_Port, M3InB_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(M3InA_GPIO_Port, M3InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M3InB_GPIO_Port, M3InB_Pin, GPIO_PIN_RESET);
        }
        break;

    case 4:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM); //M4
        if (do_brake) {
            HAL_GPIO_WritePin(M4InA_GPIO_Port, M4InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M4InB_GPIO_Port, M4InB_Pin, GPIO_PIN_SET);
        } else if (direction == 0) {
            HAL_GPIO_WritePin(M4InA_GPIO_Port, M4InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M4InB_GPIO_Port, M4InB_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(M4InA_GPIO_Port, M4InA_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M4InB_GPIO_Port, M4InB_Pin, GPIO_PIN_SET);
        }
        break;

    case 5:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM); //M5
        if (do_brake) {
            HAL_GPIO_WritePin(M5InA_GPIO_Port, M5InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M5InB_GPIO_Port, M5InB_Pin, GPIO_PIN_SET);
        } else if (direction == 0) {
            HAL_GPIO_WritePin(M5InA_GPIO_Port, M5InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M5InB_GPIO_Port, M5InB_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(M5InA_GPIO_Port, M5InA_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M5InB_GPIO_Port, M5InB_Pin, GPIO_PIN_SET);
        }
        break;

    case 6:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM); //M6
        if (do_brake) {
            HAL_GPIO_WritePin(M6InA_GPIO_Port, M6InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M6InB_GPIO_Port, M6InB_Pin, GPIO_PIN_SET);
        } else if (direction == 0) {
            HAL_GPIO_WritePin(M6InA_GPIO_Port, M6InA_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(M6InB_GPIO_Port, M6InB_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(M6InA_GPIO_Port, M6InA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(M6InB_GPIO_Port, M6InB_Pin, GPIO_PIN_RESET);
        }
        break;

    default:
        // тодорхойгүй мотор дугаар — юу ч хийхгүй
        break;
    }
}

/* -----------------------------------------------------------------------------
 *  brake — Хөдөлгүүр 1..4-ийг нэг дуудалтаар тоормослох
 *  Тоормослох логик motor_control(x, 0) дотор нэг л газар байрлана.
 *  ТЭМДЭГЛЭЛ: rack мотор 5, 6-г энэ функц зогсоодоггүй.
 * -----------------------------------------------------------------------------
 */
void brake(void)
{
    motor_control(1, 0);
    motor_control(2, 0);
    motor_control(3, 0);
    motor_control(4, 0);
}

/* ---- UART туслах функцууд ----------------------------------------------- */

uint8_t rx_data[10];   // хүлээн авах буфер

/* -----------------------------------------------------------------------------
 *  rec_uart — huart4-аас өгөгдөл уншиж OLED дээр харуулах (blocking, 100ms)
 * -----------------------------------------------------------------------------
 */
void rec_uart(void)
{
    HAL_UART_Receive(&huart4, rx_data, sizeof(rx_data), 100);

    colorFill(Black);
    setCursor(4, 2);
    printStr("RX Data: %s", rx_data);
    setScreen();
}

/* -----------------------------------------------------------------------------
 *  send_uart — huart4-ээр текст мөр илгээх
 * -----------------------------------------------------------------------------
 */
void send_uart(char *message)
{
    HAL_UART_Transmit(&huart4, (uint8_t *)message, strlen(message), 100);
}

/* -----------------------------------------------------------------------------
 *  send_uart_int — бүхэл тоог текст болгож huart4-ээр илгээх
 * -----------------------------------------------------------------------------
 */
void send_uart_int(int value)
{
    char buffer[20];
    sprintf(buffer, "%d", value);
    HAL_UART_Transmit(&huart4, (uint8_t *)buffer, strlen(buffer), 100);
}
