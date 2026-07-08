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
void selectMode(void)
{
    static bool flag = false;
    int mode = 0;

    /* ---- Горим гүйлгэх (сонгох товч дарагдтал) ---- */
    while (control_data[4][1] == 0) {

        if (control_data[4][0] == 1) {
            if (flag) {
                flag = false;
                mode++;
                if (mode == 14) mode = 0;
            }
        } else {
            flag = true;
        }

        colorFill(Black);
        setCursor(10, 10);
        printStr("SELECT MODE: %d ", mode);

        setCursor(10, 30);
        if      (mode == 0)  printStr("Let's do it!");
        else if (mode == 1)  printStr("Test joystick!");
        else if (mode == 2)  printStr("Test encoder!");
        else if (mode == 3)  printStr("Test relay!");
        else if (mode == 4)  printStr("Test sensor!");
        else if (mode == 5)  printStr("Deesh Doosh!");
        else if (mode == 6)  printStr("Test opto!");
        else if (mode == 7)  printStr("Test motor!");
        else if (mode == 8)  printStr("Test servo!");
        else if (mode == 9)  printStr("Test hand!");
        else if (mode == 10) printStr("Test BLDC!");
        else if (mode == 11) printStr("Config urgugch!");
        else if (mode == 12) printStr("Test Serial");
        else if (mode == 13) printStr("Test solinoed!");
        setScreen();
    }

    colorFill(Black);
    setCursor(10, 10);
    printStr("THIS MODE: %d ", mode);
    setScreen();

    /* ---- Сонгосон горимыг ажиллуулах ---- */
    if (mode == 0) {
        while (true) {
            //mainCode();
        }
    }
    else if (mode == 1) {
        while (true) {
            test_joyStick();
        }
    }
    else if (mode == 2) {
        while (true) {
            showEncoderStatus();
        }
    }
    else if (mode == 3) {
        while (true) {
            test_joyStick();
            testOptocoupler();
            //testRelay();
        }
    }
    else if (mode == 4) {
        while (true) {
            test_sensor();
        }
    }
    else if (mode == 5) {
        while (true) {
        }
    }
    else if (mode == 6) {
        while (true) {
            testOptocoupler();
            test_motor();
        }
    }
    else if (mode == 7) {
        while (true) {
            test_motor();
            //test_BTS7960();
        }
    }
    else if (mode == 8) {
        while (true) {
            testServo();
        }
    }
    else if (mode == 9) {
        while (true) {
        }
    }
    else if (mode == 10) {
    }
    else if (mode == 11) {
        while (true) {
            //test_motor();
            //showEncoderStatus();
        }
    }
    else if (mode == 12) {
        while (true) {
            for (int i = 0; i < 1000; i++) {
                send_uart_int(i);
                send_uart("\n");
            }
        }
    }
    else if (mode == 13) {                 // ЗАСВАР: өмнө нь handler байхгүй байсан
        while (true) {
            testOptocoupler();             // соленоидуудыг control_data-аар шалгах
        }
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
