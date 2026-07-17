/* =============================================================================
 *  default.h — Суурь драйверууд (мотор, соленоид, серво, UART)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */

#ifndef INC_DEFAULT_H_
#define INC_DEFAULT_H_

#include "main.h"
#include "ssd1306.h"
#include "general.h"
#include <stdbool.h>

/* ---- Тоног төхөөрөмжийн эхлүүлэлт ба удирдлага -------------------------- */
void generalInit(void);                                   // бүх periphery-г асаах
void controlSolenoid(uint8_t solenoidNumber, bool state); // 1..8 соленоид on/off
void controlServo(bool isServo, int angle);               // серво өнцөг өгөх
void motor_control(uint8_t type, int PWM);                // мотор 1..6, PWM: -1000..1000
void brake(void);                                         // хөдөлгүүр 1..4 тоормослох
void errorFunction(void);                                 // алдааны төлөв (buzzer гүйдэг)
void selectMode(void);                                    // OLED-оос горим сонгох

/* ---- UART туслах функцууд ----------------------------------------------- */
void send_uart(char *message);       // текст илгээх
void rec_uart(void);                 // өгөгдөл хүлээн авч OLED дээр харуулах
void send_uart_int(int value);       // бүхэл тоог текст болгож илгээх

#endif /* INC_DEFAULT_H_ */
