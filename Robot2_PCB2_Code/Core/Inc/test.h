/* =============================================================================
 *  test.h — Тоног төхөөрөмжийн тест функцууд (encoder, servo, sensor, motor)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_

#include "main.h"
#include "default.h"

void showEncoderStatus(void);   // encoder + timer-ийг OLED дээр харуулах
void sendEncoderSerial(void);   // encoder 0/1-ийг UART4-ээр илгээх (115200, 100мс тутам)
void testOptocoupler(void);     // соленоидуудыг control_data-аар шалгах
void testServo(void);           // серво + brush тест
void test_sensor(void);         // val1..val8 сенсорыг OLED дээр харуулах
void test_joyStick(void);       // джойстик/товчлуурын өгөгдлийг харуулах
void test_motor(void);          // моторуудыг джойстикоор турших
void test_pwm(void);            // бүх PWM сувгийг 0..1000 гүйлгэх

#endif /* INC_TEST_H_ */
