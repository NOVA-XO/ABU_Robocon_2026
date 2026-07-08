/* =============================================================================
 *  lpms.h — LPMS-BE2 IMU драйвер ба gyro PID-ийн интерфейс
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */

#ifndef INC_LPMS_H_
#define INC_LPMS_H_

#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// =============================================================
//  Gyro RST pin
// =============================================================
#define GYRO_RST_PORT  GPIOB
#define GYRO_RST_PIN   GPIO_PIN_0

// =============================================================
//  LPMS үндсэн функцууд
// =============================================================
void    LPMS_Init(void);
void    LPMS_Reset(void);
uint8_t LPMS_Read(void);
void    OLED_Display(void);

// =============================================================
//  Gyro PID — Yaw тэнхлэгээр эргэх
// =============================================================
void    Gyro_ZeroYaw(void);
void    Gyro_TurnInit(float angle);
int     Gyro_TurnPID(void);
uint8_t Gyro_TurnDone(void);

// =============================================================
//  Gyro PID — Налуу дээр гарах (Roll тэнхлэгээр)
// =============================================================
void    Gyro_UpInit(float target_roll);
void Gyro_UpReset(void);
int     Gyro_UpPID(void);
uint8_t Gyro_UpDone(void);

extern float yaw_anchor;
extern uint8_t yaw_anchor_set;

void  Set_Yaw_Anchor(void);
float Get_Yaw_Offset_From_Anchor(void);


// =============================================================
//  Эулерийн өнцөг
//      roll, pitch — -180..+180 (raw)
//      yaw         — 0..360     (raw, sensor-аас ирсэн)
//      yaw_rel     — -360..+360 (тасралтгүй, Gyro_ZeroYaw()-аас харьцангуй)
// =============================================================
extern float roll;
extern float pitch;
extern float yaw;
extern float yaw_rel;

#endif /* INC_LPMS_H_ */
