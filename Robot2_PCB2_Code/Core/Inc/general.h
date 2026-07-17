/* =============================================================================
 *  general.h — Хөдөлгөөний өндөр түвшний удирдлага (mecanum, серво, соленоид)
 *
 *  ЭНЭ БОЛ ROBOT2-ЫН 2 ДАХЬ PCB. Robot2_Code (1-р PCB)-аас гаралтай боловч
 *  rack, LPMS/gyro, sequence/blue/red дарааллуудыг ХАСАВ — тэдгээр тоног
 *  төхөөрөмж энэ самбар дээр байхгүй.
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 17, 2026
 *  Автор : nova
 * =============================================================================
 */

#ifndef INC_GENERAL_H_
#define INC_GENERAL_H_

#include "main.h"
#include "ssd1306.h"
#include "default.h"
#include <stdbool.h>

/* ---- Mecanum удирдлага --------------------------------------------------- */
int  applyDeadzone(int v);            // джойстик утгыг үхмэл бүсээр шүүх
void runner(void);                    // джойстикоор mecanum хөдөлгөөн

/* ---- Серво (1 серво, htim1 CH1), ГРАДУСААР — асаахад 180° ---------------- */
#define SERVO_DEG_MAX   180           // сервоны бүтэн зам (градус)
#define SERVO_HOME_DEG  SERVO_DEG_MAX // АСААХАД энэ байрлалаас эхэлнэ

void    Servo_SetDeg(int deg);        // сервог өнцгөөр тавих (0..SERVO_DEG_MAX)
void    Servo_Home(void);             // эхлэлийн байрлалд тавих (init-д ЗААВАЛ дуудна)
int     Servo_Preset_Buttons(void);   // △ → 0°/180° toggle (OLED-гүй), буц: өнцөг
void    Servo_Preset_Control(void);   // Servo_Preset_Buttons + OLED

/* ---- Соленоид 1 — D-Up товчоор toggle (debounce, давталтгүй) ------------- */
void    Solenoid_Control(void);

/* ---- ADC / OLED туслах --------------------------------------------------- */
uint16_t Read_PC3(void);
void     Read_PC3_OLED(void);

/* ---- USB CDC ------------------------------------------------------------- */
uint8_t USB_GetValue(void);      // хүлээн авсан утгыг буцаах
uint8_t USB_HasNewData(void);    // шинэ өгөгдөл ирсэн эсэх (1/0)
void    USB_ClearFlag(void);     // "шинэ өгөгдөл" flag-ыг цэвэрлэх
void    USB_Show_OLED(void);     // хүлээн авсан утгыг OLED дээр харуулах

#endif /* INC_GENERAL_H_ */