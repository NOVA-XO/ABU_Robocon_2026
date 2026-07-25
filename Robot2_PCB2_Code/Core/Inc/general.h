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

/* ---- USART2 холбоо: 1-р PCB-ээс ирэх өгөгдлийг турших ------------------- */
void    Link_Recv_Test(void);

/* ---- ҮНДСЭН ГАРЫН УДИРДЛАГА (PS5 нь PCB1-ээс USART2-оор ирнэ) ----------- */
void    PCB2_Manual(void);            // D-Up/Dn→M6, D-Left/Right→M5; ▭○△→соленоид 1,2,5

/* ---- SUN GEAR: Мотор 5 нарны араа (encoder0: 6000 count = 360°) --------- */
void    sun_gear(void);               // joystick jog + байрлал барих P-hold + UART4

/* ---- MOON GEAR: Мотор 6 / encoder1 (□→-148 △→0 O→140, ±1) --------------- */
void    moon_gear(void);              // RStickY jog + байрлал барих P-hold + UART4

/* ---- SUN-MOON: нэг мод дотор sun(face △○▭✕) + moon(D-pad) тусад нь --------- */
void    sun_moon(void);               // sun←face, moon←D-pad, хоёр P-hold + UART4

/* ---- AUTO SEQ: ▭ → sun-90 → sol5 → sol1 → moon120 (дараалсан) ----------- */
void    auto_seq(void);               // автомат дараалал + P-hold + UART4

/* ---- GRAB: PCB1-ийн GRAB команд → sun-90 → sol1+5 ON → 1сек → sol OFF ----
 *   Grab_Start() эхлүүлнэ, Grab_Service() бүр давталтад (sun барина), дуусмагц 1.
 *   Tactic_Task-аас дуудна. sun gear −90 дээрээ ҮЛДЭНЭ.                        */
void    Grab_Start(void);
uint8_t Grab_Service(void);
void    Grab_Test(void);              // шоо-авах механизмыг ТУСДАА турших (strafe-гүй)

/* ---- ШОО-АВАХ 14 COMBINATION (7 тохиолдол × sun 180° урд/ард; _f=урд _b=ард) --
 *   non-blocking: дуусвал 1. Одоохондоо ХООСОН (TODO). Эхлээд front-ыг бичнэ.   */
uint8_t grab_front_up_20_f(void);
uint8_t grab_front_up_20_b(void);
uint8_t grab_front_down_20_f(void);
void    grab_front_down_20_f_reset(void); // дараалал эхлүүлэх (Grab_Test дуудна)
uint8_t grab_front_down_20_b(void);
void    grab_front_down_20_b_reset(void); // дараалал эхлүүлэх (Grab_Test дуудна)
uint8_t grab_left_up_20_f(void);
uint8_t grab_left_up_20_b(void);
uint8_t grab_left_down_20_f(void);
uint8_t grab_left_down_20_b(void);
uint8_t grab_right_up_20_f(void);
uint8_t grab_right_up_20_b(void);
uint8_t grab_right_down_20_f(void);
uint8_t grab_right_down_20_b(void);
uint8_t grab_front_up_40_f(void);
uint8_t grab_front_up_40_b(void);

/* ---- ADC / OLED туслах --------------------------------------------------- */
uint16_t Read_PC3(void);
void     Read_PC3_OLED(void);

/* ---- USB CDC ------------------------------------------------------------- */
uint8_t USB_GetValue(void);      // хүлээн авсан утгыг буцаах
uint8_t USB_HasNewData(void);    // шинэ өгөгдөл ирсэн эсэх (1/0)
void    USB_ClearFlag(void);     // "шинэ өгөгдөл" flag-ыг цэвэрлэх
void    USB_Show_OLED(void);     // хүлээн авсан утгыг OLED дээр харуулах

#endif /* INC_GENERAL_H_ */