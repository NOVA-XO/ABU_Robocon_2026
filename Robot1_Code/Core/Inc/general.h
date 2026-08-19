/* =============================================================================
 *  general.h — Хөдөлгөөний өндөр түвшний удирдлага (differential жолоодлого)
 *              (гар удирдлагатай робот — gyro/autonomous хэсэг хасагдсан)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */

#ifndef INC_GENERAL_H_
#define INC_GENERAL_H_

#include "main.h"
#include "ssd1306.h"
#include "default.h"
#include <stdbool.h>

/* ---- Differential (tank) жолоодлого -------------------------------------- */
int  applyDeadzone(int v);            // джойстик утгыг үхмэл бүсээр шүүх
void runner(void);                    // джойстикоор differential (tank) жолоодлого

/* ---- Актуатор удирдлага -------------------------------------------------- */
void solenoidControl(void);           // L1→sol1 R1→sol3+4 L2→sol2 (debounce + toggle)

/* ---- 2 PCB: main.c-д R1_PCB (1/2)-аар сонгож build хийнэ ----------------- */
void robot1_pcb1(void);               // 1-р PCB: жолоо+соленоид+мотор5/6 (main loop бие)
void robot1_pcb2(void);               // 2-р PCB: удирдлага (бөглөнө)

/* ---- 2 PCB хоорондын UART4 холбоо: [0x0A][pwm1][pwm2][pwm3][0x0D] (main.c) - */
void r1_link_send3(uint8_t p1, uint8_t p2, uint8_t p3);  // PCB1 → PCB2 руу 3 pwm илгээх
extern volatile uint8_t r1_link_p1, r1_link_p2, r1_link_p3; // PCB2: ирсэн pwm-ууд
extern volatile uint8_t r1_link_new;  // PCB2: 1=шинэ багц (уншаад 0 болгоно)

/* ---- ADC / OLED туслах --------------------------------------------------- */
uint16_t Read_PC3(void);
void     Read_PC3_OLED(void);

/* ---- USB CDC ------------------------------------------------------------- */
uint8_t USB_GetValue(void);      // хүлээн авсан утгыг буцаах
uint8_t USB_HasNewData(void);    // шинэ өгөгдөл ирсэн эсэх (1/0)
void    USB_ClearFlag(void);     // "шинэ өгөгдөл" flag-ыг цэвэрлэх
void    USB_Show_OLED(void);     // хүлээн авсан утгыг OLED дээр харуулах

#endif /* INC_GENERAL_H_ */
