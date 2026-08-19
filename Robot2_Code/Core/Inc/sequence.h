/* =============================================================================
 *  sequence.h — Бүтэн үйлдлийн блокууд (non-blocking sequence blocks)
 *
 *  Улаан (red) ба цэнхэр (blue) тал давтан ашиглах бүтэн дарааллын блокууд.
 *  Блок бүр: xxx_function()-г main loop-д давтан дуудна, бүрэн дуусахад 1 буцаана;
 *            дахин ажиллуулахын өмнө xxx_reset() дуудна.
 *  Блокуудыг гинжлэн үндсэн автомат кодоо угсарна.
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */

#ifndef INC_SEQUENCE_H_
#define INC_SEQUENCE_H_

#include "general.h"   // Rack, Drive_Straight, brake, motor_control, val макро

/* ---- 20-ийн блокууд ------------------------------------------------------ */
uint8_t up_20_function(void);    // дуусвал 1, эс бөгөөс 0
void    up_20_reset(void);

uint8_t down_20_function(void);
void    down_20_reset(void);

/* ---- 40-ийн блокууд ------------------------------------------------------ */
uint8_t up_40_function(void);
void    up_40_reset(void);

uint8_t down_40_function(void);
void    down_40_reset(void);

/* ---- CLIMB: зам (уул) бүрийн бүтэн дэс дараалал (блокуудыг гинжилнэ) ------ */
uint8_t climb_1_function(void);   // ЗАМ 1: up_40 ×1 → down_20 ×1 → up_20 ×1 → down_20 ×2
void    climb_1_reset(void);
uint8_t climb_2_function(void);   // ЗАМ 2: up_20 ×3 → down_20 ×1 → down_40 ×1
void    climb_2_reset(void);
uint8_t climb_3_function(void);   // ЗАМ 3: up_40 ×1 → up_20 ×1 → down_20 ×3
void    climb_3_reset(void);

/* ---- AUTO CLIMB: PCB2-оос ирсэн route (g_route)-оор climb_1/2/3 сонгоно --- */
void    auto_climb(void);         // route латчилж, тэр замын climb-ыг дуустал явна

/* ---- GRAB STRAFE TEST: PCB2-ийн "Grab test" (мод 13)-тэй хамт (PCB1 strafe) - */
void    Grab_Strafe_Test(void);   // PCB2-оос ирэх 0xB7 команд дагаж зүүн/баруун strafe
void    Exit_Test(void);          // ГАРАХ маневрын рак-сунах хэсэг (D-Up эхлүүл)

/* ---- TICTACTOE: exit-ийн дараах үргэлжлэл (straight drive → val5==0 → ...) -- */
uint8_t tictactoe(void);          // non-blocking; 1 = дуусав
void    tictactoe_reset(void);
void    Tic_Tac_Toe_Test(void);   // тусдаа турших (D-Up эхлүүл)

#endif /* INC_SEQUENCE_H_ */
