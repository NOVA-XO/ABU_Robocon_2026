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

#endif /* INC_SEQUENCE_H_ */
