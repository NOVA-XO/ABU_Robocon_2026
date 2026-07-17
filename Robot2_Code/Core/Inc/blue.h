/* =============================================================================
 *  blue.h — Цэнхэр (blue) талын автомат дараалал (weapon_blue)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */

#ifndef INC_BLUE_H_
#define INC_BLUE_H_

#include "general.h"   // main.h, ssd1306.h, default.h, lpms.h-г дамжуулж авчирна

uint8_t weapon_blue(void);        // цэнхэр талбарын дараалал — дуусвал 1
void    weapon_blue_reset(void);  // эхнээс нь дахин ажиллуулах

#endif /* INC_BLUE_H_ */
