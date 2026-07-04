/* =============================================================================
 *  general.h — Хөдөлгөөний өндөр түвшний удирдлага (mecanum, rack, gyro)
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
#include "lpms.h"
#include <stdbool.h>

/* ---- Mecanum удирдлага --------------------------------------------------- */
int  applyDeadzone(int v);            // джойстик утгыг үхмэл бүсээр шүүх
void runner(void);                    // джойстикоор mecanum хөдөлгөөн
void Drive_Straight(int base_pwm);    // gyro-гоор шулуун явах

/* ---- Gyro-д суурилсан эргэлт --------------------------------------------- */
uint8_t Gyro_TurnAngle(float angle);  // харьцангуй өнцгөөр эргэх (non-blocking)

/* -----------------------------------------------------------------------------
 *  Rack_t — Нэг рак = нэг мотор + нэг encoder + өөрийн БҮРЭН PID
 * -----------------------------------------------------------------------------
 */
typedef struct {
    /* --- холболт --- */
    uint8_t  motor;       // motor_control дугаар (5 / 6)
    uint8_t  enc;         // counter[] индекс (0 / 1)

    /* --- зам ба PID тохиргоо (рак бүрт тусад нь тааруул) --- */
    int      pos_min;
    int      pos_max;     // бүтэн зам (count)
    float    kp, ki, kd;
    float    i_max;       // integral хязгаар (anti-windup)
    int      pwm_up;      // ДЭЭШ явах хурд (хүчтэй)
    int      pwm_down;    // ДООШ явах хурд (хэвийн)
    int      hold_pwm;    // байрлал барих feedforward (таталцал нөхөх)
    int      min_pwm;     // мотор хөдлөх доод босго
    int      tolerance;   // зорилтод хүрсэн гэж үзэх хүлцэл

    /* --- ISR-service горим (Rack_Service TIM7 ISR-т ажиллана) --- */
    int      target;      // зорилтот байрлал (Rack_SetTarget тавина)
    uint8_t  active;      // 1 бол Rack_Service энэ ракыг байнга барина

    /* --- дотоод төлөв (гараар бүү хүр) --- */
    float    prev_err;
    float    integral;
    uint32_t last_ms;
} Rack_t;

extern Rack_t frontRack;   // мотор 5 / counter[0]
extern Rack_t backRack;    // мотор 6 / counter[1]

void    Rack_SetHome(Rack_t *r);            // үзүүрт аваачаад дуудна (0 цэг)
void    Rack_Reset(Rack_t *r);              // PID төлвийг л цэвэрлэх
void    Rack_Jog(Rack_t *r, int dir);       // dir: +1 дээш, -1 доош, 0 зогс
uint8_t Rack_GoTo(Rack_t *r, int target);   // байрлал руу; хүрвэл 1, эс бөгөөс 0

/* ISR-service горим: зорилт тавиад TIM7 ISR-т Rack_Service байнга барина */
void    Rack_SetTarget(Rack_t *r, int target);  // зорилт тавьж, active = 1
void    Rack_Off(Rack_t *r);                    // active = 0, мотор зогсоох
void    Rack_Service(void);                     // TIM7 ISR-т дуудна (2 ракыг барина)

/* ---- ADC / OLED туслах --------------------------------------------------- */
uint16_t Read_PC3(void);
void     Read_PC3_OLED(void);

/* ---- USB CDC ------------------------------------------------------------- */
uint8_t USB_GetValue(void);      // хүлээн авсан утгыг буцаах
uint8_t USB_HasNewData(void);    // шинэ өгөгдөл ирсэн эсэх (1/0)
void    USB_ClearFlag(void);     // "шинэ өгөгдөл" flag-ыг цэвэрлэх
void    USB_Show_OLED(void);     // хүлээн авсан утгыг OLED дээр харуулах

#endif /* INC_GENERAL_H_ */
