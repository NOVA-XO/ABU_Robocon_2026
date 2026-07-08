/* =============================================================================
 *  general.h — Хөдөлгөөний өндөр түвшний удирдлага (mecanum, rack)
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

/* ---- Mecanum удирдлага --------------------------------------------------- */
int  applyDeadzone(int v);            // джойстик утгыг үхмэл бүсээр шүүх
void runner(void);                    // джойстикоор mecanum хөдөлгөөн

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
    float    d_filt;      // low-pass шүүсэн derivative (damping, шумгүй)
    uint32_t last_ms;
    uint8_t  holding;     // (нөөц) hysteresis төлөв
    uint8_t  home_prev;   // limit switch-ийн өмнөх төлөв (ирмэг таних)
} Rack_t;

/* -----------------------------------------------------------------------------
 *  ДООД LIMIT SWITCH (home / encoder 0 цэг)
 *    Pull-up резистортой тул ДАРАГДСАН (rack доод тулгуур дээр) = 0.
 *    Тиймээс "home дээр байна" = (val == 0).
 *
 *    val2 = Sen2 (PE2) → FRONT rack (мотор 5, counter[0])
 *    val1 = Sen1 (PE0) → BACK  rack (мотор 6, counter[1])
 * -----------------------------------------------------------------------------
 */
#define FRONT_RACK_HOME  (val2 == 0)   // 1 = урд рак доод тулгуур дээр
#define BACK_RACK_HOME   (val1 == 0)   // 1 = хойд рак доод тулгуур дээр

extern Rack_t frontRack;   // мотор 5 / counter[0]
extern Rack_t backRack;    // мотор 6 / counter[1]

uint8_t Rack_SetHome(Rack_t *r);            // limit switch хүртэл доошлуулж 0 цэг тогтооно
                                            //   return: 1 = амжилттай, 0 = timeout (switch олдсонгүй)
void    Rack_Reset(Rack_t *r);              // PID төлвийг л цэвэрлэх
void    Rack_Jog(Rack_t *r, int dir);       // dir: +1 дээш, -1 доош, 0 зогс
uint8_t Rack_GoTo(Rack_t *r, int target);   // байрлал руу; хүрвэл 1, эс бөгөөс 0

/* ISR-service горим: зорилт тавиад TIM7 ISR-т Rack_Service байнга барина */
void    Rack_SetTarget(Rack_t *r, int target);  // зорилт тавьж, active = 1
void    Rack_Off(Rack_t *r);                    // active = 0, мотор зогсоох
void    Rack_Service(void);                     // TIM7 ISR-т дуудна (2 ракыг барина)

/* Тааруулга: PS5 D-pad Дээш/Доош-оор тогтмол PWM тааруулах (open-loop тест) */
int     Rack_PWM_Tune(uint8_t motor);           // motor 5=front / 6=back; буцаана: одоогийн PWM
void    Rack_PWM_Tune_Dual(void);               // M5: D-Up/Down, M6: Triangle/Cross (тусад нь)

/* Оношилгоо: joystick-оор мотор/encoder/switch-ийн чиглэл, харьяаллыг ХЭМЖИХ */
void    Rack_Joystick_Test(void);               // LStick Y→M5, RStick Y→M6, Cross→enc тэглэх

/* ---- ADC / OLED туслах --------------------------------------------------- */
uint16_t Read_PC3(void);
void     Read_PC3_OLED(void);

/* ---- USB CDC ------------------------------------------------------------- */
uint8_t USB_GetValue(void);      // хүлээн авсан утгыг буцаах
uint8_t USB_HasNewData(void);    // шинэ өгөгдөл ирсэн эсэх (1/0)
void    USB_ClearFlag(void);     // "шинэ өгөгдөл" flag-ыг цэвэрлэх
void    USB_Show_OLED(void);     // хүлээн авсан утгыг OLED дээр харуулах

#endif /* INC_GENERAL_H_ */
