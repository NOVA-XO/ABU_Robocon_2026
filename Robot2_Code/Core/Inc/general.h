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
void    Gyro_TurnReset(void);         // эргэлтийг таслах / эхнээс нь эхлүүлэх
uint8_t Gyro_TurnFail(void);          // 0=амжилттай, 1=TIMEOUT, 2=чиглэл буруу
uint8_t Turn_Left_90(void);           // зүүн 90° (дуусвал 1)
uint8_t Turn_Right_90(void);          // баруун 90° (дуусвал 1)
void    Gyro_Turn_Test(void);         // D-Left/D-Right→90°, D-Down→таслах, L1..R2→рак
void    Drive_Straight_Test(void);    // ЗӨВХӨН шулуун явах: D-Up сэлгэх, L1/R1 хурд
void    Link_Status_Test(void);     // PCB2 руу PS5 дамжуулж буй эсэхийг харах
int     Rack_Preset_Buttons(void);    // рак preset товчны логик (OLED-гүй), буц: зорилт
int     Servo_Preset_Buttons(void);   // серво товчны логик (OLED-гүй), буц: өнцөг

/* ---- Серво ---------------------------------------------------------------- */
#define SERVO_DEG_MAX   180           // сервоны бүтэн зам (градус)
#define SERVO_HOME_DEG  SERVO_DEG_MAX // АСААХАД энэ байрлалаас эхэлнэ

void    Servo_SetDeg(int deg);        // сервог өнцгөөр тавих (0..SERVO_DEG_MAX)
void    Servo_Home(void);             // эхлэлийн байрлалд тавих (init-д дуудна)

/* -----------------------------------------------------------------------------
 *  Rack_t — Нэг рак = нэг мотор + нэг encoder + өөрийн БҮРЭН PID
 * -----------------------------------------------------------------------------
 */
typedef struct {
    /* --- холболт (тоног төхөөрөмж дээр ХЭМЖИЖ тогтоосон) --- */
    uint8_t  motor;       // motor_control дугаар (5 / 6)
    uint8_t  enc;         // counter[] индекс (0 / 1)
    uint8_t  home_sw;     // доод limit switch: 1 = S1 (val1), 2 = S2 (val2)

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

    /* --- 0 руу буух зан төлөв (дуудагч тавьж болно) --- */
    uint8_t  land_soft;   // 1 = ЗӨӨЛӨН газардалт (шугаман удаашрал), 0 = ХҮЧТЭЙ (бүрэн pwm_down)

    /* --- ISR-service горим (Rack_Service TIM7 ISR-т ажиллана) --- */
    int      target;      // зорилтот байрлал (Rack_SetTarget тавина)
    uint8_t  active;      // 1 бол Rack_Service энэ ракыг байнга барина

    /* --- телеметр (зөвхөн УНШИХ; rack_step бичнэ) --- */
    int      last_pwm;    // сүүлд моторт өгсөн PWM

    /* --- алдаа хянах (rack_step бичнэ; Rack_Fault уншина) --- */
    int      cmd_target;  // сүүлд rack_step-д өгсөн зорилт
    uint32_t move_t0;     // зорилт өөрчлөгдсөн / сүүлд хүрсэн агшин (timeout хэмжих)

    /* --- дотоод төлөв (гараар бүү хүр) --- */
    float    prev_err;
    float    integral;
    float    d_filt;      // low-pass шүүсэн derivative (damping, шумгүй)
    uint32_t last_ms;
    uint8_t  holding;     // (нөөц) hysteresis төлөв
    uint8_t  home_prev;   // limit switch-ийн өмнөх төлөв (ирмэг таних)
    float    down_lim;    // буултын хурдны governor-ийн одоогийн PWM хязгаар
} Rack_t;

/* -----------------------------------------------------------------------------
 *  ДООД LIMIT SWITCH (home / encoder 0 цэг)
 *    Pull-up резистортой тул ДАРАГДСАН (rack доод тулгуур дээр) = 0.
 *    Тиймээс "home дээр байна" = (val == 0).
 *
 *  ХЭМЖИЖ ТОГТООСОН (Rack_Joystick_Test):
 *    FRONT rack = мотор 6, counter[1], switch S2 (val2)
 *    BACK  rack = мотор 5, counter[0], switch S1 (val1)
 *    Чиглэл: эерэг PWM → counter ӨСНӨ (хоёуланд нь ижил)
 *
 *  Switch-ийг рак бүр өөрөө home_sw талбартаа агуулна (enc индексээр БҮҮ сонго).
 * -----------------------------------------------------------------------------
 */
#define RACK_SW_S1  1   // Sen1 (PE0) → val1
#define RACK_SW_S2  2   // Sen2 (PE2) → val2

extern Rack_t frontRack;   // мотор 6 / counter[1] / S2
extern Rack_t backRack;    // мотор 5 / counter[0] / S1

uint8_t Rack_SetHome(Rack_t *r);            // limit switch хүртэл доошлуулж 0 цэг тогтооно
                                            //   return: 1 = амжилттай, 0 = timeout (switch олдсонгүй)
void    Rack_Reset(Rack_t *r);              // PID төлвийг л цэвэрлэх
void    Rack_Jog(Rack_t *r, int dir);       // dir: +1 дээш, -1 доош, 0 зогс
uint8_t Rack_GoTo(Rack_t *r, int target);   // байрлал руу; хүрвэл 1, эс бөгөөс 0
uint8_t Rack_GoTo_Sync(int target);         // ХОЁР ракыг ХАМТ, зэрэгцүүлж; хоёул хүрвэл 1

/* ISR-service горим: зорилт тавиад TIM7 ISR-т Rack_Service байнга барина */
void    Rack_SetTarget(Rack_t *r, int target);  // зорилт тавьж, active = 1
void    Rack_Off(Rack_t *r);                    // active = 0, мотор зогсоох
void    Rack_Service(void);                     // TIM7 ISR-т дуудна (2 ракыг барина)

/* Аюулгүй байдал: rack-ийн алдаа хянах + робот зогсоох */
uint8_t Rack_Fault(void);                       // 0 = OK, 1 = timeout, 2 = sync зөрүү
void    Robot_Error(const char *msg);           // жолоо зогс, рак 0,0 руу буулга, зогс (буцдаггүй)

/* Rack удирдлага (Rack_SetHome-ын ДАРАА) */
void    Rack_Climb_Test(void);       // L1/R1→2уул 0/1000,  L2/△→front 0/1000,  R2/✕→back 0/1000
void    Rack_Preset_Control(void);   // L1→0  L2→900  R1→1350  R2→1950 (хоёулаа хамт; тогтмол preset)

/* Серво (1 серво, htim1 CH1), ГРАДУСААР — асаахад 180° */
void    Servo_Preset_Control(void);             // Cross +5°  Triangle −5°  Circle→0°  Square→180°

/* Соленоид 1 — D-Up товчоор toggle (debounce, давталтгүй) */
void    Solenoid_Control(void);

/* Гарын нэгдсэн тест: стик→runner, L1..R2→рак, △→серво, D-Up→соленоид1 */
void    test_weapon(void);

/* Тааруулгын телеметр: UART4 (115200) руу TSV — tgt/pos/pwm/integral/switch */
void    Rack_Telemetry_Serial(void);

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
