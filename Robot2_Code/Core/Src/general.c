/* =============================================================================
 *  general.c — Хөдөлгөөний өндөр түвшний удирдлага
 *              (mecanum жолоо, rack PID, gyro эргэлт/тэгшлэлт, налуу гарах/буух)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "general.h"
#include <stdlib.h>   // abs() — runner() дахь mecanum нормчлолд хэрэглэнэ
#include <stdio.h>    // sprintf() — Rack_Joystick_Test-ийн serial тайлан

/* ---- main.c дахь periphery handle-ууд ---------------------------------- */
extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;

extern ADC_HandleTypeDef hadc1;

/* ---- Глобал төлөв ------------------------------------------------------- */
extern int counter[4];            // encoder тоолуурууд
extern int control_data[5][4];    // джойстик/товчлуурын өгөгдөл
extern int timer;                 // TIM7-оос нэмэгддэг зөөлөн тоолуур

extern uint8_t usb_received_value;
extern uint8_t usb_new_data_flag;

/* ---- Серво: одоогийн өнцөг (Servo_Preset_Control бичнэ, OLED уншина) ------
 *  SERVO_DEG_MAX нь general.h-д (Servo_SetDeg-ийн дуудагчид хэрэгтэй).       */
static int g_servo_deg = SERVO_DEG_MAX;     // АСААХАД 180°-оос эхэлнэ (0..SERVO_DEG_MAX)


/* =============================================================================
 *  MECANUM УДИРДЛАГА
 * =============================================================================
 */
#define DEADZONE    10     // joystick үхмэл бүс
#define SPEED_GAIN   8     // -100..100 утгыг PWM болгох коэффициент

/* -----------------------------------------------------------------------------
 *  applyDeadzone — joystick утгыг deadzone-оор шүүх (жижиг утгыг 0 болгоно)
 * -----------------------------------------------------------------------------
 */
int applyDeadzone(int v) {
    if (v > -DEADZONE && v < DEADZONE) return 0;
    return v;
}

/* -----------------------------------------------------------------------------
 *  runner — Джойстикоор mecanum хөдөлгөөн (inverse kinematics + нормчлол)
 * -----------------------------------------------------------------------------
 */
void runner(void) {
    // --- joystick унших (-100..100) ---
    int Vy = applyDeadzone(control_data[0][1]);   // зүүн стик Y → урагш/хойш
    int Vx = applyDeadzone(control_data[0][0]);   // зүүн стик X → хажуу (strafe)
    int W  = applyDeadzone(control_data[0][2]);   // баруун стик X → эргэлт

    // joystick Y нь дээш түлхэхэд сөрөг тул урагш = эерэг болгож эргүүлнэ
    Vy = -Vy;

    // --- Mecanum inverse kinematics ---
    int fl = Vy - Vx - W;   // Урд-Зүүн    (мотор 1)
    int fr = Vy + Vx + W;   // Урд-Баруун  (мотор 2)
    int rl = Vy + Vx - W;   // Хойд-Зүүн   (мотор 3)
    int rr = Vy - Vx + W;   // Хойд-Баруун (мотор 4)

    // --- Нормчлол: хамгийн их утга 100 хэтэрвэл бүгдийг хувь тэнцүүлж багасгана ---
    int m = abs(fl);
    if (abs(fr) > m) m = abs(fr);
    if (abs(rl) > m) m = abs(rl);
    if (abs(rr) > m) m = abs(rr);
    if (m > 100) {
        fl = fl * 100 / m;
        fr = fr * 100 / m;
        rl = rl * 100 / m;
        rr = rr * 100 / m;
    }

    // --- PWM болгож моторт өгөх ---
    motor_control(1, fl * SPEED_GAIN);   // Урд-Зүүн
    motor_control(2, fr * SPEED_GAIN);   // Урд-Баруун
    motor_control(3, rl * SPEED_GAIN);   // Хойд-Зүүн
    motor_control(4, rr * SPEED_GAIN);   // Хойд-Баруун
}


/* =============================================================================
 *  Drive_Straight — LPMS gyro-гоор чиг баримжаа барьж ШУЛУУН явах
 *
 *    base_pwm: үндсэн хурд. Конвенц: урагш = СӨРӨГ (ж: урагш -300, ухрах +300).
 *    Anchor = "шулуун" гэх чиглэл (LPMS_Init дотор тогтоогддог).
 * =============================================================================
 */
#define STRAIGHT_KP        45.0f    // залруулгын хүч
#define STRAIGHT_MAX_CORR  150      // залруулгын дээд PWM

void Drive_Straight(int base_pwm) {

    LPMS_Read();                                  // stream-аас хоцрохгүй

    float offset = Get_Yaw_Offset_From_Anchor();  // anchor-аас зөрөө (-180..+180)

    float corr = STRAIGHT_KP * offset;
    if (corr >  STRAIGHT_MAX_CORR) corr =  STRAIGHT_MAX_CORR;
    if (corr < -STRAIGHT_MAX_CORR) corr = -STRAIGHT_MAX_CORR;

    int c = (int)corr;

    // Бүх мотор ижил base + зүүн(1,3)/баруун(2,4) yaw засвар (runner шиг)
    motor_control(1, base_pwm - c);   // FL (зүүн)
    motor_control(2, base_pwm + c);   // FR (баруун)
    motor_control(3, base_pwm - c);   // RL (зүүн)
    motor_control(4, base_pwm + c);   // RR (баруун)
}


/* =============================================================================
 *  RACK УДИРДЛАГА (Мотор 5, 6) — encoder-аар тогтсон БАЙРЛАЛ руу PID
 *
 *  Encoder:  counter[0] = мотор 5 ,  counter[1] = мотор 6
 *  Мотор бүр өөрийн БҮРЭН PID-тай тул дангаар нь тааруулж болно.
 *  pos_max-ийг encoder-ийн CPR-аар дахин хэмжиж бич (Rack_Jog + showEncoderStatus).
 * =============================================================================
 */
#define RACK_DT_MS       5    // PID шинэчлэх интервал (мс)
#define JOG_PWM        150    // гараар гүйлгэх хүч
#define RACK_D_ALPHA  0.7f    // derivative low-pass шүүлтүүр (0..1; их=илүү гөлгөр)

/* ---- Ойрын бүсийн зөөлрүүлэлт (хол зайнаас слам хийхээс сэргийлнэ) --------
 *  Ойрын бүсэд PWM-ийн дээд хязгаар = hold_pwm + RACK_NEAR_PWM.
 *
 *  ⚠ ЭНЭ ХЯЗГААР нь ракыг ҮРЭЛТ даван хөдөлгөхөд ХАНГАЛТТАЙ байх ЁСТОЙ.
 *    Өмнө нь 120 байсан тул front-ийн таг = 70 + 120 = 190 PWM болж, рак 796
 *    дээр (зорилтоос 104 зайд) ГАЦДАГ байв — телеметрт Fpwm яг 190 дээр тогтсон.
 *    Front нь 261+ PWM дээр хөдөлдөг тул тагийг өндөрсгөв.
 *
 *  ⚠ ЗААВАЛ: RACK_I_BAND >= RACK_NEAR_ZONE байх ёстой!
 *    Эс бөгөөс хоёрын завсарт УРХИ үүснэ: PWM нь хайчлагдсан (сул) атлаа
 *    integral нь тэглэгдсэн (тусалж чадахгүй) → рак тэнд гацвал гарах аргагүй.
 *    (Яг ийм урхи 80..120 муж дээр үүсээд front rack-ийг гацаасан.)
 */
#define RACK_NEAR_ZONE  60    // зорилтод ойрын бүс (count) — энд хүчийг зөөлрүүлнэ
#define RACK_NEAR_PWM  230    // ойрын бүсэд hold_pwm-ээс дээш нэмэх дээд засвар

/* ---- Integral (таталцлын үлдэгдлийг нөхөх нарийн засвар) ------------------
 *  hold_pwm нь таталцлын ҮНДСЭН хүчийг өгнө; integral зөвхөн ҮЛДЭГДЛИЙГ трим хийнэ.
 *  RACK_I_BAND — хураах бүс. Өргөн байвал ойртох замдаа хэт их хурааж ХАНАНА.
 *  RACK_I_DEAD — encoder шумаас integral мөлхөхөөс сэргийлэх үхмэл бүс (count).
 */
#define RACK_I_BAND     80    // үүнээс ХОЛ бол integral-ыг тэглэнэ (windup-аас сэргийлнэ)
                              // >= RACK_NEAR_ZONE (60) байх ЁСТОЙ — дээрх урхийг үзнэ үү
/*  RACK_I_DEAD — ҮРЭЛТИЙН (stiction) бүсийг бүрхэх ёстой. Механизм өөрөө түгжигддэг:
 *  телеметрээр back rack нь 10..170 PWM хооронд ОГТ хөдөлдөггүй нь батлагдсан.
 *  Хэрэв энэ бүсэд integral-ыг үргэлжлүүлэн хураавал рак хөдлөхгүй байхад integral
 *  −150 хүртэл ханаж, PWM-ийг 10 болтол буулгаад ч гэсэн рак хөдөлдөггүй (хий ажил).
 */
#define RACK_I_DEAD      8    // |алдаа| үүнээс бага бол integral-ыг хөлдөөнө

/* ---- Зөөлөн газардалт (ЗӨВХӨН 0 = доод тулгуур руу буух үед) -------------
 *  Энэ зайнаас эхлэн PWM-ийг ШУГАМААР бууруулж limit switch рүү зөөлөн хүрнэ.
 *  RACK_LAND_ZONE ↑  → эрт эхэлж удаашрана (илүү зөөлөн, гэхдээ удаан)
 *  RACK_LAND_MIN_PWM → switch дээр газардах үеийн PWM (хэт бага бол хүрэлгүй зогсоно)
 */
#define RACK_LAND_ZONE     600   // 0-оос энэ зайд орохоос эхлэн удаашруулна (count)
#define RACK_LAND_MIN_PWM   70   // газардах агшны хамгийн бага PWM

/* ---- Аюулгүй байдал (Rack_Fault) — ТЕЛЕМЕТРЭЭР ТААРУУЛСАН -----------------
 *  Хэмжилт (50мс/мөр): бүтэн зам 0→1950 нь ~25 мөр ≈ 1250мс. Өгсөх үед front/back
 *  зөрүү дээд тал нь ~70 count. Тиймээс:
 *    TIMEOUT = 1250мс + ~2x нөөц = 2500мс (гацсан ракыг ~2.5с-д барина, худал
 *             триггергүй; батарей суларвал ч бүтэн зам үүнээс богино).
 *    SYNC_MAX = хэвийн ~70-ийн ~4x = 300 (нэг рак гацвал зөрүү үүнийг хурдан
 *             давна; хэвийн 70 зөрүүнд худал триггер өгөхгүй). */
#define RACK_TIMEOUT_MS   3500   // зорилтод хүрэх дээд хугацаа; хэтэрвэл алдаа
                                 //   2500 нь удаан (ачаатай) авиралтад БУРУУ асч байв → 3500
#define RACK_SYNC_MAX      450   // Rack_Fault-ийн ХЯЗГААР (count) — синхрончлол дампуурвал барина
#define RACK_SYNC_BAND     150   // Rack_GoTo_Sync: хоёр рак нэг нэгнээсээ хамгийн ихдээ энэ зайд
                                 //   (< RACK_SYNC_MAX байх ёстой; band барихад зөрүү үүнээс хэтрэхгүй)
#define RACK_SYNC_NEAR     100   // sync шалгах: зорилтоос энэ дотор бол "хүрсэн/parked" гэж үзнэ
                                 //   (нэг рак parked, нөгөө тусдаа хол → ХУДАЛ fault өгөхгүй)

/* ---- Homing (доод limit switch хүртэл буулгах) --------------------------- */
#define HOME_PWM         250   // доошлуулах хүч (ЗӨӨЛӨН — таталцал бас тусалдаг)
#define HOME_TIMEOUT_MS 6000  // энэ хугацаанд switch дарагдахгүй бол зогсоно
#define HOME_CONFIRM      3   // зогссоны дараа баталгаажуулах уншилтын тоо
#define HOME_CONFIRM_MS   2   // баталгаажуулах уншилтын хоорондох хугацаа (мс)

Rack_t frontRack = {
    .motor = 6, .enc = 1, .home_sw = RACK_SW_S2,   // ХЭМЖСЭН: M6 / E1 / S2
    .pos_min = 0, .pos_max = 1950,   // ДЭЭД хязгаар — Rack_SetTarget зорилтыг үүн рүү хайчилна
    // FRONT нь back-аас ХОЛ ИЛҮҮ АЧААТАЙ (телеметрээр: 900-г барихад back 69 PWM,
    //   FRONT 173 PWM хэрэгтэй). Тиймээс:
    //   - kp-г 2.0 → 3.5 (back-аас өндөр): өгсөх/ойртохдоо хоцрохгүй, хангалттай хүч.
    //   - hold_pwm-ийг 70 → 150: барих хүчийг feedforward-оор ШУУД өгнө; өмнө нь
    //     integral 100 хүртэл УДААН ханаж байж нөхдөг тул мөлхөж хоцордог байв.
    .kp = 3.5f, .ki = 0.012f, .kd = 9.0f, .i_max = 12500.0f,
    // pwm_up 750: back (650)-ийн авирах ХУРДТАЙ тэнцүүлэв. 900 нь ХЭТ хурдан
    //   болж front түрүүлээд sync fault асч байв (front/back ачаа өөр тул ижил
    //   хурдад front-д арай өндөр PWM хэрэгтэй: телеметрээр ~770).
    // pwm_down 150→300: front 0 руу буухдаа 1686 дээр ГАЦдаг байв (150 хүрэлцэхгүй).
    .pwm_up = 750, .pwm_down = 350, .hold_pwm = 150,
    .min_pwm = 80, .tolerance = 15, .land_soft = 1,   // default: зөөлөн газардалт
    .target = 0, .active = 0,
    .prev_err = 0.0f, .integral = 0.0f, .d_filt = 0.0f, .last_ms = 0, .holding = 0,
    .home_prev = 0
};

Rack_t backRack = {
    .motor = 5, .enc = 0, .home_sw = RACK_SW_S1,   // ХЭМЖСЭН: M5 / E0 / S1
    .pos_min = 0, .pos_max = 1950,   // ДЭЭД хязгаар — Rack_SetTarget зорилтыг үүн рүү хайчилнаs
    // frontRack-тай ижил зарчим: удаан, ханахгүй integral (0.012 x 12500 = 150 PWM дээд)
    .kp = 3.0f, .ki = 0.012f, .kd = 10.0f, .i_max = 12500.0f,
    // hold_pwm — back rack-ийн үрэлтийн бүс 10..170 PWM (телеметрээр батлагдсан) тул
    //   түүний ДУНДАЖ. 175 нь бүсээс ДЭЭГҮҮР байсан → байнга дээш түлхэж гацаадаг байв.
    // pwm_down 150→300: back мөн 0 руу буухдаа удаан/гацдаг (front-той ижил үрэлт).
    .pwm_up = 650, .pwm_down = 350, .hold_pwm = 90,
    .min_pwm = 80, .tolerance = 15, .land_soft = 1,   // default: зөөлөн газардалт
    .target = 0, .active = 0,
    .prev_err = 0.0f, .integral = 0.0f, .d_filt = 0.0f, .last_ms = 0, .holding = 0,
    .home_prev = 0
};

/* -----------------------------------------------------------------------------
 *  rack_at_home — Тухайн ракийн доод limit switch дарагдсан эсэх
 *    return: 1 = доод тулгуур дээр байна (switch дарагдсан, pull-up тул утга 0)
 *  ISR-safe: зөвхөн GPIO уншилт.
 *
 *  Рак бүр өөрийн switch-ээ home_sw талбартаа агуулна (ХЭМЖИЖ тогтоосон):
 *    frontRack (M6/E1) → S2 (val2)   |   backRack (M5/E0) → S1 (val1)
 *  ӨМНӨ нь switch-ийг enc индексээр сонгодог байсан нь урд/хойдыг СОЛЬЖ уншиж байв.
 * -----------------------------------------------------------------------------
 */
static uint8_t rack_at_home(Rack_t *r) {
    return (r->home_sw == RACK_SW_S2) ? (uint8_t)(val2 == 0)
                                      : (uint8_t)(val1 == 0);
}

/* -----------------------------------------------------------------------------
 *  enc_zero — encoder тоолуурыг аюулгүйгээр 0 болгох
 *  counter[] нь EXTI ISR-т read-modify-write (counter[i]--) хийгддэг. Main loop-оос
 *  бичихэд EXTI дундуур таслаад хуучин утгаар дарж бичиж болзошгүй тул богино
 *  критик мужаар хамгаална. (TIM7 ISR-ээс дуудахад ч аюулгүй — PRIMASK сэргээнэ.)
 * -----------------------------------------------------------------------------
 */
static void enc_zero(int idx) {
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    counter[idx] = 0;
    __set_PRIMASK(prim);
}

/* -----------------------------------------------------------------------------
 *  Rack_SetHome — Ракыг доод limit switch хүртэл БУУЛГАЖ, encoder-ийг 0 болгох
 *
 *    - Аль хэдийн switch дээр байвал огт хөдөлгөхгүй (шууд тэглэнэ).
 *    - Эс бөгөөс HOME_PWM хүчээр зөөлөн доошлуулж, switch дарагдмагц зогсоно.
 *    - HOME_TIMEOUT_MS дотор switch дарагдахгүй бол ЗОГСООД 0 БУЦААНА
 *      (counter-ийг тэглэхгүй — байрлал тодорхойгүй тул).
 *
 *  БЛОКЛОДОГ: HAL_Delay ашигладаг тул ЗӨВХӨН main loop-оос дуудна (ISR-ээс БИШ).
 *  Дуудах үед тухайн ракийн ISR-service-ийг түр унтраана (Rack_Service зөрчихгүй).
 *
 *  return: 1 = амжилттай home хийсэн,  0 = timeout (switch олдсонгүй)
 * -----------------------------------------------------------------------------
 */
uint8_t Rack_SetHome(Rack_t *r) {
    uint8_t was_active = r->active;
    r->active = 0;                       // ISR-service түр унтраа

    uint32_t t0 = HAL_GetTick();
    uint8_t  ok = 0;

    while (1) {
        if (rack_at_home(r)) {
            motor_control(r->motor, 0);  // тэр даруй зогсоо (brake)

            // Bounce шүүх: зогссоны дараа дараалан баталгаажуулна
            uint8_t stable = 1;
            for (int i = 0; i < HOME_CONFIRM; i++) {
                HAL_Delay(HOME_CONFIRM_MS);
                if (!rack_at_home(r)) { stable = 0; break; }
            }
            if (stable) { ok = 1; break; }
        }

        if (HAL_GetTick() - t0 > HOME_TIMEOUT_MS) {   // switch олдсонгүй
            motor_control(r->motor, 0);
            ok = 0;
            break;
        }

        motor_control(r->motor, -HOME_PWM);           // сөрөг = доош
    }

    motor_control(r->motor, 0);

    if (ok) {
        enc_zero(r->enc);         // зөвхөн switch олдсон үед л 0 цэг тогтооно
        r->target = 0;
    }

    r->prev_err  = 0.0f;
    r->integral  = 0.0f;
    r->d_filt    = 0.0f;
    r->holding   = 0;
    r->home_prev = rack_at_home(r);
    r->active    = was_active;

    return ok;
}

/* -----------------------------------------------------------------------------
 *  Rack_Reset — PID дотоод төлвийг л цэвэрлэх (counter-ийг хөндөхгүй)
 * -----------------------------------------------------------------------------
 */
void Rack_Reset(Rack_t *r) {
    r->prev_err  = 0.0f;
    r->integral  = 0.0f;
    r->d_filt    = 0.0f;
    r->holding   = 0;
    r->home_prev = rack_at_home(r);
}

/* -----------------------------------------------------------------------------
 *  Rack_Jog — Гараар гүйлгэх (dir > 0 дээш, dir < 0 доош, 0 зогс)
 * -----------------------------------------------------------------------------
 */
void Rack_Jog(Rack_t *r, int dir) {
    if (dir > 0)      motor_control(r->motor,  JOG_PWM);
    else if (dir < 0) motor_control(r->motor, -JOG_PWM);
    else              motor_control(r->motor, 0);
}

/* -----------------------------------------------------------------------------
 *  rack_step — Нэг ракийн PID-ийн НЭГ алхам (Rack_GoTo ба Rack_Service хуваалцана)
 *    return: 1 = хүрсэн (tolerance дотор), 0 = хөдөлж байна
 *  ISR-safe: зөвхөн HAL_GetTick, float, motor_control ашиглана.
 * -----------------------------------------------------------------------------
 */
static uint8_t rack_step(Rack_t *r, int target) {
    if (target > r->pos_max) target = r->pos_max;
    if (target < r->pos_min) target = r->pos_min;

    // ---- Доод limit switch ДАРАГДСАН БОЛ encoder-ийг ШУУД 0 болгоно ----
    //      Зөвхөн ирмэг дээр биш, дарагдсан ХУГАЦААНД тасралтгүй тэглэнэ:
    //      рак доод тулгуур дээр зогсож байхад хуримтлагдсан алдаа арилна.
    //      Дөнгөж хүрсэн АГШИНД PID-ийн дотоод төлвийг цэвэрлэнэ.
    uint8_t at_home = rack_at_home(r);
    if (at_home) {
        enc_zero(r->enc);
        if (!r->home_prev) {          // switch-д дөнгөж хүрлээ
            r->integral = 0.0f;
            r->prev_err = 0.0f;
            r->d_filt   = 0.0f;
        }
    }
    r->home_prev = at_home;

    int   pos   = counter[r->enc];
    float error = (float)(target - pos);
    float aerr  = (error < 0.0f) ? -error : error;

    uint8_t reached = (aerr < (float)r->tolerance);
    // Дээш өргөгдсөн байрлал уу? (таталцлыг барих шаардлагатай эсэх)
    uint8_t raised  = (target > r->pos_min + r->tolerance);

    // ---- Алдаа хянах цаг: зорилт өөрчлөгдсөн ЭСВЭЛ хүрсэн үед timeout-ыг тэглэнэ.
    //      Хүрсэн хэвээр байвал move_t0 шинэчлэгдсээр байх тул зогсоод удвал л
    //      timeout болно (Rack_Fault уншина). ISR-safe: зөвхөн HAL_GetTick.
    if (target != r->cmd_target) {
        r->cmd_target = target;
        r->move_t0    = HAL_GetTick();
    } else if (reached) {
        r->move_t0    = HAL_GetTick();
    }

    // ---- Доод тулгуур (0) дээр хүрсэн бол ЧӨЛӨӨЛ (coast) ----
    if (reached && !raised) {
        motor_control(r->motor, 0);
        r->last_pwm = 0;
        r->integral = 0.0f;
        r->prev_err = 0.0f;
        r->d_filt   = 0.0f;
        return 1;
    }

    // ---- ТАСРАЛТГҮЙ PID + feedforward + шүүсэн damping (тогтмол интервалаар) ----
    //   Плант тогтмол PWM дээр тогтвортой (open-loop тестээр батлагдсан) тул
    //   гогцоог ЗӨӨЛӨН, overdamped байлгана:
    //     - feedforward (hold_pwm) — суурь барих хүч
    //     - integral — ЖИНГ автоматаар нөхнө (gravity comp; жин нэмэгдвэл өөрөө өснө)
    //     - шүүсэн derivative — damping (encoder ±1 шумыг дарж савалгааг сэргийлнэ)
    if (HAL_GetTick() - r->last_ms >= RACK_DT_MS) {
        r->last_ms = HAL_GetTick();

        // Integral (таталцлын ҮЛДЭГДЛИЙГ трим хийнэ) — 3 муж:
        //   1. ХОЛ (aerr >= RACK_I_BAND) : тэглэнэ — ойртох замдаа хурааж ханахаас
        //      сэргийлнэ.
        //   2. ОЙР (RACK_I_DEAD < aerr < RACK_I_BAND) : хурааж үлдэгдлийг нөхнө.
        //      Зорилтоос ДЭЭГҮҮР байвал (error < 0) integral БУЦАЖ ТАЙЛАГДАНА —
        //      ингэж л байнгын хазайлтыг арилгана.
        //   3. МАШ ОЙР (aerr <= RACK_I_DEAD) : хөлдөнө — encoder шумаас мөлхөхгүй.
        //
        //   ӨМНӨ нь tolerance дотор ХӨЛДӨӨДӨГ байсан нь integral-ыг ХАНАСАН утган
        //   дээр нь түгжиж, байнгын дээш хазайлт (+9..+15 count) үүсгэж байв.
        if (aerr >= RACK_I_BAND) {
            r->integral = 0.0f;
        } else if (aerr > (float)RACK_I_DEAD) {
            r->integral += error;
            if (r->integral >  r->i_max) r->integral =  r->i_max;
            if (r->integral < -r->i_max) r->integral = -r->i_max;
        }

        // Derivative = алдааны өөрчлөлт (= -хурд). Low-pass шүүж damping өгнө:
        // encoder ±1 квант шумыг дарж, бодит хөдөлгөөний хурдыг л үлдээнэ.
        float raw_d = error - r->prev_err;
        r->prev_err = error;
        r->d_filt   = RACK_D_ALPHA * r->d_filt + (1.0f - RACK_D_ALPHA) * raw_d;

        float output = (r->kp * error) + (r->ki * r->integral) + (r->kd * r->d_filt);

        // Таталцлын FEEDFORWARD — өргөгдсөн байрлалд суурь дээш bias
        if (raised) output += (float)r->hold_pwm;

        int lim = (error > 0.0f) ? r->pwm_up : r->pwm_down;
        // Gain scheduling: зорилтод ойрхон (RACK_NEAR_ZONE) бол хүчийг зөөлрүүлж
        // хол зайнаас слам хийж хэтрүүлэхээс сэргийлнэ.
        if (aerr < (float)RACK_NEAR_ZONE) {
            int soft = r->hold_pwm + RACK_NEAR_PWM;
            if (lim > soft) lim = soft;
        }

        // ---- 0 (доод тулгуур) руу буух үед: land_soft-оос хамаарна ----
        //   ЗӨӨЛӨН (land_soft=1): хол байхад pwm_down, switch рүү ойртох тусам
        //     ШУГАМААР удааширч RACK_LAND_MIN_PWM хүртэл буурна → зөөлөн газардана.
        //   ХҮЧТЭЙ (land_soft=0): удаашруулахгүй — бүрэн pwm_down-оор шууд татна
        //     (ачаатай ракыг тусад нь буулгахад хэрэгтэй).
        uint8_t landing = (!raised && error < 0.0f);
        if (landing && r->land_soft && aerr < (float)RACK_LAND_ZONE) {
            int span = r->pwm_down - RACK_LAND_MIN_PWM;
            int land = RACK_LAND_MIN_PWM + (int)(((float)span * aerr) / (float)RACK_LAND_ZONE);
            if (lim > land) lim = land;
        }

        if (output >  lim) output =  lim;
        if (output < -lim) output = -lim;

        int pwm = (int)output;

        // min_pwm floor зөвхөн ТОМ зөрөөнд (хол хөдөлгөөн, статик үрэлт даван гарах).
        // Ойрын жижиг залруулгыг зөөлөн үлдээж хэлбэлзэхээс сэргийлнэ.
        // ГАЗАРДАХ үед floor хэрэглэхгүй — эс бөгөөс удаашруулах профайлыг эвдэнэ.
        if (aerr > (float)RACK_NEAR_ZONE && !landing) {
            if (pwm > 0 && pwm <  r->min_pwm) pwm =  r->min_pwm;
            if (pwm < 0 && pwm > -r->min_pwm) pwm = -r->min_pwm;
        }

        // ХАМГААЛАЛТ: доод limit switch дарагдсан бол ДООШ (сөрөг PWM) жолоодохгүй.
        // Механизмыг тулгуур руу шахаж мотор түгжихээс сэргийлнэ.
        if (at_home && pwm < 0) pwm = 0;

        r->last_pwm = pwm;              // телеметр
        motor_control(r->motor, pwm);
    }

    return reached ? 1 : 0;
}

/* -----------------------------------------------------------------------------
 *  Rack_GoTo — Ракыг target байрлал руу PID-ээр аваачих (main loop-оос дуудна)
 *    return: 1 = хүрсэн (tolerance дотор), 0 = хөдөлж байна
 *  ТЭМДЭГЛЭЛ: ижил ракийг Rack_Service (ISR)-тэй ЗЭРЭГ бүү удирд.
 * -----------------------------------------------------------------------------
 */
uint8_t Rack_GoTo(Rack_t *r, int target) {
    return rack_step(r, target);
}

/* -----------------------------------------------------------------------------
 *  Rack_GoTo_Sync — ХОЁР ракыг ХАМТ нэг зорилт руу, зэрэгцүүлж (main loop-оос)
 *    return: 1 = ХОЁУЛАА зорилтод хүрсэн, 0 = хөдөлж байна
 *
 *  Хурдан рак нөгөөгөөсөө RACK_SYNC_BAND-аас ЦААШ гарахгүй: өөрийн завсрын
 *  зорилтыг (нөгөөгийн байрлал ± band) хүртэл хязгаарлаж, удаанийг ХҮЛЭЭНЭ.
 *  Ингэснээр front/back ачаа/хурд ямар ч байсан ЗЭРЭГ хөдөлнө (open-loop
 *  pwm_up тааруулга шаардлагагүй болно). Дээш ба доош хоёуланд ажиллана.
 *
 *  Гацсан ракыг Rack_Fault-ийн TIMEOUT барина (хурдан рак band дээр хүлээхэд
 *  гацсан нь эцсийн зорилтдоо хүрэхгүй тул).
 * -----------------------------------------------------------------------------
 */
uint8_t Rack_GoTo_Sync(int target) {
    int fpos = counter[frontRack.enc];
    int bpos = counter[backRack.enc];

    // Front-ийг back-аас ±band дотор барих; back-ийг front-аас ±band дотор барих
    int ftgt = target;
    if (ftgt > bpos + RACK_SYNC_BAND) ftgt = bpos + RACK_SYNC_BAND;
    if (ftgt < bpos - RACK_SYNC_BAND) ftgt = bpos - RACK_SYNC_BAND;

    int btgt = target;
    if (btgt > fpos + RACK_SYNC_BAND) btgt = fpos + RACK_SYNC_BAND;
    if (btgt < fpos - RACK_SYNC_BAND) btgt = fpos - RACK_SYNC_BAND;

    Rack_GoTo(&frontRack, ftgt);
    Rack_GoTo(&backRack,  btgt);

    // ХОЁУЛАА ЭЦСИЙН зорилтод хүрсэн үед л дуусна
    int fe = fpos - target; if (fe < 0) fe = -fe;
    int be = bpos - target; if (be < 0) be = -be;
    return (fe < frontRack.tolerance) && (be < backRack.tolerance);
}

/* -----------------------------------------------------------------------------
 *  Rack_SetTarget — Зорилтот байрлал тавьж, ISR-service-ийг идэвхжүүлэх
 *  Үүний дараа Rack_Service (TIM7 ISR) уг ракыг зорилтод нь байнга барина.
 * -----------------------------------------------------------------------------
 */
void Rack_SetTarget(Rack_t *r, int target) {
    if (target > r->pos_max) target = r->pos_max;
    if (target < r->pos_min) target = r->pos_min;
    r->target = target;
    r->active = 1;
}

/* -----------------------------------------------------------------------------
 *  Rack_Off — ISR-service-ийг унтрааж, моторыг зогсоох
 * -----------------------------------------------------------------------------
 */
void Rack_Off(Rack_t *r) {
    r->active = 0;
    motor_control(r->motor, 0);
}

/* -----------------------------------------------------------------------------
 *  Rack_Service — Идэвхтэй рак(ууд)-ыг зорилтод нь барих (TIM7 ISR-т дуудна)
 *  Зогсож байхад ч тогтмол давтамжтай ажиллаж байрлалыг барина.
 * -----------------------------------------------------------------------------
 */
void Rack_Service(void) {
    if (frontRack.active) rack_step(&frontRack, frontRack.target);
    if (backRack.active)  rack_step(&backRack,  backRack.target);
}

/* -----------------------------------------------------------------------------
 *  Rack_Fault — Rack-ийн алдааг хянах (main loop-оос дуудна)
 *    return: 0 = OK ,  1 = TIMEOUT (зорилтод хугацаандаа хүрсэнгүй)
 *                      2 = SYNC (хоёр рак ижил зорилттой ч хэт зөрсөн)
 *
 *  cmd_target / move_t0-ыг rack_step хөтөлдөг. Зорилтод хүрч байвал move_t0
 *  байнга шинэчлэгддэг тул зогсоод удвал л timeout болно (буруу дуудалтгүй).
 *  ISR-ээс БҮҮ дууд — main loop-оос дуудаж, алдаа гарвал Robot_Error руу ор.
 * -----------------------------------------------------------------------------
 */
uint8_t Rack_Fault(void) {
    uint32_t now = HAL_GetTick();
    Rack_t *racks[2] = { &frontRack, &backRack };

    // 1) TIMEOUT — рак бүр: зорилтоос хол байсаар RACK_TIMEOUT_MS хэтэрвэл
    for (int i = 0; i < 2; i++) {
        Rack_t *r = racks[i];
        int err = counter[r->enc] - r->cmd_target;
        if (err < 0) err = -err;
        if (err >= r->tolerance && (now - r->move_t0) > RACK_TIMEOUT_MS)
            return 1;
    }

    // 2) SYNC — ЗӨВХӨН хоёулаа зорилтоос ХОЛ (хамт хөдөлж) байхад л шалгана.
    //    Нэг рак зорилтдоо хүрчихсэн (parked) атал нөгөө нь ТУСАД НЬ хол зайд
    //    хөдлөх нь хууль ёсны (ж: up_40 S5 — front 0-д, back тусдаа 0 руу буужа).
    //    Өмнө нь энэ тохиолдол ХУДАЛ sync fault өгч back-ийн буултыг тасалдаг байв.
    if (frontRack.cmd_target == backRack.cmd_target) {
        int ft = frontRack.cmd_target;
        int fe = counter[frontRack.enc] - ft; if (fe < 0) fe = -fe;
        int be = counter[backRack.enc]  - ft; if (be < 0) be = -be;

        if (fe > RACK_SYNC_NEAR && be > RACK_SYNC_NEAR) {   // хоёулаа зорилтоос хол
            int d = counter[frontRack.enc] - counter[backRack.enc];
            if (d < 0) d = -d;
            if (d > RACK_SYNC_MAX)
                return 2;
        }
    }

    return 0;
}

/* -----------------------------------------------------------------------------
 *  Robot_Error — Аюулгүй зогсолт: жолоо зогсоо, рак 0,0 руу буулга, БУЦАХГҮЙ
 *    msg: OLED дээр харуулах шалтгаан ("TIMEOUT" / "SYNC" г.м.)
 *
 *  1. Жолооны мотор + ISR-service зогсооно (робот байрандаа).
 *  2. Хоёр ракыг ЗӨӨЛӨН 0 руу буулгаж, доод switch (0,0) хүртэл барина.
 *  3. Buzzer дуугаргаж, OLED дээр алдаа + switch төлөв харуулна. БУЦАХГҮЙ.
 * -----------------------------------------------------------------------------
 */
void Robot_Error(const char *msg) {
    brake();                       // жолоо зогс (мотор 1-4)
    frontRack.active = 0;          // ISR-service унтраа (давхар удирдлагагүй)
    backRack.active  = 0;
    frontRack.land_soft = 1;       // зөөлөн буулт
    backRack.land_soft  = 1;

    while (1) {                    // БУЦАХГҮЙ аюулгүй төлөв
        Rack_GoTo(&frontRack, 0);  // хоёр ракыг 0 руу (switch дээр coast)
        Rack_GoTo(&backRack,  0);

        buzzer;                    // сонсголын дохио

        colorFill(Black);
        setCursor(6, 2);
        printStr("!! ERROR !!");
        setCursor(6, 22);
        printStr("%s", msg);
        setCursor(6, 42);
        printStr("F:%d B:%d", counter[frontRack.enc], counter[backRack.enc]);
        setScreen();
    }
}

/* -----------------------------------------------------------------------------
 *  Rack_PWM_Tune — PS5 D-pad-аар тогтмол (open-loop) PWM-ийг бодит цагт тааруулах
 *
 *    D-Up   (control_data[2][2]) → PWM +алхам
 *    D-Down (control_data[2][0]) → PWM −алхам
 *
 *  motor: тааруулах мотор (5 = front rack, 6 = back rack).
 *  PWM-ийг encoder feedback-ГҮЙгээр шууд өгнө — таталцлыг бариулах утгыг олоход,
 *  мөн савалгаа удирдлагынх уу (feedback) эсвэл механикийнх уу гэдгийг ялгахад.
 *  Одоогийн PWM болон encoder утгыг OLED дээр харуулна.
 *
 *  Ашиглах: main loop дотор бусад rack удирдлагыг ЗОГСООЖ, зөвхөн энийг дууд:
 *      while (1) { get_data(); Rack_PWM_Tune(5); }
 * -----------------------------------------------------------------------------
 */
#define TUNE_STEP     10      // нэг алхмын PWM өөрчлөлт
#define TUNE_RATE_MS  120     // товч дарж байхад алхам хийх давтамж (мс)

int Rack_PWM_Tune(uint8_t motor) {
    static int      pwm = 0;
    static uint32_t t   = 0;

    int up   = control_data[2][2];   // D-Up
    int down = control_data[2][0];   // D-Down

    // Товч дарагдсан бол TUNE_RATE_MS тутам нэг алхам (тонших = нарийн, барих = хурдан)
    if ((up || down) && (HAL_GetTick() - t >= TUNE_RATE_MS)) {
        t = HAL_GetTick();
        if (up)   pwm += TUNE_STEP;
        if (down) pwm -= TUNE_STEP;
        if (pwm >  1000) pwm =  1000;
        if (pwm < -1000) pwm = -1000;
    }

    motor_control(motor, pwm);

    // OLED дээр харуулах (100мс-д нэг)
    static uint32_t st = 0;
    if (HAL_GetTick() - st >= 100) {
        st = HAL_GetTick();
        colorFill(Black);
        setCursor(10, 6);
        printStr("M%d PWM:%d", motor, pwm);
        setCursor(10, 30);
        printStr("F:%d B:%d", counter[0], counter[1]);
        setScreen();
    }

    return pwm;
}

/* -----------------------------------------------------------------------------
 *  Товчны DEBOUNCE + edge/repeat таних туслах
 *    - Түүхий 1/0 хэлбэлзлийг BTN_DEBOUNCE_MS тогтвортой болтол шүүнэ.
 *    - Цэвэр даралт (rising edge) бүрт НЭГ алхам.
 *    - Дараад барьж байвал BTN_REPEAT_MS тутам давтана.
 * -----------------------------------------------------------------------------
 */
#define BTN_DEBOUNCE_MS  25    // тогтвортой гэж үзэх хугацаа
#define BTN_REPEAT_MS   120    // барьж байхад давтах интервал

typedef struct {
    uint8_t  stable;      // debounced төлөв
    uint8_t  raw_last;    // сүүлийн түүхий уншилт
    uint32_t change_ms;   // түүхий төлөв өөрчлөгдсөн хугацаа
    uint32_t repeat_ms;   // сүүлийн repeat алхмын хугацаа
} Btn_t;

// return: энэ дуудалтад "нэг алхам хий" гэвэл 1 (rising edge эсвэл repeat)
static uint8_t btn_step(Btn_t *b, uint8_t raw) {
    uint32_t now = HAL_GetTick();

    if (raw != b->raw_last) {          // түүхий төлөв өөрчлөгдвөл цаг эхлүүлнэ
        b->raw_last  = raw;
        b->change_ms = now;
    }

    // тогтвортой болтол хүлээж байж л debounced төлвийг шинэчилнэ
    if ((now - b->change_ms) >= BTN_DEBOUNCE_MS && b->stable != raw) {
        b->stable = raw;
        if (raw) {                     // цэвэр шинэ даралт → эхний алхам
            b->repeat_ms = now;
            return 1;
        }
    }

    if (b->stable && (now - b->repeat_ms) >= BTN_REPEAT_MS) {  // барьж байвал давтах
        b->repeat_ms = now;
        return 1;
    }

    return 0;
}


/* =============================================================================
 *  RACK_CLIMB_TEST — L1/R1 ХОЁУЛАНГ, L2/R2 нэг ракийг тусад нь удирдана
 *  (front/back-ийг ТУСАД НЬ хөдөлгөж шатанд авирахад турших горим)
 *
 *    L1 → ХОЁУЛАА  0       R1 → ХОЁУЛАА  1000
 *    L2 → FRONT    0       R2 → BACK     0
 *
 *  L1/R1 нь хоёр ракийг зэрэг; L2/R2 нь нөгөөг нь ХӨНДӨХГҮЙ, зөвхөн нэгийг нь
 *  0 руу буулгана. Rack_SetTarget зорилтыг тавьж, TIM7 ISR доторх Rack_Service
 *  байрлалыг тасралтгүй БАРИНА (таталцлыг PID нөхнө).
 *
 *  Түвшнийг өөрчлөх бол доорх #define-үүдийг засна.
 *
 *  ⚠ Урьдчилан Rack_SetHome() ажиллуулж encoder-ийн 0 цэгийг доод limit switch
 *    дээр тогтоосон байх ЁСТОЙ. Эс бөгөөс түвшнүүд асаах үеийн санамсаргүй
 *    байрлалаас тоологдоно.
 * =============================================================================
 */
#define RACK_LEVEL_DOWN     0   // L1 / L2 / R2 → доод байрлал
#define RACK_LEVEL_UP    1000   // R1 → дээш (хоёулаа)

/* -----------------------------------------------------------------------------
 *  btn_rising — Debounce хийсэн ЦЭВЭР ШИНЭ даралт (0→1), ДАВТАЛТГҮЙ
 *
 *  btn_step нь товчийг барьж байхад BTN_REPEAT_MS тутам давтдаг. Preset-д
 *  давталт хэрэггүй тул давталтгүй хувилбар.
 * -----------------------------------------------------------------------------
 */
static uint8_t btn_rising(Btn_t *b, uint8_t raw) {
    uint32_t now = HAL_GetTick();

    if (raw != b->raw_last) {          // түүхий төлөв өөрчлөгдвөл цаг эхлүүлнэ
        b->raw_last  = raw;
        b->change_ms = now;
    }

    if ((now - b->change_ms) >= BTN_DEBOUNCE_MS && b->stable != raw) {
        b->stable = raw;
        if (raw) return 1;             // зөвхөн 0 → 1 шилжилтэд
    }

    return 0;
}

void Rack_Climb_Test(void) {
    static Btn_t bL1 = {0}, bR1 = {0}, bL2 = {0}, bR2 = {0};
    static int   tgtFront = 0, tgtBack = 0;   // зорилтууд (homing-ийн дараа 0)

    /* control_data[3]:  [0]=L1  [1]=R1  [2]=L2  [3]=R2
     * land_soft: ХОЁУЛАА зэрэг 0 → ЗӨӨЛӨН,  ТУСАД НЬ 0 → ХҮЧТЭЙ */
    if (btn_rising(&bL1, (uint8_t)control_data[3][0])) {   // L1 → ХОЁУЛАА доош (зөөлөн)
        tgtFront = RACK_LEVEL_DOWN;  frontRack.land_soft = 1;
        tgtBack  = RACK_LEVEL_DOWN;  backRack.land_soft  = 1;
    }
    if (btn_rising(&bR1, (uint8_t)control_data[3][1])) {   // R1 → ХОЁУЛАА дээш
        tgtFront = RACK_LEVEL_UP;
        tgtBack  = RACK_LEVEL_UP;
    }
    if (btn_rising(&bL2, (uint8_t)control_data[3][2])) {   // L2 → зөвхөн FRONT доош (хүчтэй)
        tgtFront = RACK_LEVEL_DOWN;  frontRack.land_soft = 0;
    }
    if (btn_rising(&bR2, (uint8_t)control_data[3][3])) {   // R2 → зөвхөн BACK доош (хүчтэй)
        tgtBack  = RACK_LEVEL_DOWN;  backRack.land_soft = 0;
    }

    /* Зорилтыг тавина — байрлалыг Rack_Service (TIM7 ISR) тасралтгүй барина */
    Rack_SetTarget(&backRack,  tgtBack);
    Rack_SetTarget(&frontRack, tgtFront);

    /* --- OLED (100мс тутам): СЕРВО өнцөг + rack-ийн зорилт/бодит байрлал --- */
    static uint32_t t = 0;
    if (HAL_GetTick() - t >= 100) {
        t = HAL_GetTick();
        colorFill(Black);
        setCursor(2, 2);
        printStr("SRV:%d deg", g_servo_deg);   // серво (Servo_Preset_Control бичдэг)
        setCursor(2, 18);
        printStr("B %d>%d", tgtBack, counter[backRack.enc]);     // зорилт>бодит
        setCursor(2, 38);
        printStr("F %d>%d", tgtFront, counter[frontRack.enc]);
        setScreen();
    }
}


/* =============================================================================
 *  RACK PRESET — товч дармагц ХОЁУЛАА тогтсон ТҮВШИН рүү (тогтмол байрлалууд)
 *
 *    L1 → 0       L2 → 900       R1 → 1350       R2 → 1950
 *
 *  Climb_Test-ээс ялгаатай нь: front/back-ийг ТУСАД НЬ биш, үргэлж ХОЁУЛАНГ
 *  нь нэг ижил түвшин рүү явуулна (0..1950 тогтмол preset).
 * =============================================================================
 */
#define RACK_CLIMB_L0     0     // L1
#define RACK_CLIMB_L1   900     // L2
#define RACK_CLIMB_L2  1350     // R1
#define RACK_CLIMB_L3  1950     // R2 (дээд; pos_max-тай тэнцүү)

/*  Товчны логик — OLED-ГҮЙ. Өөр тесттэй хослуулахад (ж: Gyro_Turn_Test)
 *  дэлгэц мөргөлдөхгүйн тулд тусад нь салгав. Буцаах: одоогийн зорилт.     */
int Rack_Preset_Buttons(void) {
    static Btn_t bL1 = {0}, bR1 = {0}, bL2 = {0}, bR2 = {0};
    static int   target = 0;

    /* control_data[3]:  [0]=L1  [1]=R1  [2]=L2  [3]=R2 */
    if (btn_rising(&bL1, (uint8_t)control_data[3][0])) target = RACK_CLIMB_L0;  // 0
    if (btn_rising(&bL2, (uint8_t)control_data[3][2])) target = RACK_CLIMB_L1;  // 900
    if (btn_rising(&bR1, (uint8_t)control_data[3][1])) target = RACK_CLIMB_L2;  // 1350
    if (btn_rising(&bR2, (uint8_t)control_data[3][3])) target = RACK_CLIMB_L3;  // 1950

    /* ХОЁУЛАНГ нь нэг түвшин рүү — байрлалыг Rack_Service (TIM7 ISR) барина */
    Rack_SetTarget(&frontRack, target);
    Rack_SetTarget(&backRack,  target);

    return target;
}

void Rack_Preset_Control(void) {
    int target = Rack_Preset_Buttons();

    /* --- OLED (100мс тутам): зорилт + хоёр ракийн бодит байрлал --- */
    static uint32_t t = 0;
    if (HAL_GetTick() - t >= 100) {
        t = HAL_GetTick();
        colorFill(Black);
        setCursor(2, 2);
        printStr("PRESET:%d", target);
        setCursor(2, 22);
        printStr("B at:%d", counter[backRack.enc]);
        setCursor(2, 42);
        printStr("F at:%d", counter[frontRack.enc]);
        setScreen();
    }
}


/* =============================================================================
 *  СЕРВО УДИРДЛАГА — ГРАДУСААР (1 серво, htim1 CH1)
 *
 *    Triangle △  →  0° ↔ 180° СЭЛГЭХ (debounce, давталтгүй)
 *
 *  Өөр товч ашиглахгүй — 0/180-аас өөр байрлал хэрэггүй тул алхмаар тааруулах
 *  (✕ +5° / ○ 0° / ▭ 180°) хувилбарыг хассан.
 *
 *  Одоогийн өнцөг g_servo_deg-д хадгалагдаж OLED дээр харагдана.
 *  АСААХАД 180°-оос эхэлнэ (эхлэлийн байрлал).
 *
 *  ⚠ controlServo нь 0..SERVO_UNIT_MAX-аас гарсан утга авбал errorFunction()
 *    руу орж роботыг БҮРЭН ГАЦААНА (buzzer-ийн хязгааргүй гогцоо). Тиймээс
 *    градусыг хааж, дараа нь л unit болгож хөрвүүлнэ.
 * =============================================================================
 */
#define SERVO_UNIT_MAX  24000   // controlServo-ийн дээд утга (= SERVO_DEG_MAX градус)

/* -----------------------------------------------------------------------------
 *  servo_deg_to_units — градус (0..180) → controlServo-ийн unit (0..24000)
 *  Хязгаарыг ЭНД хаана — errorFunction()-аас сэргийлнэ.
 * -----------------------------------------------------------------------------
 */
static int servo_deg_to_units(int deg) {
    if (deg < 0)             deg = 0;
    if (deg > SERVO_DEG_MAX) deg = SERVO_DEG_MAX;
    return (deg * SERVO_UNIT_MAX) / SERVO_DEG_MAX;   // 180° → 24000
}

/* -----------------------------------------------------------------------------
 *  Servo_SetDeg — сервог өнцгөөр тавих (автомат дараалалд ашиглана)
 *
 *  g_servo_deg-ийг МӨН шинэчилнэ — эс тэгвээс △ toggle нь одоогийн байрлалаас
 *  шийддэг тул автомат дараалал сервог зөөсний дараа буруу тал руу үсэрнэ.
 * -----------------------------------------------------------------------------
 */
void Servo_SetDeg(int deg) {
    if (deg < 0)             deg = 0;
    if (deg > SERVO_DEG_MAX) deg = SERVO_DEG_MAX;
    g_servo_deg = deg;
    controlServo(true, servo_deg_to_units(deg));   // htim1 CH1
}

/* -----------------------------------------------------------------------------
 *  Servo_Home — эхлэлийн байрлалд (180°) тавих.  Init-д ЗААВАЛ дуудна.
 *
 *  g_servo_deg нь 180-аар эхэлдэг ч энэ нь зөвхөн ПРОГРАММЫН утга — hardware
 *  руу бичигдэхгүй. Өмнө нь зөвхөн Servo_Preset_Buttons-ийн inited блок бичдэг
 *  байсан тул автомат дараалал (тэр функцийг дуудахгүй) үед серво асахдаа
 *  хаана байснаа тэндээ үлдэж, g_servo_deg-тэй зөрж байв.
 * -----------------------------------------------------------------------------
 */
void Servo_Home(void) {
    Servo_SetDeg(SERVO_HOME_DEG);
}

/*  Товчны логик — OLED-ГҮЙ. Өөр тесттэй хослуулахад (ж: Gyro_Turn_Test)
 *  дэлгэц мөргөлдөхгүйн тулд тусад нь салгав. Буцаах: одоогийн өнцөг.      */
int Servo_Preset_Buttons(void) {
    static Btn_t   bTri   = {0};
    static uint8_t inited = 0;

    if (!inited) {                 // асаахад мэдэгдэж буй байрлалд тавина
        controlServo(true, servo_deg_to_units(g_servo_deg));
        inited = 1;
    }

    /* --- △ : нэг даралт = 0° ↔ 180° сэлгэх (debounce, давталтгүй) --- */
    if (btn_rising(&bTri, (uint8_t)control_data[1][2])) {
        g_servo_deg = (g_servo_deg >= SERVO_DEG_MAX / 2) ? 0 : SERVO_DEG_MAX;
        controlServo(true, servo_deg_to_units(g_servo_deg));   // htim1 CH1
    }

    return g_servo_deg;
}

void Servo_Preset_Control(void) {
    int deg = Servo_Preset_Buttons();

    /* --- OLED (100мс тутам): өнцөг + timer-т бичигдсэн бодит unit --- */
    static uint32_t t = 0;
    if (HAL_GetTick() - t >= 100) {
        t = HAL_GetTick();
        colorFill(Black);
        setCursor(2, 2);
        printStr("SERVO");
        setCursor(2, 22);
        printStr("%d deg", deg);
        setCursor(2, 42);
        printStr("u:%d", servo_deg_to_units(deg));
        setScreen();
    }
}


/* =============================================================================
 *  СОЛЕНОИД 1 — D-Up товчоор TOGGLE (нэг даралт = нэг сэлгэлт)
 *
 *    control_data[2][2] = D-Up  →  соленоид 1
 *
 *  Товч дарах бүрд соленоид ON ↔ OFF сэлгэнэ. Дараад БАРЬЖ байхад давтахгүй
 *  (зөвхөн 0→1 ирмэг дээр), механик чичиргээг BTN_DEBOUNCE_MS-ээр шүүнэ.
 *  Асаахад мэдэгдэж буй OFF төлөвт тавина.
 * =============================================================================
 */
#define SOLENOID1_BTN   2   // control_data[2][SOLENOID1_BTN] = D-Up
#define SOLENOID1_NUM   1   // controlSolenoid-ийн дугаар

void Solenoid_Control(void) {
    static Btn_t   btn    = {0};
    static bool    state  = false;
    static uint8_t inited = 0;

    if (!inited) {                                    // программын төлөвтэй нийцүүлнэ
        controlSolenoid(SOLENOID1_NUM, false);
        inited = 1;
    }

    if (btn_rising(&btn, (uint8_t)control_data[2][SOLENOID1_BTN])) {
        state = !state;                               // ON ↔ OFF
        controlSolenoid(SOLENOID1_NUM, state);
    }
}


/* =============================================================================
 *  RACK ТЕЛЕМЕТР — тааруулгад хэрэгтэй бүх утгыг UART4 (115200) руу TSV-ээр
 *
 *  Багана (эхэнд нэг удаа толгой мөр гарна):
 *    Btgt  — back rack-ийн ЗОРИЛТ        Ftgt — front rack-ийн зорилт
 *    Bpos  — back rack-ийн БОДИТ байрлал  Fpos — front rack-ийн бодит байрлал
 *    Bpwm  — back-д ӨГСӨН PWM             Fpwm — front-д өгсөн PWM
 *    Bi    — integral-ийн PWM ХУВЬ НЭМЭР (ki x integral) — windup харагдана
 *    S1 S2 — limit switch (0 = ДАРАГДСАН)
 *
 *  TSV тул шууд spreadsheet-д буулгаж график гаргаж болно.
 *
 *  send_uart нь БЛОКЛОДОГ (~5мс) ч PID нь TIM7 ISR-т ажилладаг тул
 *  удирдлагад НӨЛӨӨЛӨХГҮЙ.
 *
 *  ⚠ huart4-ийг LPMS gyro мөн ашигладаг — LPMS идэвхтэй үед БҮҮ дууд.
 * =============================================================================
 */
#define RACK_TLM_MS  50   // илгээх интервал (мс) — шилжилтийг барихад хангалттай

void Rack_Telemetry_Serial(void) {
    static uint32_t t   = 0;
    static uint8_t  hdr = 0;

    if (!hdr) {
        send_uart("Btgt\tBpos\tBpwm\tBi\tFtgt\tFpos\tFpwm\tFi\tS1\tS2\r\n");
        hdr = 1;
    }

    if (HAL_GetTick() - t < RACK_TLM_MS) return;
    t = HAL_GetTick();

    char buf[128];
    sprintf(buf, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
            backRack.target,  counter[backRack.enc],  backRack.last_pwm,
            (int)(backRack.ki  * backRack.integral),
            frontRack.target, counter[frontRack.enc], frontRack.last_pwm,
            (int)(frontRack.ki * frontRack.integral),
            (int)val1, (int)val2);
    send_uart(buf);
}

/* -----------------------------------------------------------------------------
 *  Rack_PWM_Tune_Dual — ХОЁР моторыг ТУСАД нь open-loop PWM тааруулах (debounce-тай)
 *
 *    Мотор 5 (front): D-Up (control_data[2][2]) +,  D-Down (control_data[2][0]) −
 *    Мотор 6 (back) : Triangle (control_data[1][2]) +,  Cross (control_data[1][0]) −
 *
 *  Мотор бүр өөрийн PWM-тэй, тусдаа тааруулагдана. OLED дээр M5/M6 хоёуланг харуулна.
 * -----------------------------------------------------------------------------
 */
void Rack_PWM_Tune_Dual(void) {
    static int   pwm5 = 0, pwm6 = 0;
    static Btn_t bUp = {0}, bDn = {0}, bTri = {0}, bCr = {0};

    // --- Мотор 5 (front): D-Up / D-Down ---
    if (btn_step(&bUp, control_data[2][2])) pwm5 += TUNE_STEP;   // D-Up
    if (btn_step(&bDn, control_data[2][0])) pwm5 -= TUNE_STEP;   // D-Down

    // --- Мотор 6 (back): Triangle / Cross ---
    if (btn_step(&bTri, control_data[1][2])) pwm6 += TUNE_STEP;  // Triangle
    if (btn_step(&bCr,  control_data[1][0])) pwm6 -= TUNE_STEP;  // Cross

    if (pwm5 >  1000) pwm5 =  1000;
    if (pwm5 < -1000) pwm5 = -1000;
    if (pwm6 >  1000) pwm6 =  1000;
    if (pwm6 < -1000) pwm6 = -1000;

    motor_control(5, pwm5);
    motor_control(6, pwm6);

    // OLED дээр хоёуланг харуулах (100мс-д нэг)
    static uint32_t st = 0;
    if (HAL_GetTick() - st >= 100) {
        st = HAL_GetTick();
        colorFill(Black);
        setCursor(10, 6);
        printStr("M5:%d M6:%d", pwm5, pwm6);
        setCursor(10, 30);
        printStr("F:%d B:%d", counter[0], counter[1]);
        setScreen();
    }
}

/* =============================================================================
 *  ОНОШИЛГОО: Rack-ийн параметрүүдийг ХЭМЖИЖ тогтоох joystick тест
 *
 *  ЗОРИЛГО: дараах 6 зүйлийг таамаглалгүй, хэмжилтээр тогтооно —
 *    1. Аль мотор аль ракыг хөдөлгөдөг (5 → front уу, back уу?)
 *    2. Аль encoder (counter[0] / counter[1]) тэр ракийнх вэ?
 *    3. Эерэг PWM өгөхөд rack ДЭЭШ үү, ДООШ уу?
 *    4. Rack дээш явахад counter ӨСНӨ үү, БУУРНА уу?
 *    5. Аль limit switch (S1=val1 / S2=val2) тэр ракийнх вэ?
 *    6. Бүтэн замын encoder тоолол (pos_max)
 *
 *  УДИРДЛАГА:
 *    Зүүн стик Y   → Мотор 5   (стик ДЭЭШ = эерэг PWM)
 *    Баруун стик Y → Мотор 6   (стик ДЭЭШ = эерэг PWM)
 *    Cross (✕)     → encoder-үүдийг 0 болгох
 *
 *  АВТОМАТ ИЛРҮҮЛЭЛТ (2, 4, 5-ыг код өөрөө тогтооно):
 *    Нэг моторыг ГАНЦААРАА жолоодоход л хэмжинэ (хоёулаа зэрэг явбал алгасна).
 *      - Мотор → encoder хос : аль encoder илүү тоолсноор
 *      - Чиглэлийн нийцэл    : '+' = +PWM өгөхөд counter ӨСНӨ (НИЙЦНЭ)
 *                              '-' = +PWM өгөхөд counter БУУРНА (УРВУУ)
 *      - Мотор → limit switch: тухайн мотор явж байхад ирмэг өгсөн switch
 *
 *  OLED:
 *    M5/M6 = өгсөн PWM,   E0/E1 = counter,   S1/S2 = limit switch (0 = ДАРАГДСАН)
 *    4-р мөр: "5>E0+ 6>E1-"  ← мотор>encoder ба чиглэлийн тэмдэг ('?' = хараахан үгүй)
 *
 *  SERIAL (UART4, 115200, 250мс тутам):
 *    M5>E0 dir:+ sw:S2 | M6>E1 dir:- sw:S1 | E0:.. E1:.. S1:.. S2:..
 *
 *  1 ба 6-г ХҮН тогтооно: стикээ түлхээд аль ФИЗИК рак (урд/хойд) хөдөлснийг
 *  хараарай — firmware үүнийг мэдэх боломжгүй.
 *
 *  Cross (✕) = encoder БОЛОН илрүүлэгчийн хуримтлалыг тэглэж шинээр хэмжинэ.
 *
 *  АЮУЛГҮЙ БАЙДАЛ: rack PID (ISR-service) унтраалттай, PWM дээд тал нь
 *  JTEST_MAX_PWM. Чиглэл тодорхойгүй тул автомат хамгаалалт БАЙХГҮЙ —
 *  limit switch 0 болмогц стикээ суллаарай.
 * =============================================================================
 */
#define JTEST_MAX_PWM   250   // аюулгүйн дээд PWM (чиглэл олох хангалттай)
#define JTEST_DEADZONE   12   // стикийн үхмэл бүс

#define JTEST_MIN_COUNTS  20   // хос тогтоохын өмнө шаардагдах хамгийн бага тоолол

void Rack_Joystick_Test(void) {
    // Rack PID ISR-service унтраалттай эсэхийг баталгаажуулна (зөрчихгүй)
    frontRack.active = 0;
    backRack.active  = 0;

    /* ---- Илрүүлэгчийн хуримтлагдах төлөв ------------------------------------
     *  acc[m][e] — мотор m ГАНЦААРАА ажиллах үед encoder e хэдэн count хөдөлсөн
     *              (абсолют). Их нь тэр моторын encoder.
     *  dir[m][e] — ТЭМДЭГТЭЙ хуримтлал: "+PWM өгөхөд Δcount" (сөрөг PWM-ийг
     *              урвуулж нэмнэ). > 0 бол +PWM → counter ӨСНӨ (чиглэл НИЙЦНЭ).
     *  sw[m]     — тухайн мотор ажиллаж байхад ирмэг өгсөн limit switch (1=S1, 2=S2)
     * ------------------------------------------------------------------------ */
    static int32_t acc[2][2] = {{0, 0}, {0, 0}};   // [0]=M5, [1]=M6
    static int32_t dir[2][2] = {{0, 0}, {0, 0}};
    static uint8_t sw[2]     = {0, 0};             // 0 = тодорхойгүй
    static int     prev0 = 0, prev1 = 0;
    static uint8_t ps1 = 0, ps2 = 0;
    static uint8_t inited = 0;

    if (!inited) {
        prev0 = counter[0];
        prev1 = counter[1];
        ps1 = (val1 == 0);
        ps2 = (val2 == 0);
        inited = 1;
    }

    // --- Стикээс PWM. ХЭМЖСЭН: түүхий стик ДЭЭШ = ЭЕРЭГ тул эргүүлэхгүй шууд авна ---
    int ly = control_data[0][1];   // зүүн стик Y
    int ry = control_data[0][3];   // баруун стик Y
    if (ly > -JTEST_DEADZONE && ly < JTEST_DEADZONE) ly = 0;
    if (ry > -JTEST_DEADZONE && ry < JTEST_DEADZONE) ry = 0;

    int pwm5 = ly * JTEST_MAX_PWM / 100;
    int pwm6 = ry * JTEST_MAX_PWM / 100;

    motor_control(5, pwm5);
    motor_control(6, pwm6);

    /* ---- Хэмжилт: encoder-ийн өөрчлөлт + limit switch-ийн ирмэг -------------
     *  ЗӨВХӨН нэг мотор ажиллаж байх үед л хамааруулна. Хоёулаа зэрэг явбал
     *  аль нь аль encoder-ийг хөдөлгөснийг ялгаж чадахгүй тул алгасна.
     * ------------------------------------------------------------------------ */
    int c0 = counter[0], c1 = counter[1];
    int d0 = c0 - prev0, d1 = c1 - prev1;
    prev0 = c0; prev1 = c1;

    uint8_t s1 = (val1 == 0);      // 1 = ДАРАГДСАН (pull-up тул 0 = дарагдсан)
    uint8_t s2 = (val2 == 0);

    int m = -1;                    // аль мотор ГАНЦААРАА явж байна: 0=M5, 1=M6
    int pwm = 0;
    if (pwm5 != 0 && pwm6 == 0)      { m = 0; pwm = pwm5; }
    else if (pwm6 != 0 && pwm5 == 0) { m = 1; pwm = pwm6; }

    if (m >= 0) {
        acc[m][0] += (d0 < 0) ? -d0 : d0;
        acc[m][1] += (d1 < 0) ? -d1 : d1;

        // +PWM үед Δ; −PWM бол тэмдгийг урвуулж нэмнэ → үр дүн нь "+PWM-ийн Δ"
        dir[m][0] += (pwm > 0) ? d0 : -d0;
        dir[m][1] += (pwm > 0) ? d1 : -d1;

        if (s1 != ps1) sw[m] = 1;   // энэ мотор явж байхад S1 ирмэг өглөө
        if (s2 != ps2) sw[m] = 2;
    }
    ps1 = s1; ps2 = s2;

    // --- Cross (✕): encoder + илрүүлэгчийн хуримтлалыг тэглэх (шинээр хэмжих) ---
    static uint8_t cross_prev = 0;
    uint8_t cross = (uint8_t)control_data[1][0];
    if (cross && !cross_prev) {
        enc_zero(0);
        enc_zero(1);
        prev0 = 0; prev1 = 0;
        for (int i = 0; i < 2; i++) {
            acc[i][0] = acc[i][1] = 0;
            dir[i][0] = dir[i][1] = 0;
            sw[i] = 0;
        }
    }
    cross_prev = cross;

    /* ---- Дүгнэлт: мотор → encoder хос, чиглэлийн нийцэл ---------------------
     *  enc[m] : тухайн моторын encoder (илүү их тоолсон нь).  -1 = тодорхойгүй
     *  sgn[m] : '+' = +PWM өгөхөд counter ӨСНӨ (НИЙЦНЭ)
     *           '-' = +PWM өгөхөд counter БУУРНА (УРВУУ)
     * ------------------------------------------------------------------------ */
    int  enc[2];
    char sgn[2];
    for (int i = 0; i < 2; i++) {
        if (acc[i][0] + acc[i][1] < JTEST_MIN_COUNTS) {
            enc[i] = -1;           // хангалттай хөдөлгөөн хараахан алга
            sgn[i] = '?';
        } else {
            enc[i] = (acc[i][0] > acc[i][1]) ? 0 : 1;
            int32_t d = dir[i][enc[i]];
            sgn[i] = (d > 0) ? '+' : ((d < 0) ? '-' : '?');
        }
    }

    // --- OLED (100мс тутам) ---
    static uint32_t t = 0;
    if (HAL_GetTick() - t >= 100) {
        t = HAL_GetTick();
        colorFill(Black);
        setCursor(2, 2);
        printStr("M5:%d M6:%d", pwm5, pwm6);
        setCursor(2, 18);
        printStr("E0:%d E1:%d", counter[0], counter[1]);
        setCursor(2, 34);
        printStr("S1:%d S2:%d", (int)val1, (int)val2);
        setCursor(2, 50);
        // Ж: "5>E0+ 6>E1-"  (E? = хараахан тодорхойгүй)
        if (enc[0] < 0) printStr("5>E?? ");
        else            printStr("5>E%d%c ", enc[0], sgn[0]);
        if (enc[1] < 0) printStr("6>E??");
        else            printStr("6>E%d%c", enc[1], sgn[1]);
        setScreen();
    }

    // --- Serial тайлан UART4 (115200), 250мс тутам ---
    static uint32_t ts = 0;
    if (HAL_GetTick() - ts >= 250) {
        ts = HAL_GetTick();
        char buf[110];
        const char *sw_txt[3] = { "??", "S1", "S2" };
        sprintf(buf,
                "M5>E%c dir:%c sw:%s | M6>E%c dir:%c sw:%s | E0:%d E1:%d S1:%d S2:%d\r\n",
                (enc[0] < 0) ? '?' : (char)('0' + enc[0]), sgn[0], sw_txt[sw[0]],
                (enc[1] < 0) ? '?' : (char)('0' + enc[1]), sgn[1], sw_txt[sw[1]],
                counter[0], counter[1], (int)val1, (int)val2);
        send_uart(buf);
    }
}


/* =============================================================================
 *  Gyro_TurnAngle — Одоогийн чигээсээ RELATIVE өнцгөөр эргэх PID
 *    angle: +90 = нэг тийш 90°, -90 = нөгөө тийш, ±180 = эргэх
 *    return: 1 = дууссан, 0 = эргэж байна (loop-д давтан дууд)
 *
 *  Эхний дуудалтад одоогийн чигийг 0° болгож (Gyro_ZeroYaw), зорилтыг angle
 *  болгоно. Тасралтгүй yaw_rel ашигладаг тул ±180 хилийн орчимд асуудалгүй —
 *  ямар ч чигт байсан "тэр чигээсээ angle зэрэг" эргэнэ (ж: 30° → 120°).
 * =============================================================================
 */
/*  Тааруулга: робот дугуйгаараа газар зуран эргэдэг тул СТАТИК ҮРЭЛТ өндөр.
 *  KP=5 / MIN_PWM=50 үед 14° алдаа дээр ~110 PWM өгч байсан нь хөдөлгөхөд
 *  хүрэлцээгүй → 76° дээр гацаж TIMEOUT болсон. Тиймээс зорилтын ойролцоох
 *  эрх мэдлийг (KP) болон үрэлт таслах доод хүчийг (MIN_PWM) нэмэв.        */
#define TURN_KP        12.0f     // 14° алдаа → 168 PWM (өмнө 70)
#define TURN_KI        0.25f     // гацвал integral нэмж 150 хүртэл PWM өгнө
#define TURN_KD        80.0f     // KP өссөн тул тоормосыг мөн нэмэв
#define TURN_D_ALPHA   0.6f      // derivative шүүлтүүр (LPMS 90Hz, PID 10ms → чимээтэй)
#define TURN_I_MAX     600.0f
#define TURN_I_BAND    25.0f     // зөвхөн ийм ойр орсны дараа integral хуримтлана
#define TURN_DT_MS     10        // PID интервал (LPMS-ийн 11ms-тэй ойролцоо)
#define TURN_MAX_PWM   450
#define TURN_MIN_PWM   200       // статик үрэлт таслах босго (энэнээс доош = гацна)
#define TURN_TOL       1.5f      // ±° хүлцэл (3.0 үед 87-д зогсдог байсан)
#define TURN_HOLD_MS   250       // tol дотор энэ хугацаанд барих → дууссан
#define TURN_TIMEOUT_MS   5000   // энэ хугацаанд эргэж дуусахгүй бол зогсоно
#define TURN_ACCEPT    5.0f      // timeout болоход ийм ойр байвал алдаа биш гэж үзнэ
#define TURN_RUNAWAY_DEG  30.0f  // алдаа эхнийхээсээ ийм хэмжээгээр ӨСВӨЛ = тэмдэг буруу

/*  ⚠ TURN_MOTOR_SIGN — PWM-ийн тэмдэг ба yaw_rel-ийн тэмдгийг НИЙЦҮҮЛНЭ.
 *    Шаардлага: pwm > 0 үед yaw_rel ӨСӨХ ёстой. Эсрэг бол PID нь сөрөг биш
 *    ЭЕРЭГ feedback болж, робот зогсохгүй эргэлдэнэ (гацсан гогцоо).
 *    Хэрэв OLED дээр "DIR?" гарвал энэ тэмдгийг эсрэгээр нь болго.        */
#define TURN_MOTOR_SIGN  (+1)

/* Дотоод төлөв — Gyro_TurnReset()-ээр гаднаас цэвэрлэнэ */
static uint8_t  turn_started   = 0;
static float    turn_tgt       = 0.0f;
static float    turn_perr      = 0.0f;
static float    turn_integ     = 0.0f;
static float    turn_dfilt     = 0.0f;
static uint32_t turn_zone_t    = 0;
static uint32_t turn_t0        = 0;
static float    turn_err0      = 0.0f;
static uint8_t  turn_fail      = 0;   // 0=OK, 1=TIMEOUT, 2=DIR (тэмдэг буруу)

/* Эргэлтийг таслах / дахин эхнээс нь эхлүүлэхэд (блок reset дотроос) */
void Gyro_TurnReset(void) {
    turn_started = 0;
    turn_zone_t  = 0;
    turn_fail    = 0;
}

/* Сүүлийн эргэлт хэрхэн дууссан: 0=амжилттай, 1=TIMEOUT, 2=чиглэл буруу */
uint8_t Gyro_TurnFail(void) {
    return turn_fail;
}

uint8_t Gyro_TurnAngle(float angle) {
    // ---- Эхний дуудалт: одоогийн чигийг 0 болгож, зорилт = angle ----
    if (!turn_started) {
        LPMS_Read();         // DMA буферээс шинэ өгөгдөл (blocking БИШ)
        Gyro_ZeroYaw();      // одоогийн чиг = 0°
        LPMS_Read();         // yaw_rel-ийн шинэ суурийг тогтоох

        turn_tgt     = angle;   // тэг цэгээс angle зэрэг эргэнэ (харьцангуй)
        turn_perr    = angle;   // эхний derivative үсрэлтээс сэргийлнэ
        turn_integ   = 0.0f;
        turn_dfilt   = 0.0f;
        turn_zone_t  = 0;
        turn_started = 1;
        turn_fail    = 0;
        turn_t0      = HAL_GetTick();
        turn_err0    = (angle < 0.0f) ? -angle : angle;   // |эхний алдаа|
        timer        = 0;
        brake();
        return 0;
    }

    LPMS_Read();
    float current = yaw_rel;   // тасралтгүй (-360..+360), зөвхөн энэ эргэлтэд харьцангуй

    // ---- Алдаа (wrap хэрэггүй — yaw_rel тасралтгүй) ----
    float error = turn_tgt - current;
    float aerr  = (error < 0.0f) ? -error : error;

    // ---- ХАМГААЛАЛТ: алдаа БАГАСАХЫН оронд ӨССӨН = PWM/gyro тэмдэг зөрчилтэй.
    //      Ингэвэл PID нь эерэг feedback болж робот тасралтгүй эргэлдэнэ. ----
    if (aerr > turn_err0 + TURN_RUNAWAY_DEG) {
        brake();
        turn_fail    = 2;      // → TURN_MOTOR_SIGN-ийг эсрэгээр нь болго
        turn_started = 0;
        return 1;
    }

    // ---- ХАМГААЛАЛТ: хугацаа хэтэрсэн ----
    //   Гэхдээ зорилтдоо TURN_ACCEPT дотор ойрхон байвал (үрэлтээс болж сүүлийн
    //   хэдэн градусыг чадаагүй) үүнийг алдаа гэж үзэхгүй — автомат дараалал
    //   2°-ын зөрүүнээс болж Robot_Error рүү орох ёсгүй.
    if (HAL_GetTick() - turn_t0 >= TURN_TIMEOUT_MS) {
        brake();
        turn_fail    = (aerr < TURN_ACCEPT) ? 0 : 1;
        turn_started = 0;
        return 1;
    }

    // ---- Дууссан эсэх (tol дотор HOLD_MS барих) ----
    if (error < TURN_TOL && error > -TURN_TOL) {
        brake();
        if (turn_zone_t == 0) turn_zone_t = HAL_GetTick();
        if (HAL_GetTick() - turn_zone_t >= TURN_HOLD_MS) {
            turn_started = 0;          // дараагийн эргэлтэд дахин init
            return 1;
        }
        return 0;
    } else {
        turn_zone_t = 0;
    }

    // ---- PID тогтмол интервалаар ----
    if (timer > TURN_DT_MS) {
        timer = 0;

        // Integral — зөвхөн ойрхон (anti-windup)
        if (error < TURN_I_BAND && error > -TURN_I_BAND) {
            turn_integ += error;
            if (turn_integ >  TURN_I_MAX) turn_integ =  TURN_I_MAX;
            if (turn_integ < -TURN_I_MAX) turn_integ = -TURN_I_MAX;
        } else {
            turn_integ = 0.0f;
        }

        // Derivative — шүүсэн (эргэх хурдны тоормос)
        float draw   = error - turn_perr;
        turn_perr    = error;
        turn_dfilt   = (TURN_D_ALPHA * turn_dfilt) + ((1.0f - TURN_D_ALPHA) * draw);

        float output = (TURN_KP * error)
                     + (TURN_KI * turn_integ)
                     + (TURN_KD * turn_dfilt);

        if (output >  TURN_MAX_PWM) output =  TURN_MAX_PWM;
        if (output < -TURN_MAX_PWM) output = -TURN_MAX_PWM;

        int pwm = (int)output;

        // Min PWM — эргэхэд хангалттай хүч (зөвхөн tol-ийн гадна)
        if (pwm > 0 && pwm <  TURN_MIN_PWM) pwm =  TURN_MIN_PWM;
        if (pwm < 0 && pwm > -TURN_MIN_PWM) pwm = -TURN_MIN_PWM;

        // ЗҮҮН (1,3) vs БАРУУН (2,4) — TURN_MOTOR_SIGN нь pwm>0 → yaw_rel өсөх
        // болгож нийцүүлнэ (нийцэхгүй бол дээрх runaway хамгаалалт барина).
        int m = TURN_MOTOR_SIGN * pwm;
        motor_control(1,  m);   // зүүн
        motor_control(3,  m);   // зүүн
        motor_control(2, -m);   // баруун
        motor_control(4, -m);   // баруун
    }

    return 0;
}


/* =============================================================================
 *  Turn_Left_90 / Turn_Right_90 — 90° эргэх (non-blocking)
 *    Loop-д давтан дуудна; дуусахад 1 буцаана, дараа нь өөрөө reset хийгдэнэ.
 *
 *  ⚠ ЧИГЛЭЛ: LPMS хэрхэн суусанаас yaw-ийн тэмдэг хамаарна. Хэрэв Turn_Left_90
 *    баруун тийш эргэвэл доорх TURN_LEFT_SIGN-ийг +1 болго (нэг л мөр).
 * =============================================================================
 */
#define TURN_LEFT_SIGN  (+1.0f)   // зүүн эргэлтийн yaw тэмдэг (буруу бол -1.0f)

uint8_t Turn_Left_90(void)  { return Gyro_TurnAngle(TURN_LEFT_SIGN * 90.0f); }
uint8_t Turn_Right_90(void) { return Gyro_TurnAngle(-TURN_LEFT_SIGN * 90.0f); }


/* =============================================================================
 *  Drive_Straight_Test — ЗӨВХӨН шулуун явахыг турших (өөр юу ч ажиллахгүй)
 *
 *    D-Up   → эхлэх / зогсоох (эхлэхэд одоогийн чигийг anchor болгоно)
 *    D-Down → зогсоох
 *    L1 / R1 → хурд −50 / +50  (STRAIGHT_KP хурдаас хамаарч өөр байж болно)
 *
 *  OLED: ON/OFF,  pwm = суурь хурд,  off = anchor-аас ХАЗАЙЛТ (°) ← ГОЛ ТОО,
 *        max = хамгийн их хазайлт (эхлэснээс хойш).
 *
 *  Шулуун явж байвал off нь 0-ийн эргэн тойронд бага үлдэнэ. max нь нэг
 *  гүйлтийн муу тохиолдлыг хэлнэ — нүдээр харснаас найдвартай.
 * =============================================================================
 */
#define STRAIGHT_TEST_PWM  (-400)   // урагш = СӨРӨГ (жолооны конвенц)

void Drive_Straight_Test(void) {
    static Btn_t   bUp = {0}, bDown = {0}, bL1 = {0}, bR1 = {0};
    static uint8_t on      = 0;
    static int     pwm     = STRAIGHT_TEST_PWM;
    static float   max_off = 0.0f;

    /* --- Хурд тааруулах (урагш = СӨРӨГ тул R1 = хурдан = илүү сөрөг) --- */
    if (btn_rising(&bL1, (uint8_t)control_data[3][0])) pwm += 50;   // L1 → удаан
    if (btn_rising(&bR1, (uint8_t)control_data[3][1])) pwm -= 50;   // R1 → хурдан
    if (pwm >    0) pwm =    0;
    if (pwm < -900) pwm = -900;

    /* --- D-Up → сэлгэх.  Эхлэхэд anchor тавихгүй бол Drive_Straight нь
           хуучин/санамсаргүй чиг рүү залруулна.                          --- */
    if (btn_rising(&bUp, (uint8_t)control_data[2][2])) {
        if (on) { on = 0; brake(); }
        else    { Set_Yaw_Anchor();  max_off = 0.0f;  on = 1; }
    }
    if (btn_rising(&bDown, (uint8_t)control_data[2][0])) { on = 0; brake(); }

    float off = 0.0f;
    if (on) {
        Drive_Straight(pwm);                 // дотроо LPMS_Read() дуудна
        off = Get_Yaw_Offset_From_Anchor();
        float a = (off < 0.0f) ? -off : off;
        if (a > max_off) max_off = a;
    } else {
        LPMS_Read();                         // зогсолтод ч DMA буферээ хоослох
        off = Get_Yaw_Offset_From_Anchor();
    }

    static uint32_t t = 0;
    if (HAL_GetTick() - t >= 100) {
        t = HAL_GetTick();
        colorFill(Black);
        setCursor(2, 2);
        printStr("STR %s", on ? "ON" : "OFF");
        setCursor(2, 18);
        printStr("pwm:%d", pwm);
        setCursor(2, 34);
        printStr("off:%d", (int)off);
        setCursor(2, 50);
        printStr("max:%d", (int)max_off);
        setScreen();
    }
}


/* =============================================================================
 *  Gyro_Turn_Test — 90° эргэлтийн туршилт РАК ӨРГӨГДСӨН үед
 *
 *    Стик    → runner() (гарын жолоо)
 *    D-Left  → зүүн 90°      D-Right → баруун 90°     D-Down → таслах (зогс)
 *    D-Up    → Drive_Straight() сэлгэх (gyro-гоор шулуун урагш)
 *    L1 → рак 0    L2 → рак 900    R1 → рак 1350    R2 → рак 1950
 *    △ → серво 0° ↔ 180° сэлгэх
 *
 *  Рак дээшлэхэд массын төв өндөрсөж инерц нэмэгддэг тул PID-ийн зан төлөв
 *  өөр байж болно — тиймээс бүгдийг нэг тестэд нийлүүлэв.
 *
 *  ⚠ runner(), эргэлтийн PID, Drive_Straight() ГУРВУУЛАА 1-4 моторыг бичнэ.
 *    Тиймээс mode нь тэднээс ЗӨВХӨН НЭГИЙГ идэвхжүүлнэ — эс тэгвээс хоорондоо
 *    дарж бичээд аль нь ч зөв ажиллахгүй (ж: джойстик төвдөө байхад 0 PWM
 *    бичээд эргэлтийн PID-ийг үхүүлнэ).
 *
 *  OLED: M = горим (- жолоо / L / R / S),  Y = чиг,  P = ракийн зорилт,
 *        S = сервоны өнцөг,  F/B = ракуудын бодит байрлал,  ms = эргэлтийн хугацаа.
 * =============================================================================
 */
#define MODE_JOY  0   // джойстикийн жолоо
#define MODE_TL   1   // зүүн 90° эргэлт
#define MODE_TR   2   // баруун 90° эргэлт
#define MODE_STR  3   // gyro-гоор шулуун явах

void Gyro_Turn_Test(void) {
    static Btn_t    bLeft = {0}, bRight = {0}, bDown = {0}, bUp = {0};
    static uint8_t  mode    = MODE_JOY;
    static uint32_t t0      = 0;
    static uint32_t last_ms = 0;

    /* --- Рак ба серво: OLED-гүй хувилбарууд (доор нэг дэлгэц зурна) --- */
    int rack_tgt  = Rack_Preset_Buttons();
    int servo_deg = Servo_Preset_Buttons();

    /* --- Горим сонгох: D-pad.  main.c:106 — [2][1]=D-Right, [2][3]=D-Left --- */
    if (mode == MODE_JOY && btn_rising(&bLeft,  (uint8_t)control_data[2][3])) {
        Gyro_TurnReset();  mode = MODE_TL;  t0 = HAL_GetTick();
    }
    if (mode == MODE_JOY && btn_rising(&bRight, (uint8_t)control_data[2][1])) {
        Gyro_TurnReset();  mode = MODE_TR;  t0 = HAL_GetTick();
    }
    /* D-Up → шулуун явахыг сэлгэнэ. Эхлэхэд ОДООГИЙН чигийг anchor болгоно —
       Drive_Straight нь тэр anchor-аас хазайхыг л залруулна.               */
    if ((mode == MODE_JOY || mode == MODE_STR) &&
        btn_rising(&bUp, (uint8_t)control_data[2][2])) {
        if (mode == MODE_STR) { mode = MODE_JOY;  brake(); }
        else                  { Set_Yaw_Anchor();  mode = MODE_STR; }
    }
    if (btn_rising(&bDown, (uint8_t)control_data[2][0])) {   // D-Down → яаралтай таслах
        Gyro_TurnReset();  mode = MODE_JOY;  brake();
    }

    static uint8_t fail = 0;
    if (mode == MODE_TL) {
        if (Turn_Left_90())  { last_ms = HAL_GetTick() - t0;  fail = Gyro_TurnFail();  mode = MODE_JOY; }
    } else if (mode == MODE_TR) {
        if (Turn_Right_90()) { last_ms = HAL_GetTick() - t0;  fail = Gyro_TurnFail();  mode = MODE_JOY; }
    } else if (mode == MODE_STR) {
        Drive_Straight(STRAIGHT_TEST_PWM);   // дотроо LPMS_Read() дуудна
    } else {
        LPMS_Read();      // зогсолтод ч DMA буферээ хоослох (хоцрохгүй)
        runner();
    }

    static uint32_t t = 0;
    if (HAL_GetTick() - t >= 100) {
        t = HAL_GetTick();
        char m = (mode == MODE_TL)  ? 'L' : (mode == MODE_TR) ? 'R'
               : (mode == MODE_STR) ? 'S' : '-';
        colorFill(Black);
        setCursor(2, 2);
        printStr("M:%c Y:%d", m, (int)yaw_rel);
        setCursor(2, 18);
        printStr("P:%d S:%d", rack_tgt, servo_deg);
        setCursor(2, 34);
        printStr("F:%d B:%d", counter[frontRack.enc], counter[backRack.enc]);
        setCursor(2, 50);
        /* fail: 0 = амжилттай (хугацаа), 1 = TIMEOUT, 2 = чиглэл буруу */
        if      (fail == 2) printStr("DIR? flip");
        else if (fail == 1) printStr("TIMEOUT");
        else                printStr("ms:%lu", (unsigned long)last_ms);
        setScreen();
    }
}


/* =============================================================================
 *  ADC (PC3) унших
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 *  Read_PC3 — ADC1-ээс нэг утга унших (12-bit)
 * -----------------------------------------------------------------------------
 */
uint16_t Read_PC3(void)
{
    uint16_t value = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return value;
}

/* -----------------------------------------------------------------------------
 *  Read_PC3_OLED — ADC утгыг OLED дэлгэц дээр харуулах
 * -----------------------------------------------------------------------------
 */
void Read_PC3_OLED(void)
{
    uint16_t adc_value = Read_PC3();

    colorFill(Black);
    setCursor(10, 10);
    printStr("ADC: %d", adc_value);
    setScreen();
}


/* =============================================================================
 *  USB CDC ФУНКЦУУД
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 *  USB_GetValue — Хүлээн авсан утгыг буцаах (хэзээ ч дуудаж болно)
 * -----------------------------------------------------------------------------
 */
uint8_t USB_GetValue(void)
{
    return usb_received_value;
}

/* -----------------------------------------------------------------------------
 *  USB_HasNewData — Шинэ өгөгдөл ирсэн эсэх (1 = бий, 0 = үгүй)
 * -----------------------------------------------------------------------------
 */
uint8_t USB_HasNewData(void)
{
    return usb_new_data_flag;
}

/* -----------------------------------------------------------------------------
 *  USB_ClearFlag — flag-ыг цэвэрлэх (өгөгдлийг уншиж дууссаны дараа дуудна)
 * -----------------------------------------------------------------------------
 */
void USB_ClearFlag(void)
{
    usb_new_data_flag = 0;
}

/* -----------------------------------------------------------------------------
 *  USB_Show_OLED — USB утгыг OLED дэлгэц дээр төлөвийн текстээр харуулах
 * -----------------------------------------------------------------------------
 */
void USB_Show_OLED(void)
{
    colorFill(Black);

    setCursor(10, 10);
    printStr("USB: %d", usb_received_value);

    setCursor(10, 40);

    // Утганд тохирох текст харуулах
    if (usb_received_value == '0' || usb_received_value == 0)
    {
        printStr("Status: hooson");
    }
    else if (usb_received_value == '1' || usb_received_value == 1)
    {
        printStr("Status: fake");
    }
    else if (usb_received_value == '2' || usb_received_value == 2)
    {
        printStr("Status: real");
    }
    else
    {
        printStr("Status: ???");
    }

    setScreen();
}
