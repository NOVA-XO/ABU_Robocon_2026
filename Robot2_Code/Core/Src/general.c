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
#define RACK_NEAR_ZONE 120    // зорилтод ойрын бүс (count) — энд хүчийг зөөлрүүлнэ
#define RACK_NEAR_PWM  120    // ойрын бүсэд hold_pwm-ээс дээш нэмэх дээд засвар
/* ---- Integral (таталцлын үлдэгдлийг нөхөх нарийн засвар) ------------------
 *  hold_pwm нь таталцлын ҮНДСЭН хүчийг өгнө; integral зөвхөн ҮЛДЭГДЛИЙГ трим хийнэ.
 *  RACK_I_BAND — хураах бүс. Өргөн байвал ойртох замдаа хэт их хурааж ХАНАНА.
 *  RACK_I_DEAD — encoder шумаас integral мөлхөхөөс сэргийлэх үхмэл бүс (count).
 */
#define RACK_I_BAND     80    // үүнээс ХОЛ бол integral-ыг тэглэнэ (windup-аас сэргийлнэ)
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

/* ---- Homing (доод limit switch хүртэл буулгах) --------------------------- */
#define HOME_PWM         150   // доошлуулах хүч (ЗӨӨЛӨН — таталцал бас тусалдаг)
#define HOME_TIMEOUT_MS 6000  // энэ хугацаанд switch дарагдахгүй бол зогсоно
#define HOME_CONFIRM      3   // зогссоны дараа баталгаажуулах уншилтын тоо
#define HOME_CONFIRM_MS   2   // баталгаажуулах уншилтын хоорондох хугацаа (мс)

Rack_t frontRack = {
    .motor = 6, .enc = 1, .home_sw = RACK_SW_S2,   // ХЭМЖСЭН: M6 / E1 / S2
    .pos_min = 0, .pos_max = 1950,   // ДЭЭД хязгаар — Rack_SetTarget зорилтыг үүн рүү хайчилна
    // ki нь УДААН байх ёстой: integral += error (5мс тутам) тул 0.08 нь ~160 PWM/сек
    //   өсгөж ХАНАДАГ байв → +14 count байнгын хазайлт. 0.012 нь ~24 PWM/сек.
    // i_max: дээд хувь нэмэр = 0.012 x 12500 = 150 PWM (ханахгүй, трим хийх орон зайтай).
    .kp = 2.0f, .ki = 0.012f, .kd = 9.0f, .i_max = 12500.0f,
    // hold_pwm — ЖИНХЭНЭ тэнцвэр (integral тайлагдсаны дараа 902 ба 1349 дээр хоёуланд
    //   нь PWM≈70 дээр тогтсон). 135 нь ХЭТ ӨНДӨР байсан: алдаа 0 болсон ч дээш
    //   түлхсээр байж ракыг зорилтоосоо давуулж үрэлтэнд гацаадаг байв.
    .pwm_up = 600, .pwm_down = 250, .hold_pwm = 70,
    .min_pwm = 80, .tolerance = 15,
    .target = 0, .active = 0,
    .prev_err = 0.0f, .integral = 0.0f, .d_filt = 0.0f, .last_ms = 0, .holding = 0,
    .home_prev = 0
};

Rack_t backRack = {
    .motor = 5, .enc = 0, .home_sw = RACK_SW_S1,   // ХЭМЖСЭН: M5 / E0 / S1
    .pos_min = 0, .pos_max = 1950,   // ДЭЭД хязгаар — Rack_SetTarget зорилтыг үүн рүү хайчилна
    // frontRack-тай ижил зарчим: удаан, ханахгүй integral (0.012 x 12500 = 150 PWM дээд)
    .kp = 3.0f, .ki = 0.012f, .kd = 10.0f, .i_max = 12500.0f,
    // hold_pwm — back rack-ийн үрэлтийн бүс 10..170 PWM (телеметрээр батлагдсан) тул
    //   түүний ДУНДАЖ. 175 нь бүсээс ДЭЭГҮҮР байсан → байнга дээш түлхэж гацаадаг байв.
    .pwm_up = 650, .pwm_down = 250, .hold_pwm = 90,
    .min_pwm = 80, .tolerance = 15,
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

        // ---- ЗӨӨЛӨН ГАЗАРДАЛТ: ЗӨВХӨН 0 (доод тулгуур) руу буух үед ----
        //   Хол байхад pwm_down-оор ХУРДАН, limit switch рүү ойртох тусам
        //   ШУГАМААР удааширч RACK_LAND_MIN_PWM хүртэл буурна → switch дээр
        //   зөөлөн газардана (шаагиж мөргөхгүй).
        //     aerr = RACK_LAND_ZONE  →  бүрэн pwm_down
        //     aerr = 0               →  RACK_LAND_MIN_PWM
        uint8_t landing = (!raised && error < 0.0f);
        if (landing && aerr < (float)RACK_LAND_ZONE) {
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
 *  RACK PRESET — L1/L2/R1/R2 дармагц ХОЁУЛАА тогтсон ӨНДӨРТ шууд очно
 *
 *    L1 → 0     (доод / home)
 *    L2 → 900
 *    R1 → 1350
 *    R2 → 1950  (дээд; pos_max-тай тэнцүү)
 *
 *  Товч дармагц front (M6) ба back (M5) рак ХОЁУЛАА тухайн түвшин рүү явна
 *  (алхам алхмаар биш — ШУУД). Rack_SetTarget зорилтыг тавьж, TIM7 ISR доторх
 *  Rack_Service байрлалыг тасралтгүй БАРИНА (таталцлыг PID нөхнө).
 *
 *  ⚠ Урьдчилан Rack_SetHome() ажиллуулж encoder-ийн 0 цэгийг доод limit switch
 *    дээр тогтоосон байх ЁСТОЙ. Эс бөгөөс түвшнүүд асаах үеийн санамсаргүй
 *    байрлалаас тоологдоно.
 * =============================================================================
 */

/* control_data[3] индекс:  0 = L1 ,  1 = R1 ,  2 = L2 ,  3 = R2
 * Түвшнийг өөрчлөх бол ЗӨВХӨН энэ хүснэгтийг засна. */
#define RACK_BTN_COUNT  4
static const int RACK_BTN_LEVEL[RACK_BTN_COUNT] = {
    0,      /* [3][0] L1 → 0    (доод / home) */
    1350,   /* [3][1] R1 → 1350               */
    900,    /* [3][2] L2 → 900                */
    1950    /* [3][3] R2 → 1950 (дээд)        */
};

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

void Rack_Preset_Control(void) {
    static Btn_t btn[RACK_BTN_COUNT] = {0};
    static int   target = 0;           // одоогийн зорилт (homing-ийн дараа 0)

    /* Аль нэг товч дармагц ХОЁУЛАНГ нь тэр түвшин рүү явуулна */
    for (int i = 0; i < RACK_BTN_COUNT; i++) {
        if (btn_rising(&btn[i], (uint8_t)control_data[3][i])) {
            target = RACK_BTN_LEVEL[i];
            break;                     // хэд зэрэг дарагдвал эхнийх нь ялна
        }
    }

    /* Зорилтыг тавина — байрлалыг Rack_Service (TIM7 ISR) тасралтгүй барина */
    Rack_SetTarget(&backRack,  target);
    Rack_SetTarget(&frontRack, target);

    /* --- OLED (100мс тутам): зорилт + хоёр ракийн бодит байрлал --- */
    static uint32_t t = 0;
    if (HAL_GetTick() - t >= 100) {
        t = HAL_GetTick();
        colorFill(Black);
        setCursor(2, 2);
        printStr("TARGET:%d", target);
        setCursor(2, 22);
        printStr("B at:%d", counter[backRack.enc]);
        setCursor(2, 42);
        printStr("F at:%d", counter[frontRack.enc]);
        setScreen();
    }
}


/* =============================================================================
 *  СЕРВО PRESET — товч дармагц тогтсон ӨНЦӨГТ шууд очно (1 серво, htim1 CH1)
 *
 *    Cross    → 0       (доод / хаалттай)
 *    Square   → 8000
 *    Triangle → 16000
 *    Circle   → 24000   (дээд / нээлттэй)
 *
 *  Өнцөг өөрчлөх бол ЗӨВХӨН SERVO_PRESET хүснэгтийг засна.
 *
 *  ⚠ controlServo нь 0..SERVO_ANGLE_MAX-аас гарсан утга авбал errorFunction()
 *    руу орж роботыг БҮРЭН ГАЦААНА (buzzer-ийн хязгааргүй гогцоо). Тиймээс
 *    дуудахын өмнө ЗААВАЛ хаана — хүснэгтийг буруу засвал ч аюулгүй байлгана.
 * =============================================================================
 */
#define SERVO_ANGLE_MAX     24000   // controlServo-ийн зөвшөөрөгдөх дээд утга
#define SERVO_PRESET_COUNT      4

/* control_data[1] индекс:  0 = Cross ,  1 = Square ,  2 = Triangle ,  3 = Circle */
static const int SERVO_PRESET[SERVO_PRESET_COUNT] = {
    0,       /* [1][0] Cross    → 0     (хаалттай) */
    8000,    /* [1][1] Square   → 8000             */
    16000,   /* [1][2] Triangle → 16000            */
    24000    /* [1][3] Circle   → 24000 (нээлттэй) */
};

void Servo_Preset_Control(void) {
    static Btn_t  btn[SERVO_PRESET_COUNT] = {0};
    static int    angle  = 0;      // одоогийн өнцөг (эхлэхдээ 0)
    static uint8_t inited = 0;

    if (!inited) {                 // асаахад мэдэгдэж буй байрлалд тавина
        controlServo(true, angle);
        inited = 1;
    }

    for (int i = 0; i < SERVO_PRESET_COUNT; i++) {
        if (btn_rising(&btn[i], (uint8_t)control_data[1][i])) {
            int a = SERVO_PRESET[i];

            if (a < 0)               a = 0;                 // ХАМГААЛАЛТ: errorFunction-аас
            if (a > SERVO_ANGLE_MAX) a = SERVO_ANGLE_MAX;   //   сэргийлнэ (робот гацахгүй)

            angle = a;
            controlServo(true, angle);   // htim1 CH1
            break;                       // хэд зэрэг дарагдвал эхнийх нь ялна
        }
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
#define TURN_KP        5.0f
#define TURN_KI        0.0f
#define TURN_KD        0.0f
#define TURN_I_MAX     100.0f
#define TURN_MAX_PWM   450
#define TURN_MIN_PWM   50
#define TURN_TOL       2.0f      // ±° хүлцэл
#define TURN_HOLD_MS   250       // tol дотор энэ хугацаанд барих → дууссан

uint8_t Gyro_TurnAngle(float angle) {
    static uint8_t  started   = 0;
    static float    target    = 0.0f;
    static float    prev_err  = 0.0f;
    static float    integral  = 0.0f;
    static uint32_t in_zone_t = 0;

    // ---- Эхний дуудалт: одоогийн чигийг 0 болгож, зорилт = angle ----
    if (!started) {
        // Сенсорыг тогтворжуулах
        for (int i = 0; i < 5; i++) {
            LPMS_Read();
            HAL_Delay(5);
        }
        Gyro_ZeroYaw();      // одоогийн чиг = 0°
        LPMS_Read();         // yaw_rel-ийн шинэ суурийг тогтоох

        target    = angle;   // тэг цэгээс angle зэрэг эргэнэ (харьцангуй)
        prev_err  = 0.0f;
        integral  = 0.0f;
        in_zone_t = 0;
        started   = 1;
        timer     = 0;
        return 0;
    }

    LPMS_Read();
    float current = yaw_rel;   // тасралтгүй (-360..+360), зөвхөн энэ эргэлтэд харьцангуй

    // ---- Алдаа (wrap хэрэггүй — yaw_rel тасралтгүй) ----
    float error = target - current;

    // ---- Дууссан эсэх (tol дотор HOLD_MS барих) ----
    if (error < TURN_TOL && error > -TURN_TOL) {
        if (in_zone_t == 0) in_zone_t = HAL_GetTick();
        if (HAL_GetTick() - in_zone_t >= TURN_HOLD_MS) {
            brake();
            started = 0;          // дараагийн эргэлтэд дахин init
            return 1;
        }
        brake();
        return 0;
    } else {
        in_zone_t = 0;
    }

    // ---- PID тогтмол интервалаар ----
    if (timer > 5) {
        timer = 0;

        // Integral — зөвхөн ойрхон (anti-windup)
        if (error < 30.0f && error > -30.0f) {
            integral += error;
            if (integral >  TURN_I_MAX) integral =  TURN_I_MAX;
            if (integral < -TURN_I_MAX) integral = -TURN_I_MAX;
        } else {
            integral = 0.0f;
        }

        float derivative = error - prev_err;
        prev_err = error;

        float output = (TURN_KP * error) + (TURN_KI * integral) + (TURN_KD * derivative);

        if (output >  TURN_MAX_PWM) output =  TURN_MAX_PWM;
        if (output < -TURN_MAX_PWM) output = -TURN_MAX_PWM;

        int pwm = (int)output;

        // Min PWM — эргэхэд хангалттай хүч (зөвхөн tol-ийн гадна)
        if (pwm > 0 && pwm <  TURN_MIN_PWM) pwm =  TURN_MIN_PWM;
        if (pwm < 0 && pwm > -TURN_MIN_PWM) pwm = -TURN_MIN_PWM;

        // ЗҮҮН (1,3) vs БАРУУН (2,4) — runner-ийн эргэлттэй ИЖИЛ бүлэглэл
        motor_control(1, -pwm);   // зүүн
        motor_control(3, -pwm);   // зүүн
        motor_control(2,  pwm);   // баруун
        motor_control(4,  pwm);   // баруун
    }

    return 0;
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
