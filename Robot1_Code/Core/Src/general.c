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

/* ---- Homing (доод limit switch хүртэл буулгах) --------------------------- */
#define HOME_PWM         150   // доошлуулах хүч (ЗӨӨЛӨН — таталцал бас тусалдаг)
#define HOME_TIMEOUT_MS 6000  // энэ хугацаанд switch дарагдахгүй бол зогсоно
#define HOME_CONFIRM      3   // зогссоны дараа баталгаажуулах уншилтын тоо
#define HOME_CONFIRM_MS   2   // баталгаажуулах уншилтын хоорондох хугацаа (мс)

Rack_t frontRack = {
    .motor = 6, .enc = 1,
    .pos_min = 0, .pos_max = 1800,
    .kp = 2.0f, .ki = 0.3f, .kd = 5.0f, .i_max = 1000.0f,
    .pwm_up = 600, .pwm_down = 250, .hold_pwm = 100,   // дээш хурдан, доош хэвийн
    .min_pwm = 80, .tolerance = 10,
    .target = 0, .active = 0,
    .prev_err = 0.0f, .integral = 0.0f, .d_filt = 0.0f, .last_ms = 0, .holding = 0,
    .home_prev = 0
};

Rack_t backRack = {
    .motor = 5, .enc = 0,
    .pos_min = 0, .pos_max = 1800,
    .kp = 4.0f, .ki = 0.3f, .kd = 6.0f, .i_max = 1000.0f,
    .pwm_up = 650, .pwm_down = 250, .hold_pwm = 130,
    .min_pwm = 80, .tolerance = 10,
    .target = 0, .active = 0,
    .prev_err = 0.0f, .integral = 0.0f, .d_filt = 0.0f, .last_ms = 0, .holding = 0,
    .home_prev = 0
};

/* -----------------------------------------------------------------------------
 *  rack_at_home — Тухайн ракийн доод limit switch дарагдсан эсэх
 *    return: 1 = доод тулгуур дээр байна (switch дарагдсан, pull-up тул утга 0)
 *  ISR-safe: зөвхөн GPIO уншилт.
 *    enc 0 = FRONT (val2) ,  enc 1 = BACK (val1)   ← солигдсоныг санаарай
 * -----------------------------------------------------------------------------
 */
static uint8_t rack_at_home(Rack_t *r) {
    return (r->enc == 0) ? (uint8_t)FRONT_RACK_HOME : (uint8_t)BACK_RACK_HOME;
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

    // ---- Доод limit switch: ДАРАГДСАН ЭХНИЙ АГШИНД encoder-ийг 0 болгож дахин
    //      home хийнэ (encoder-ийн хуримтлагдсан алдааг арилгана).
    //      Тасралтгүй биш ЗӨВХӨН ИРМЭГ дээр тэглэнэ: switch гэмтэж байнга
    //      дарагдсан үед counter-ийг 0-д гацааж PID-г дээш зугтаахаас сэргийлнэ.
    uint8_t at_home = rack_at_home(r);
    if (at_home && !r->home_prev) {
        enc_zero(r->enc);
        r->integral = 0.0f;
        r->prev_err = 0.0f;
        r->d_filt   = 0.0f;
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

        // Integral (gravity comp): зорилтод ойрхон л хура (anti-windup)
        if (aerr < 250.0f) {
            r->integral += error;
            if (r->integral >  r->i_max) r->integral =  r->i_max;
            if (r->integral < -r->i_max) r->integral = -r->i_max;
        } else {
            r->integral = 0.0f;
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
        if (output >  lim) output =  lim;
        if (output < -lim) output = -lim;

        int pwm = (int)output;

        // min_pwm floor зөвхөн ТОМ зөрөөнд (хол хөдөлгөөн, статик үрэлт даван гарах).
        // Ойрын жижиг залруулгыг зөөлөн үлдээж хэлбэлзэхээс сэргийлнэ.
        if (aerr > (float)RACK_NEAR_ZONE) {
            if (pwm > 0 && pwm <  r->min_pwm) pwm =  r->min_pwm;
            if (pwm < 0 && pwm > -r->min_pwm) pwm = -r->min_pwm;
        }

        // ХАМГААЛАЛТ: доод limit switch дарагдсан бол ДООШ (сөрөг PWM) жолоодохгүй.
        // Механизмыг тулгуур руу шахаж мотор түгжихээс сэргийлнэ.
        if (at_home && pwm < 0) pwm = 0;

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
 *  OLED:
 *    M5/M6 = өгсөн PWM,   E0/E1 = counter[0]/counter[1]
 *    S1/S2 = val1/val2 limit switch   (1 = суларсан, 0 = ДАРАГДСАН)
 *
 *  АЮУЛГҮЙ БАЙДАЛ: rack PID (ISR-service) унтраалттай, PWM дээд тал нь
 *  JTEST_MAX_PWM. Чиглэл тодорхойгүй тул автомат хамгаалалт БАЙХГҮЙ —
 *  limit switch 0 болмогц стикээ суллаарай.
 * =============================================================================
 */
#define JTEST_MAX_PWM   250   // аюулгүйн дээд PWM (чиглэл олох хангалттай)
#define JTEST_DEADZONE   12   // стикийн үхмэл бүс

void Rack_Joystick_Test(void) {
    // Rack PID ISR-service унтраалттай эсэхийг баталгаажуулна (зөрчихгүй)
    frontRack.active = 0;
    backRack.active  = 0;

    // --- Стикээс PWM. Түүхий стик ДЭЭШ = СӨРӨГ тул эргүүлж "дээш = эерэг" болгоно ---
    int ly = control_data[0][1];   // зүүн стик Y
    int ry = control_data[0][3];   // баруун стик Y
    if (ly > -JTEST_DEADZONE && ly < JTEST_DEADZONE) ly = 0;
    if (ry > -JTEST_DEADZONE && ry < JTEST_DEADZONE) ry = 0;

    int pwm5 = (-ly) * JTEST_MAX_PWM / 100;
    int pwm6 = (-ry) * JTEST_MAX_PWM / 100;

    motor_control(5, pwm5);
    motor_control(6, pwm6);

    // --- Cross (✕): encoder-үүдийг тэглэх (нэг даралт = нэг удаа) ---
    static uint8_t cross_prev = 0;
    uint8_t cross = (uint8_t)control_data[1][0];
    if (cross && !cross_prev) {
        enc_zero(0);
        enc_zero(1);
    }
    cross_prev = cross;

    // --- OLED (100мс тутам) ---
    static uint32_t t = 0;
    if (HAL_GetTick() - t >= 100) {
        t = HAL_GetTick();
        colorFill(Black);
        setCursor(6, 2);
        printStr("M5:%d M6:%d", pwm5, pwm6);
        setCursor(6, 22);
        printStr("E0:%d E1:%d", counter[0], counter[1]);
        setCursor(6, 42);
        printStr("S1:%d S2:%d", (int)val1, (int)val2);
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
