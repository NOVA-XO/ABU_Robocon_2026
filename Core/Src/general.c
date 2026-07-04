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
#define RACK_DT_MS    5       // PID шинэчлэх интервал (мс)
#define JOG_PWM     150       // гараар гүйлгэх хүч

Rack_t frontRack = {
    .motor = 5, .enc = 0,
    .pos_min = 0, .pos_max = 1800,
    .kp = 2.0f, .ki = 0.3f, .kd = 5.0f, .i_max = 1000.0f,
    .pwm_up = 600, .pwm_down = 250, .hold_pwm = 100,   // дээш хурдан, доош хэвийн
    .min_pwm = 80, .tolerance = 10,
    .target = 0, .active = 0,
    .prev_err = 0.0f, .integral = 0.0f, .last_ms = 0
};

Rack_t backRack = {
    .motor = 6, .enc = 1,
    .pos_min = 0, .pos_max = 1800,
    .kp = 4.0f, .ki = 0.3f, .kd = 6.0f, .i_max = 1000.0f,
    .pwm_up = 650, .pwm_down = 250, .hold_pwm = 130,
    .min_pwm = 80, .tolerance = 10,
    .target = 0, .active = 0,
    .prev_err = 0.0f, .integral = 0.0f, .last_ms = 0
};

/* -----------------------------------------------------------------------------
 *  Rack_SetHome — Одоогийн байрлалыг 0 цэг болгох (үзүүрт аваачаад дуудна)
 * -----------------------------------------------------------------------------
 */
void Rack_SetHome(Rack_t *r) {
    counter[r->enc] = 0;
    r->prev_err = 0.0f;
    r->integral = 0.0f;
}

/* -----------------------------------------------------------------------------
 *  Rack_Reset — PID дотоод төлвийг л цэвэрлэх (counter-ийг хөндөхгүй)
 * -----------------------------------------------------------------------------
 */
void Rack_Reset(Rack_t *r) {
    r->prev_err = 0.0f;
    r->integral = 0.0f;
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

    int   pos   = counter[r->enc];
    float error = (float)(target - pos);

    // ---- Хүрсэн эсэх ----
    if (error < r->tolerance && error > -r->tolerance) {
        // Дээш өргөсөн байрлал бол жижиг hold-PWM-ээр БАРЬ;
        // доод тулгуур (0) дээр бол чөлөөт (coast) OK.
        if (target > r->pos_min + r->tolerance)
            motor_control(r->motor, r->hold_pwm);
        else
            motor_control(r->motor, 0);

        r->prev_err = 0.0f;
        return 1;
    }

    // ---- PID тогтмол интервалаар ----
    if (HAL_GetTick() - r->last_ms >= RACK_DT_MS) {
        r->last_ms = HAL_GetTick();

        // Conditional integration — зөвхөн target-т ОЙРХОН (windup-аас сэргийлнэ)
        if (error < 250.0f && error > -250.0f) {
            r->integral += error;
            if (r->integral >  r->i_max) r->integral =  r->i_max;
            if (r->integral < -r->i_max) r->integral = -r->i_max;
        } else {
            r->integral = 0.0f;
        }

        float derivative = error - r->prev_err;
        r->prev_err = error;

        float output = (r->kp * error) + (r->ki * r->integral) + (r->kd * derivative);

        int lim = (error > 0.0f) ? r->pwm_up : r->pwm_down;
        if (output >  lim) output =  lim;
        if (output < -lim) output = -lim;

        int pwm = (int)output;

        if (error > (float)r->tolerance || error < -(float)r->tolerance) {
            if (pwm > 0 && pwm <  r->min_pwm) pwm =  r->min_pwm;
            if (pwm < 0 && pwm > -r->min_pwm) pwm = -r->min_pwm;
        }

        motor_control(r->motor, pwm);
    }

    return 0;
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
