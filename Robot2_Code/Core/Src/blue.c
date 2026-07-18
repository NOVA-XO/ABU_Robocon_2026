/* =============================================================================
 *  blue.c — Цэнхэр (blue) талын автомат дараалал
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "blue.h"

extern int counter[4];              // encoder тоолуурууд
extern ADC_HandleTypeDef hadc1;

/* ---- Тохиргоо ------------------------------------------------------------ */
#define WB_DRIVE_PWM   (-400)   // хурдан урагш = СӨРӨГ (жолооны конвенц)
#define WB_DRIVE_SLOW  (-100)   // намуухан урагш (val8 руу дөхөх — хэт хурдан байв)
#define WB_RACK_UP      1900    // ракийн дээд байрлал. pos_max(1950)-аас ДООШ:
                                //   1950-д барихад хүрч чадалгүй RACK TIMEOUT болдог.
#define WB_SERVO_DOWN      0    // рак өргөгдсөний дараах сервоны өнцөг (буулгах)
#define WB_SERVO_UP      180    // серво буцаах өнцөг
#define WB_SERVO_MS      500    // серво буух хугацаа (хүлээнэ)
#define WB_VAL8_HOLD_MS  200    // val8 ийм хугацаанд ТАСРАЛТГҮЙ 0 байвал "шахагдсан"
#define WB_VAL3_WAIT_MS 1000    // val3 мэдэрсний дараа хүлээх хугацаа (1 секунд)
#define WB_END_DRIVE_MS  500    // эцсийн ухрах хугацаа (500мс)
#define WB_END_PWM      (+400)   // эцсийн явалт: УХРАХ (урагш=сөрөг тул эерэг=ухрах)
#define WB_SOLENOID        1    // val3==0 үед асаах соленоид
#define WB_RACK_FULL    1950    // ухрах+эргэх үед хоёр ракийг ДҮҮРЭН дээш (pos_max)
#define WB_TURN_DEG    180.0f   // эцэст эргэх өнцөг

/* ---- val3 align (val3==1 бол зүүн тийш зэрэгцүүлэх) ---------------------- */
#define WB_ALIGN_STRAFE  250    // зүүн тийш хажуулах хурд
#define WB_ALIGN_FRONT    50    // УРД хоёр дугуй (1,2)-ийн хажуулах суурь нэмэлт PWM
#define WB_ALIGN_FWD    (-120)  // урагш bias — val8-ыг шахсаар байлгах (сөрөг=урагш)
#define WB_ALIGN_KP      8.0f   // gyro чиг барих коэффициент (Drive_Straight-тэй ижил маяг)
#define WB_ALIGN_WMAX    150    // gyro залруулгын дээд PWM

/* ---- Төлөвүүд ------------------------------------------------------------ */
typedef enum {
    WB_S0_DRIVE = 0,    // val5 == 0 болтол gyro-гоор шулуун урагш
    WB_S1_RACK_UP,      // хоёр рак ХАМТ 1900 руу
    WB_S3_PUSH_VAL8,    // намуухан урагш + серво ЗЭРЭГ буулгах, val8 ТАСРАЛТГҮЙ 0 болтол
    WB_S4_ALIGN_VAL3,   // val3==1 бол зүүн тийш (gyro+val8), val3==0 бол → 1сек хүлээх
    WB_S4B_WAIT,        // val3 мэдэрсэн → 1 сек хүлээгээд → соленоид1 + серво 180
    WB_S5_END_DRIVE,    // 500мс УХРАХ + ракуудыг 1950 руу
    WB_S6_TURN,         // 180° эргэх (ракууд 1950-д)
    WB_DONE
} wb_state_t;

static wb_state_t wb_st       = WB_S0_DRIVE;
static uint8_t    wb_anchored = 0;
static uint8_t    wb_srv_init = 0;   // S3-д сервог нэг л удаа буулгах
static uint32_t   wb_wait_t0  = 0;   // val3-ийн дараах 1сек хүлээлтийн эхлэл
static uint32_t   wb_v8_t0    = 0;   // val8 тасралтгүй 0 болсон эхлэлийн агшин
static uint32_t   wb_end_t0   = 0;   // эцсийн ухрах явалтын эхлэл
static uint8_t    wb_s5_init  = 0;   // S5-д ракуудыг 1950 руу нэг л удаа командлах
static uint8_t    wb_s6_init  = 0;   // S6-д эргэлтийг нэг л удаа reset хийх

/* -----------------------------------------------------------------------------
 *  wb_move — ХАЖУУ + УРАГШ + gyro ЧИГ БАРИХ (mecanum, closed-loop чиг)
 *
 *    vx  : хажуу.  vx < 0 → ЗҮҮН,  vx > 0 → баруун  (wb_strafe-ийн конвенц)
 *    fwd : урагш bias.  сөрөг = урагш (Drive_Straight-ийн конвенц)
 *
 *  gyro залруулга (c): Drive_Straight-тэй ИЖИЛ маяг — зүүн мотор(1,3) −c,
 *  баруун мотор(2,4) +c. Тэр функц шулуун явахад туршигдсан тул тэмдэг зөв.
 *  Ингэснээр зэрэгцэх зуур робот чигээ (anchor) барина, эргэлдэхгүй.
 * -----------------------------------------------------------------------------
 */
static void wb_move(int vx, int fwd)
{
    LPMS_Read();                                  // gyro шинэ өгөгдөл
    float off = Get_Yaw_Offset_From_Anchor();     // anchor-аас хазайлт (°)
    int   c   = (int)(WB_ALIGN_KP * off);
    if (c >  WB_ALIGN_WMAX) c =  WB_ALIGN_WMAX;
    if (c < -WB_ALIGN_WMAX) c = -WB_ALIGN_WMAX;

    /* Урд хоёр дугуй (1,2) сул тул хажуулах хүчийг нь суурь WB_ALIGN_FRONT-оор
       нэмнэ — хажуулах ЧИГЛЭЛД (vx-ийн тэмдгээр). Хойд (3,4) хэвээр.        */
    int vxf = vx;
    if      (vxf > 0) vxf += WB_ALIGN_FRONT;
    else if (vxf < 0) vxf -= WB_ALIGN_FRONT;

    motor_control(1, -vxf + fwd - c);   // Урд-Зүүн  (+ суурь хүч)
    motor_control(2,  vxf + fwd + c);   // Урд-Баруун (+ суурь хүч)
    motor_control(3,  vx  + fwd - c);   // Хойд-Зүүн
    motor_control(4, -vx  + fwd + c);   // Хойд-Баруун
}

/* -----------------------------------------------------------------------------
 *  wb_show — OLED (100мс тутам).  ⚠ Хурдыг хязгаарлахгүй бол ~25мс блоклож
 *  LPMS-ийн DMA буфер халина (sequence.c-ийн Seq_Show-тэй ижил шалтгаан).
 *    S = төлөв,  v5 = сенсор,  off = anchor-аас хазайлт (°),  F/B = рак,
 *    d = хоёр ракийн ЗӨРҮҮ (RACK_SYNC_MAX = 450 хүрвэл алдаа)
 *
 *  Алдааны мэдээллийг ЭНД харуулахгүй — Robot_Error өөрийн дэлгэцтэй бөгөөд
 *  түүнийг дуудсаны дараа энэ функц хэзээ ч ажиллахгүй.
 * -----------------------------------------------------------------------------
 */
static void wb_show(void)
{
    static uint32_t t = 0;
    if (HAL_GetTick() - t < 100) return;
    t = HAL_GetTick();

    int fp = counter[frontRack.enc];
    int bp = counter[backRack.enc];
    int d  = fp - bp;  if (d < 0) d = -d;

    colorFill(Black);
    setCursor(2, 2);
    printStr("BLUE S:%d", (int)wb_st);
    setCursor(2, 18);
    printStr("v5:%d v8:%d v3:%d", (int)val5, (int)val8, (int)val3);
    setCursor(2, 34);
    printStr("F:%d B:%d", fp, bp);
    setCursor(2, 50);
    printStr("d:%d", d);
    setScreen();
}

/* -----------------------------------------------------------------------------
 *  weapon_blue_reset — эхнээс нь дахин ажиллуулах
 * -----------------------------------------------------------------------------
 */
void weapon_blue_reset(void)
{
    wb_st       = WB_S0_DRIVE;
    wb_anchored = 0;
    wb_srv_init = 0;
    wb_wait_t0  = 0;
    wb_v8_t0    = 0;
    wb_end_t0   = 0;
    wb_s5_init  = 0;
    wb_s6_init  = 0;
}

/* -----------------------------------------------------------------------------
 *  weapon_blue — цэнхэр талбарын дараалал (non-blocking state machine)
 *
 *    S0: val5 == 0 болтол Drive_Straight-ээр шулуун урагш
 *    S1: хоёр рак ХАМТ 1900 руу (Rack_GoTo_Sync)
 *    S3: НАМУУХАН урагш + сервог ЗЭРЭГ буулгах, val8 ТАСРАЛТГҮЙ 0 болтол
 *    S4: val3==1 бол → val3==0 болтол ЗҮҮН тийш (gyro+val8).  val3==0 бол → S4B.
 *    S4B: val3 мэдэрсэн → 1 сек хүлээгээд → соленоид1 ON + серво 180°
 *    S5: 500мс УХРАХ + хоёр ракыг 1950 руу (дүүрэн дээш)
 *    S6: 180° эргэх (ракууд 1950-д) → дуусна
 *
 *  Дуусвал 1, эс бөгөөс 0 буцаана. Loop-д давтан дуудна.
 * -----------------------------------------------------------------------------
 */
uint8_t weapon_blue(void)
{
    /* ---- РАКИЙН АЛДАА — БҮХ төлөвт шалгана ---------------------------------
     *  Rack_Service нь TIM7 ISR-д ажилладаг тул рак зөвхөн S1-д биш, аль ч
     *  төлөвт гацаж/зөрж болно (ж: 1950 дээр барьж байхад front сунжирвал).
     *  Rack_Fault нь түгждэггүй, тухайн агшинд бодогддог — рак зорилтдоо
     *  байхад 0 буцаана, тиймээс S0-д (рак 0 дээр зогсож байхад) худал
     *  эерэг өгөхгүй.  ⚠ Robot_Error нь БУЦАХГҮЙ (аюулгүй төлөвт гацна).   */
    uint8_t f = Rack_Fault();
    if (f) Robot_Error(f == 1 ? "RACK TIMEOUT" : "RACK SYNC");

    wb_show();

    switch (wb_st) {

    /* -------- S0: val5 == 0 болтол шулуун урагш -------- */
    case WB_S0_DRIVE:
        /* Эхний дуудалтад одоогийн чигийг anchor болгоно — Drive_Straight нь
           тэр anchor-аас хазайхыг залруулна. Anchor тавихгүй бол хуучин
           чиг рүү залруулж хажуу тийш явна.                                */
        if (!wb_anchored) {
            Set_Yaw_Anchor();
            wb_anchored = 1;
        }

        if (val5 == 0) {          // сенсор идэвхжив → зогсоод дараагийн алхам
            brake();
            wb_st = WB_S1_RACK_UP;
            break;
        }

        Drive_Straight(WB_DRIVE_PWM);   // дотроо LPMS_Read() дуудна
        break;

    /* -------- S1: хоёр рак ХАМТ 1900 руу -------- */
    case WB_S1_RACK_UP:
        LPMS_Read();              // жолоо зогссон ч DMA буферээ хоослох
        /* Rack_GoTo_Sync — leader-follower: түрүүлсэн рак нөгөөгөө хүлээнэ.
           Тусад нь Rack_SetTarget хийвэл хүнд front хоцорч Rack_Fault=SYNC. */
        if (Rack_GoTo_Sync(WB_RACK_UP)) {
            wb_st = WB_S3_PUSH_VAL8;   // серво буух хүлээлт БАЙХГҮЙ — S3-д зэрэг буулгана
        }
        break;

    /* -------- S3: НАМУУХАН урагш + серво ЗЭРЭГ буулгах, val8 0 болтол -------- */
    case WB_S3_PUSH_VAL8:
        /* Сервог зогсолгүй, урагш явахтайгаа ЗЭРЭГ буулгана (нэг л удаа команд).
           Anchor-ыг ДАХИН тавихгүй — S0-д тогтоосон нь талбарын "урагш" чиг.
           val8 чичиргээнд 0/1 хэлбэлзэж болзошгүй тул WB_VAL8_HOLD_MS хугацаанд
           ТАСРАЛТГҮЙ 0 байж байж "шахагдсан" гэнэ.                            */
        if (!wb_srv_init) {
            Servo_SetDeg(WB_SERVO_DOWN);   // урагш явж байхдаа серво буулгах
            wb_srv_init = 1;
        }

        if (val8 == 0) {
            if (wb_v8_t0 == 0) wb_v8_t0 = HAL_GetTick();   // 0 болсон эхлэл
            if (HAL_GetTick() - wb_v8_t0 >= WB_VAL8_HOLD_MS) {
                wb_st = WB_S4_ALIGN_VAL3;   // жолоо ЗОГСООХГҮЙ — S4 үргэлжлүүлнэ
                break;
            }
        } else {
            wb_v8_t0 = 0;                  // 0-оос гарвал тоолуурыг тэглэнэ
        }

        Drive_Straight(WB_DRIVE_SLOW);    // намуухан — дотроо LPMS_Read() дуудна
        break;

    /* -------- S4: val3 align -------- */
    case WB_S4_ALIGN_VAL3:
        /* val3==0 (мэдэрсэн) бол → 1сек хүлээх төлөв рүү.  val3==1 бол → val3==0
           болтол ЗҮҮН тийш зэрэгцэнэ (gyro чиг барьж, val8-ыг шахсаар).       */
        if (val3 == 0) {
            wb_wait_t0 = HAL_GetTick();
            wb_st      = WB_S4B_WAIT;
            break;
        }

        /* vx сөрөг = ЗҮҮН,  fwd сөрөг = урагш (val8 шахалт) */
        wb_move(-WB_ALIGN_STRAFE, WB_ALIGN_FWD);
        break;

    /* -------- S4B: val3 мэдэрсэн → 1 сек хүлээгээд соленоид1 + серво180 -------- */
    case WB_S4B_WAIT:
        /* Байрлалаа алдахгүйн тулд намуухан урагш ШАХСААР 1 сек хүлээнэ. */
        if (HAL_GetTick() - wb_wait_t0 >= WB_VAL3_WAIT_MS) {
            brake();
            controlSolenoid(WB_SOLENOID, true);   // соленоид 1 ON
            Servo_SetDeg(WB_SERVO_UP);            // серво → 180°
            wb_end_t0 = HAL_GetTick();
            wb_st     = WB_S5_END_DRIVE;
            break;
        }

        Drive_Straight(WB_DRIVE_SLOW);    // 1сек хүлээх зуур байрлалаа барина
        break;

    /* -------- S5: 1 секунд шулуун УХРАХ -------- */
    case WB_S5_END_DRIVE:
        /* Энэ фазд ракуудыг ДҮҮРЭН дээш (1950). 1900-аас ~50 count дээшлэх
           жижиг хөдөлгөөн тул sync fault-гүй; Rack_Service (ISR) барина.    */
        if (!wb_s5_init) {
            Rack_SetTarget(&frontRack, WB_RACK_FULL);
            Rack_SetTarget(&backRack,  WB_RACK_FULL);
            wb_s5_init = 1;
        }

        if (HAL_GetTick() - wb_end_t0 >= WB_END_DRIVE_MS) {
            brake();
            wb_st = WB_S6_TURN;
            break;
        }
        Drive_Straight(WB_END_PWM);       // 500мс — УХРАХ (gyro чиг барина)
        break;

    /* -------- S6: 180° эргэх (ракууд 1950-д) -------- */
    case WB_S6_TURN:
        /* Ракууд 1950-д хэвээр (ISR барьсаар). Эргэлт нь мотор 1-4-ийг л
           ашиглана — рак (мотор 5,6)-тай зөрчилдөхгүй.                       */
        if (!wb_s6_init) {
            Gyro_TurnReset();   // өмнөх эргэлтийн үлдэгдэл төлвийг цэвэрлэх
            wb_s6_init = 1;
        }
        if (Gyro_TurnAngle(WB_TURN_DEG)) {   // дуусвал 1 (дотроо LPMS_Read)
            brake();
            wb_st = WB_DONE;
        }
        break;

    /* -------- Дууссан -------- */
    case WB_DONE:
        LPMS_Read();              // DMA буферээ хоослох (хоцрохгүй)
        return 1;
    }

    return 0;
}