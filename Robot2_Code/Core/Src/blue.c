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
#define WB_DRIVE_PWM   (-400)   // урагш = СӨРӨГ (жолооны конвенц)
#define WB_RACK_UP      1950    // ракийн дээд байрлал (= pos_max)
#define WB_SERVO_DEG       0    // рак өргөгдсөний дараах сервоны өнцөг
#define WB_STRAFE_PWM    250    // хажуу тийш хурд
#define WB_STRAFE_MS     500    // хажуу тийш хугацаа

/*  ⚠ WB_STRAFE_SIGN — Vx-ийн тэмдэг: ЗҮҮН = сөрөг.
 *    Гарал үүсэл: runner-д цэвэр эргэлтэд (Vx=0,Vy=0) W>0 нь мотор1 сөрөг /
 *    мотор2 эерэг өгдөг — энэ нь Gyro_TurnAngle-ийн ЗҮҮН эргэлтийн ЭСРЭГ тал.
 *    ⇒ W>0 = баруун ⇒ стик баруун = эерэг ⇒ Vx>0 = баруун хажуулалт.
 *    Энэ бол хэмжилт биш ЛОГИК дүгнэлт — робот БАРУУН тийш явбал +1 болго.  */
#define WB_STRAFE_SIGN   (-1)

/* ---- Төлөвүүд ------------------------------------------------------------ */
typedef enum {
    WB_S0_DRIVE = 0,    // val5 == 0 болтол gyro-гоор шулуун урагш
    WB_S1_RACK_UP,      // хоёр рак ХАМТ 1950 руу
    WB_S2_SERVO_STRAFE, // серво 0° + ЗЭРЭГ зүүн тийш 500мс
    WB_S3_DRIVE2,       // val8 == 0 болтол дахин шулуун урагш
    WB_DONE
} wb_state_t;

static wb_state_t wb_st       = WB_S0_DRIVE;
static uint8_t    wb_anchored = 0;
static uint8_t    wb_s2_init  = 0;
static uint32_t   wb_s2_t0    = 0;

/* -----------------------------------------------------------------------------
 *  wb_strafe — ХАЖУУ тийш (mecanum).  runner-ийн inverse kinematics-ээс
 *  Vy = 0, W = 0 тавьсан тохиолдол:  fl=-Vx  fr=+Vx  rl=+Vx  rr=-Vx
 *    vx < 0 → зүүн,  vx > 0 → баруун
 * -----------------------------------------------------------------------------
 */
static void wb_strafe(int vx)
{
    motor_control(1, -vx *1.4);   // Урд-Зүүн
    motor_control(2,  vx *1.4);   // Урд-Баруун
    motor_control(3,  vx);   // Хойд-Зүүн
    motor_control(4, -vx);   // Хойд-Баруун
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
    printStr("v5:%d v8:%d o:%d", (int)val5, (int)val8,
             (int)Get_Yaw_Offset_From_Anchor());
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
    wb_s2_init  = 0;
}

/* -----------------------------------------------------------------------------
 *  weapon_blue — цэнхэр талбарын дараалал (non-blocking state machine)
 *
 *    S0: val5 == 0 болтол Drive_Straight-ээр шулуун урагш
 *    S1: хоёр рак ХАМТ 1950 руу (Rack_GoTo_Sync)
 *    S2: серво 0° + ЗЭРЭГ зүүн тийш 500мс
 *    S3: val8 == 0 болтол дахин шулуун урагш (S0-ийн anchor-аар)
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

    /* -------- S1: хоёр рак ХАМТ 1950 руу -------- */
    case WB_S1_RACK_UP:
        LPMS_Read();              // жолоо зогссон ч DMA буферээ хоослох
        /* Rack_GoTo_Sync — leader-follower: түрүүлсэн рак нөгөөгөө хүлээнэ.
           Тусад нь Rack_SetTarget хийвэл хүнд front хоцорч Rack_Fault=SYNC. */
        if (Rack_GoTo_Sync(WB_RACK_UP)) {
            wb_st = WB_S2_SERVO_STRAFE;
        }
        break;

    /* -------- S2: серво 0° + ЗЭРЭГ зүүн тийш 500мс -------- */
    case WB_S2_SERVO_STRAFE:
        LPMS_Read();

        /* Серво нэг удаа тавигдаад цааш өөрөө буунa (эргэх холбоогүй).
           Тиймээс "хүлээхийн" оронд хажуулалтыг ЗЭРЭГ гүйцэтгэнэ.       */
        if (!wb_s2_init) {
            Servo_SetDeg(WB_SERVO_DEG);
            wb_s2_t0   = HAL_GetTick();
            wb_s2_init = 1;
        }

        if (HAL_GetTick() - wb_s2_t0 >= WB_STRAFE_MS) {
            brake();
            wb_st = WB_S3_DRIVE2;
            break;
        }

        wb_strafe(WB_STRAFE_SIGN * WB_STRAFE_PWM);
        break;

    /* -------- S3: val8 == 0 болтол дахин шулуун урагш -------- */
    case WB_S3_DRIVE2:
        /* Anchor-ыг ДАХИН тавихгүй — S0-д тогтоосон нь талбарын "урагш" чиг.
           Хажуулалт бага зэрэг эргүүлсэн байвал Drive_Straight түүнийг эргүүлж
           залруулна. Энд дахин anchor тавибал тэр хазайлт ТОГТМОЛ болно.    */
        if (val8 == 0) {
            brake();
            wb_st = WB_DONE;
            break;
        }

        Drive_Straight(WB_DRIVE_PWM);   // дотроо LPMS_Read() дуудна
        break;

    /* -------- Дууссан -------- */
    case WB_DONE:
        LPMS_Read();              // DMA буферээ хоослох (хоцрохгүй)
        return 1;
    }

    return 0;
}