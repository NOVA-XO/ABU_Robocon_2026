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
#define WB_DRIVE_PWM   (-700)   // хурдан урагш = СӨРӨГ.  −1000 бол дээд тагт хүрч
                                //   gyro залруулгад зай үлдэхгүй тул −900 (±100 зай).
#define WB_DRIVE_SLOW  (-100)   // намуухан урагш (val8 руу дөхөх — хэт хурдан байв)
#define WB_RACK_BASE_LOW  1400  // урагш явахдаа BACK рак ЭНД (FRONT нь +offset өндөр)
#define WB_FRONT_OFFSET     100  // FRONT ракыг BACK-аас ИЛҮҮ өндөр барих (тонгойхыг нөхнө)
#define WB_RACK_UP        1950  // val5 мэдэрч ЗОГССОНЫ дараа хоёр рак ЭНД дээшилнэ.
                                //   pos_max(2000)-аас доош тул барихад timeout болохгүй.
#define WB_SERVO_DOWN      0    // рак өргөгдсөний дараах сервоны өнцөг (буулгах)
#define WB_SERVO_UP      180    // серво буцаах өнцөг
#define WB_SERVO_MS      500    // серво буух хугацаа (хүлээнэ)
#define WB_VAL8_HOLD_MS  200    // val8 ийм хугацаанд ТАСРАЛТГҮЙ 0 байвал "шахагдсан"
#define WB_VAL3_WAIT_MS  500    // val3 мэдэрсний (0) дараа хүлээх хугацаа (500мс)
#define WB_SOL_WAIT_MS   500    // соленоид 1 асаасны дараа хүлээх хугацаа (500мс)
#define WB_END_DRIVE_MS  700    // эцсийн УХРАХ хугацаа (700мс)
#define WB_END_PWM      (+400)   // эцсийн явалт: УХРАХ (урагш=сөрөг тул эерэг=ухрах)
#define WB_SOLENOID        1    // val3==0 үед асаах соленоид
#define WB_TURN_DEG    180.0f   // эцэст эргэх өнцөг

/* ---- val3 align (val3==1 бол зүүн тийш зэрэгцүүлэх) ---------------------- */
#define WB_ALIGN_STRAFE  250    // зүүн тийш хажуулах хурд
#define WB_ALIGN_FRONT    100    // УРД хоёр дугуй (1,2)-ийн хажуулах суурь нэмэлт PWM
#define WB_ALIGN_FWD    (-120)  // урагш bias — val8-ыг шахсаар байлгах (сөрөг=урагш)
#define WB_ALIGN_KP      8.0f   // gyro чиг барих коэффициент (Drive_Straight-тэй ижил маяг)
#define WB_ALIGN_WMAX    150    // gyro залруулгын дээд PWM

/* ---- Төлөвүүд ------------------------------------------------------------ */
typedef enum {
    WB_S0_DRIVE = 0,    // val5 == 0 болтол gyro-гоор шулуун урагш
    WB_S1_RACK_UP,      // хоёр рак ХАМТ 1950 руу
    WB_S3_PUSH_VAL8,    // намуухан урагш + серво ЗЭРЭГ буулгах, val8 ТАСРАЛТГҮЙ 0 болтол
    WB_S4_ALIGN_VAL3,   // val3==1 бол зүүн тийш (gyro+val8), val3==0 бол → 1сек хүлээх
    WB_S4B_WAIT,        // val3 мэдэрсэн → 1 сек хүлээгээд → соленоид1 + серво 180
    WB_S4C_SOL_WAIT,    // соленоид1 асаасны дараа 500мс хүлээх
    WB_S5_END_DRIVE,    // 500мс УХРАХ (ракууд 1950-д)
    WB_S6_TURN,         // 180° эргэх (ракууд 1950-д)
    WB_DONE
} wb_state_t;

static wb_state_t wb_st       = WB_S0_DRIVE;
static uint8_t    wb_anchored = 0;
static uint8_t    wb_srv_init = 0;   // S3-д сервог нэг л удаа буулгах
static uint32_t   wb_wait_t0  = 0;   // val3-ийн дараах 1сек хүлээлтийн эхлэл
static uint32_t   wb_v8_t0    = 0;   // val8 тасралтгүй 0 болсон эхлэлийн агшин
static uint32_t   wb_end_t0   = 0;   // эцсийн ухрах явалтын эхлэл
static uint8_t    wb_s6_init  = 0;   // S6-д эргэлтийг нэг л удаа reset хийх
static int        wb_rack_base = WB_RACK_BASE_LOW;   // BACK зорилт (урагш 1400 → val5 1950)
static int        wb_front_off = WB_FRONT_OFFSET;    // FRONT-ийн нэмэлт өндөр (урагш 50 → val5 0)

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
    wb_s6_init  = 0;
    wb_rack_base = WB_RACK_BASE_LOW;
    wb_front_off = WB_FRONT_OFFSET;
}

/* -----------------------------------------------------------------------------
 *  weapon_blue — цэнхэр талбарын дараалал (non-blocking state machine)
 *
 *    S0: val5 == 0 болтол урагш + ракийг ЗЭРЭГ 1400 руу өргөх.  val5 → зогсоод
 *        рак 1950 руу дээшлүүлнэ.
 *    S1: рак 1950-д хүрч тогтмогц → S3.
 *    S3: НАМУУХАН урагш + серво 0 буулгах, val8 болтол → S4.
 *    S4: val3==1 бол → val3==0 болтол ЗҮҮН тийш (gyro+val8).  val3==0 бол → S4B.
 *    S4B: val3==0 → 500мс хүлээгээд → соленоид1 ON.
 *    S4C: 500мс хүлээгээд → серво 180°.
 *    S5: 700мс УХРАХ (шулуун).
 *    S6: 180° эргэх → дуусна.
 *
 *  Дуусвал 1, эс бөгөөс 0 буцаана. Loop-д давтан дуудна.
 * -----------------------------------------------------------------------------
 */
uint8_t weapon_blue(void)
{
    /* ---- РАКИЙН АЛДАА — БҮХ төлөвт шалгана ---------------------------------
     *  Rack_Service нь TIM7 ISR-д ажилладаг тул рак аль ч төлөвт гацаж/зөрж
     *  болно. Rack_Fault нь түгждэггүй, тухайн агшинд бодогддог — рак зорилтдоо
     *  байхад 0 буцаана.  ⚠ Robot_Error нь БУЦАХГҮЙ (аюулгүй төлөвт гацна).   */
    uint8_t f = Rack_Fault();
    if (f) Robot_Error(f == 1 ? "RACK TIMEOUT" : "RACK SYNC");

    /* ---- РАКИЙГ ҮРГЭЛЖ ИДЭВХТЭЙ, СИНХРОН БАРИХ (band 20) -------------------
     *  Аль ч төлөвт рак идэвхгүй үлдвэл (рак_step дуудагдахгүй) cmd_target/move_t0
     *  хөлдөж, open-loop сажирч ХУДАЛ timeout болдог. Тиймээс ЭНД нэг л удаа,
     *  бүх төлөвт дуудна. Target нь эхнээс эцэс хүртэл 1950 (тогтмол).       
     *  (мотор 5,6 — жолоо/эргэлтийн мотор 1-4-тэй зөрчилдөхгүй.)               */
    /* СИНХРОН (leader-follower), гэхдээ FRONT нь BACK-аас wb_front_off-оор ӨНДӨР.
       Урагш явахад front тонгойхыг нөхнө; хоёр рак хамт хөдлөх тул тогтвортой.  */
    uint8_t rack_done = Rack_GoTo_Sync_Front(wb_rack_base, wb_front_off);

    wb_show();

    switch (wb_st) {

    /* -------- S0: val5 == 0 болтол шулуун урагш (рак дээд талд ЗЭРЭГ өгсөж байна) -------- */
    case WB_S0_DRIVE:
        /* Эхний дуудалтад одоогийн чигийг anchor болгоно — Drive_Straight нь
           тэр anchor-аас хазайхыг залруулна. Anchor тавихгүй бол хуучин
           чиг рүү залруулж хажуу тийш явна.                                */
        if (!wb_anchored) {
            Set_Yaw_Anchor();
            wb_anchored = 1;
        }

        if (val5 == 0) {          // val5 мэдэрлээ → урагш ЗОГСООД рак 1950 руу дээшлүүлнэ
            brake();
            wb_rack_base = WB_RACK_UP;  // одоо л хоёулаа 1950 руу
            wb_front_off = 0;           // зогсоод тонгойхгүй тул front offset хэрэггүй
            wb_st = WB_S1_RACK_UP;      // рак 1950-д тогтмогц S3 руу
            break;
        }

        Drive_Straight(WB_DRIVE_PWM);   // дотроо LPMS_Read() дуудна
        break;

    /* -------- S1: рак 1950-д хүрч ТОГТМОГЦ дараагийн алхам (робот зогссон) -------- */
    case WB_S1_RACK_UP:
        LPMS_Read();              // жолоо зогссон ч DMA буферээ хоослох
        if (rack_done) {          // дээд талын Rack_GoTo_Sync 1950-д хүрлээ
            wb_st = WB_S3_PUSH_VAL8;
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
                brake();
                wb_st = WB_S4_ALIGN_VAL3;   // val8 шахагдлаа → val3 align руу
                break;
            }
        } else {
            wb_v8_t0 = 0;                  // 0-оос гарвал тоолуурыг тэглэнэ
        }

        Drive_Straight(WB_DRIVE_SLOW);    // намуухан — дотроо LPMS_Read() дуудна
        break;

    /* -------- S4: val3 align -------- */
    case WB_S4_ALIGN_VAL3:
        /* val3==0 (мэдэрсэн) бол → 500мс хүлээх төлөв рүү.  val3==1 бол → val3==0
           болтол ЗҮҮН тийш зэрэгцэнэ (gyro чиг барьж, val8-ыг шахсаар).       */
        if (val3 == 0) {
            wb_wait_t0 = HAL_GetTick();
            wb_st      = WB_S4B_WAIT;
            break;
        }

        /* vx сөрөг = ЗҮҮН,  fwd сөрөг = урагш (val8 шахалт) */
        wb_move(-WB_ALIGN_STRAFE, WB_ALIGN_FWD);
        break;

    /* -------- S4B: val3==0 → 500мс хүлээгээд соленоид1 ON -------- */
    case WB_S4B_WAIT:
        /* Байрлалаа алдахгүйн тулд намуухан урагш ШАХСААР 500мс хүлээнэ. */
        if (HAL_GetTick() - wb_wait_t0 >= WB_VAL3_WAIT_MS) {
            brake();
            controlSolenoid(WB_SOLENOID, true);   // соленоид 1 ON
            wb_wait_t0  = HAL_GetTick();          // дараагийн 500мс хүлээлт
            wb_st       = WB_S4C_SOL_WAIT;
            break;
        }

        Drive_Straight(WB_DRIVE_SLOW);    // 500мс хүлээх зуур байрлалаа барина
        break;

    /* -------- S4C: соленоид1-ийн дараа 500мс хүлээгээд → серво 180 → ухрах -------- */
    case WB_S4C_SOL_WAIT:
        LPMS_Read();                      // зогссон ч DMA буферээ хоослох
        if (HAL_GetTick() - wb_wait_t0 >= WB_SOL_WAIT_MS) {
            Servo_SetDeg(WB_SERVO_UP);    // УХРАХЫН ӨМНӨ серво → 180°
            wb_end_t0 = HAL_GetTick();
            wb_st     = WB_S5_END_DRIVE;
        }
        break;

    /* -------- S5: 700мс УХРАХ (шулуун, gyro чиг барина) → эргэлт -------- */
    case WB_S5_END_DRIVE:
        if (HAL_GetTick() - wb_end_t0 >= WB_END_DRIVE_MS) {
            brake();
            wb_st = WB_S6_TURN;
            break;
        }
        Drive_Straight(WB_END_PWM);       // 700мс — УХРАХ (урагш=сөрөг тул эерэг)
        break;

    /* -------- S6: 180° эргэх → ДУУССАН -------- */
    case WB_S6_TURN:
        if (!wb_s6_init) {
            Gyro_TurnReset();   // өмнөх эргэлтийн үлдэгдэл төлвийг цэвэрлэх
            wb_s6_init = 1;
        }
        if (Gyro_TurnAngle(WB_TURN_DEG)) {   // дуусвал 1 (дотроо LPMS_Read)
            brake();
            wb_st = WB_DONE;
        }
        break;

    /* -------- ДУУССАН / БАРИХ: урагш зогссон, рак дээд дуудлагад 1950-д барина -------- */
    case WB_DONE:
        LPMS_Read();              // DMA буферээ хоослох (хоцрохгүй)
        return 1;

    default:                      // ТҮР comment-д байгаа S1..S6 (хүрэхгүй)
        break;
    }

    return 0;
}