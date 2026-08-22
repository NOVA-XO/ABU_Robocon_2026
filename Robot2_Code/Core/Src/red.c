/* =============================================================================
 *  red.c — Улаан (red) талын автомат зэвсгийн дараалал (weapon_red)
 *
 *  weapon_blue-ийн ТОЛИН ТУСГАЛ: бүх алхам ижил, зөвхөн align (зэрэгцүүлэх)
 *  strafe нь ЭСРЭГ тал руу (val3-аар БАРУУН) ба эцсийн 180° эргэлт эсрэг чиг
 *  (−180°). Талбарын улаан/цэнхэр эхлэл нь толин тусгал тул хажуулах чиг л
 *  өөрчлөгдөнө.
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Aug 22, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "red.h"

extern int counter[4]; // encoder тоолуурууд

/* ---- Тохиргоо (weapon_blue-тэй ижил, WR_ угтвартай) --------------------- */
#define WR_DRIVE_PWM                                                           \
  (-700)                     // хурдан урагш = СӨРӨГ.  −1000 бол дээд тагт хүрч
                             //   gyro залруулгад зай үлдэхгүй тул −700.
#define WR_DRIVE_SLOW (-100) // намуухан урагш (val8 руу дөхөх)
#define WR_RACK_BASE_LOW                                                       \
  1400                    // урагш явахдаа BACK рак ЭНД (FRONT нь +offset өндөр)
#define WR_FRONT_OFFSET 0 // FRONT/BACK ТЭГШ
#define WR_RACK_UP 1950   // val5 мэдэрч ЗОГССОНЫ дараа хоёр рак ЭНД дээшилнэ.
#define WR_RACK_FINAL 1800  // 180° эргэсний ДАРАА рак хоёулаа ЭНД (бүтэн) очно.
#define WR_SERVO_DOWN 0     // рак өргөгдсөний дараах сервоны өнцөг (буулгах)
#define WR_SERVO_UP 180     // серво буцаах өнцөг
#define WR_VAL8_HOLD_MS 200 // val8 ийм хугацаанд ТАСРАЛТГҮЙ 0 → "шахагдсан"
#define WR_VAL3_WAIT_MS 500 // val3 мэдэрсний (0) дараа хүлээх хугацаа (500мс)
#define WR_SOL_WAIT_MS 500  // соленоид 1 асаасны дараа хүлээх хугацаа (500мс)
#define WR_END_DRIVE_MS 700 // эцсийн УХРАХ хугацаа (700мс)
#define WR_END_PWM (+400)   // эцсийн явалт: УХРАХ (урагш=сөрөг тул эерэг=ухрах)
#define WR_SOLENOID 1       // val3==0 үед асаах соленоид
#define WR_TURN_DEG (-180.0f) // эцэст эргэх өнцөг — ТОЛИН ТУСГАЛ (blue нь +180)

/* ---- val3 align (val3==1 бол БАРУУН тийш зэрэгцүүлэх — blue нь ЗҮҮН) ----- */
#define WR_ALIGN_STRAFE 250 // хажуулах хурд
#define WR_ALIGN_FRONT 100 // УРД хоёр дугуй (1,2)-ийн хажуулах суурь нэмэлт PWM
#define WR_ALIGN_FWD                                                           \
  (-120)                  // урагш bias — val8-ыг шахсаар байлгах (сөрөг=урагш)
#define WR_ALIGN_KP 8.0f  // gyro чиг барих коэффициент
#define WR_ALIGN_WMAX 150 // gyro залруулгын дээд PWM

/* ---- Төлөвүүд (weapon_blue-тэй ижил) ------------------------------------ */
typedef enum {
  WR_S0_DRIVE = 0,  // val5 == 0 болтол gyro-гоор шулуун урагш
  WR_S1_RACK_UP,    // хоёр рак ХАМТ 1950 руу
  WR_S3_PUSH_VAL8,  // намуухан урагш + серво ЗЭРЭГ буулгах, val8 ТАСРАЛТГҮЙ 0
  WR_S4_ALIGN_VAL3, // val3==1 бол БАРУУН тийш (gyro+val8), val3==0 бол → хүлээх
  WR_S4B_WAIT,      // val3 мэдэрсэн → 500мс хүлээгээд → соленоид1 + серво 180
  WR_S4C_SOL_WAIT,  // соленоид1 асаасны дараа 500мс хүлээх
  WR_S5_END_DRIVE,  // 700мс УХРАХ (ракууд 1950-д)
  WR_S6_TURN,       // −180° эргэх (ракууд 1950-д)
  WR_S7_RACK,       // эргэсний ДАРАА рак хоёулаа 1800 руу (бүтэн)
  WR_DONE
} wr_state_t;

static wr_state_t wr_st = WR_S0_DRIVE;
static uint8_t wr_anchored = 0;
static uint8_t wr_srv_init = 0; // S3-д сервог нэг л удаа буулгах
static uint32_t wr_wait_t0 = 0; // val3-ийн дараах хүлээлтийн эхлэл
static uint32_t wr_v8_t0 = 0;   // val8 тасралтгүй 0 болсон эхлэлийн агшин
static uint32_t wr_end_t0 = 0;  // эцсийн ухрах явалтын эхлэл
static uint8_t wr_s6_init = 0;  // S6-д эргэлтийг нэг л удаа reset хийх
static int wr_rack_base = WR_RACK_BASE_LOW; // BACK зорилт
static int wr_front_off = WR_FRONT_OFFSET;  // FRONT-ийн нэмэлт өндөр

/* -----------------------------------------------------------------------------
 *  wr_move — ХАЖУУ + УРАГШ + gyro ЧИГ БАРИХ (mecanum, closed-loop чиг)
 *    vx  : хажуу.  vx < 0 → ЗҮҮН,  vx > 0 → баруун
 *    fwd : урагш bias.  сөрөг = урагш (Drive_Straight-ийн конвенц)
 *  (weapon_blue-ийн wb_move-тэй ЯГ ижил — тэмдэг vx-ийн дуудалтаас шийдэгдэнэ.)
 * -----------------------------------------------------------------------------
 */
static void wr_move(int vx, int fwd) {
  LPMS_Read();                              // gyro шинэ өгөгдөл
  float off = Get_Yaw_Offset_From_Anchor(); // anchor-аас хазайлт (°)
  int c = (int)(WR_ALIGN_KP * off);
  if (c > WR_ALIGN_WMAX)
    c = WR_ALIGN_WMAX;
  if (c < -WR_ALIGN_WMAX)
    c = -WR_ALIGN_WMAX;

  /* Урд хоёр дугуй (1,2) сул тул хажуулах хүчийг WR_ALIGN_FRONT-оор нэмнэ. */
  int vxf = vx;
  if (vxf > 0)
    vxf += WR_ALIGN_FRONT;
  else if (vxf < 0)
    vxf -= WR_ALIGN_FRONT;

  motor_control(1, -vxf + fwd - c); // Урд-Зүүн
  motor_control(2, vxf + fwd + c);  // Урд-Баруун
  motor_control(3, vx + fwd - c);   // Хойд-Зүүн
  motor_control(4, -vx + fwd + c);  // Хойд-Баруун
}

/* -----------------------------------------------------------------------------
 *  wr_show — OLED (100мс тутам).  Хурдыг хязгаарлахгүй бол LPMS DMA буфер
 * халина.
 * -----------------------------------------------------------------------------
 */
static void wr_show(void) {
  static uint32_t t = 0;
  if (HAL_GetTick() - t < 100)
    return;
  t = HAL_GetTick();

  int fp = counter[frontRack.enc];
  int bp = counter[backRack.enc];
  int d = fp - bp;
  if (d < 0)
    d = -d;

  colorFill(Black);
  setCursor(2, 2);
  printStr("RED S:%d", (int)wr_st);
  setCursor(2, 18);
  printStr("v5:%d v8:%d v3:%d", (int)val5, (int)val8, (int)val3);
  setCursor(2, 34);
  printStr("F:%d B:%d", fp, bp);
  setCursor(2, 50);
  printStr("d:%d", d);
  setScreen();
}

/* -----------------------------------------------------------------------------
 *  weapon_red_reset — эхнээс нь дахин ажиллуулах
 * -----------------------------------------------------------------------------
 */
void weapon_red_reset(void) {
  wr_st = WR_S0_DRIVE;
  wr_anchored = 0;
  wr_srv_init = 0;
  wr_wait_t0 = 0;
  wr_v8_t0 = 0;
  wr_end_t0 = 0;
  wr_s6_init = 0;
  wr_rack_base = WR_RACK_BASE_LOW;
  wr_front_off = WR_FRONT_OFFSET;
}

/* -----------------------------------------------------------------------------
 *  weapon_red — улаан талбарын дараалал (non-blocking state machine)
 *
 *    weapon_blue-тэй ижил урсгал; ялгаа:
 *      • S4 align: БАРУУН тийш (+WR_ALIGN_STRAFE), blue нь ЗҮҮН.
 *      • S6 эргэлт: −180° (толин тусгал), blue нь +180°.
 *
 *  Дуусвал 1, эс бөгөөс 0 буцаана. Loop-д давтан дуудна.
 * -----------------------------------------------------------------------------
 */
uint8_t weapon_red(void) {
  /* ---- РАКИЙН АЛДАА — БҮХ төлөвт шалгана ---- */
  uint8_t f = Rack_Fault();
  if (f)
    Robot_Error(f == 1 ? "RACK TIMEOUT" : "RACK SYNC");

  /* ---- РАКИЙГ ҮРГЭЛЖ ИДЭВХТЭЙ, СИНХРОН БАРИХ (FRONT нь +wr_front_off өндөр)
   */
  uint8_t rack_done = Rack_GoTo_Sync_Front(wr_rack_base, wr_front_off);

  wr_show();

  switch (wr_st) {

  /* -------- S0: val5 == 0 болтол шулуун урагш (рак дээд талд ЗЭРЭГ өгсөж) --
   */
  case WR_S0_DRIVE:
    if (!wr_anchored) {
      Set_Yaw_Anchor();
      wr_anchored = 1;
    }

    if (val5 == 0) { // val5 мэдэрлээ → урагш ЗОГСООД рак 1950 руу дээшлүүлнэ
      brake();
      wr_rack_base = WR_RACK_UP;
      wr_front_off = 0;
      wr_st = WR_S1_RACK_UP;
      break;
    }

    Drive_Straight(WR_DRIVE_PWM); // дотроо LPMS_Read() дуудна
    break;

  /* -------- S1: рак 1950-д хүрч ТОГТМОГЦ дараагийн алхам -------- */
  case WR_S1_RACK_UP:
    LPMS_Read(); // жолоо зогссон ч DMA буферээ хоослох
    if (rack_done) {
      wr_st = WR_S3_PUSH_VAL8;
    }
    break;

  /* -------- S3: НАМУУХАН урагш + серво ЗЭРЭГ буулгах, val8 0 болтол --------
   */
  case WR_S3_PUSH_VAL8:
    if (!wr_srv_init) {
      Servo_SetDeg(WR_SERVO_UP); // урагш явж байхдаа серво буулгах
      wr_srv_init = 1;
    }

    if (val8 == 0) {
      if (wr_v8_t0 == 0)
        wr_v8_t0 = HAL_GetTick();
      if (HAL_GetTick() - wr_v8_t0 >= WR_VAL8_HOLD_MS) {
        brake();
        wr_st = WR_S4_ALIGN_VAL3;
        break;
      }
    } else {
      wr_v8_t0 = 0;
    }

    Drive_Straight(WR_DRIVE_SLOW); // намуухан
    break;

  /* -------- S4: val3 align — БАРУУН тийш (blue нь ЗҮҮН) -------- */
  case WR_S4_ALIGN_VAL3:
    if (val3 == 0) {
      wr_wait_t0 = HAL_GetTick();
      wr_st = WR_S4B_WAIT;
      break;
    }

    /* vx ЭЕРЭГ = БАРУУН,  fwd сөрөг = урагш (val8 шахалт) */
    wr_move(+WR_ALIGN_STRAFE, WR_ALIGN_FWD);
    break;

  /* -------- S4B: val3==0 → 500мс хүлээгээд соленоид1 ON -------- */
  case WR_S4B_WAIT:
    if (HAL_GetTick() - wr_wait_t0 >= WR_VAL3_WAIT_MS) {
      brake();
      controlSolenoid(WR_SOLENOID, true); // соленоид 1 ON
      wr_wait_t0 = HAL_GetTick();
      wr_st = WR_S4C_SOL_WAIT;
      break;
    }

    Drive_Straight(WR_DRIVE_SLOW); // хүлээх зуур байрлалаа барина
    break;

  /* -------- S4C: соленоид1-ийн дараа 500мс → серво 180 → ухрах -------- */
  case WR_S4C_SOL_WAIT:
    LPMS_Read();
    if (HAL_GetTick() - wr_wait_t0 >= WR_SOL_WAIT_MS) {
      Servo_SetDeg(WR_SERVO_DOWN); // УХРАХЫН ӨМНӨ серво → 180°
      wr_end_t0 = HAL_GetTick();
      wr_st = WR_S5_END_DRIVE;
    }
    break;

  /* -------- S5: 700мс УХРАХ (шулуун, gyro чиг барина) → эргэлт -------- */
  case WR_S5_END_DRIVE:
    if (HAL_GetTick() - wr_end_t0 >= WR_END_DRIVE_MS) {
      brake();
      wr_st = WR_S6_TURN;
      break;
    }
    Drive_Straight(WR_END_PWM); // УХРАХ (урагш=сөрөг тул эерэг)
    break;

  /* -------- S6: −180° эргэх (толин тусгал) → рак руу -------- */
  case WR_S6_TURN:
    if (!wr_s6_init) {
      Gyro_TurnReset();
      wr_s6_init = 1;
    }
    if (Gyro_TurnAngle(WR_TURN_DEG)) { // дуусвал 1 (дотроо LPMS_Read)
      brake();
      wr_rack_base = WR_RACK_FINAL;
      wr_front_off = 0;
      wr_st = WR_S7_RACK;
    }
    break;

  /* -------- S7: эргэсний ДАРАА рак хоёулаа 1800 руу → ДУУССАН -------- */
  case WR_S7_RACK:
    brake();
    if (rack_done)
      wr_st = WR_DONE;
    break;

  /* -------- ДУУССАН / БАРИХ -------- */
  case WR_DONE:
    LPMS_Read();
    return 1;

  default:
    break;
  }

  return 0;
}
