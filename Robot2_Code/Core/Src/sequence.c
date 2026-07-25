/* =============================================================================
 *  sequence.c — Бүтэн үйлдлийн блокууд (non-blocking sequence blocks)
 *
 *  red.c / blue.c тал давтан ашиглах бүтэн дарааллын блокууд.
 *  Блок бүр өөрийн төлөвтэй (static) state machine:
 *    - Дотоод while давталт, HAL_Delay БАЙХГҮЙ.
 *    - Нэг дуудалт = нэг алхам, тэр даруй буцна.
 *    - Хугацааны хүлээлтийг wait_ms()-ээр non-blocking хийнэ.
 *
 *  Блокууд:  up_20_function   — 20-д гарах  (red.c-ийн хуучин Sequence-ийн
 * логик) down_20_function — 20-оос буух (загвар — бөглөх) up_40_function   —
 * 40-д гарах  (загвар — бөглөх) down_40_function — 40-оос буух (загвар —
 * бөглөх)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "sequence.h"

extern int counter[4]; // encoder тоолуурууд (OLED дээр харуулах)

/* ---- Рак байрлалууд ------------------------------------------------------ */
#define RACK_UP 1000 // рак дээд (барих) байрлал — 20-ийн блокууд
#define RACK_UP_40                                                             \
  1950              // рак дээд байрлал — 40-ийн блокууд (pos_max-тай тэнцүү)
#define RACK_DOWN 0 // рак доод байрлал

/* =============================================================================
 *  НИЙТЛЭГ ТУСЛАХ ФУНКЦУУД (бүх блок хуваалцана)
 * =============================================================================
 */

/* Нэг зэрэг зөвхөн НЭГ блок ажиллана гэж үзвэл нэг хүлээлтийн таймер хангалттай
 */
static uint8_t wait_active = 0;
static uint32_t wait_t0 = 0;

/* -----------------------------------------------------------------------------
 *  wait_ms — non-blocking хугацааны хүлээлт (return: 1 = дүүрсэн, 0 = хүлээж
 * байна)
 * -----------------------------------------------------------------------------
 */
__attribute__((unused)) // дараагийн алхмуудад хэрэглэгдэнэ (хүлээлт)
static uint8_t wait_ms(uint32_t ms) {
  if (!wait_active) {
    wait_active = 1;
    wait_t0 = HAL_GetTick();
  }
  if (HAL_GetTick() - wait_t0 >= ms) {
    wait_active = 0;
    return 1;
  }
  return 0;
}

/* -----------------------------------------------------------------------------
 *  Rack_Hold — Хоёр ракыг тухайн байрлалд барих (нэг алхам)
 * -----------------------------------------------------------------------------
 */
__attribute__((unused)) // дараагийн алхмуудад хэрэглэгдэнэ (рак барих)
static void Rack_Hold(int f_pos, int b_pos) {
  Rack_GoTo(&frontRack, f_pos);
  Rack_GoTo(&backRack, b_pos);
}

/* Sequence блокуудын урагш явах ҮНДСЭН хурд (Drive_Open → gyro чиг барина).
   Урагш = СӨРӨГ конвенц. Бүх блок үүнийг ашиглана — нэг газраас тааруулна.   */
#define SEQ_DRIVE_PWM (-250)
#define SEQ_DRIVE_PWM_SLOW (-200) // зарим блокуудад бага хурд хэрэгтэй
#define EXIT_DRIVE_PWM (-1000)    // гарах маневр: val5 руу MAX хурдаар урагш
#define EXIT_RACK 700             // гарах маневр: 2 дахь урагшаас өмнө хоёр рак ЭНД

/* -----------------------------------------------------------------------------
 *  Drive_Open — GYRO-гоор чиг барьж шулуун явах (Drive_Straight-ийн бүрхүүл)
 *    pwm: урагш = СӨРӨГ, ухрах = ЭЕРЭГ.
 *
 *  Өмнө нь open-loop (4 моторт ижил PWM) байсан — удаан явбал хазайдаг байв.
 *  Одоо Drive_Straight-ийг дуудна: anchor-аас хазайхыг gyro-гоор залруулна.
 *  ⚠ Дуудахаас ӨМНӨ climb эхлэхэд Set_Yaw_Anchor() тавьсан байх ЁСТОЙ
 *    (selectMode-ийн climb горимд хийгддэг), эс бөгөөс хуучин anchor руу засна.
 * -----------------------------------------------------------------------------
 */
static void Drive_Open(int pwm) {
  Drive_Straight(pwm); // дотроо LPMS_Read() + gyro чиг барих
}

/* -----------------------------------------------------------------------------
 *  Seq_Show — Блокийн нэр, төлөв, encoder, сенсорыг OLED дээр харуулах (100мс-д
 * нэг)
 * -----------------------------------------------------------------------------
 */
static void Seq_Show(const char *name, int st) {
  static uint32_t t = 0;
  if (HAL_GetTick() - t < 100)
    return;
  t = HAL_GetTick();

  colorFill(Black);
  setCursor(10, 2);
  printStr("%s S:%d", name, st);
  setCursor(10, 22);
  printStr("F:%d B:%d", counter[0], counter[1]);
  setCursor(10, 42);
  printStr("v1:%d v2:%d", val1, val2);
  setScreen();
}

/* =============================================================================
 *  up_20_function — 20-д гарах бүтэн дараалал
 *  (red.c-ийн хуучин блоклодог Sequence-ийн non-blocking хувилбар)
 * =============================================================================
 */
enum {
  U20_S0_DRIVE = 0,  // val5 == 0 болтол шулуун урагш
  U20_S1_RACK_UP,    // хоёр ракыг 1000 (RACK_UP) руу
  U20_S2_DRIVE,      // рак UP БАРЬЖ, val7 == 0 болтол урагш
  U20_S3_FRONT_DOWN, // front ракыг 0 руу (ХҮЧТЭЙ); back-ийг UP барих
  U20_S4_DRIVE,      // front 0 / back UP БАРЬЖ, val4 == 0 болтол урагш
  U20_S5_BACK_DOWN,  // back ракыг 0 руу (ХҮЧТЭЙ); front 0-д барих
  U20_S6_DRIVE,      // хоёр рак 0 БАРЬЖ, val3 == 0 болтол урагш
  U20_DONE
  /* дараагийн алхмуудыг ЭНД нэмнэ (U20_S7_...) */
};

static int u20_state = U20_S0_DRIVE;

uint8_t up_20_function(void) {

  Seq_Show("UP20", u20_state);

  switch (u20_state) {

  // -------- S0: val5 == 0 болтол шулуун урагш (gyro-ГҮЙ, open-loop) --------
  case U20_S0_DRIVE:
    Drive_Open(
        SEQ_DRIVE_PWM); // урагш (конвенц: урагш = сөрөг); gyro чиг барина
    if (val5 == 0) {
      brake();
      u20_state = U20_S1_RACK_UP; // → хоёр рак дээш
    }
    break;

  // -------- S1: хоёр ракыг 1000 (RACK_UP) руу; хоёул хүрэхэд дараагийн алхам
  // --------
  case U20_S1_RACK_UP:
    if (Rack_GoTo_Sync(RACK_UP)) { // ХОЁУЛАА зэрэгцүүлж дээш
      u20_state = U20_S2_DRIVE;    // → рак барьж урагш
    }
    break;

  // -------- S2: рак UP БАРЬЖ, val7 == 0 болтол урагш --------
  //   val7-ийг ЗӨВХӨН энд шалгана — S1 дуусаж, рак аль хэдийн 1000 дээр гарсан
  //   тул "рак дээшлэхээс өмнө val7 == 0" гэдэг худал триггер боломжгүй.
  case U20_S2_DRIVE:
    Rack_Hold(RACK_UP, RACK_UP); // ракыг 1000-д БАРЬСААР (таталцлаар унахгүй)
    Drive_Open(SEQ_DRIVE_PWM);   // урагш (gyro чиг барина)
    if (val7 == 0) {
      brake();
      frontRack.land_soft = 0; // front-ийг ТУСАД НЬ буулгах тул ХҮЧТЭЙ
      u20_state = U20_S3_FRONT_DOWN;
    }
    break;

  // -------- S3: front ракыг 0 руу (ХҮЧТЭЙ); back-ийг UP барих --------
  case U20_S3_FRONT_DOWN: {
    uint8_t f = Rack_GoTo(&frontRack, RACK_DOWN);
    Rack_GoTo(&backRack, RACK_UP); // back-ийг 1000-д барьсаар
    if (f) {
      u20_state = U20_S4_DRIVE; // → front 0 / back UP барьж урагш
    }
    break;
  }

  // -------- S4: front 0 / back UP БАРЬЖ, val4 == 0 болтол урагш --------
  case U20_S4_DRIVE:
    Rack_Hold(RACK_DOWN, RACK_UP); // front 0 (coast), back 1000 барих
    Drive_Open(SEQ_DRIVE_PWM);     // урагш (gyro чиг барина)
    if (val4 == 0) {
      brake();
      backRack.land_soft = 0; // back-ийг ТУСАД НЬ буулгах тул ХҮЧТЭЙ
      u20_state = U20_S5_BACK_DOWN;
    }
    break;

  // -------- S5: back ракыг 0 руу (ХҮЧТЭЙ); front 0-д барих --------
  case U20_S5_BACK_DOWN: {
    Rack_GoTo(&frontRack, RACK_DOWN); // front 0-д (coast)
    uint8_t b = Rack_GoTo(&backRack, RACK_DOWN);
    if (b) {
      u20_state = U20_S6_DRIVE; // → хоёр рак 0 барьж урагш
    }
    break;
  }

  // -------- S6: хоёр рак 0 БАРЬЖ, val3 == 0 болтол урагш --------
  case U20_S6_DRIVE:
    Rack_Hold(RACK_DOWN, RACK_DOWN); // хоёулаа 0 (coast)
    Drive_Open(SEQ_DRIVE_PWM);       // урагш (gyro чиг барина)
    if (val3 == 0) {
      brake();
      u20_state = U20_DONE;
    }
    break;

  // -------- дараагийн алхмуудыг ЭНД case-ээр нэмнэ --------

  // -------- Нэг мөчлөг дуусав (давталтыг climb функц удирдана) --------
  case U20_DONE:
    brake();
    return 1;
  }

  return 0;
}

void up_20_reset(void) {
  u20_state = U20_S0_DRIVE;
  wait_active = 0;
}

/* =============================================================================
 *  down_20_function — 20-оос БУУХ дараалал (шат доошоо; УХРАХГҮЙ, урагшаа явна)
 *  20 см өндөр шатнаас буулгах.
 * =============================================================================
 */
enum {
  D20_S0_DRIVE = 0, // val6 == 1 болтол шулуун урагш
  D20_S1_FRONT_UP,  // front ракыг 1000 руу
  D20_S2_DRIVE,     // front 1000 БАРЬЖ, val3 == 1 болтол урагш
  D20_S3_DRIVE,     // front 1000 БАРЬЖ, val4 == 1 болтол урагш
  D20_S4_BACK_UP,   // back ракыг 1000 руу (front-ийг 1000 барих)
  D20_S5_DRIVE,     // хоёр рак 1000 БАРЬЖ, 500мс урагш
  D20_S6_DOWN,      // хоёр ракыг ЗЭРЭГ 0 руу (ЗӨӨЛӨН), зогсох
  D20_DONE
  /* дараагийн алхмуудыг ЭНД нэмнэ (D20_S7_...) */
};

static int d20_state = D20_S0_DRIVE;

uint8_t down_20_function(void) {

  Seq_Show("DOWN20", d20_state);

  switch (d20_state) {

  // -------- S0: val6 == 1 болтол шулуун урагш (gyro-ГҮЙ; УХРАХГҮЙ) --------
  case D20_S0_DRIVE:
    Drive_Open(SEQ_DRIVE_PWM); // урагш (конвенц: урагш = сөрөг)
    if (val6 == 1) {
      brake();
      d20_state = D20_S1_FRONT_UP; // → front рак дээш
    }
    break;

  // -------- S1: front ракыг 1000 руу; хүрэхэд дараагийн алхам --------
  case D20_S1_FRONT_UP: {
    uint8_t f = Rack_GoTo(&frontRack, RACK_UP);
    if (f) {
      d20_state = D20_S2_DRIVE; // → front барьж урагш
    }
    break;
  }

  // -------- S2: front 1000 БАРЬЖ, val3 == 1 болтол урагш --------
  case D20_S2_DRIVE:
    Rack_Hold(RACK_UP, RACK_DOWN); // front 1000 барих, back 0 (coast)
    Drive_Open(SEQ_DRIVE_PWM);     // урагш (gyro чиг барина)
    if (val3 == 1) {
      brake();
      d20_state = D20_S3_DRIVE; // → val4 хүртэл үргэлжлүүлнэ
    }
    break;

  // -------- S3: front 1000 БАРЬЖ, val4 == 1 болтол урагш --------
  case D20_S3_DRIVE:
    Rack_Hold(RACK_UP, RACK_DOWN);  // front 1000 барих, back 0 (coast)
    Drive_Open(SEQ_DRIVE_PWM_SLOW); // урагш (gyro чиг барина)
    if (val4 == 1) {
      brake();
      d20_state = D20_S4_BACK_UP; // → back рак дээш
    }
    break;

  // -------- S4: back ракыг 1000 руу (front-ийг 1000 барих) --------
  case D20_S4_BACK_UP: {
    Rack_GoTo(&frontRack, RACK_UP); // front 1000 барьсаар
    uint8_t b = Rack_GoTo(&backRack, RACK_UP);
    if (b) {
      d20_state = D20_S5_DRIVE; // → 500мс урагш
    }
    break;
  }

  // -------- S5: хоёр рак 1000 БАРЬЖ, 500мс урагш --------
  case D20_S5_DRIVE:
    Rack_Hold(RACK_UP, RACK_UP); // хоёулаа 1000 барих
    Drive_Open(SEQ_DRIVE_PWM);   // урагш (gyro чиг барина)
    if (wait_ms(1000)) {
      brake();
      frontRack.land_soft = 1; // ХОЁУЛАА ЗЭРЭГ 0 → ЗӨӨЛӨН буулт
      backRack.land_soft = 1;
      d20_state = D20_S6_DOWN;
    }
    break;

  // -------- S6: хоёр ракыг ЗЭРЭГ 0 руу (ЗӨӨЛӨН); хоёул хүрэхэд зогсоно
  // --------
  case D20_S6_DOWN:
    if (Rack_GoTo_Sync(RACK_DOWN)) { // ХОЁУЛАА зэрэгцүүлж доош
      brake();
      d20_state = D20_DONE;
    }
    break;

    // -------- дараагийн алхмуудыг ЭНД case-ээр нэмнэ --------

  case D20_DONE:
    brake();
    return 1;
  }

  return 0;
}

void down_20_reset(void) {
  d20_state = D20_S0_DRIVE;
  wait_active = 0;
}

/* =============================================================================
 *  up_40_function — 40-д гарах бүтэн дараалал   (ЗАГВАР — бөглөх)
 * =============================================================================
 */
/* up_20-той ЯГ АДИЛ, зөвхөн рак 1950 (RACK_UP_40) руу гардаг (1000 биш). */
enum {
  U40_S0_DRIVE = 0,  // val5 == 0 болтол шулуун урагш
  U40_S1_RACK_UP,    // хоёр ракыг 1950 (RACK_UP_40) руу
  U40_S2_DRIVE,      // рак UP БАРЬЖ, val7 == 0 болтол урагш
  U40_S3_FRONT_DOWN, // front ракыг 0 руу (ХҮЧТЭЙ); back-ийг UP барих
  U40_S4_DRIVE,      // front 0 / back UP БАРЬЖ, val4 == 0 болтол урагш
  U40_S5_BACK_DOWN,  // back ракыг 0 руу (ХҮЧТЭЙ); front 0-д барих
  U40_S6_DRIVE,      // хоёр рак 0 БАРЬЖ, val3 == 0 болтол урагш
  U40_DONE
};

static int u40_state = U40_S0_DRIVE;

uint8_t up_40_function(void) {

  Seq_Show("UP40", u40_state);

  switch (u40_state) {

  // -------- S0: val5 == 0 болтол шулуун урагш --------
  case U40_S0_DRIVE:
    Drive_Open(SEQ_DRIVE_PWM); // урагш (gyro чиг барина)
    if (val5 == 0) {
      brake();
      u40_state = U40_S1_RACK_UP;
    }
    break;

  // -------- S1: хоёр ракыг 1950 руу; хоёул хүрэхэд дараагийн алхам --------
  case U40_S1_RACK_UP:
    if (Rack_GoTo_Sync(RACK_UP_40)) { // ХОЁУЛАА зэрэгцүүлж дээш
      u40_state = U40_S2_DRIVE;
    }
    break;

  // -------- S2: рак 1950 БАРЬЖ, val7 == 0 болтол урагш --------
  case U40_S2_DRIVE:
    Rack_Hold(RACK_UP_40, RACK_UP_40); // ракыг 1950-д БАРЬСААР
    Drive_Open(SEQ_DRIVE_PWM);
    if (val7 == 0) {
      brake();
      frontRack.land_soft = 0; // front-ийг ТУСАД НЬ буулгах тул ХҮЧТЭЙ
      u40_state = U40_S3_FRONT_DOWN;
    }
    break;

  // -------- S3: front ракыг 0 руу (ХҮЧТЭЙ); back-ийг UP барих --------
  case U40_S3_FRONT_DOWN: {
    uint8_t f = Rack_GoTo(&frontRack, RACK_DOWN);
    Rack_GoTo(&backRack, RACK_UP_40); // back-ийг 1950-д барьсаар
    if (f) {
      u40_state = U40_S4_DRIVE;
    }
    break;
  }

  // -------- S4: front 0 / back 1950 БАРЬЖ, val4 == 0 болтол урагш --------
  case U40_S4_DRIVE:
    Rack_Hold(RACK_DOWN, RACK_UP_40); // front 0 (coast), back 1950 барих
    Drive_Open(SEQ_DRIVE_PWM);
    if (val4 == 0) {
      brake();
      backRack.land_soft = 0; // back-ийг ТУСАД НЬ буулгах тул ХҮЧТЭЙ
      u40_state = U40_S5_BACK_DOWN;
    }
    break;

  // -------- S5: back ракыг 0 руу (ХҮЧТЭЙ); front 0-д барих --------
  case U40_S5_BACK_DOWN: {
    Rack_GoTo(&frontRack, RACK_DOWN); // front 0-д (coast)
    uint8_t b = Rack_GoTo(&backRack, RACK_DOWN);
    if (b) {
      u40_state = U40_S6_DRIVE;
    }
    break;
  }

  // -------- S6: хоёр рак 0 БАРЬЖ, val3 == 0 болтол урагш --------
  case U40_S6_DRIVE:
    Rack_Hold(RACK_DOWN, RACK_DOWN); // хоёулаа 0 (coast)
    Drive_Open(SEQ_DRIVE_PWM);
    if (val3 == 0) {
      brake();
      u40_state = U40_DONE;
    }
    break;

  // -------- Нэг мөчлөг дуусав (давталтыг climb функц удирдана) --------
  case U40_DONE:
    brake();
    return 1;
  }

  return 0;
}

void up_40_reset(void) {
  u40_state = U40_S0_DRIVE;
  wait_active = 0;
}

/* =============================================================================
 *  down_40_function — 40-оос БУУХ дараалал
 *  down_20-той ЯГ АДИЛ, гэхдээ рак 1950 (RACK_UP_40) руу гардаг (1000 биш).
 * =============================================================================
 */
enum {
  D40_S0_DRIVE = 0, // val6 == 1 болтол шулуун урагш
  D40_S1_FRONT_UP,  // front ракыг 1950 руу
  D40_S2_DRIVE,     // front 1950 БАРЬЖ, val3 == 1 болтол урагш
  D40_S3_DRIVE,     // front 1950 БАРЬЖ, val4 == 1 болтол урагш
  D40_S4_BACK_UP,   // back ракыг 1950 руу (front-ийг 1950 барих)
  D40_S5_DRIVE,     // хоёр рак 1950 БАРЬЖ, 500мс урагш
  D40_S6_DOWN,      // хоёр ракыг ЗЭРЭГ 0 руу (ЗӨӨЛӨН), зогсох
  D40_DONE
};

static int d40_state = D40_S0_DRIVE;

uint8_t down_40_function(void) {

  Seq_Show("DOWN40", d40_state);

  switch (d40_state) {

  // -------- S0: val6 == 1 болтол шулуун урагш (gyro-ГҮЙ; УХРАХГҮЙ) --------
  case D40_S0_DRIVE:
    Drive_Open(SEQ_DRIVE_PWM); // урагш (конвенц: урагш = сөрөг)
    if (val6 == 1) {
      brake();
      d40_state = D40_S1_FRONT_UP; // → front рак дээш
    }
    break;

  // -------- S1: front ракыг 1950 руу; хүрэхэд дараагийн алхам --------
  case D40_S1_FRONT_UP: {
    uint8_t f = Rack_GoTo(&frontRack, RACK_UP_40);
    if (f) {
      d40_state = D40_S2_DRIVE; // → front барьж урагш
    }
    break;
  }

  // -------- S2: front 1950 БАРЬЖ, val3 == 1 болтол урагш --------
  case D40_S2_DRIVE:
    Rack_Hold(RACK_UP_40, RACK_DOWN); // front 1950 барих, back 0 (coast)
    Drive_Open(SEQ_DRIVE_PWM);        // урагш (gyro чиг барина)
    if (val3 == 1) {
      brake();
      d40_state = D40_S3_DRIVE; // → val4 хүртэл үргэлжлүүлнэ
    }
    break;

  // -------- S3: front 1950 БАРЬЖ, val4 == 1 болтол урагш --------
  case D40_S3_DRIVE:
    Rack_Hold(RACK_UP_40, RACK_DOWN); // front 1950 барих, back 0 (coast)
    Drive_Open(SEQ_DRIVE_PWM_SLOW);   // урагш (gyro чиг барина)
    if (val4 == 1) {
      brake();
      d40_state = D40_S4_BACK_UP; // → back рак дээш
    }
    break;

  // -------- S4: back ракыг 1950 руу (front-ийг 1950 барих) --------
  case D40_S4_BACK_UP: {
    Rack_GoTo(&frontRack, RACK_UP_40); // front 1950 барьсаар
    uint8_t b = Rack_GoTo(&backRack, RACK_UP_40);
    if (b) {
      d40_state = D40_S5_DRIVE; // → 500мс урагш
    }
    break;
  }

  // -------- S5: хоёр рак 1950 БАРЬЖ, 500мс урагш --------
  case D40_S5_DRIVE:
    Rack_Hold(RACK_UP_40, RACK_UP_40); // хоёулаа 1950 барих
    Drive_Open(SEQ_DRIVE_PWM);         // урагш (gyro чиг барина)
    if (wait_ms(500)) {
      brake();
      frontRack.land_soft = 1; // ХОЁУЛАА ЗЭРЭГ 0 → ЗӨӨЛӨН буулт
      backRack.land_soft = 1;
      d40_state = D40_S6_DOWN;
    }
    break;

  // -------- S6: хоёр ракыг ЗЭРЭГ 0 руу (ЗӨӨЛӨН); хоёул хүрэхэд зогсоно
  // --------
  case D40_S6_DOWN:
    if (Rack_GoTo_Sync(RACK_DOWN)) { // ХОЁУЛАА зэрэгцүүлж доош
      brake();
      d40_state = D40_DONE;
    }
    break;

  case D40_DONE:
    brake();
    return 1;
  }

  return 0;
}

void down_40_reset(void) {
  d40_state = D40_S0_DRIVE;
  wait_active = 0;
}

/* =============================================================================
 *  CLIMB — зам (уул) бүрийн БҮТЭН дэс дараалал: up/down блокуудыг тодорхой
 *          тоогоор гинжлэн угсарна. Блок бүр НЭГ мөчлөг хийж 1 буцаана; тоог
 *          climb функц удирдана (дотоод давталт устгагдсан).
 *
 *  Ашиглах: climbN_function()-г main loop-д давтан дуудна, бүрэн дуусахад 1;
 *           дахин ажиллуулахын өмнө climbN_reset() дуудна.
 * =============================================================================
 */

/* Блокийг n удаа ажиллуулна (мөчлөг бүрийн хооронд reset). Бүгд дуусахад 1. */
typedef uint8_t (*seq_fn)(void);
typedef void (*seq_reset_fn)(void);

static uint8_t run_n(seq_fn fn, seq_reset_fn reset, int n, int *cnt) {
  if (fn()) { // нэг мөчлөг дуусав
    (*cnt)++;
    if (*cnt >= n) {
      *cnt = 0;
      return 1;
    } // бүгд дуусав
    reset(); // дараагийн мөчлөгт бэлдэнэ
  }
  return 0;
}

/* -----------------------------------------------------------------------------
 *  climb_1 — ЗАМ 1:  up_40 ×1  →  down_20 ×1  →  up_20 ×1  →  down_20 ×2
 * -----------------------------------------------------------------------------
 */
enum { C1_UP40 = 0, C1_DOWN20A, C1_UP20, C1_DOWN20B, C1_DONE };
static int c1_state = C1_UP40;
static int c1_cnt = 0;

uint8_t climb_1_function(void) {
  switch (c1_state) {
  case C1_UP40:
    if (run_n(up_40_function, up_40_reset, 1, &c1_cnt)) {
      down_20_reset();
      c1_state = C1_DOWN20A;
    }
    break;
  case C1_DOWN20A:
    if (run_n(down_20_function, down_20_reset, 1, &c1_cnt)) {
      up_20_reset();
      c1_state = C1_UP20;
    }
    break;
  case C1_UP20:
    if (run_n(up_20_function, up_20_reset, 1, &c1_cnt)) {
      down_20_reset();
      c1_state = C1_DOWN20B;
    }
    break;
  case C1_DOWN20B:
    if (run_n(down_20_function, down_20_reset, 2, &c1_cnt)) {
      c1_state = C1_DONE;
    }
    break;
  case C1_DONE:
    brake();
    return 1;
  }
  return 0;
}

void climb_1_reset(void) {
  c1_state = C1_UP40;
  c1_cnt = 0;
  up_40_reset(); // эхний блок
}

/* -----------------------------------------------------------------------------
 *  climb_2 — ЗАМ 2:  up_20 ×3  →  down_20 ×1  →  down_40 ×1
 * -----------------------------------------------------------------------------
 */
enum { C2_UP20 = 0, C2_DOWN20, C2_DOWN40, C2_DONE };
static int c2_state = C2_UP20;
static int c2_cnt = 0;

uint8_t climb_2_function(void) {
  switch (c2_state) {
  case C2_UP20:
    if (run_n(up_20_function, up_20_reset, 3, &c2_cnt)) {
      down_20_reset();
      c2_state = C2_DOWN20;
    }
    break;
  case C2_DOWN20:
    if (run_n(down_20_function, down_20_reset, 1, &c2_cnt)) {
      down_40_reset();
      c2_state = C2_DOWN40;
    }
    break;
  case C2_DOWN40:
    if (run_n(down_40_function, down_40_reset, 1, &c2_cnt)) {
      c2_state = C2_DONE;
    }
    break;
  case C2_DONE:
    brake();
    return 1;
  }
  return 0;
}

void climb_2_reset(void) {
  c2_state = C2_UP20;
  c2_cnt = 0;
  up_20_reset(); // эхний блок
}

/* -----------------------------------------------------------------------------
 *  climb_3 — ЗАМ 3:  up_40 ×1  →  up_20 ×1  →  down_20 ×3
 * -----------------------------------------------------------------------------
 */
enum { C3_UP40 = 0, C3_UP20, C3_DOWN20, C3_DONE };
static int c3_state = C3_UP40;
static int c3_cnt = 0;

uint8_t climb_3_function(void) {
  switch (c3_state) {
  case C3_UP40:
    if (run_n(up_40_function, up_40_reset, 1, &c3_cnt)) {
      up_20_reset();
      c3_state = C3_UP20;
    }
    break;
  case C3_UP20:
    if (run_n(up_20_function, up_20_reset, 1, &c3_cnt)) {
      down_20_reset();
      c3_state = C3_DOWN20;
    }
    break;
  case C3_DOWN20:
    if (run_n(down_20_function, down_20_reset, 3, &c3_cnt)) {
      c3_state = C3_DONE;
    }
    break;
  case C3_DONE:
    brake();
    return 1;
  }
  return 0;
}

void climb_3_reset(void) {
  c3_state = C3_UP40;
  c3_cnt = 0;
  up_40_reset(); // эхний блок
}

/* =============================================================================
 *  auto_climb — PCB2-оос ирсэн ЗАМ (g_route)-оор climb_1/2/3-ыг АВТОМАТААР
 *
 *  Тактик setup-д PCB2 хамгийн сайн баганыг USART2-оор илгээж, main.c-ийн
 *  g_route (1/2/3)-д хадгалагдана. Энэ мод эхлэхэд route-ыг ЛАТЧилж (нэг л
 * удаа), тэр замын climb-ыг ДУУСТАЛ ажиллуулна — дунд нь g_route өөрчлөгдсөн ч
 * замаа СОЛИХГҮЙ (тогтвортой). g_route хараахан ирээгүй (0) бол хүлээнэ.
 *
 *  ⚠ Дүрэм 10.7: route нь тоглоом эхлэхээс ӨМНӨ (setup) ирдэг тул энэ нь
 *    тоглоомын үеийн гадаад тушаал БИШ — R2 өөрөө автоматаар гүйцэтгэнэ.
 * =============================================================================
 */
extern volatile uint8_t g_route; // main.c: PCB2-оос ирсэн зам (1/2/3), 0=алга
extern volatile uint8_t g_route_set; // main.c: 1 = тактик БАТАЛГААЖСАН (SET)
extern volatile uint8_t g_grab_done; // main.c: 1 = PCB2 grab дуусгасан (ack)
extern volatile uint8_t g_grab_ans;  // main.c: PCB2 "энд grab уу?" хариу (1/0)
extern volatile uint8_t g_grab_ans_rdy; // main.c: 1 = хариу шинээр ирсэн
extern volatile uint8_t
    g_strafe_cmd; // main.c: PCB2 strafe команд (0=зогс 1=зүүн 2=баруун)
extern volatile uint32_t
    g_strafe_ms; // main.c: сүүлд strafe команд ирсэн үе (шинэлэг)
extern volatile uint32_t route_rx_n;  // main.c: USART2 RX нийт байт (оношилгоо)
extern volatile uint32_t route_pkt_n; // main.c: бүрэн задарсан багц
void Link_Send_Grab(void);            // main.c: PCB2 руу GRAB команд илгээх
void Link_Query_Block(uint8_t block); // main.c: "энэ блок дээр grab уу?" асуух

/* grab handshake-ийн тохиргоо */
#define GRAB_STRAFE 250    // хажуулах хурд (PWM)
#define GRAB_MS 500        // (ашиглагдахгүй болсон — strafe-ийг PCB2 удирдана)
#define GRAB_RESEND_MS 300 // GRAB командыг давтан илгээх интервал
#define GRAB_STRAFE_FRESH_MS                                                   \
  700 // strafe команд энэ хугацаанд шинэлэг байвал л биелүүлнэ
#define GRAB_FWD_PWM                                                           \
  SEQ_DRIVE_PWM_SLOW             // strafe-ийн ӨМНӨ урагш (gyro шулуун) хурд
#define GRAB_FWD_TIMEOUT_MS 3000 // val7==1 болохгүй бол хамгаалалт → strafe рүү

/* -----------------------------------------------------------------------------
 *  grab_step — GRAB дараалал (non-blocking). Strafe-ийг ОДОО PCB2 удирдана:
 *    0) УРАГШ (val7==1 болтол, gyro шулуун) → PCB2 руу GRAB илгээ
 *    1) PCB2-ийн strafe командыг (0xB7: зүүн/баруун/зогс) биелүүлж, "done"
 * хүлээх — strafe команд GRAB_STRAFE_FRESH_MS-д шинэлэг байвал л хөдөлнө
 * (алдагдсан пакетаас хамгаална: команд хуучирвал автоматаар зогсоно).
 *    Дуусвал 1. st/t0-г дуудагч тэглэнэ (t0 = grab эхэлсэн үе → case 0
 * timeout).
 * -----------------------------------------------------------------------------
 */
static uint8_t grab_step(uint8_t *st, uint32_t *t0) {
  switch (*st) {
  case 0: /* УРАГШ — val7 == 1 болтол (эсвэл timeout хамгаалалт) → GRAB эхлүүл
           */
    if (val7 == 1 || HAL_GetTick() - *t0 >= GRAB_FWD_TIMEOUT_MS) {
      brake();
      Link_Send_Grab(); // PCB2 дараалал эхлүүл
      g_grab_done = 0;
      g_strafe_cmd = 0; // цэвэр эхлэл (зогс)
      *st = 1;
      *t0 = HAL_GetTick(); // GRAB давтан илгээх таймер
      break;
    }
    Drive_Open(GRAB_FWD_PWM); // урагш (gyro чиг барина)
    break;

  case 1: /* PCB2 удирдлагаар strafe хийж, "done" хүлээх */
    // GRAB алдагдсан бол давтан илгээх (PCB2 хариу өгөх хүртэл)
    if (!g_grab_done && HAL_GetTick() - *t0 >= GRAB_RESEND_MS) {
      Link_Send_Grab();
      *t0 = HAL_GetTick();
    }
    // PCB2-ийн strafe командыг биелүүлэх (ЗӨВХӨН шинэлэг команд бол)
    if (g_strafe_cmd != 0 &&
        HAL_GetTick() - g_strafe_ms <= GRAB_STRAFE_FRESH_MS) {
      if (g_strafe_cmd == 1)
        Strafe_Gyro(-GRAB_STRAFE); // зүүн
      else
        Strafe_Gyro(+GRAB_STRAFE); // баруун (2)
    } else {
      brake(); // команд алга / хуучирсан → зогс
    }
    if (g_grab_done) {
      brake();
      *st = 2;
    }
    break;

  default:
    return 1; // 2 = дуусав
  }
  return 0;
}

/* =============================================================================
 *  CLIMB STEP-LIST — route бүр = блокуудын дараалал (блок бүр = нэг move).
 *    move дуусах бүрд PCB2-оос "энд grab уу?" асууна.
 *  ⚠ ROUTE1 нь БАТЛАГДСАН (1→4→7→10). ROUTE2/ROUTE3 нь ТААМАГ — блокийн
 *    дараалал/өндрийг батлуулах шаардлагатай.
 * =============================================================================
 */
typedef struct {
  seq_fn fn;
  seq_reset_fn rst;
  uint8_t count;
  uint8_t block;
} ClimbStep_t;

static const ClimbStep_t ROUTE1[] = {
    /* багана 1: 1→4→7→10 (БАТЛАГДСАН) */
    {up_40_function, up_40_reset, 1, 1},
    {down_20_function, down_20_reset, 1, 4},
    {up_20_function, up_20_reset, 1, 7},
    {down_20_function, down_20_reset, 2, 10},
};
static const ClimbStep_t ROUTE2[] = {
    /* багана 2: 2→5→8→11  ⚠ ТААМАГ */
    {up_20_function, up_20_reset, 1, 2},
    {up_20_function, up_20_reset, 1, 5},
    {up_20_function, up_20_reset, 1, 8},
    {down_20_function, down_20_reset, 1, 11},
};
static const ClimbStep_t ROUTE3[] = {
    /* багана 3: 3→6→9→12  ⚠ ТААМАГ */
    {up_40_function, up_40_reset, 1, 3},
    {up_20_function, up_20_reset, 1, 6},
    {down_20_function, down_20_reset, 1, 9},
    {down_20_function, down_20_reset, 2, 12},
};

#define QUERY_RESEND_MS 200   // "grab уу?" асуултыг давтах интервал
#define QUERY_TIMEOUT_MS 2000 // хариугүй бол SKIP

/* -----------------------------------------------------------------------------
 *  Grab_Strafe_Test — PCB2-ийн "Grab test" (мод 13)-тэй ХАМТ ажиллах (PCB1 тал).
 *    PCB2-оос ирэх strafe командыг (0xB7) дагаж ЗҮҮН/БАРУУН strafe хийнэ (gyro).
 *    Эхлэхэд одоогийн чигийг anchor болгоно. Команд шинэлэг байвал л хөдөлнө.
 *    ⚠ LPMS идэвхтэй байх ёстой (gyro). PCB2-д мод 13, PCB1-д ЭНЭ модыг зэрэг тавина.
 * -----------------------------------------------------------------------------
 */
void Grab_Strafe_Test(void) {
  static uint8_t anchored = 0;
  if (!anchored) {
    Set_Yaw_Anchor(); // одоогийн чиг = strafe барих heading
    anchored = 1;
  }
  if (g_strafe_cmd != 0 && HAL_GetTick() - g_strafe_ms <= GRAB_STRAFE_FRESH_MS) {
    if (g_strafe_cmd == 1)
      Strafe_Gyro(-GRAB_STRAFE); // зүүн (дотроо LPMS_Read)
    else
      Strafe_Gyro(+GRAB_STRAFE); // баруун
  } else {
    brake();
    LPMS_Read(); // зогсолтод ч DMA буферээ хоослох (хоцрохгүй)
  }

  static uint32_t t = 0;
  if (HAL_GetTick() - t >= 100) {
    t = HAL_GetTick();
    const char *d = (g_strafe_cmd == 1)   ? "LEFT"
                    : (g_strafe_cmd == 2) ? "RIGHT"
                                          : "stop";
    colorFill(Black);
    setCursor(10, 2);
    printStr("GRAB STRAFE");
    setCursor(10, 30);
    printStr("cmd:%s", d);
    setScreen();
  }
}

void auto_climb(void) {
  static uint8_t phase = 0; // 0=WAIT 1=RUN 2=DONE
  static const ClimbStep_t *steps = 0;
  static uint8_t nsteps = 0, route = 0;
  static uint8_t si = 0;    // одоогийн step
  static int cnt = 0;       // run_n тоологч
  static uint8_t sub = 0;   // 0=MOVE 1=QUERY 2=GRAB
  static uint8_t moved = 0; // step-ийн эхэнд reset хийсэн эсэх
  static uint8_t gst = 0;   // grab дэд төлөв
  static uint32_t gt0 = 0, qt0 = 0;
  static uint8_t ex = 0; // гарах маневр: 0=зүүн 90° эргэх 1=val5 урагш 2=дуусав
  static uint8_t ex_init = 0; // эргэлтийн Gyro_TurnReset хийсэн эсэх

  switch (phase) {
  case 0: /* ===== WAIT — SET (LOCK) болтол ХӨДЛӨХГҮЙ ===== */
    brake();
    LPMS_Read(); // gyro DMA буферээ хоослож шинэ байлгах (RUN эхлэхэд anchor
                 // зөв)
    if (g_route_set && g_route >= 1 && g_route <= 3) {
      route = g_route;
      if (route == 1) {
        steps = ROUTE1;
        nsteps = 4;
      } else if (route == 2) {
        steps = ROUTE2;
        nsteps = 4;
      } else {
        steps = ROUTE3;
        nsteps = 4;
      }
      si = 0;
      sub = 0;
      moved = 0;
      cnt = 0;
      Set_Yaw_Anchor(); // урагшийн чиг = SET дарсан агшны heading (climb
                        // "урагш")
      phase = 1;
    }
    break;

  case 1:           /* ===== RUN — step бүр: MOVE → QUERY → (GRAB) ===== */
    if (sub == 0) { /* --- MOVE: тухайн блок руу --- */
      if (!moved) {
        steps[si].rst();
        cnt = 0;
        moved = 1;
      }
      if (run_n(steps[si].fn, steps[si].rst, steps[si].count, &cnt)) {
        brake();
        Set_Yaw_Anchor();                  // strafe-ийн чиг барих
        Link_Query_Block(steps[si].block); // "энд grab уу?"
        g_grab_ans_rdy = 0;
        qt0 = HAL_GetTick();
        sub = 1;
      }
    } else if (sub == 1) { /* --- QUERY: PCB2 хариу хүлээх --- */
      brake();
      if (HAL_GetTick() - qt0 >= QUERY_RESEND_MS) {
        Link_Query_Block(steps[si].block);
        qt0 = HAL_GetTick();
      }
      if (g_grab_ans_rdy) {
        g_grab_ans_rdy = 0;
        if (g_grab_ans) {
          gst = 0;
          gt0 = HAL_GetTick();
          sub = 2;
        } // → GRAB
        else {
          si++;
          moved = 0;
          sub = 0;
        } // SKIP
      } else if (HAL_GetTick() - qt0 >= QUERY_TIMEOUT_MS) { // хариугүй → SKIP
        si++;
        moved = 0;
        sub = 0;
      }
    } else { /* --- GRAB: зүүн→PCB2→баруун --- */
      if (grab_step(&gst, &gt0)) {
        si++;
        moved = 0;
        sub = 0;
      }
    }
    if (si >= nsteps)
      phase = 2; // бүх step дуусав
    break;

  default: /* 2 = DONE → ГАРАХ: зүүн90° → val5 MAX урагш → баруун90° → val5 MAX урагш → зогс */
    if (ex == 0) { // (1) зүүн 90° эргэх
      if (!ex_init) {
        Gyro_TurnReset(); // өмнөх эргэлтийн үлдэгдлийг цэвэрлэх
        ex_init = 1;
      }
      if (Turn_Left_90()) { // дуусвал 1
        brake();
        Set_Yaw_Anchor(); // шинэ чиг = "урагш" (Drive_Open үүнийг барина)
        ex = 1;
      }
    } else if (ex == 1) { // (2) val5 == 0 болтол урагш — MAX хурд
      if (val5 == 0) {
        brake();
        Gyro_TurnReset(); // 2 дахь эргэлтэд бэлдэх
        ex = 2;
      } else {
        Drive_Open(EXIT_DRIVE_PWM); // MAX хурдаар урагш (gyro чиг барина)
      }
    } else if (ex == 2) { // (3) баруун −90° эргэ
      if (Turn_Right_90()) {
        brake();
        Set_Yaw_Anchor(); // шинэ чиг = "урагш" (2 дахь урагш явалт)
        ex = 3;
      }
    } else if (ex == 3) { // (4) хоёр ракыг 700 руу ЗЭРЭГ (жолоо зогсож байхад)
      brake();
      if (Rack_GoTo_Sync(EXIT_RACK)) { // хоёул 700-д хүрэхэд
        ex = 4;
      }
    } else if (ex == 4) { // (5) val5 == 0 болтол ДАХИН урагш — MAX (рак 700 барина)
      Rack_Hold(EXIT_RACK, EXIT_RACK);
      if (val5 == 0) {
        brake(); // val5 мэдэрлээ → одоо ракыг 0 руу
        ex = 5;
      } else {
        Drive_Open(EXIT_DRIVE_PWM); // MAX хурдаар урагш (gyro чиг барина)
      }
    } else if (ex == 5) { // (6) хоёр ракыг 0 руу буулгах (жолоо зогсож байхад)
      brake();
      if (Rack_GoTo_Sync(RACK_DOWN)) { // хоёул 0-д хүрэхэд
        ex = 6;
      }
    } else { // (7) бүрэн дуусав
      brake();
    }
    break;
  }

  /* --- OLED (100мс) — MOVE (Seq_Show эзэмшинэ) үед зурахгүй --- */
  static uint32_t t = 0;
  if (HAL_GetTick() - t >= 100) {
    t = HAL_GetTick();
    if (phase == 0) {
      colorFill(Black);
      setCursor(2, 2);
      printStr("AUTO CLIMB");
      setCursor(2, 22);
      printStr("WAIT SET");
      setCursor(2, 44);
      printStr("rx:%u pk:%u", (unsigned)route_rx_n, (unsigned)route_pkt_n);
      setScreen();
    } else if (phase == 1 && sub != 0) { // QUERY / GRAB (MOVE-д Seq_Show)
      colorFill(Black);
      setCursor(2, 2);
      printStr("blk %d r%d", steps[si].block, route);
      setCursor(2, 22);
      printStr("%s", sub == 1   ? "query pcb2"
                     : gst == 0 ? "LEFT"
                     : gst == 1 ? "wait"
                     : gst == 2 ? "RIGHT"
                                : "end");
      setCursor(2, 44);
      printStr("ans:%d dn:%d", g_grab_ans, g_grab_done);
      setScreen();
    } else if (phase == 2) {
      colorFill(Black);
      setCursor(2, 2);
      printStr("AUTO CLIMB");
      setCursor(2, 26);
      printStr("route %d DONE", route);
      setScreen();
    }
  }
}
