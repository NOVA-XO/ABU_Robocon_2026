/* =============================================================================
 *  sequence.c — Бүтэн үйлдлийн блокууд (non-blocking sequence blocks)
 *
 *  red.c / blue.c тал давтан ашиглах бүтэн дарааллын блокууд.
 *  Блок бүр өөрийн төлөвтэй (static) state machine:
 *    - Дотоод while давталт, HAL_Delay БАЙХГҮЙ.
 *    - Нэг дуудалт = нэг алхам, тэр даруй буцна.
 *    - Хугацааны хүлээлтийг wait_ms()-ээр non-blocking хийнэ.
 *
 *  Блокууд:  up_20_function   — 20-д гарах  (red.c-ийн хуучин Sequence-ийн логик)
 *            down_20_function — 20-оос буух (загвар — бөглөх)
 *            up_40_function   — 40-д гарах  (загвар — бөглөх)
 *            down_40_function — 40-оос буух (загвар — бөглөх)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "sequence.h"

extern int counter[4];   // encoder тоолуурууд (OLED дээр харуулах)

/* ---- Рак байрлалууд ------------------------------------------------------ */
#define RACK_UP    1000     // рак дээд (барих) байрлал — 20-ийн блокууд
#define RACK_UP_40 1950     // рак дээд байрлал — 40-ийн блокууд (pos_max-тай тэнцүү)
#define RACK_DOWN     0     // рак доод байрлал


/* =============================================================================
 *  НИЙТЛЭГ ТУСЛАХ ФУНКЦУУД (бүх блок хуваалцана)
 * =============================================================================
 */

/* Нэг зэрэг зөвхөн НЭГ блок ажиллана гэж үзвэл нэг хүлээлтийн таймер хангалттай */
static uint8_t  wait_active = 0;
static uint32_t wait_t0     = 0;

/* -----------------------------------------------------------------------------
 *  wait_ms — non-blocking хугацааны хүлээлт (return: 1 = дүүрсэн, 0 = хүлээж байна)
 * -----------------------------------------------------------------------------
 */
__attribute__((unused))   // дараагийн алхмуудад хэрэглэгдэнэ (хүлээлт)
static uint8_t wait_ms(uint32_t ms) {
    if (!wait_active) {
        wait_active = 1;
        wait_t0     = HAL_GetTick();
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
__attribute__((unused))   // дараагийн алхмуудад хэрэглэгдэнэ (рак барих)
static void Rack_Hold(int f_pos, int b_pos) {
    Rack_GoTo(&frontRack, f_pos);
    Rack_GoTo(&backRack,  b_pos);
}

/* -----------------------------------------------------------------------------
 *  Drive_Open — gyro-ГҮЙ (open-loop) шулуун явах: 4 моторт ижил PWM
 *    pwm: урагш = СӨРӨГ, ухрах = ЭЕРЭГ (Drive_Straight-тай ижил конвенц).
 *  Чиг баримжаа засварлахгүй тул удаан хугацаанд явбал бага зэрэг хазайж болно.
 * -----------------------------------------------------------------------------
 */
static void Drive_Open(int pwm) {
    motor_control(1, pwm);   // FL
    motor_control(2, pwm);   // FR
    motor_control(3, pwm);   // RL
    motor_control(4, pwm);   // RR
}

/* -----------------------------------------------------------------------------
 *  Seq_Show — Блокийн нэр, төлөв, encoder, сенсорыг OLED дээр харуулах (100мс-д нэг)
 * -----------------------------------------------------------------------------
 */
static void Seq_Show(const char *name, int st) {
    static uint32_t t = 0;
    if (HAL_GetTick() - t < 100) return;
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
    U20_S0_DRIVE = 0,   // val5 == 0 болтол шулуун урагш
    U20_S1_RACK_UP,     // хоёр ракыг 1000 (RACK_UP) руу
    U20_S2_DRIVE,       // рак UP БАРЬЖ, val7 == 0 болтол урагш
    U20_S3_FRONT_DOWN,  // front ракыг 0 руу (ХҮЧТЭЙ); back-ийг UP барих
    U20_S4_DRIVE,       // front 0 / back UP БАРЬЖ, val4 == 0 болтол урагш
    U20_S5_BACK_DOWN,   // back ракыг 0 руу (ХҮЧТЭЙ); front 0-д барих
    U20_S6_DRIVE,       // хоёр рак 0 БАРЬЖ, val3 == 0 болтол урагш
    U20_DONE
    /* дараагийн алхмуудыг ЭНД нэмнэ (U20_S7_...) */
};

static int u20_state = U20_S0_DRIVE;

uint8_t up_20_function(void) {

    Seq_Show("UP20", u20_state);

    switch (u20_state) {

    // -------- S0: val5 == 0 болтол шулуун урагш (gyro-ГҮЙ, open-loop) --------
    case U20_S0_DRIVE:
        Drive_Open(-200);                // урагш (конвенц: урагш = сөрөг); gyro ашиглахгүй
        if (val5 == 0) {
            brake();
            u20_state = U20_S1_RACK_UP;  // → хоёр рак дээш
        }
        break;

    // -------- S1: хоёр ракыг 1000 (RACK_UP) руу; хоёул хүрэхэд дараагийн алхам --------
    case U20_S1_RACK_UP:
        if (Rack_GoTo_Sync(RACK_UP)) {   // ХОЁУЛАА зэрэгцүүлж дээш
            u20_state = U20_S2_DRIVE;    // → рак барьж урагш
        }
        break;

    // -------- S2: рак UP БАРЬЖ, val7 == 0 болтол урагш --------
    //   val7-ийг ЗӨВХӨН энд шалгана — S1 дуусаж, рак аль хэдийн 1000 дээр гарсан
    //   тул "рак дээшлэхээс өмнө val7 == 0" гэдэг худал триггер боломжгүй.
    case U20_S2_DRIVE:
        Rack_Hold(RACK_UP, RACK_UP);     // ракыг 1000-д БАРЬСААР (таталцлаар унахгүй)
        Drive_Open(-200);                // урагш (gyro-гүй)
        if (val7 == 0) {
            brake();
            frontRack.land_soft = 0;     // front-ийг ТУСАД НЬ буулгах тул ХҮЧТЭЙ
            u20_state = U20_S3_FRONT_DOWN;
        }
        break;

    // -------- S3: front ракыг 0 руу (ХҮЧТЭЙ); back-ийг UP барих --------
    case U20_S3_FRONT_DOWN: {
        uint8_t f = Rack_GoTo(&frontRack, RACK_DOWN);
        Rack_GoTo(&backRack, RACK_UP);   // back-ийг 1000-д барьсаар
        if (f) {
            u20_state = U20_S4_DRIVE;    // → front 0 / back UP барьж урагш
        }
        break;
    }

    // -------- S4: front 0 / back UP БАРЬЖ, val4 == 0 болтол урагш --------
    case U20_S4_DRIVE:
        Rack_Hold(RACK_DOWN, RACK_UP);   // front 0 (coast), back 1000 барих
        Drive_Open(-200);                // урагш (gyro-гүй)
        if (val4 == 0) {
            brake();
            backRack.land_soft = 0;      // back-ийг ТУСАД НЬ буулгах тул ХҮЧТЭЙ
            u20_state = U20_S5_BACK_DOWN;
        }
        break;

    // -------- S5: back ракыг 0 руу (ХҮЧТЭЙ); front 0-д барих --------
    case U20_S5_BACK_DOWN: {
        Rack_GoTo(&frontRack, RACK_DOWN);            // front 0-д (coast)
        uint8_t b = Rack_GoTo(&backRack, RACK_DOWN);
        if (b) {
            u20_state = U20_S6_DRIVE;    // → хоёр рак 0 барьж урагш
        }
        break;
    }

    // -------- S6: хоёр рак 0 БАРЬЖ, val3 == 0 болтол урагш --------
    case U20_S6_DRIVE:
        Rack_Hold(RACK_DOWN, RACK_DOWN); // хоёулаа 0 (coast)
        Drive_Open(-200);                // урагш (gyro-гүй)
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
    u20_state   = U20_S0_DRIVE;
    wait_active = 0;
}


/* =============================================================================
 *  down_20_function — 20-оос БУУХ дараалал (шат доошоо; УХРАХГҮЙ, урагшаа явна)
 *  20 см өндөр шатнаас буулгах.
 * =============================================================================
 */
enum {
    D20_S0_DRIVE = 0,   // val6 == 1 болтол шулуун урагш
    D20_S1_FRONT_UP,    // front ракыг 1000 руу
    D20_S2_DRIVE,       // front 1000 БАРЬЖ, val3 == 1 болтол урагш
    D20_S3_DRIVE,       // front 1000 БАРЬЖ, val4 == 1 болтол урагш
    D20_S4_BACK_UP,     // back ракыг 1000 руу (front-ийг 1000 барих)
    D20_S5_DRIVE,       // хоёр рак 1000 БАРЬЖ, 500мс урагш
    D20_S6_DOWN,        // хоёр ракыг ЗЭРЭГ 0 руу (ЗӨӨЛӨН), зогсох
    D20_DONE
    /* дараагийн алхмуудыг ЭНД нэмнэ (D20_S7_...) */
};

static int d20_state = D20_S0_DRIVE;

uint8_t down_20_function(void) {

    Seq_Show("DOWN20", d20_state);

    switch (d20_state) {

    // -------- S0: val6 == 1 болтол шулуун урагш (gyro-ГҮЙ; УХРАХГҮЙ) --------
    case D20_S0_DRIVE:
        Drive_Open(-200);                // урагш (конвенц: урагш = сөрөг)
        if (val6 == 1) {
            brake();
            d20_state = D20_S1_FRONT_UP; // → front рак дээш
        }
        break;

    // -------- S1: front ракыг 1000 руу; хүрэхэд дараагийн алхам --------
    case D20_S1_FRONT_UP: {
        uint8_t f = Rack_GoTo(&frontRack, RACK_UP);
        if (f) {
            d20_state = D20_S2_DRIVE;    // → front барьж урагш
        }
        break;
    }

    // -------- S2: front 1000 БАРЬЖ, val3 == 1 болтол урагш --------
    case D20_S2_DRIVE:
        Rack_Hold(RACK_UP, RACK_DOWN);   // front 1000 барих, back 0 (coast)
        Drive_Open(-200);                // урагш (gyro-гүй)
        if (val3 == 1) {
            brake();
            d20_state = D20_S3_DRIVE;    // → val4 хүртэл үргэлжлүүлнэ
        }
        break;

    // -------- S3: front 1000 БАРЬЖ, val4 == 1 болтол урагш --------
    case D20_S3_DRIVE:
        Rack_Hold(RACK_UP, RACK_DOWN);   // front 1000 барих, back 0 (coast)
        Drive_Open(-200);                // урагш (gyro-гүй)
        if (val4 == 1) {
            brake();
            d20_state = D20_S4_BACK_UP;  // → back рак дээш
        }
        break;

    // -------- S4: back ракыг 1000 руу (front-ийг 1000 барих) --------
    case D20_S4_BACK_UP: {
        Rack_GoTo(&frontRack, RACK_UP);              // front 1000 барьсаар
        uint8_t b = Rack_GoTo(&backRack, RACK_UP);
        if (b) {
            d20_state = D20_S5_DRIVE;    // → 500мс урагш
        }
        break;
    }

    // -------- S5: хоёр рак 1000 БАРЬЖ, 500мс урагш --------
    case D20_S5_DRIVE:
        Rack_Hold(RACK_UP, RACK_UP);     // хоёулаа 1000 барих
        Drive_Open(-200);                // урагш (gyro-гүй)
        if (wait_ms(1000)) {
            brake();
            frontRack.land_soft = 1;     // ХОЁУЛАА ЗЭРЭГ 0 → ЗӨӨЛӨН буулт
            backRack.land_soft  = 1;
            d20_state = D20_S6_DOWN;
        }
        break;

    // -------- S6: хоёр ракыг ЗЭРЭГ 0 руу (ЗӨӨЛӨН); хоёул хүрэхэд зогсоно --------
    case D20_S6_DOWN:
        if (Rack_GoTo_Sync(RACK_DOWN)) {   // ХОЁУЛАА зэрэгцүүлж доош
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
    d20_state   = D20_S0_DRIVE;
    wait_active = 0;
}


/* =============================================================================
 *  up_40_function — 40-д гарах бүтэн дараалал   (ЗАГВАР — бөглөх)
 * =============================================================================
 */
/* up_20-той ЯГ АДИЛ, зөвхөн рак 1950 (RACK_UP_40) руу гардаг (1000 биш). */
enum {
    U40_S0_DRIVE = 0,   // val5 == 0 болтол шулуун урагш
    U40_S1_RACK_UP,     // хоёр ракыг 1950 (RACK_UP_40) руу
    U40_S2_DRIVE,       // рак UP БАРЬЖ, val7 == 0 болтол урагш
    U40_S3_FRONT_DOWN,  // front ракыг 0 руу (ХҮЧТЭЙ); back-ийг UP барих
    U40_S4_DRIVE,       // front 0 / back UP БАРЬЖ, val4 == 0 болтол урагш
    U40_S5_BACK_DOWN,   // back ракыг 0 руу (ХҮЧТЭЙ); front 0-д барих
    U40_S6_DRIVE,       // хоёр рак 0 БАРЬЖ, val3 == 0 болтол урагш
    U40_DONE
};

static int u40_state = U40_S0_DRIVE;

uint8_t up_40_function(void) {

    Seq_Show("UP40", u40_state);

    switch (u40_state) {

    // -------- S0: val5 == 0 болтол шулуун урагш --------
    case U40_S0_DRIVE:
        Drive_Open(-200);                   // урагш (gyro-гүй)
        if (val5 == 0) {
            brake();
            u40_state = U40_S1_RACK_UP;
        }
        break;

    // -------- S1: хоёр ракыг 1950 руу; хоёул хүрэхэд дараагийн алхам --------
    case U40_S1_RACK_UP:
        if (Rack_GoTo_Sync(RACK_UP_40)) {   // ХОЁУЛАА зэрэгцүүлж дээш
            u40_state = U40_S2_DRIVE;
        }
        break;

    // -------- S2: рак 1950 БАРЬЖ, val7 == 0 болтол урагш --------
    case U40_S2_DRIVE:
        Rack_Hold(RACK_UP_40, RACK_UP_40);  // ракыг 1950-д БАРЬСААР
        Drive_Open(-200);
        if (val7 == 0) {
            brake();
            frontRack.land_soft = 0;        // front-ийг ТУСАД НЬ буулгах тул ХҮЧТЭЙ
            u40_state = U40_S3_FRONT_DOWN;
        }
        break;

    // -------- S3: front ракыг 0 руу (ХҮЧТЭЙ); back-ийг UP барих --------
    case U40_S3_FRONT_DOWN: {
        uint8_t f = Rack_GoTo(&frontRack, RACK_DOWN);
        Rack_GoTo(&backRack, RACK_UP_40);   // back-ийг 1950-д барьсаар
        if (f) {
            u40_state = U40_S4_DRIVE;
        }
        break;
    }

    // -------- S4: front 0 / back 1950 БАРЬЖ, val4 == 0 болтол урагш --------
    case U40_S4_DRIVE:
        Rack_Hold(RACK_DOWN, RACK_UP_40);   // front 0 (coast), back 1950 барих
        Drive_Open(-200);
        if (val4 == 0) {
            brake();
            backRack.land_soft = 0;         // back-ийг ТУСАД НЬ буулгах тул ХҮЧТЭЙ
            u40_state = U40_S5_BACK_DOWN;
        }
        break;

    // -------- S5: back ракыг 0 руу (ХҮЧТЭЙ); front 0-д барих --------
    case U40_S5_BACK_DOWN: {
        Rack_GoTo(&frontRack, RACK_DOWN);            // front 0-д (coast)
        uint8_t b = Rack_GoTo(&backRack, RACK_DOWN);
        if (b) {
            u40_state = U40_S6_DRIVE;
        }
        break;
    }

    // -------- S6: хоёр рак 0 БАРЬЖ, val3 == 0 болтол урагш --------
    case U40_S6_DRIVE:
        Rack_Hold(RACK_DOWN, RACK_DOWN);    // хоёулаа 0 (coast)
        Drive_Open(-200);
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
    u40_state   = U40_S0_DRIVE;
    wait_active = 0;
}


/* =============================================================================
 *  down_40_function — 40-оос БУУХ дараалал
 *  down_20-той ЯГ АДИЛ, гэхдээ рак 1950 (RACK_UP_40) руу гардаг (1000 биш).
 * =============================================================================
 */
enum {
    D40_S0_DRIVE = 0,   // val6 == 1 болтол шулуун урагш
    D40_S1_FRONT_UP,    // front ракыг 1950 руу
    D40_S2_DRIVE,       // front 1950 БАРЬЖ, val3 == 1 болтол урагш
    D40_S3_DRIVE,       // front 1950 БАРЬЖ, val4 == 1 болтол урагш
    D40_S4_BACK_UP,     // back ракыг 1950 руу (front-ийг 1950 барих)
    D40_S5_DRIVE,       // хоёр рак 1950 БАРЬЖ, 500мс урагш
    D40_S6_DOWN,        // хоёр ракыг ЗЭРЭГ 0 руу (ЗӨӨЛӨН), зогсох
    D40_DONE
};

static int d40_state = D40_S0_DRIVE;

uint8_t down_40_function(void) {

    Seq_Show("DOWN40", d40_state);

    switch (d40_state) {

    // -------- S0: val6 == 1 болтол шулуун урагш (gyro-ГҮЙ; УХРАХГҮЙ) --------
    case D40_S0_DRIVE:
        Drive_Open(-200);                   // урагш (конвенц: урагш = сөрөг)
        if (val6 == 1) {
            brake();
            d40_state = D40_S1_FRONT_UP;    // → front рак дээш
        }
        break;

    // -------- S1: front ракыг 1950 руу; хүрэхэд дараагийн алхам --------
    case D40_S1_FRONT_UP: {
        uint8_t f = Rack_GoTo(&frontRack, RACK_UP_40);
        if (f) {
            d40_state = D40_S2_DRIVE;       // → front барьж урагш
        }
        break;
    }

    // -------- S2: front 1950 БАРЬЖ, val3 == 1 болтол урагш --------
    case D40_S2_DRIVE:
        Rack_Hold(RACK_UP_40, RACK_DOWN);   // front 1950 барих, back 0 (coast)
        Drive_Open(-200);                   // урагш (gyro-гүй)
        if (val3 == 1) {
            brake();
            d40_state = D40_S3_DRIVE;       // → val4 хүртэл үргэлжлүүлнэ
        }
        break;

    // -------- S3: front 1950 БАРЬЖ, val4 == 1 болтол урагш --------
    case D40_S3_DRIVE:
        Rack_Hold(RACK_UP_40, RACK_DOWN);   // front 1950 барих, back 0 (coast)
        Drive_Open(-200);                   // урагш (gyro-гүй)
        if (val4 == 1) {
            brake();
            d40_state = D40_S4_BACK_UP;     // → back рак дээш
        }
        break;

    // -------- S4: back ракыг 1950 руу (front-ийг 1950 барих) --------
    case D40_S4_BACK_UP: {
        Rack_GoTo(&frontRack, RACK_UP_40);              // front 1950 барьсаар
        uint8_t b = Rack_GoTo(&backRack, RACK_UP_40);
        if (b) {
            d40_state = D40_S5_DRIVE;       // → 500мс урагш
        }
        break;
    }

    // -------- S5: хоёр рак 1950 БАРЬЖ, 500мс урагш --------
    case D40_S5_DRIVE:
        Rack_Hold(RACK_UP_40, RACK_UP_40);  // хоёулаа 1950 барих
        Drive_Open(-200);                   // урагш (gyro-гүй)
        if (wait_ms(500)) {
            brake();
            frontRack.land_soft = 1;        // ХОЁУЛАА ЗЭРЭГ 0 → ЗӨӨЛӨН буулт
            backRack.land_soft  = 1;
            d40_state = D40_S6_DOWN;
        }
        break;

    // -------- S6: хоёр ракыг ЗЭРЭГ 0 руу (ЗӨӨЛӨН); хоёул хүрэхэд зогсоно --------
    case D40_S6_DOWN:
        if (Rack_GoTo_Sync(RACK_DOWN)) {   // ХОЁУЛАА зэрэгцүүлж доош
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
    d40_state   = D40_S0_DRIVE;
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
typedef void    (*seq_reset_fn)(void);

static uint8_t run_n(seq_fn fn, seq_reset_fn reset, int n, int *cnt) {
    if (fn()) {                       // нэг мөчлөг дуусав
        (*cnt)++;
        if (*cnt >= n) { *cnt = 0; return 1; }   // бүгд дуусав
        reset();                      // дараагийн мөчлөгт бэлдэнэ
    }
    return 0;
}

/* -----------------------------------------------------------------------------
 *  climb_1 — ЗАМ 1:  up_40 ×1  →  down_20 ×1  →  up_20 ×1  →  down_20 ×2
 * -----------------------------------------------------------------------------
 */
enum { C1_UP40 = 0, C1_DOWN20A, C1_UP20, C1_DOWN20B, C1_DONE };
static int c1_state = C1_UP40;
static int c1_cnt   = 0;

uint8_t climb_1_function(void) {
    switch (c1_state) {
    case C1_UP40:
        if (run_n(up_40_function,   up_40_reset,   1, &c1_cnt)) { down_20_reset(); c1_state = C1_DOWN20A; }
        break;
    case C1_DOWN20A:
        if (run_n(down_20_function, down_20_reset, 1, &c1_cnt)) { up_20_reset();   c1_state = C1_UP20; }
        break;
    case C1_UP20:
        if (run_n(up_20_function,   up_20_reset,   1, &c1_cnt)) { down_20_reset(); c1_state = C1_DOWN20B; }
        break;
    case C1_DOWN20B:
        if (run_n(down_20_function, down_20_reset, 2, &c1_cnt)) { c1_state = C1_DONE; }
        break;
    case C1_DONE:
        brake();
        return 1;
    }
    return 0;
}

void climb_1_reset(void) {
    c1_state = C1_UP40;
    c1_cnt   = 0;
    up_40_reset();          // эхний блок
}

/* -----------------------------------------------------------------------------
 *  climb_2 — ЗАМ 2:  up_20 ×3  →  down_20 ×1  →  down_40 ×1
 * -----------------------------------------------------------------------------
 */
enum { C2_UP20 = 0, C2_DOWN20, C2_DOWN40, C2_DONE };
static int c2_state = C2_UP20;
static int c2_cnt   = 0;

uint8_t climb_2_function(void) {
    switch (c2_state) {
    case C2_UP20:
        if (run_n(up_20_function,   up_20_reset,   3, &c2_cnt)) { down_20_reset(); c2_state = C2_DOWN20; }
        break;
    case C2_DOWN20:
        if (run_n(down_20_function, down_20_reset, 1, &c2_cnt)) { down_40_reset(); c2_state = C2_DOWN40; }
        break;
    case C2_DOWN40:
        if (run_n(down_40_function, down_40_reset, 1, &c2_cnt)) { c2_state = C2_DONE; }
        break;
    case C2_DONE:
        brake();
        return 1;
    }
    return 0;
}

void climb_2_reset(void) {
    c2_state = C2_UP20;
    c2_cnt   = 0;
    up_20_reset();          // эхний блок
}

/* -----------------------------------------------------------------------------
 *  climb_3 — ЗАМ 3:  up_40 ×1  →  up_20 ×1  →  down_20 ×3
 * -----------------------------------------------------------------------------
 */
enum { C3_UP40 = 0, C3_UP20, C3_DOWN20, C3_DONE };
static int c3_state = C3_UP40;
static int c3_cnt   = 0;

uint8_t climb_3_function(void) {
    switch (c3_state) {
    case C3_UP40:
        if (run_n(up_40_function,   up_40_reset,   1, &c3_cnt)) { up_20_reset();   c3_state = C3_UP20; }
        break;
    case C3_UP20:
        if (run_n(up_20_function,   up_20_reset,   1, &c3_cnt)) { down_20_reset(); c3_state = C3_DOWN20; }
        break;
    case C3_DOWN20:
        if (run_n(down_20_function, down_20_reset, 3, &c3_cnt)) { c3_state = C3_DONE; }
        break;
    case C3_DONE:
        brake();
        return 1;
    }
    return 0;
}

void climb_3_reset(void) {
    c3_state = C3_UP40;
    c3_cnt   = 0;
    up_40_reset();          // эхний блок
}
