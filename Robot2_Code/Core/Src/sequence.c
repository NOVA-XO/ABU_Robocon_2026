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
#define RACK_UP     900     // рак дээд (барих) байрлал
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
static void Rack_Hold(int f_pos, int b_pos) {
    Rack_GoTo(&frontRack, f_pos);
    Rack_GoTo(&backRack,  b_pos);
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
    U20_S0_DRIVE = 0,   // val7 == 0 болтол шулуун урагш
    U20_S0_WAIT,        // 1000 мс хүлээх
    U20_S1_RACK_UP,     // хоёр ракыг UP руу
    U20_S2_DRIVE,       // рак UP барьж, сенсорын нөхцлөөр урагш
    U20_S3_FRONT_DOWN,  // front ракыг DOWN (back UP барих)
    U20_S4_DRIVE,       // front DOWN / back UP барьж, val6 нөхцлөөр урагш
    U20_S4_WAIT,        // 1000 мс хүлээх
    U20_S5_BACK_DOWN,   // back ракыг DOWN
    U20_S6_DRIVE,       // рак DOWN барьж, val3&&val4 нөхцлөөр урагш
    U20_DONE
};

static int u20_state = U20_S0_DRIVE;

uint8_t up_20_function(void) {

    Seq_Show("UP20", u20_state);

    switch (u20_state) {

    // -------- S0: val7 == 0 болтол шулуун урагш --------
    case U20_S0_DRIVE:
        Drive_Straight(-200);
        if (val7 == 0) {
            brake();
            u20_state = U20_S0_WAIT;
        }
        break;

    // -------- S0 хүлээлт: 1000 мс --------
    case U20_S0_WAIT:
        brake();
        if (wait_ms(1000)) {
            u20_state = U20_S1_RACK_UP;
        }
        break;

    // -------- S1: хоёр ракыг UP руу --------
    case U20_S1_RACK_UP: {
        uint8_t f = Rack_GoTo(&frontRack, RACK_UP);
        uint8_t b = Rack_GoTo(&backRack,  RACK_UP);
        if (f && b) {
            u20_state = U20_S2_DRIVE;
        }
        break;
    }

    // -------- S2: рак UP барьж, сенсорын нөхцлөөр урагш --------
    case U20_S2_DRIVE:
        Rack_Hold(RACK_UP, RACK_UP);
        if (val1 && val2 && val3 && val4) {
            Drive_Straight(-300);
        }
        if (!val1 && !val2 && val3 && val4) {
            brake();
            u20_state = U20_S3_FRONT_DOWN;
        }
        break;

    // -------- S3: front ракыг DOWN (back UP барих) --------
    case U20_S3_FRONT_DOWN: {
        uint8_t f = Rack_GoTo(&frontRack, RACK_DOWN);
        Rack_GoTo(&backRack, RACK_UP);
        if (f) {
            u20_state = U20_S4_DRIVE;
        }
        break;
    }

    // -------- S4: front DOWN / back UP барьж, val6 нөхцлөөр урагш --------
    case U20_S4_DRIVE:
        Rack_Hold(RACK_DOWN, RACK_UP);
        if (val6) {
            Drive_Straight(-300);
        } else {
            brake();
            u20_state = U20_S4_WAIT;
        }
        break;

    // -------- S4 хүлээлт: 1000 мс (back-ийг UP барьсаар) --------
    case U20_S4_WAIT:
        Rack_Hold(RACK_DOWN, RACK_UP);
        if (wait_ms(1000)) {
            u20_state = U20_S5_BACK_DOWN;
        }
        break;

    // -------- S5: back ракыг DOWN --------
    case U20_S5_BACK_DOWN: {
        Rack_GoTo(&frontRack, RACK_DOWN);
        uint8_t b = Rack_GoTo(&backRack, RACK_DOWN);
        if (b) {
            u20_state = U20_S6_DRIVE;
        }
        break;
    }

    // -------- S6: рак DOWN барьж, val3&&val4 нөхцлөөр урагш --------
    case U20_S6_DRIVE:
        Rack_Hold(RACK_DOWN, RACK_DOWN);
        if (val3 && val4) {
            Drive_Straight(-300);
        }
        if (!val3 && !val4) {
            brake();
            u20_state = U20_DONE;
        }
        break;

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
 *  down_20_function — 20-оос буух бүтэн дараалал   (ЗАГВАР — бөглөх)
 * =============================================================================
 */
enum {
    D20_S0_START = 0,
    D20_DONE
};

static int d20_state = D20_S0_START;

uint8_t down_20_function(void) {

    Seq_Show("DOWN20", d20_state);

    switch (d20_state) {

    case D20_S0_START:
        // TODO: down_20 үйлдлүүдийг энд бич. Ашиглаж болох барилгын материал:
        //   Drive_Straight(200);                         // ухрах (эерэг = ухрах)
        //   if (val7 == 0) { brake(); d20_state = ...; }  // сенсорын нөхцөл
        //   Rack_GoTo(&frontRack, RACK_UP);               // рак хөдөлгөх
        //   Rack_Hold(RACK_DOWN, RACK_UP);                // рак барих
        //   if (wait_ms(1000)) d20_state = ...;           // non-blocking хүлээлт
        d20_state = D20_DONE;   // одоохондоо шууд дуусгана — бөглөсний дараа устга
        break;

    case D20_DONE:
        brake();
        return 1;
    }

    return 0;
}

void down_20_reset(void) {
    d20_state   = D20_S0_START;
    wait_active = 0;
}


/* =============================================================================
 *  up_40_function — 40-д гарах бүтэн дараалал   (ЗАГВАР — бөглөх)
 * =============================================================================
 */
enum {
    U40_S0_START = 0,
    U40_DONE
};

static int u40_state = U40_S0_START;

uint8_t up_40_function(void) {

    Seq_Show("UP40", u40_state);

    switch (u40_state) {

    case U40_S0_START:
        // TODO: up_40 үйлдлүүдийг энд бич (up_20-той адил хэв маягаар,
        //       гэхдээ 40-д тохирсон рак байрлал / зай / нөхцлөөр).
        u40_state = U40_DONE;   // бөглөсний дараа устга
        break;

    case U40_DONE:
        brake();
        return 1;
    }

    return 0;
}

void up_40_reset(void) {
    u40_state   = U40_S0_START;
    wait_active = 0;
}


/* =============================================================================
 *  down_40_function — 40-оос буух бүтэн дараалал   (ЗАГВАР — бөглөх)
 * =============================================================================
 */
enum {
    D40_S0_START = 0,
    D40_DONE
};

static int d40_state = D40_S0_START;

uint8_t down_40_function(void) {

    Seq_Show("DOWN40", d40_state);

    switch (d40_state) {

    case D40_S0_START:
        // TODO: down_40 үйлдлүүдийг энд бич.
        d40_state = D40_DONE;   // бөглөсний дараа устга
        break;

    case D40_DONE:
        brake();
        return 1;
    }

    return 0;
}

void down_40_reset(void) {
    d40_state   = D40_S0_START;
    wait_active = 0;
}
