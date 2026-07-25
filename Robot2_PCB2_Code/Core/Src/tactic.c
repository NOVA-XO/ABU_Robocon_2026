/* =============================================================================
 *  tactic.c — "Scroll & Zone Tracker" + R2 багана-зам сонгогч (ABU Robocon 2026)
 *
 *  Энэ 3x4 сүлжээ = Meihua Forest-ийн 12 БЛОК (1..12). Дээд мөр = R2 Entrance
 *  (блок 1,2,3), доод мөр = R2 Exit (блок 10,11,12). Оператор setup-ийн үеэр
 *  R2-ийн болон Fake KFS-ийг бүртгэж, төв контроллер руу илгээнэ.
 *
 *  R1 KFS-ийг ТЭМДЭГЛЭХГҮЙ: R1 (гар робот) хурдан тул өөрийн 3 scroll-оо шууд
 *  цуглуулж дуусгана — R2-ийн зам төлөвлөлтөд саад болохгүй.
 *
 *  БОКСЫН ТӨЛӨВ = KFS ТӨРӨЛ (3 төлөв):
 *    "---" хоосон · "R2S" R2 KFS · "FAK" Fake KFS (хүрвэл violation)
 *
 *  R2-ИЙН ХӨДӨЛГӨӨНИЙ ЗАГВАР (бодит робот):
 *    • R2 нэг БАГАНААР шулуун доошоо явна: 1→4→7→10, эсвэл 2→5→8→11, эсвэл
 *      3→6→9→12 (Entrance-ээс Exit хүртэл чигээрээ).
 *    • Явах замдаа тухайн байрнаасаа УРД/ЗҮҮН/БАРУУН талын R2 scroll-ыг авна
 *      (өөрөөр хэлбэл багана + зэргэлдээ зүүн/баруун баганы R2-ууд).
 *    • Багананд Fake байвал тэр багана ХААГДана (Fake дээр зогсож/хүрч болохгүй).
 *
 *  СОНГОЛТ: хаагдаагүй 3 баганаас ХАМГИЙН ОЛОН R2 цуглуулахыг санал болгоно.
 *    Жишээ (Fake=5): багана2 хаагдана → 1-4-7-10 vs 3-6-9-12, аль нь их R2-той.
 *
 *  ХОЁР ХУУДАС (L1 товчоор сэлгэнэ):
 *    • GRID — сүлжээ засах: бокс бүрт "N SSS" (дугаар + төлөв), курсор навигаци.
 *    • PLAN — СҮЛЖЭЭ дээр визуал: сонгосон зам (багана) БҮТЭН ЦАГААН, авах R2
 *      scroll-ыг сумаар (замын зүг) тэмдэглэж харуулна.
 *
 *  УДИРДЛАГА (PS5, PCB1-ээс USART2-оор control_data-д ирнэ):
 *    • Навигаци (GRID) — D-pad ЭСВЭЛ зүүн стик: дээш/доош/зүүн/баруун, wrap-around,
 *      debounce 30ms, auto-repeat (450ms дараа 150ms).
 *    • Төлөв тавих (GRID) — ✕→"---"  ○→"R2S"  △→"FAK"  (rising-edge, давталтгүй).
 *    • L1 — GRID ↔ PLAN хуудас сэлгэх.
 *
 *  UART (өөрчлөлт бүрд, huart4=115200):
 *    "GRID:---,R2S,---;...\n"  — сүлжээний бүтэн зураг
 *    "PLAN:v=1;best=1;route=1,4,7,10;grab=1F,2R,4F,7F;exit=10\n"
 *      route = R2 явах блокууд (Entrance→Exit), grab = авах scroll <блок><тал>
 *      (F=урд/route, L=зүүн, R=баруун), exit = гарах блок
 *
 *  ⚠ ДҮРМИЙН АНХААРУУЛГА: R2 нь тоглоом эхэлсний дараа АВТОМАТ байх ёстой
 *    (rule 10.7). Энэ tracker-ийг зөвхөн setup буюу эхлэхээс ӨМНӨ ашиглана.
 *  ⚠ setScreen() нь ssd1306.c-д I2C timeout-той — салсан дэлгэц дээр гацахгүй.
 * =============================================================================
 */
#include "tactic.h"
#include "ssd1306.h"
#include "default.h"      // send_uart() — huart4 (115200)
#include "general.h"      // Grab_Start()/Grab_Service() — шоо-авах дараалал
#include <stdio.h>        // snprintf()
#include <string.h>       // strlen()
#include <stdbool.h>

/* ---- PS5 удирдлага (main.c-д тодорхойлогдож, USART2 ISR-ээр дүүрдэг) ----- */
extern int control_data[5][4];

/* ---- USART2 (PA2 TX) — PCB1 руу route/ack илгээх ------------------------ */
extern UART_HandleTypeDef huart2;

/* ---- PCB1-ээс ирэх GRAB хүсэлт (main.c-ийн huart2 ISR тавина) ----------- */
extern volatile uint8_t grab_request;

/* ---- PCB1-ээс ирэх QUERY ("энэ блок дээр grab уу?") -------------------- */
extern volatile uint8_t query_request;   // 1 = асуув
extern volatile uint8_t query_block;      // асуусан блок (1..12)

/* =============================================================================
 *  Сүлжээний геометр (пикселээр)
 * =============================================================================
 */
#define T_COLS       3
#define T_ROWS       4
#define BOX_W        42
#define BOX_H        16
#define FONT_W       6
#define FONT_H       8

#define GRID_X0      ((128 - T_COLS * BOX_W) / 2)   // = 1
#define GRID_Y0      ((64  - T_ROWS * BOX_H) / 2)   // = 0

#define ST_AT(i)     (grid[(i) / T_COLS][(i) % T_COLS])   // линейн индекс → төлөв

/* =============================================================================
 *  Боксын төлөвүүд = KFS төрлүүд (R1-гүй, 3 төлөв)
 * =============================================================================
 */
typedef enum {
    ST_EMPTY = 0,   // "---"  хоосон
    ST_FAKE,        // "FAK"  Fake KFS
    ST_R2,          // "R2S"  R2 KFS
    ST_COUNT
} TacticState;

static const char *const STATE_STR[ST_COUNT] = { "---", "FAK", "R2S" };

/* Нүүр товч → шууд онооx төлөв (control_data[1]: [0]=✕ [1]=▭ [2]=△ [3]=○) */
#define ST_NONE  0xFF
static const uint8_t FACE_SET_STATE[4] = { ST_EMPTY, ST_NONE, ST_FAKE, ST_R2 };

/* Навигацийн чиглэл ба хуудас */
enum { DIR_NONE = 0, DIR_UP, DIR_DOWN, DIR_LEFT, DIR_RIGHT };
enum { VIEW_GRID = 0, VIEW_PLAN = 1 };

/* =============================================================================
 *  Товчны цаг хугацаа (ms) ба стикийн босго
 * =============================================================================
 */
#define NAV_DEBOUNCE_MS      30
#define NAV_REPEAT_FIRST_MS  450
#define NAV_REPEAT_NEXT_MS   150
#define SEL_DEBOUNCE_MS      30
#define STICK_TH             60

/* =============================================================================
 *  Модулийн төлөв
 * =============================================================================
 */
static uint8_t grid[T_ROWS][T_COLS];   // static → бүгд ST_EMPTY
static uint8_t cur_row = 0;
static uint8_t cur_col = 0;
static uint8_t g_view  = VIEW_GRID;

typedef struct { uint8_t stable, raw_last; uint32_t change_ms, next_ms; } Nav_t;
typedef struct { uint8_t stable, raw_last; uint32_t change_ms; } Edge_t;

/* Дүрмийн шалгалт */
typedef struct { uint8_t nR2, nFake, nEmpty; bool valid; const char *msg; } Valid_t;

/* Багана бүрийн зам (эхлэл блок = c+1) */
typedef struct {
    bool    blocked;      // багананд Fake байвал true
    uint8_t r2count;      // энэ баганаар авах R2 тоо (багана + зүүн/баруун)
    uint8_t blocks[T_ROWS];
} ColPath_t;

typedef struct { ColPath_t col[T_COLS]; int8_t best; } Plan_t;

static Valid_t g_valid;
static Plan_t  g_plan;

/* =============================================================================
 *  Оролт: навигаци + debounce/repeat
 * =============================================================================
 */
static uint8_t read_nav_raw(void) {
    int lx = control_data[0][0];
    int ly = control_data[0][1];   // дээш = ЭЕРЭГ

    uint8_t up    = control_data[2][2] || (ly >  STICK_TH);
    uint8_t down  = control_data[2][0] || (ly < -STICK_TH);
    uint8_t left  = control_data[2][3] || (lx < -STICK_TH);
    uint8_t right = control_data[2][1] || (lx >  STICK_TH);

    if (up)    return DIR_UP;
    if (down)  return DIR_DOWN;
    if (left)  return DIR_LEFT;
    if (right) return DIR_RIGHT;
    return DIR_NONE;
}

static uint8_t nav_step(Nav_t *n, uint8_t raw) {
    uint32_t now = HAL_GetTick();
    if (raw != n->raw_last) { n->raw_last = raw; n->change_ms = now; }
    if ((now - n->change_ms) < NAV_DEBOUNCE_MS) return DIR_NONE;
    if (raw != n->stable) {
        n->stable = raw;
        if (raw) { n->next_ms = now + NAV_REPEAT_FIRST_MS; return raw; }
        return DIR_NONE;
    }
    if (raw && (int32_t)(now - n->next_ms) >= 0) {
        n->next_ms = now + NAV_REPEAT_NEXT_MS;
        return raw;
    }
    return DIR_NONE;
}

static uint8_t edge_rising(Edge_t *e, uint8_t raw) {
    uint32_t now = HAL_GetTick();
    if (raw != e->raw_last) { e->raw_last = raw; e->change_ms = now; }
    if ((now - e->change_ms) >= SEL_DEBOUNCE_MS && e->stable != raw) {
        e->stable = raw;
        if (raw) return 1;
    }
    return 0;
}

/* =============================================================================
 *  Дүрмийн шалгалт (R2 тоо, Fake тоо, Fake entrance дээр биш)
 * =============================================================================
 */
static void validate(Valid_t *v) {
    int nR2 = 0, nF = 0;
    for (int i = 0; i < 12; i++) {
        if      (ST_AT(i) == ST_R2)   nR2++;
        else if (ST_AT(i) == ST_FAKE) nF++;
    }
    v->nR2 = (uint8_t)nR2; v->nFake = (uint8_t)nF;
    v->nEmpty = (uint8_t)(12 - nR2 - nF);

    v->valid = true;  v->msg = "ready";
    if (nR2 != 4 || nF != 1) { v->valid = false; v->msg = "need R2:4 F:1"; return; }
    for (int i = 0; i < 3; i++)                      // блок 1,2,3 = cells 0,1,2
        if (ST_AT(i) == ST_FAKE) { v->valid = false; v->msg = "Fake on entrance"; return; }
}

/* =============================================================================
 *  plan_compute — 3 баганы шулуун замыг үнэлэх
 *    Багана c (0..2): блокууд (0..3, c). Fake байвал хаагдана. Үгүй бол багана +
 *    зэргэлдээ зүүн/баруун баганы R2-уудыг тоолно (урд/зүүн/баруун авалт).
 *    best = хаагдаагүй, хамгийн олон R2-той багана.
 * =============================================================================
 */
static void plan_compute(Plan_t *out) {
    out->best = -1;
    int bestCount = -1;

    for (int c = 0; c < T_COLS; c++) {
        ColPath_t *p = &out->col[c];
        for (int r = 0; r < T_ROWS; r++) p->blocks[r] = (uint8_t)(r * T_COLS + c + 1);

        /* багананд Fake байвал хаагдана */
        p->blocked = false;
        for (int r = 0; r < T_ROWS; r++)
            if (grid[r][c] == ST_FAKE) { p->blocked = true; break; }

        /* R2 тоолол: багана + зэргэлдээ зүүн/баруун (урд нь баганадаа орсон) */
        p->r2count = 0;
        if (!p->blocked) {
            for (int r = 0; r < T_ROWS; r++)
                for (int cc = c - 1; cc <= c + 1; cc++)
                    if (cc >= 0 && cc < T_COLS && grid[r][cc] == ST_R2) p->r2count++;

            if ((int)p->r2count > bestCount) { bestCount = p->r2count; out->best = (int8_t)c; }
        }
    }
}

static void recompute(void) {
    validate(&g_valid);
    plan_compute(&g_plan);
}

/* =============================================================================
 *  UART илгээх
 * =============================================================================
 */
static void send_grid_uart(void) {
    char buf[64];
    int  n = 0;
    n += snprintf(buf + n, sizeof(buf) - n, "GRID:");
    for (uint8_t r = 0; r < T_ROWS; r++) {
        for (uint8_t c = 0; c < T_COLS; c++) {
            n += snprintf(buf + n, sizeof(buf) - n, "%s", STATE_STR[grid[r][c]]);
            if (c < T_COLS - 1) buf[n++] = ',';
        }
        buf[n++] = (r < T_ROWS - 1) ? ';' : '\n';
    }
    buf[n] = '\0';
    send_uart(buf);
}

/* -----------------------------------------------------------------------------
 *  send_plan_uart — R2-ийн БҮРЭН төлөвлөгөө: аль замаар явах + хаанаас авах.
 *
 *    "PLAN:v=1;best=1;route=1,4,7,10;grab=1F,2R,4F,7F;exit=10\n"
 *      best  = сонгосон баганы эхлэл блок (1/2/3)
 *      route = R2 явах блокууд дараалалтай (Entrance→Exit)
 *      grab  = авах scroll (ДООШЛОХ дарааллаар) — <блок><тал>:
 *                F = замын дагуу урд (route баганад)
 *                L = зүүн багана (best-1),  R = баруун багана (best+1)
 *      exit  = гарах блок (10/11/12)
 *    Зам байхгүй бол бүх талбар "-".
 * -----------------------------------------------------------------------------
 */
static void send_plan_uart(void) {
    char buf[96];
    int  n    = 0;
    int  best = g_plan.best;

    n += snprintf(buf + n, sizeof(buf) - n, "PLAN:v=%d;best=", g_valid.valid ? 1 : 0);
    if (best < 0) {
        n += snprintf(buf + n, sizeof(buf) - n, "-;route=-;grab=-;exit=-");
    } else {
        n += snprintf(buf + n, sizeof(buf) - n, "%d;route=", best + 1);
        for (int r = 0; r < T_ROWS; r++)
            n += snprintf(buf + n, sizeof(buf) - n, "%s%d", r ? "," : "", r * T_COLS + best + 1);

        n += snprintf(buf + n, sizeof(buf) - n, ";grab=");
        int first = 1;
        for (int r = 0; r < T_ROWS; r++) {                 // доошлох дарааллаар
            if (grid[r][best] == ST_R2) {                  // урд (route багана)
                n += snprintf(buf + n, sizeof(buf) - n, "%s%dF", first ? "" : ",", r * T_COLS + best + 1);
                first = 0;
            }
            if (best - 1 >= 0 && grid[r][best - 1] == ST_R2) {         // зүүн багана
                n += snprintf(buf + n, sizeof(buf) - n, "%s%dL", first ? "" : ",", r * T_COLS + (best - 1) + 1);
                first = 0;
            }
            if (best + 1 < T_COLS && grid[r][best + 1] == ST_R2) {     // баруун багана
                n += snprintf(buf + n, sizeof(buf) - n, "%s%dR", first ? "" : ",", r * T_COLS + (best + 1) + 1);
                first = 0;
            }
        }
        if (first) buf[n++] = '-';                          // авах scroll байхгүй

        n += snprintf(buf + n, sizeof(buf) - n, ";exit=%d", 3 * T_COLS + best + 1);
    }
    buf[n++] = '\n';
    buf[n]   = '\0';
    send_uart(buf);
}

/* =============================================================================
 *  Рендер
 * =============================================================================
 */
static void draw_text(uint8_t x, uint8_t y, const char *s, SSD1306_COLOR color) {
    setCursor(x, y);
    while (*s) printCh(*s++, Font_6x8, color);
}

/* GRID хуудас: бокс бүрт "N SSS", курсор урвуу, хамгийн сайн багана доод зураасаар */
static void render_grid(void) {
    colorFill(Black);
    for (uint8_t r = 0; r < T_ROWS; r++) {
        for (uint8_t c = 0; c < T_COLS; c++) {
            uint8_t x = GRID_X0 + c * BOX_W;
            uint8_t y = GRID_Y0 + r * BOX_H;
            char    cell[10];
            snprintf(cell, sizeof(cell), "%d %s", r * T_COLS + c + 1, STATE_STR[grid[r][c]]);
            uint8_t tx = x + (uint8_t)((BOX_W - (int)strlen(cell) * FONT_W) / 2);
            uint8_t ty = y + (BOX_H - FONT_H) / 2;
            if (r == cur_row && c == cur_col) {
                ssd1306_FillRectangle(x, y, x + BOX_W - 1, y + BOX_H - 1, White);
                draw_text(tx, ty, cell, Black);
            } else {
                ssd1306_DrawRectangle(x, y, x + BOX_W - 1, y + BOX_H - 1, White);
                draw_text(tx, ty, cell, White);
            }
        }
    }
    setScreen();
}

/* -----------------------------------------------------------------------------
 *  PLAN хуудас — СҮЛЖЭЭ ДЭЭР визуал:
 *    • Сонгосон зам (хамгийн сайн багана) = БҮТЭН ЦАГААН баганаар (R2 явах зам),
 *      дотор нь блокын дугаар (хар).
 *    • Авах R2 scroll:
 *        - зам дээрх бол   "N R2" (цагаан бокст хараар)
 *        - хажуугийн бол   "R2>" / "<R2" (замын зүг заасан сум) — тэндээс авна
 *        - хүрэхгүй R2 бол  "R2" (сумгүй — энэ замд алдагдана)
 *    • Fake = "FAK" (замд ороогүй, зайлсхийнэ).
 * -----------------------------------------------------------------------------
 */
static void render_plan(void) {
    int best = g_plan.best;
    colorFill(Black);

    for (uint8_t r = 0; r < T_ROWS; r++) {
        for (uint8_t c = 0; c < T_COLS; c++) {
            uint8_t x = GRID_X0 + c * BOX_W;
            uint8_t y = GRID_Y0 + r * BOX_H;
            uint8_t s = grid[r][c];
            char    txt[8];
            txt[0] = '\0';

            if (best >= 0 && c == best) {
                /* Сонгосон зам — бүтэн цагаан */
                ssd1306_FillRectangle(x, y, x + BOX_W - 1, y + BOX_H - 1, White);
                if (s == ST_R2) snprintf(txt, sizeof(txt), "%d R2", r * T_COLS + c + 1);
                else            snprintf(txt, sizeof(txt), "%d",    r * T_COLS + c + 1);
                uint8_t tx = x + (uint8_t)((BOX_W - (int)strlen(txt) * FONT_W) / 2);
                draw_text(tx, y + (BOX_H - FONT_H) / 2, txt, Black);
            } else {
                ssd1306_DrawRectangle(x, y, x + BOX_W - 1, y + BOX_H - 1, White);
                if (s == ST_R2) {
                    bool grabbed = (best >= 0) && (c == best - 1 || c == best + 1);
                    if (grabbed) strcpy(txt, (c < best) ? "R2>" : "<R2");  // сум замын зүг
                    else         strcpy(txt, "R2");                        // хүрэхгүй scroll
                } else if (s == ST_FAKE) {
                    strcpy(txt, "FAK");
                }
                if (txt[0]) {
                    uint8_t tx = x + (uint8_t)((BOX_W - (int)strlen(txt) * FONT_W) / 2);
                    draw_text(tx, y + (BOX_H - FONT_H) / 2, txt, White);
                }
            }
        }
    }
    setScreen();
}

/* =============================================================================
 *  PCB1 РУУ ЗАМ (route) ДАМЖУУЛАХ — USART2 (PA2 TX)
 *
 *  ЗӨВХӨН SET (Options) дарахад л илгээнэ — PREVIEW БАЙХГҮЙ. Тиймээс SET
 *  хийтэл PCB1-д ЯМАР Ч тактик (route) байхгүй.
 *    Багц: [0xB3][route][0x0A]   (route = хамгийн сайн багана 1/2/3)
 *
 *  ⚠ Setup-д л дуудна (тоглоом эхлэхээс ӨМНӨ). Дүрэм 10.7: тоглоом эхэлсний
 *    дараа R2 руу тушаал явуулахгүй. Энэ нь самбар ХООРОНДЫН wired холбоо тул
 *    12.11 (R1↔R2 утасгүй)-т ч хамаарахгүй.
 *  ⚠ Утас: PCB2 PA2 (TX) → PCB1 PA3 (RX),  GND нийтлэг.
 * =============================================================================
 */
static void send_route_link(void) {
    uint8_t route  = (g_plan.best >= 0) ? (uint8_t)(g_plan.best + 1) : 0;
    uint8_t pkt[3] = { 0xB3, route, 0x0A };   // зөвхөн SET (баталгаажсан)
    HAL_UART_Transmit(&huart2, pkt, 3, 20);
}

/* PCB1 руу "GRAB дуусгав" ack (USART2 TX): [0xB5][0x00][0x0A] */
static void send_grab_done(void) {
    uint8_t pkt[3] = { 0xB5, 0x00, 0x0A };
    HAL_UART_Transmit(&huart2, pkt, 3, 20);
}

/* PCB1-ийн "энэ блок дээр grab уу?"-д хариулах: [0xB6][1/0][0x0A] */
static void send_query_ans(uint8_t grab) {
    uint8_t pkt[3] = { 0xB6, grab, 0x0A };
    HAL_UART_Transmit(&huart2, pkt, 3, 20);
}

/* -----------------------------------------------------------------------------
 *  query_grab_decision — блок N дээр grab хийх үү (тактикаар)?
 *    Дүрэм: route дахь N-ийн ДАРААГИЙН блокт R2 scroll байвал энд бэлдэж grab.
 *      block N → row=(N-1)/3, col=(N-1)%3;  дараагийн блок = grid[row+1][col].
 *    (Хажуугийн багана/L-R сонголтыг хожим нэмж болно.)
 * -----------------------------------------------------------------------------
 */
static uint8_t query_grab_decision(uint8_t block) {
    if (block < 1 || block > 12) return 0;
    uint8_t row = (uint8_t)((block - 1) / T_COLS);
    uint8_t col = (uint8_t)((block - 1) % T_COLS);
    if (row + 1 >= T_ROWS) return 0;                 // сүүлийн мөр — дараагийн блок алга
    return (grid[row + 1][col] == ST_R2) ? 1 : 0;
}

/* =============================================================================
 *  Tactic_Task — тактикийн НЭГ давталт (selectMode-оос дуудна)
 * =============================================================================
 */
void Tactic_Task(void) {
    static Nav_t   nav      = {0};
    static Edge_t  face[4]  = {0};
    static Edge_t  view_btn = {0};
    static Edge_t  set_btn  = {0};   // Options → тактик SET
    static uint8_t  set_armed  = 0;  // Options-ийг нэг тавьсны дараа л SET зэвсэглэнэ
    static uint8_t  set_route  = 0;  // сүүлд SET хийсэн route (мессежинд)
    static uint32_t set_msg_ms = 0;  // SET мессежийн эхлэл (0 = идэвхгүй)
    static uint32_t grab_msg_ms = 0; // GRAB мессежийн эхлэл (0 = идэвхгүй)
    static uint8_t inited   = 0;
    static uint8_t dirty    = 1;

    if (!inited) {
        recompute();
        send_grid_uart();
        send_plan_uart();
        inited = 1;
    }

    /* --- PCB1-ээс GRAB ирвэл: ЗӨВХӨН front-down-20 грабыг ажиллуулна ---
     *  grab_front_down_20_f (non-blocking) бүр давталтад дуудна; дуусмагц PCB1
     *  руу "done" (0xB5). ЭРТ (мессежийн return-оос өмнө) дуудна — эс бөгөөс зогсоно.
     *  ⚠ Одоохондоо нэг л төрлийн шоо (front-down-20) холбов.                    */
    static uint8_t grabbing = 0;
    if (grab_request) {
        grab_request = 0;
        if (!grabbing) {
            grab_front_down_20_f_reset();
            grabbing = 1;
        }
    }
    if (grabbing && grab_front_down_20_f()) { // дараалал дуусав → PCB1 руу ack
        send_grab_done();
        grabbing = 0;
        grab_msg_ms = HAL_GetTick();
        dirty = 1;
    }

    /* --- PCB1 "энэ блок дээр grab уу?" асуувал: тактикаар шийдэж хариулна --- */
    if (query_request) {
        query_request = 0;
        send_query_ans(query_grab_decision(query_block));
    }

    /* --- L1: GRID ↔ PLAN хуудас сэлгэх --- */
    if (edge_rising(&view_btn, (uint8_t)control_data[3][0])) {
        g_view = (g_view == VIEW_GRID) ? VIEW_PLAN : VIEW_GRID;
        dirty  = 1;
    }

    /* --- Options: тактикийг SET → PCB1 руу БАТАЛГААЖСАН route (0xB3) ---
     *  Зам байгаа (best багана тодорхой, best>=0) бол илгээнэ. Найдвартай
     *  байдлын үүднээс 3 удаа. OLED дээр ~1.2сек баталгаажуулалт харуулна —
     *  ингэснээр товч ажилласан эсэх нь харагдана (утас шалгахад тус болно).    */
    if (control_data[4][1] == 0) set_armed = 1;   // Options тавигдсан → одоо зэвсэглэв
    if (set_armed && edge_rising(&set_btn, (uint8_t)control_data[4][1])) {
        uint8_t route = (g_plan.best >= 0) ? (uint8_t)(g_plan.best + 1) : 0;
        if (route >= 1) {
            send_route_link();     // ЗӨВХӨН энд — SET дарж байж л тактик явна
            send_route_link();     // найдвартай байдлын үүднээс 3 удаа
            send_route_link();
            set_route  = route;
            set_msg_ms = HAL_GetTick();
            dirty      = 1;
        }
    }

    /* --- Засварлах зөвхөн GRID хуудсанд --- */
    if (g_view == VIEW_GRID) {
        switch (nav_step(&nav, read_nav_raw())) {
            case DIR_UP:    cur_row = (cur_row + T_ROWS - 1) % T_ROWS; dirty = 1; break;
            case DIR_DOWN:  cur_row = (cur_row + 1)          % T_ROWS; dirty = 1; break;
            case DIR_LEFT:  cur_col = (cur_col + T_COLS - 1) % T_COLS; dirty = 1; break;
            case DIR_RIGHT: cur_col = (cur_col + 1)          % T_COLS; dirty = 1; break;
            default: break;
        }

        for (int i = 0; i < 4; i++) {
            uint8_t ns = FACE_SET_STATE[i];
            if (ns == ST_NONE) continue;
            if (edge_rising(&face[i], (uint8_t)control_data[1][i])) {
                if (grid[cur_row][cur_col] != ns) {
                    grid[cur_row][cur_col] = ns;
                    recompute();
                    send_grid_uart();
                    send_plan_uart();
                    dirty = 1;
                }
            }
        }
    }

    /* --- GRAB баталгаажуулалтын мессеж (~1.2сек) --- */
    if (grab_msg_ms) {
        if (dirty) {
            colorFill(Black);
            setCursor(10, 14); printStr("GRAB DONE");
            setCursor(10, 38); printStr("sun -90 hold");
            setScreen();
            dirty = 0;
        }
        if (HAL_GetTick() - grab_msg_ms >= 1200) { grab_msg_ms = 0; dirty = 1; }
        return;
    }

    /* --- SET баталгаажуулалтын мессеж (~1.2сек, grid-ийн оронд) --- */
    if (set_msg_ms) {
        if (dirty) {
            colorFill(Black);
            setCursor(10, 14); printStr("SET route %d", set_route);
            setCursor(10, 38); printStr("sent -> PCB1");
            setScreen();
            dirty = 0;
        }
        if (HAL_GetTick() - set_msg_ms >= 1200) { set_msg_ms = 0; dirty = 1; }
        return;
    }

    if (dirty) {
        if (g_view == VIEW_GRID) render_grid();
        else                     render_plan();
        dirty = 0;
    }
}
