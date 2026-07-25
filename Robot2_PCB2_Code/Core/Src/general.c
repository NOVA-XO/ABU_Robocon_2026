/* =============================================================================
 *  general.c — Хөдөлгөөний өндөр түвшний удирдлага
 *              (mecanum жолоо, серво, соленоид, ADC, USB)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "general.h"
#include <stdio.h>  // sprintf()
#include <stdlib.h> // abs() — runner() дахь mecanum нормчлолд хэрэглэнэ

/* ---- main.c дахь periphery handle-ууд ---------------------------------- */
extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2; // PA2/PA3 — 1-р PCB-тэй холбоо

extern ADC_HandleTypeDef hadc1;

/* ---- Глобал төлөв ------------------------------------------------------- */
extern int counter[4];         // encoder тоолуурууд
extern int control_data[5][4]; // джойстик/товчлуурын өгөгдөл
extern int timer;              // TIM7-оос нэмэгддэг зөөлөн тоолуур

extern uint8_t usb_received_value;
extern uint8_t usb_new_data_flag;

/* ---- USART2 холбоо (main.c-д тодорхойлогдсон, RX ISR бичнэ) ------------- */
extern volatile uint32_t link_byte_n; // ирсэн байт
extern volatile uint32_t link_pkt;    // бүрэн задарсан багц
extern volatile uint32_t link_ms;     // сүүлийн багцын агшин

/* ---- Серво: одоогийн өнцөг (Servo_Preset_Control бичнэ, OLED уншина) ------
 *  SERVO_DEG_MAX нь general.h-д (Servo_SetDeg-ийн дуудагчид хэрэгтэй).       */
static int g_servo_deg =
    SERVO_DEG_MAX; // АСААХАД 180°-оос эхэлнэ (0..SERVO_DEG_MAX)

/* =============================================================================
 *  MECANUM УДИРДЛАГА
 * =============================================================================
 */
#define DEADZONE 10  // joystick үхмэл бүс
#define SPEED_GAIN 8 // -100..100 утгыг PWM болгох коэффициент

/* -----------------------------------------------------------------------------
 *  applyDeadzone — joystick утгыг deadzone-оор шүүх (жижиг утгыг 0 болгоно)
 * -----------------------------------------------------------------------------
 */
int applyDeadzone(int v) {
  if (v > -DEADZONE && v < DEADZONE)
    return 0;
  return v;
}

/* -----------------------------------------------------------------------------
 *  runner — Джойстикоор mecanum хөдөлгөөн (inverse kinematics + нормчлол)
 * -----------------------------------------------------------------------------
 */
void runner(void) {
  // --- joystick унших (-100..100) ---
  int Vy = applyDeadzone(control_data[0][1]); // зүүн стик Y → урагш/хойш
  int Vx = applyDeadzone(control_data[0][0]); // зүүн стик X → хажуу (strafe)
  int W = applyDeadzone(control_data[0][2]);  // баруун стик X → эргэлт

  // joystick Y нь дээш түлхэхэд сөрөг тул урагш = эерэг болгож эргүүлнэ
  Vy = -Vy;

  // --- Mecanum inverse kinematics ---
  int fl = Vy - Vx - W; // Урд-Зүүн    (мотор 1)
  int fr = Vy + Vx + W; // Урд-Баруун  (мотор 2)
  int rl = Vy + Vx - W; // Хойд-Зүүн   (мотор 3)
  int rr = Vy - Vx + W; // Хойд-Баруун (мотор 4)

  // --- Нормчлол: хамгийн их утга 100 хэтэрвэл бүгдийг хувь тэнцүүлж багасгана
  // ---
  int m = abs(fl);
  if (abs(fr) > m)
    m = abs(fr);
  if (abs(rl) > m)
    m = abs(rl);
  if (abs(rr) > m)
    m = abs(rr);
  if (m > 100) {
    fl = fl * 100 / m;
    fr = fr * 100 / m;
    rl = rl * 100 / m;
    rr = rr * 100 / m;
  }

  // --- PWM болгож моторт өгөх ---
  motor_control(1, fl * SPEED_GAIN); // Урд-Зүүн
  motor_control(2, fr * SPEED_GAIN); // Урд-Баруун
  motor_control(3, rl * SPEED_GAIN); // Хойд-Зүүн
  motor_control(4, rr * SPEED_GAIN); // Хойд-Баруун
}

/* -----------------------------------------------------------------------------
 *  Товчны DEBOUNCE + edge/repeat таних туслах
 *    - Түүхий 1/0 хэлбэлзлийг BTN_DEBOUNCE_MS тогтвортой болтол шүүнэ.
 *    - Цэвэр даралт (rising edge) бүрт НЭГ алхам.
 *    - Дараад барьж байвал BTN_REPEAT_MS тутам давтана.
 * -----------------------------------------------------------------------------
 */
#define BTN_DEBOUNCE_MS 25 // тогтвортой гэж үзэх хугацаа
#define BTN_REPEAT_MS 120  // барьж байхад давтах интервал

typedef struct {
  uint8_t stable;     // debounced төлөв
  uint8_t raw_last;   // сүүлийн түүхий уншилт
  uint32_t change_ms; // түүхий төлөв өөрчлөгдсөн хугацаа
  uint32_t repeat_ms; // сүүлийн repeat алхмын хугацаа
} Btn_t;

// return: энэ дуудалтад "нэг алхам хий" гэвэл 1 (rising edge эсвэл repeat)
//   unused: одоогоор зөвхөн btn_rising ашиглагдаж байна (давталттай хувилбар нь
//   утга алхмаар тааруулахад хэрэг болно — тиймээс хасалгүй үлдээв).
__attribute__((unused)) static uint8_t btn_step(Btn_t *b, uint8_t raw) {
  uint32_t now = HAL_GetTick();

  if (raw != b->raw_last) { // түүхий төлөв өөрчлөгдвөл цаг эхлүүлнэ
    b->raw_last = raw;
    b->change_ms = now;
  }

  // тогтвортой болтол хүлээж байж л debounced төлвийг шинэчилнэ
  if ((now - b->change_ms) >= BTN_DEBOUNCE_MS && b->stable != raw) {
    b->stable = raw;
    if (raw) { // цэвэр шинэ даралт → эхний алхам
      b->repeat_ms = now;
      return 1;
    }
  }

  if (b->stable &&
      (now - b->repeat_ms) >= BTN_REPEAT_MS) { // барьж байвал давтах
    b->repeat_ms = now;
    return 1;
  }

  return 0;
}

/* -----------------------------------------------------------------------------
 *  btn_rising — Debounce хийсэн ЦЭВЭР ШИНЭ даралт (0→1), ДАВТАЛТГҮЙ
 *
 *  btn_step нь товчийг барьж байхад BTN_REPEAT_MS тутам давтдаг. Preset-д
 *  давталт хэрэггүй тул давталтгүй хувилбар.
 * -----------------------------------------------------------------------------
 */
static uint8_t btn_rising(Btn_t *b, uint8_t raw) {
  uint32_t now = HAL_GetTick();

  if (raw != b->raw_last) { // түүхий төлөв өөрчлөгдвөл цаг эхлүүлнэ
    b->raw_last = raw;
    b->change_ms = now;
  }

  if ((now - b->change_ms) >= BTN_DEBOUNCE_MS && b->stable != raw) {
    b->stable = raw;
    if (raw)
      return 1; // зөвхөн 0 → 1 шилжилтэд
  }

  return 0;
}

/* =============================================================================
 *  СЕРВО УДИРДЛАГА — ГРАДУСААР (1 серво, htim1 CH1)
 *
 *    Triangle △  →  0° ↔ 180° СЭЛГЭХ (debounce, давталтгүй)
 *
 *  Өөр товч ашиглахгүй — 0/180-аас өөр байрлал хэрэггүй тул алхмаар тааруулах
 *  (✕ +5° / ○ 0° / ▭ 180°) хувилбарыг хассан.
 *
 *  Одоогийн өнцөг g_servo_deg-д хадгалагдаж OLED дээр харагдана.
 *  АСААХАД 180°-оос эхэлнэ (эхлэлийн байрлал).
 *
 *  ⚠ controlServo нь 0..SERVO_UNIT_MAX-аас гарсан утга авбал errorFunction()
 *    руу орж роботыг БҮРЭН ГАЦААНА (buzzer-ийн хязгааргүй гогцоо). Тиймээс
 *    градусыг хааж, дараа нь л unit болгож хөрвүүлнэ.
 * =============================================================================
 */
#define SERVO_UNIT_MAX                                                         \
  24000 // controlServo-ийн дээд утга (= SERVO_DEG_MAX градус)

/* -----------------------------------------------------------------------------
 *  servo_deg_to_units — градус (0..180) → controlServo-ийн unit (0..24000)
 *  Хязгаарыг ЭНД хаана — errorFunction()-аас сэргийлнэ.
 * -----------------------------------------------------------------------------
 */
static int servo_deg_to_units(int deg) {
  if (deg < 0)
    deg = 0;
  if (deg > SERVO_DEG_MAX)
    deg = SERVO_DEG_MAX;
  return (deg * SERVO_UNIT_MAX) / SERVO_DEG_MAX; // 180° → 24000
}

/* -----------------------------------------------------------------------------
 *  Servo_SetDeg — сервог өнцгөөр тавих (автомат дараалалд ашиглана)
 *
 *  g_servo_deg-ийг МӨН шинэчилнэ — эс тэгвээс △ toggle нь одоогийн байрлалаас
 *  шийддэг тул автомат дараалал сервог зөөсний дараа буруу тал руу үсэрнэ.
 * -----------------------------------------------------------------------------
 */
void Servo_SetDeg(int deg) {
  if (deg < 0)
    deg = 0;
  if (deg > SERVO_DEG_MAX)
    deg = SERVO_DEG_MAX;
  g_servo_deg = deg;
  controlServo(true, servo_deg_to_units(deg)); // htim1 CH1
}

/* -----------------------------------------------------------------------------
 *  Servo_Home — эхлэлийн байрлалд (180°) тавих.  Init-д ЗААВАЛ дуудна.
 *
 *  g_servo_deg нь 180-аар эхэлдэг ч энэ нь зөвхөн ПРОГРАММЫН утга — hardware
 *  руу бичигдэхгүй. Өмнө нь зөвхөн Servo_Preset_Buttons-ийн inited блок бичдэг
 *  байсан тул автомат дараалал (тэр функцийг дуудахгүй) үед серво асахдаа
 *  хаана байснаа тэндээ үлдэж, g_servo_deg-тэй зөрж байв.
 * -----------------------------------------------------------------------------
 */
void Servo_Home(void) { Servo_SetDeg(SERVO_HOME_DEG); }

/*  Товчны логик — OLED-ГҮЙ (дэлгэцийг дуудагч нь өөрөө зурна).
 *  Буцаах: одоогийн өнцөг.                                                 */
int Servo_Preset_Buttons(void) {
  static Btn_t bTri = {0};
  static uint8_t inited = 0;

  if (!inited) { // асаахад мэдэгдэж буй байрлалд тавина
    controlServo(true, servo_deg_to_units(g_servo_deg));
    inited = 1;
  }

  /* --- △ : нэг даралт = 0° ↔ 180° сэлгэх (debounce, давталтгүй) --- */
  if (btn_rising(&bTri, (uint8_t)control_data[1][2])) {
    g_servo_deg = (g_servo_deg >= SERVO_DEG_MAX / 2) ? 0 : SERVO_DEG_MAX;
    controlServo(true, servo_deg_to_units(g_servo_deg)); // htim1 CH1
  }

  return g_servo_deg;
}

void Servo_Preset_Control(void) {
  int deg = Servo_Preset_Buttons();

  /* --- OLED (100мс тутам): өнцөг + timer-т бичигдсэн бодит unit --- */
  static uint32_t t = 0;
  if (HAL_GetTick() - t >= 100) {
    t = HAL_GetTick();
    colorFill(Black);
    setCursor(2, 2);
    printStr("SERVO");
    setCursor(2, 22);
    printStr("%d deg", deg);
    setCursor(2, 42);
    printStr("u:%d", servo_deg_to_units(deg));
    setScreen();
  }
}

/* =============================================================================
 *  СОЛЕНОИД 1 — D-Up товчоор TOGGLE (нэг даралт = нэг сэлгэлт)
 *
 *    control_data[2][2] = D-Up  →  соленоид 1
 *
 *  Товч дарах бүрд соленоид ON ↔ OFF сэлгэнэ. Дараад БАРЬЖ байхад давтахгүй
 *  (зөвхөн 0→1 ирмэг дээр), механик чичиргээг BTN_DEBOUNCE_MS-ээр шүүнэ.
 *  Асаахад мэдэгдэж буй OFF төлөвт тавина.
 * =============================================================================
 */
#define SOLENOID1_BTN 2 // control_data[2][SOLENOID1_BTN] = D-Up
#define SOLENOID1_NUM 1 // controlSolenoid-ийн дугаар

void Solenoid_Control(void) {
  static Btn_t btn = {0};
  static bool state = false;
  static uint8_t inited = 0;

  if (!inited) { // программын төлөвтэй нийцүүлнэ
    controlSolenoid(SOLENOID1_NUM, false);
    inited = 1;
  }

  if (btn_rising(&btn, (uint8_t)control_data[2][SOLENOID1_BTN])) {
    state = !state; // ON ↔ OFF
    controlSolenoid(SOLENOID1_NUM, state);
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
uint16_t Read_PC3(void) {
  uint16_t value = 0;
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
    value = HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);
  return value;
}

/* -----------------------------------------------------------------------------
 *  Read_PC3_OLED — ADC утгыг OLED дэлгэц дээр харуулах
 * -----------------------------------------------------------------------------
 */
void Read_PC3_OLED(void) {
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
uint8_t USB_GetValue(void) { return usb_received_value; }

/* -----------------------------------------------------------------------------
 *  USB_HasNewData — Шинэ өгөгдөл ирсэн эсэх (1 = бий, 0 = үгүй)
 * -----------------------------------------------------------------------------
 */
uint8_t USB_HasNewData(void) { return usb_new_data_flag; }

/* -----------------------------------------------------------------------------
 *  USB_ClearFlag — flag-ыг цэвэрлэх (өгөгдлийг уншиж дууссаны дараа дуудна)
 * -----------------------------------------------------------------------------
 */
void USB_ClearFlag(void) { usb_new_data_flag = 0; }

/* -----------------------------------------------------------------------------
 *  USB_Show_OLED — USB утгыг OLED дэлгэц дээр төлөвийн текстээр харуулах
 * -----------------------------------------------------------------------------
 */
void USB_Show_OLED(void) {
  colorFill(Black);

  setCursor(10, 10);
  printStr("USB: %d", usb_received_value);

  setCursor(10, 40);

  // Утганд тохирох текст харуулах
  if (usb_received_value == '0' || usb_received_value == 0) {
    printStr("Status: hooson");
  } else if (usb_received_value == '1' || usb_received_value == 1) {
    printStr("Status: fake");
  } else if (usb_received_value == '2' || usb_received_value == 2) {
    printStr("Status: real");
  } else {
    printStr("Status: ???");
  }

  setScreen();
}

/* =============================================================================
 *  Link_Recv_Test — 1-р PCB-ээс ирэх PS5 удирдлагыг шалгах (USART2, PA3 = RX)
 *
 *  PS5 нэг ширхэг, PCB1-д залгаастай. PCB1 нь ESP32-оос ирсэн 23 байтыг яг тэр
 *  хэвээр дамжуулж, энд ps5_feed() задлаад control_data[5][4]-г дүүргэнэ.
 *  Тиймээс энэ самбарын БҮХ код (runner, серво, соленоид) ӨӨРЧЛӨЛТГҮЙ ажиллана.
 *
 *  Утас: PCB1 PA2 (TX) → PCB2 PA3 (RX),  БАС GND ↔ GND (заавал).
 *
 *  Уншиж дүгнэх — b (байт) ба p (багц) хоёрыг ЗААВАЛ хамт харна:
 *    b өсөж, p өсөж    → бүх зүйл зөв
 *    b өсөж, p ЗОГССОН → байт ирж байгаа ч багц бүрдэхгүй: baud зөрөх юм уу
 *                        байт алдагдаж байна (framing хэзээ ч бүтэхгүй)
 *    b ч өсөхгүй       → утас/чиглэл буруу, GND алга, эсвэл PCB1 асаагүй
 *    STALE             → өмнө ирж байсан, одоо тасарсан
 * =============================================================================
 */
void Link_Recv_Test(void) {
  static uint32_t t = 0;
  if (HAL_GetTick() - t < 100)
    return;
  t = HAL_GetTick();

  /* volatile-ыг НЭГ л удаа уншина — ISR дунд нь өөрчилвөл дэлгэц зөрнө */
  uint32_t b = link_byte_n;
  uint32_t p = link_pkt;
  uint32_t age = HAL_GetTick() - link_ms;

  colorFill(Black);
  setCursor(2, 2);
  printStr("LINK RX");
  setCursor(2, 18);
  printStr("b:%lu p:%lu", (unsigned long)b, (unsigned long)p);
  setCursor(2, 34);
  printStr("LX:%d LY:%d", control_data[0][0], control_data[0][1]);
  setCursor(2, 50);
  if (p == 0 && b == 0)
    printStr("NO DATA");
  else if (p == 0)
    printStr("BYTES, NO PKT");
  else if (age > 500)
    printStr("STALE %lums", (unsigned long)age);
  else
    printStr("OK");
  setScreen();
}

/* =============================================================================
 *  PCB2_Manual — 2 дахь PCB-ийн ҮНДСЭН ГАРЫН УДИРДЛАГА
 *
 *  Удирдлага НЭГ PS5-аас ирнэ: PCB1 ESP32-оос PS5 багцаа аваад USART2-оор
 *  энэ самбар руу дамжуулна. huart2 callback тэр байтуудыг ps5_feed-д оруулж
 *  control_data-г дүүргэдэг (main.c үз). Тиймээс доорх бүх товч/стик PCB1-ийн
 *  дамжуулсан утгаас уншигдана.
 *
 *  МОТОР (товч дарж байхад эргэнэ, тавихад зогсоно):
 *    D-Up  → мотор 6 (+)      D-Down  → мотор 6 (−)
 *    D-Left → мотор 5 (+)     D-Right → мотор 5 (−)
 *
 *  СОЛЕНОИД (нэг даралт = toggle, debounce):
 *    ▭ → 1    ○ → 2    △ → 5
 *
 *  ⚠ Мотор эсрэг чиглэлд явбал товчны хосыг соль эсвэл PWM #define-ийн тэмдгийг
 *    эргүүл.
 * =============================================================================
 */
/* Товч дарж байхад мотор эргэх тогтмол PWM (мотор бүрт тусад нь тааруул) */
#define PCB2_M5_PWM 1000 // D-Left (+) / D-Right (−)
#define PCB2_M6_PWM 1000 // D-Up   (+) / D-Down  (−)

/* -----------------------------------------------------------------------------
 *  hold_axis — хоёр товчийг нэг тэнхлэг болгох (дарж байхад эргэнэ)
 *    Хоёуланг нь ЗЭРЭГ дарвал 0. Debounce хэрэггүй: тогтмол PWM тул түүхий
 *    хэлбэлзэл нэг л loop-д мотор бага зэрэг цохих төдий.
 * -----------------------------------------------------------------------------
 */
static int hold_axis(uint8_t pos, uint8_t neg, int pwm) {
  if (pos && !neg)
    return pwm;
  if (neg && !pos)
    return -pwm;
  return 0;
}

void PCB2_Manual(void) {
  static Btn_t bs[3] = {0};    // соленоид 1,2,5-ийн rising-edge мэдэгч
  static bool st[3] = {false}; //   тэдгээрийн одоогийн toggle төлөв
  static uint8_t inited = 0;

  if (!inited) { // программын төлөвтэй нийцүүлнэ
    controlSolenoid(1, false);
    controlSolenoid(2, false);
    controlSolenoid(5, false);
    inited = 1;
  }

  /* --- Мотор: ТОВЧ дарж байхад эргэнэ ---
   *   control_data[2]: [0]=D-Down [1]=D-Right [2]=D-Up [3]=D-Left
   *   D-Up  → M6 (+)    D-Down  → M6 (−)
   *   D-Left → M5 (+)   D-Right → M5 (−)                                   */
  int m6 = hold_axis((uint8_t)control_data[2][2], // D-Up   (+)
                     (uint8_t)control_data[2][0], // D-Down (−)
                     PCB2_M6_PWM);
  int m5 = hold_axis((uint8_t)control_data[2][3], // D-Left  (+)
                     (uint8_t)control_data[2][1], // D-Right (−)
                     PCB2_M5_PWM);
  motor_control(5, m5);
  motor_control(6, m6);

  /* --- Соленоид (нэг даралт = toggle, debounce) ---
   *   ▭ → 1   ○ → 2   △ → 5   (control_data[1]: [0]=✕ [1]=▭ [2]=△ [3]=○) */
  static const uint8_t sol_num[3] = {1, 2, 5};
  const uint8_t sol_btn[3] = {
      (uint8_t)control_data[1][1], // ▭ Square   → 1
      (uint8_t)control_data[1][3], // ○ Circle   → 2
      (uint8_t)control_data[1][2], // △ Triangle → 5
  };
  for (int i = 0; i < 3; i++) {
    if (btn_rising(&bs[i], sol_btn[i])) {
      st[i] = !st[i];
      controlSolenoid(sol_num[i], st[i]);
    }
  }

  /* --- OLED (100мс тутам) --- */
  static uint32_t t = 0;
  if (HAL_GetTick() - t >= 100) {
    t = HAL_GetTick();

    char sol[4]; // соленоидын төлөв: "125" / '-'
    sol[0] = st[0] ? '1' : '-';
    sol[1] = st[1] ? '2' : '-';
    sol[2] = st[2] ? '5' : '-';
    sol[3] = '\0';

    colorFill(Black);
    setCursor(2, 2);
    printStr("PCB2 MANUAL");
    setCursor(2, 18);
    printStr("M5:%d E0:%d", m5, counter[0]);
    setCursor(2, 34);
    printStr("M6:%d E1:%d", m6, counter[1]);
    setCursor(2, 50);
    printStr("SOL:%s", sol);
    setScreen();
  }
}

/* =============================================================================
 *  SUN GEAR — Мотор 5 нарны араа (encoder0-оор өнцгийн байрлал)
 *
 *  ХЭМЖИЖ ТОГТООСОН: араа бүтэн эргэхэд encoder0 = 6000 count = 360°.
 *  ХЭМЖСЭН: эерэг motor PWM → counter0 БУУРНА (тиймээс SUN_DIR = -1).
 *  0 цэг = АСААХ агшны байрлал (counter[0] == 0).
 *
 *  УДИРДЛАГА (joystick jog + байрлал барих P-хяналт):
 *    • Зүүн стик Y-ийг түлхэхэд арааг эргүүлнэ (JOG); тавихад тэр байрлалыг
 *      санаж, P-controller-аар чанга барина (HOLD).
 *    • Товч:  △→0°   ○→90°   ▭→−90°   ✕→180°   (P-хяналт тийш нь аваачна).
 *    • HOLD үед алдаа ±SUN_HOLD_DB (count) дотор орвол мотор ЗОГСоно → error
 * ~1-2.
 *
 *  ⚠ ЭНЭ САМБАРТ LPMS БАЙХГҮЙ тул UART4 (115200) СУЛ — sun_gear тааруулгын
 *    телеметрийг тэндүүр цацна:  "SUN mode tgt=.. cnt=.. err=.. pwm=.."
 * =============================================================================
 */
#define SUN_CNT_PER_REV 6000 // encoder0: 1 бүтэн эргэлт (ХЭМЖСЭН)
#define SUN_DIR (-1)         // ХЭМЖСЭН: эерэг motor PWM → counter0 БУУРНА
#define SUN_JOG_MAX 1000     // joystick бүрэн хазайлт (±100) → энэ PWM
#define SUN_HOLD_KP 4        // байрлал барих P коэфф (PWM нэг count алдаанд)
#define SUN_HOLD_MIN 130     // засах доод PWM (статик үрэлтийг давах)
#define SUN_HOLD_MAX 1000    // засах дээд PWM (хол зайд бүрэн хурд)
#define SUN_HOLD_DB 2        // ±энэ count дотор — мотор ЗОГС (error 1-2 барих)

static int sun_last_pwm = 0; // сүүлд моторт өгсөн PWM (зөвхөн телеметрт)

/* deg → encoder0 count */
static int sun_deg_to_cnt(int deg) {
  return (int)((long)deg * SUN_CNT_PER_REV / 360);
}

/* -----------------------------------------------------------------------------
 *  sun_hold — target руу байрлал барих P-хяналт. Буцаах нь "logical PWM"
 *    (эерэг = counter0-г ӨСГӨХ). Deadband дотор 0 буцаана → чичрэхгүй.
 * -----------------------------------------------------------------------------
 */
static int sun_hold(int target) {
  int err = target - counter[0];
  int a = err < 0 ? -err : err;
  if (a <= SUN_HOLD_DB)
    return 0;                // тэвчих бүсэд — зогс
  int mag = SUN_HOLD_KP * a; // алдаатай пропорциональ
  if (mag < SUN_HOLD_MIN)
    mag = SUN_HOLD_MIN; // үрэлт давах доод хүч
  if (mag > SUN_HOLD_MAX)
    mag = SUN_HOLD_MAX; // дээд хязгаар
  return err > 0 ? mag : -mag;
}

/* -----------------------------------------------------------------------------
 *  sun_gear — joystick jog + байрлал барих P-хяналт (+ ✕/△/○ preset)
 *    Стик түлхэхэд эргүүлж (JOG), тавихад сүүлийн байрлалыг чанга барина
 * (HOLD). 100мс тутам UART4 руу телеметр цацаж, OLED шинэчилнэ.
 * -----------------------------------------------------------------------------
 */
void sun_gear(void) {
  static int hold_target = 0;
  static uint8_t inited = 0;
  static Btn_t btr = {0}, bc = {0}, bsq = {0}, bx = {0};

  if (!inited) {
    hold_target = counter[0];
    inited = 1;
  } // асаах байрлалаа бари

  /* --- Preset товч: зорилтот өнцөг тавина --- */
  if (btn_rising(&btr, (uint8_t)control_data[1][2]))
    hold_target = sun_deg_to_cnt(0); // △ → 0°
  if (btn_rising(&bc, (uint8_t)control_data[1][3]))
    hold_target = sun_deg_to_cnt(90); // ○ → 90°
  if (btn_rising(&bsq, (uint8_t)control_data[1][1]))
    hold_target = sun_deg_to_cnt(-90); // ▭ → −90°
  if (btn_rising(&bx, (uint8_t)control_data[1][0]))
    hold_target = sun_deg_to_cnt(180); // ✕ → 180°

  /* --- Joystick jog эсвэл байрлал барих --- */
  int stick = applyDeadzone(control_data[0][1]); // зүүн стик Y
  int logical;                                   // эерэг = counter0 өсгөх
  const char *mode;
  if (stick != 0) {
    logical = stick * SUN_JOG_MAX / 100; // -1000..1000 (jog)
    hold_target = counter[0];            // тавихад эндээ барина
    mode = "JOG";
  } else {
    logical = sun_hold(hold_target); // байрлал барих P-хяналт
    mode = "HOLD";
  }

  int pwm = logical * SUN_DIR; // logical → бодит мотор чиглэл
  motor_control(5, pwm);
  sun_last_pwm = pwm;

  static uint32_t t = 0;
  if (HAL_GetTick() - t >= 100) {
    t = HAL_GetTick();

    /* --- UART4 (115200) телеметр --- */
    int err = hold_target - counter[0];
    char line[80];
    int n = snprintf(line, sizeof(line),
                     "SUN\t%s\ttgt=%d\tcnt=%d\terr=%d\tpwm=%d\n", mode,
                     hold_target, counter[0], err, sun_last_pwm);
    HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)n, 20);

    /* --- OLED --- */
    colorFill(Black);
    setCursor(2, 2);
    printStr("SUN %s", mode);
    setCursor(2, 18);
    printStr("tgt:%d", hold_target);
    setCursor(2, 34);
    printStr("cnt:%d", counter[0]);
    setCursor(2, 50);
    printStr("err:%d", err);
    setScreen();
  }
}

/* =============================================================================
 *  MOON GEAR — Мотор 6 / encoder1 — joystick jog + байрлал барих P-хяналт
 *
 *  ХЭМЖСЭН: эерэг motor PWM → counter1 БУУРНА (тиймээс MOON_DIR = -1).
 *  3 PRESET ТӨЛӨВ (encoder1 count-оор ШУУД):  ▭ → -148   △ → 0   ○ → 140
 *    Баруун стик Y-ээр jog; тавихад сүүлийн байрлалыг P-хяналтаар барина.
 *    Алдаа ±MOON_HOLD_DB (=1) count дотор орвол мотор ЗОГСоно (±1 хүлцэл).
 *
 *  ⚠ UART4 (115200) телеметр:  "MOON mode tgt=.. enc1=.. err=.. pwm=.."
 * =============================================================================
 */
#define MOON_DIR (-1)      // ХЭМЖСЭН: эерэг motor PWM → counter1 БУУРНА
#define MOON_JOG_MAX 1000  // баруун стик бүрэн хазайлт (±100) → энэ PWM
#define MOON_HOLD_KP 8     // байрлал барих P коэфф (PWM нэг count алдаанд)
#define MOON_HOLD_MIN 130  // засах доод PWM (статик үрэлтийг давах)
#define MOON_HOLD_MAX 1000 // засах дээд PWM
#define MOON_HOLD_DB 3     // ±энэ count дотор — мотор ЗОГС (±1 хүлцэл)
#define MOON_HOLD_KI                                                           \
  0.06f // integral — ачаа/үрэлт даван зорилтод ЯГ хүрэх (steady-state алдаа
        // арилгах)
#define MOON_HOLD_IMAX 600 // integral хувь нэмрийн дээд (anti-windup)

/* 3 preset төлөв (encoder1 count) */
#define MOON_POS_SQ (-145) // ▭ Square
#define MOON_POS_TR 0      // △ Triangle
#define MOON_POS_CI 140    // ○ Circle

static int moon_last_pwm = 0; // сүүлд моторт өгсөн PWM (зөвхөн телеметрт)

/* target руу байрлал барих PI-хяналт (logical PWM, эерэг = counter1 ӨСГӨХ)
   Integral — зорилтод ойртоход P буурч ачаа/үрэлтэнд гацдгийг арилгана
   (steady-state алдаа). Зорилт солигдвол integral reset. */
static int moon_hold(int target) {
  static int i_tgt = 0x7FFFFFFF;
  static float integ = 0.0f;
  if (target != i_tgt) {
    integ = 0.0f;
    i_tgt = target;
  } // зорилт солигдвол reset
  int err = target - counter[1];
  int a = err < 0 ? -err : err;
  if (a <= MOON_HOLD_DB) {
    integ = 0.0f;
    return 0;
  } // тэвчих бүс — зогс, integ ТЭГЛЭ (windup/чичрэлт зогсооно)
  integ += (float)err * MOON_HOLD_KI; // integral хураах
  if (integ > (float)MOON_HOLD_IMAX)
    integ = (float)MOON_HOLD_IMAX; // anti-windup
  if (integ < -(float)MOON_HOLD_IMAX)
    integ = -(float)MOON_HOLD_IMAX;
  int out = MOON_HOLD_KP * err + (int)integ; // P + I
  int m = out < 0 ? -out : out;
  if (m < MOON_HOLD_MIN)
    m = MOON_HOLD_MIN;
  if (m > MOON_HOLD_MAX)
    m = MOON_HOLD_MAX;
  return out > 0 ? m : -m;
}

void moon_gear(void) {
  static int hold_target = 0;
  static uint8_t inited = 0;
  static Btn_t bsq = {0}, btr = {0}, bc = {0};

  if (!inited) {
    hold_target = counter[1];
    inited = 1;
  } // асаах байрлалаа бари

  /* --- Preset товч: 3 төлөв (encoder1 count) --- */
  if (btn_rising(&btr, (uint8_t)control_data[1][2]))
    hold_target = MOON_POS_TR; // △ → 0
  if (btn_rising(&bc, (uint8_t)control_data[1][3]))
    hold_target = MOON_POS_CI; // ○ → 140
  if (btn_rising(&bsq, (uint8_t)control_data[1][1]))
    hold_target = MOON_POS_SQ; // ▭ → -148

  /* --- Joystick jog эсвэл байрлал барих --- */
  int stick = applyDeadzone(control_data[0][3]); // баруун стик Y
  int logical;                                   // эерэг = counter1 өсгөх
  const char *mode;
  if (stick != 0) {
    logical = stick * MOON_JOG_MAX / 100; // -1000..1000 (jog)
    hold_target = counter[1];             // тавихад эндээ барина
    mode = "JOG";
  } else {
    logical = moon_hold(hold_target); // байрлал барих
    mode = "HOLD";
  }

  int pwm = logical * MOON_DIR; // logical → бодит мотор чиглэл
  motor_control(6, pwm);
  moon_last_pwm = pwm;

  static uint32_t t = 0;
  if (HAL_GetTick() - t >= 100) {
    t = HAL_GetTick();

    /* --- UART4 (115200) телеметр --- */
    int err = hold_target - counter[1];
    char line[80];
    int n = snprintf(line, sizeof(line),
                     "MOON\t%s\ttgt=%d\tenc1=%d\terr=%d\tpwm=%d\n", mode,
                     hold_target, counter[1], err, moon_last_pwm);
    HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)n, 20);

    /* --- OLED --- */
    colorFill(Black);
    setCursor(2, 2);
    printStr("MOON %s", mode);
    setCursor(2, 18);
    printStr("tgt:%d", hold_target);
    setCursor(2, 34);
    printStr("enc1:%d", counter[1]);
    setCursor(2, 50);
    printStr("err:%d", err);
    setScreen();
  }
}

/* =============================================================================
 *  SUN-MOON — sun (m5/enc0) ба moon (m6/enc1)-ыг НЭГ мод дотор ТУСАД нь удирдах
 *
 *  Хоёр гар зэрэг ажиллана, гэхдээ командыг ТУСАД нь өгнө:
 *    • SUN  ← face товч:  △→0°   ○→90°   ▭→−90°   ✕→180°
 *    • MOON ← D-pad:      Up→140   Down→−148   Left→0    (encoder1 count)
 *  Товч дарахад тухайн гар зорилтоо аваад P-hold-оор байрлалаа барина
 *  (sun_hold / moon_hold-ыг ашиглана).
 *
 *  ⚠ ДООРХ preset утгуудыг өөрийн хүссэнээр ТААРУУЛ.
 *  UART4 (115200):  "SM  S tgt=.. cnt=.. e=..  M tgt=.. cnt=.. e=.."
 * =============================================================================
 */
/* SUN preset (face товч, ГРАДУС) */
#define SM_SUN_TR 0     /* △ Triangle */
#define SM_SUN_CI 90    /* ○ Circle   */
#define SM_SUN_SQ (-90) /* ▭ Square   */
#define SM_SUN_X 180    /* ✕ Cross    */

/* MOON preset (D-pad, encoder1 COUNT) */
#define SM_MOON_UP 140    /* D-Up   */
#define SM_MOON_DN (-148) /* D-Down */
#define SM_MOON_L 0       /* D-Left */

void sun_moon(void) {
  static int sun_tgt = 0;  // encoder0 count (зорилт)
  static int moon_tgt = 0; // encoder1 count (зорилт)
  static uint8_t inited = 0;
  static Btn_t btr = {0}, bc = {0}, bsq = {0}, bx = {0}; // sun: face товч
  static Btn_t bup = {0}, bdn = {0}, bl = {0};           // moon: D-pad

  if (!inited) {
    sun_tgt = counter[0];
    moon_tgt = counter[1];
    inited = 1;
  }

  /* --- SUN ← face товч (△○▭✕), градус ---
   *   control_data[1]: [0]=✕ [1]=▭ [2]=△ [3]=○                          */
  if (btn_rising(&btr, (uint8_t)control_data[1][2]))
    sun_tgt = sun_deg_to_cnt(SM_SUN_TR); // △ → 0°
  if (btn_rising(&bc, (uint8_t)control_data[1][3]))
    sun_tgt = sun_deg_to_cnt(SM_SUN_CI); // ○ → 90°
  if (btn_rising(&bsq, (uint8_t)control_data[1][1]))
    sun_tgt = sun_deg_to_cnt(SM_SUN_SQ); // ▭ → −90°
  if (btn_rising(&bx, (uint8_t)control_data[1][0]))
    sun_tgt = sun_deg_to_cnt(SM_SUN_X); // ✕ → 180°

  /* --- MOON ← D-pad, encoder1 count ---
   *   control_data[2]: [0]=D-Down [1]=D-Right [2]=D-Up [3]=D-Left        */
  if (btn_rising(&bup, (uint8_t)control_data[2][2]))
    moon_tgt = SM_MOON_UP; // D-Up   → 140
  if (btn_rising(&bdn, (uint8_t)control_data[2][0]))
    moon_tgt = SM_MOON_DN; // D-Down → −148
  if (btn_rising(&bl, (uint8_t)control_data[2][3]))
    moon_tgt = SM_MOON_L; // D-Left → 0

  /* --- Хоёр гарыг ЗЭРЭГ байрлалд барих --- */
  int sun_pwm = sun_hold(sun_tgt) * SUN_DIR;
  int moon_pwm = moon_hold(moon_tgt) * MOON_DIR;
  motor_control(5, sun_pwm);
  motor_control(6, moon_pwm);
  sun_last_pwm = sun_pwm;
  moon_last_pwm = moon_pwm;

  static uint32_t t = 0;
  if (HAL_GetTick() - t >= 100) {
    t = HAL_GetTick();

    /* --- UART4 (115200) телеметр (sun + moon) --- */
    char line[96];
    int n = snprintf(line, sizeof(line),
                     "SM\tS tgt=%d cnt=%d e=%d\tM tgt=%d cnt=%d e=%d\n",
                     sun_tgt, counter[0], sun_tgt - counter[0], moon_tgt,
                     counter[1], moon_tgt - counter[1]);
    HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)n, 20);

    /* --- OLED --- */
    colorFill(Black);
    setCursor(2, 2);
    printStr("SUN-MOON");
    setCursor(2, 18);
    printStr("S %d/%d", counter[0], sun_tgt);
    setCursor(2, 34);
    printStr("M %d/%d", counter[1], moon_tgt);
    setCursor(2, 50);
    printStr("eS%d eM%d", sun_tgt - counter[0], moon_tgt - counter[1]);
    setScreen();
  }
}

/* =============================================================================
 *  AUTO SEQ — ▭ (Square) дархад автомат ДАРААЛСАН дараалал (PCB2)
 *
 *    1) sun  → -90°           (хүрэх хүртэл хүлээнэ)
 *    2) solenoid 5 → ON
 *    3) solenoid 1 → ON       (SEQ_SOL_MS хүлээгээд)
 *    4) moon → 120 count      (хүрэх хүртэл хүлээнэ)  → DONE
 *
 *  Алхам бүр өмнөхөө дуусгана (sun/moon-ийг encoder-оор шалгана, соленоидын
 *  дараа хугацаагаар хүлээнэ). Явцад нь sun/moon-ыг P-hold-оор барина.
 *  ▭ дахин дарвал эхнээс нь (соленоидуудыг унтрааж) дахин эхэлнэ.
 * =============================================================================
 */
#define SEQ_SUN_DEG (-90) // 1-р алхам: sun өнцөг
#define SEQ_MOON_CNT 120  // сүүлийн алхам: moon count
#define SEQ_SOL_MS 500    // соленоид АССаны дараа хүлээх (ms)

/* Зорилтод суусан (P-hold deadband дотор) эсэх */
static int sun_reached(int target) {
  int e = target - counter[0];
  return (e < 0 ? -e : e) <= SUN_HOLD_DB;
}
static int moon_reached(int target) {
  int e = target - counter[1];
  return (e < 0 ? -e : e) <= MOON_HOLD_DB;
}

void auto_seq(void) {
  static uint8_t state = 0;
  static int sun_tgt = 0;  // encoder0 count
  static int moon_tgt = 0; // encoder1 count
  static uint32_t t0 = 0;  // соленоидын хүлээлтийн эхлэл
  static Btn_t bsq = {0};
  static uint8_t inited = 0;

  if (!inited) {
    sun_tgt = counter[0];
    moon_tgt = counter[1];
    controlSolenoid(5, false);
    controlSolenoid(1, false);
    inited = 1;
  }

  /* ▭ дархад дарааллыг ЭХНЭЭС нь эхлүүлнэ (аль ч төлвөөс) */
  if (btn_rising(&bsq, (uint8_t)control_data[1][1])) {
    controlSolenoid(5, false); // цэвэр эхлэл
    controlSolenoid(1, false);
    sun_tgt = sun_deg_to_cnt(SEQ_SUN_DEG); // алхам 1: sun -90
    state = 1;
  }

  const char *msg = "idle";
  switch (state) {
  case 0: // сул зогсолт
    msg = "idle press []";
    break;

  case 1: // sun -90 хүрэх хүртэл
    msg = "1 sun -90";
    if (sun_reached(sun_tgt)) {
      controlSolenoid(5, true); // алхам 2: solenoid5 ON
      t0 = HAL_GetTick();
      state = 2;
    }
    break;

  case 2: // sol5-ийн дараа хүлээгээд sol1
    msg = "2 sol5 on";
    if (HAL_GetTick() - t0 >= SEQ_SOL_MS) {
      controlSolenoid(1, true); // алхам 3: solenoid1 ON
      t0 = HAL_GetTick();
      state = 3;
    }
    break;

  case 3: // sol1-ийн дараа хүлээгээд moon 120
    msg = "3 sol1 on";
    if (HAL_GetTick() - t0 >= SEQ_SOL_MS) {
      moon_tgt = SEQ_MOON_CNT; // алхам 4: moon 120
      state = 4;
    }
    break;

  case 4: // moon 120 хүрэх хүртэл
    msg = "4 moon 120";
    if (moon_reached(moon_tgt))
      state = 5;
    break;

  default: // 5 = DONE
    msg = "DONE";
    break;
  }

  /* --- sun/moon-ыг ҮРГЭЛЖ P-hold-оор барина (аль ч төлөвт) --- */
  int sun_pwm = sun_hold(sun_tgt) * SUN_DIR;
  int moon_pwm = moon_hold(moon_tgt) * MOON_DIR;
  motor_control(5, sun_pwm);
  motor_control(6, moon_pwm);
  sun_last_pwm = sun_pwm;
  moon_last_pwm = moon_pwm;

  static uint32_t t = 0;
  if (HAL_GetTick() - t >= 100) {
    t = HAL_GetTick();

    /* --- UART4 (115200) телеметр --- */
    char line[96];
    int n = snprintf(line, sizeof(line), "AUTO\ts%d %s\tS=%d/%d\tM=%d/%d\n",
                     state, msg, counter[0], sun_tgt, counter[1], moon_tgt);
    HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)n, 20);

    /* --- OLED --- */
    colorFill(Black);
    setCursor(2, 2);
    printStr("AUTO SEQ");
    setCursor(2, 18);
    printStr("%s", msg);
    setCursor(2, 34);
    printStr("S %d/%d", counter[0], sun_tgt);
    setCursor(2, 50);
    printStr("M %d/%d", counter[1], moon_tgt);
    setScreen();
  }
}

/* =============================================================================
 *  Шоо-авах бэлтгэлийн дараалал (PCB1-ийн GRAB команд → Tactic_Task-аас дуудна)
 *    Grab_Start()  — дараалал эхлүүлнэ (grab_request ирэхэд).
 *    Grab_Service()— БҮР давталтад дуудна; sun/moon-ыг барьж, дуусмагц 1
 * буцаана. Алхам (PCB2 удирдана, strafe-ийг PCB1-д командална): 1) sun gear −90
 *    2) хүрмэгц  sol1 ON + sol5 ON,  moon → 130
 *    3) moon 130 хүрмэгц  ЗҮҮН strafe (PCB1) — val1 && val2 хоёул 1 болтол
 *    4) зогсоод  moon → 140
 *    5) moon 140 хүрмэгц  sol1 OFF + sol5 OFF,  moon → 0
 *    6) moon 0 хүрмэгц  БАРУУН strafe (буцах) — зүүн strafe-тэй ижил хугацаа
 *    7) зогсоод  done → PCB1 (sun −90 дээрээ ҮЛДЭНЭ)
 *  Strafe командыг [0xB7][dir][0x0A]-аар 80мс тутам давтан илгээнэ (PCB1 нь
 *  300мс шинэлэг команд ирэхгүй бол автоматаар зогсоно → алдагдсан пакетаас
 * хамгаална).
 * =============================================================================
 */
#define GRAB_SUN_DEG (-90)   // бэлтгэлийн sun өнцөг
#define GRAB_MOON_A 130      // strafe-ийн ӨМНӨХ moon count
#define GRAB_MOON_B 140      // strafe-ийн ДАРААХ moon count
#define GRAB_MOON_HOME 0     // буцах moon count
#define GRAB_STRAFE_STOP 0   // PCB1-д: strafe зогс
#define GRAB_STRAFE_LEFT 1   // PCB1-д: ЗҮҮН strafe
#define GRAB_STRAFE_RIGHT 2  // PCB1-д: БАРУУН strafe (буцах)
#define GRAB_STRAFE_TX_MS 80 // strafe командыг давтан илгээх интервал (ms)
#define GRAB_STRAFE_TIMEOUT                                                    \
  5000 // val1&val2 хэзээ ч 1 болохгүй бол хамгаалалт (ms)
#define GRAB_MOON_B_WAIT_MS                                                    \
  500 // moon 140-д хүрсний дараа sol унтраахаас өмнө хүлээх (ms)
#define GRAB_STRAFE_EXTRA_MS                                                   \
  500 // val хангагдсаны дараа НЭМЖ зүүн явах хугацаа (ms)
#define GRAB_SOL_OFF_WAIT_MS                                                   \
  1000 // sol1 OFF болсны дараа moon буцахаас өмнө хүлээх (механик, ms)

static uint8_t grab_state = 0; // 0=idle 1..6 = алхам
static int grab_sun_tgt = 0;   // encoder0 count (−90°)
static int grab_moon_tgt = 0;  // encoder1 count (120/140/0)
static uint32_t grab_t0 =
    0; // strafe эхэлсэн үе (t_left тооцох / буцах / 140 хүлээлт)
static uint32_t grab_tx = 0; // strafe команд давтан илгээх таймер
static uint32_t grab_t_left =
    0; // ЗҮҮН strafe үргэлжилсэн хугацаа (буцахад ашиглана)
static uint8_t grab_wait_on = 0; // 1 = moon 140-ийн 500ms хүлээлт эхэлсэн
static uint8_t grab_holding =
    0; // 1 = sun/moon-ыг P-hold-оор барих (grab эхэлсэн)

/* PCB1 руу strafe команд: [0xB7][dir][0x0A] (USART2 TX) */
static void send_strafe_cmd(uint8_t dir) {
  uint8_t pkt[3] = {0xB7, dir, 0x0A};
  HAL_UART_Transmit(&huart2, pkt, 3, 20);
}

/* moon зорилтод ТОГТСОН эсэх — жижиг алдаа (±GRAB_MOON_TOL) тэвчиж,
   GRAB_MOON_SETTLE_MS хугацаанд хүлцэлд ТОГТвол л "yg хүрсэн" гэнэ.
   moon_reached (±MOON_HOLD_DB=1) хэт нарийн тул moon бага зэрэг алдаатай үед
   120-д хэзээ ч хүрэхгүй гацаж болзошгүй байв. Ингэснээр val-ыг зөвхөн moon
   120-д ЯГ тогтсоны дараа шалгана. */
#define GRAB_MOON_TOL 4         // moon "хүрсэн" хүлцэл (count)
#define GRAB_MOON_SETTLE_MS 120 // энэ хугацаанд хүлцэлд тогтвол "yg хүрсэн"

static uint32_t grab_moon_t = 0;
static uint8_t grab_moon_in = 0;
static uint8_t grab_moon_settled(int target) {
  int e = target - counter[1];
  if (e < 0)
    e = -e;
  if (e <= GRAB_MOON_TOL) {
    if (!grab_moon_in) {
      grab_moon_in = 1;
      grab_moon_t = HAL_GetTick();
    }
    return (HAL_GetTick() - grab_moon_t >= GRAB_MOON_SETTLE_MS);
  }
  grab_moon_in = 0; // хүлцлээс гарвал суналтын таймер тэглэнэ
  return 0;
}

void Grab_Start(void) {
  if (grab_state != 0)
    return; // дараалал явж байвал дахин эхлүүлэхгүй (GRAB давталтыг үл тоох)
  grab_sun_tgt = sun_deg_to_cnt(GRAB_SUN_DEG);
  grab_moon_tgt = counter[1]; // одоогийн байрлалдаа барих (120 хүртэл)
  grab_moon_in = 0;
  grab_wait_on = 0;
  grab_holding = 1;
  grab_state = 1;
}

uint8_t Grab_Service(void) {
  uint8_t done = 0;
  switch (grab_state) {
  case 1: // sun −90 хүрэх → sol1+5 ON, moon 130
    if (sun_reached(grab_sun_tgt)) {
      controlSolenoid(1, true);
      controlSolenoid(5, true);
      grab_moon_tgt = GRAB_MOON_A; // 120
      grab_state = 2;
    }
    break;

  case 2: // moon 130 хүрэх → ЗҮҮН strafe эхлүүл
    if (grab_moon_settled(grab_moon_tgt)) {
      grab_t0 = HAL_GetTick(); // зүүн strafe эхэл (t_left тооцох)
      grab_tx = 0;             // эхний командыг шууд илгээх
      grab_state = 3;
    }
    break;

  case 3: // ЗҮҮН strafe (PCB1) — val1 && val2 хоёул 1 болтол
    if (HAL_GetTick() - grab_tx >= GRAB_STRAFE_TX_MS) {
      send_strafe_cmd(GRAB_STRAFE_LEFT); // давтан илгээх
      grab_tx = HAL_GetTick();
    }
    if ((val1 == 1 && val2 == 1) ||
        HAL_GetTick() - grab_t0 >= GRAB_STRAFE_TIMEOUT) {
      send_strafe_cmd(GRAB_STRAFE_STOP); // зогс (2 удаа — найдвартай)
      send_strafe_cmd(GRAB_STRAFE_STOP);
      grab_t_left = HAL_GetTick() - grab_t0; // зүүн strafe хугацаа
      grab_moon_tgt = GRAB_MOON_B;           // 140
      grab_state = 4;
    }
    break;

  case 4: // moon 140 хүрэх → 500ms хүлээ → sol1+5 OFF, moon 0
    if (grab_moon_settled(grab_moon_tgt)) {
      if (!grab_wait_on) { // 140-д ДӨНГӨЖ тогтлоо → 500ms таймер эхлүүл
        grab_wait_on = 1;
        grab_t0 = HAL_GetTick();
      } else if (HAL_GetTick() - grab_t0 >= GRAB_MOON_B_WAIT_MS) {
        controlSolenoid(1, false);
        controlSolenoid(5, false);
        grab_moon_tgt = GRAB_MOON_HOME; // 0
        grab_wait_on = 0;
        grab_state = 5;
      }
    }
    break;

  case 5: // moon 0 хүрэх → БАРУУН strafe (буцах)
    if (grab_moon_settled(grab_moon_tgt)) {
      grab_t0 = HAL_GetTick(); // буцах strafe эхэл
      grab_tx = 0;
      grab_state = 6;
    }
    break;

  case 6: // БАРУУН strafe — зүүнтэй ижил хугацаа → зогс, дуусав
    if (HAL_GetTick() - grab_tx >= GRAB_STRAFE_TX_MS) {
      send_strafe_cmd(GRAB_STRAFE_RIGHT);
      grab_tx = HAL_GetTick();
    }
    if (HAL_GetTick() - grab_t0 >= grab_t_left) {
      send_strafe_cmd(GRAB_STRAFE_STOP);
      send_strafe_cmd(GRAB_STRAFE_STOP);
      grab_state = 0; // дуусав (sun −90 дээрээ ҮЛДЭНЭ)
      done = 1;
    }
    break;

  default:
    break;
  }
  /* grab эхэлсний дараа sun (−90) ба moon (зорилт)-ыг ҮРГЭЛЖ P-hold-оор барина
   */
  if (grab_holding) {
    int sun_pwm = sun_hold(grab_sun_tgt) * SUN_DIR;
    int moon_pwm = moon_hold(grab_moon_tgt) * MOON_DIR;
    motor_control(5, sun_pwm);
    motor_control(6, moon_pwm);
    sun_last_pwm = sun_pwm;
    moon_last_pwm = moon_pwm;
  }
  return done;
}

/* =============================================================================
 *  Grab_Test — шоо-авах БҮТЭН дарааллыг ТУСДАА турших (strafe-ТЭЙ, PCB1-тэй
 * ХАМТ). △ → Grab_Start (дараалал эхлүүлнэ),  ✕ → таслах/reset. Grab_Service
 * бүтэн ажиллана: sun−90 → sol1+5 → moon120 → ЗҮҮН strafe (PCB1 руу 0xB7
 * команд, val1&val2==1) → moon140 → 500ms → sol OFF, moon0 → БАРУУН strafe
 * (буцах) → дуусав.  ⚠ PCB1 нь "Grab strafe" мод дээр байх ёстой.
 * =============================================================================
 */
void Grab_Test(void) {
  static Btn_t bF = {0}, bB = {0}, breset = {0};
  static uint8_t active = 0,
                 done = 0; // 0=none 1=front_down_20_f 2=front_down_20_b
  static uint8_t inited = 0;

  if (!inited) {
    controlSolenoid(1, false);
    controlSolenoid(5, false);
    inited = 1;
  }

  /* △ → grab_front_down_20_f (урд харсан) */
  if (btn_rising(&bF, (uint8_t)control_data[1][2])) {
    grab_front_down_20_f_reset();
    active = 1;
    done = 0;
  }
  /* ○ → grab_front_down_20_b (ард харсан) */
  if (btn_rising(&bB, (uint8_t)control_data[1][3])) {
    grab_front_down_20_b_reset();
    active = 2;
    done = 0;
  }
  /* ✕ → зогсоох: соленоид унтрааж, мотор зогсоох (P-hold-ыг таслана) */
  if (btn_rising(&breset, (uint8_t)control_data[1][0])) {
    controlSolenoid(1, false);
    controlSolenoid(5, false);
    motor_control(5, 0);
    motor_control(6, 0);
    active = 0;
    done = 0;
  }
  /* ЭХЭЛСЭН бол ҮРГЭЛЖ дуудна — дуусны дараа ч sun/moon-ыг P-hold-оор БАРЬНА */
  if (active == 1)
    done = grab_front_down_20_f();
  else if (active == 2)
    done = grab_front_down_20_b();

  static uint32_t to = 0;
  if (HAL_GetTick() - to >= 100) {
    to = HAL_GetTick();
    colorFill(Black);
    setCursor(2, 2);
    printStr("GRAB FD20 %s", active == 2 ? "b" : "f");
    setCursor(2, 22);
    printStr("S:%d M:%d", counter[0], counter[1]);
    setCursor(2, 42);
    printStr("%s", !active ? "idle" : done ? "DONE hold" : "RUN");
    setScreen();
  }
}

/* =============================================================================
 *  ШОО-АВАХ 14 COMBINATION  (7 тохиолдол × sun gear 180° урд/ард харах)
 *  ────────────────────────────────────────────────────────────────────────────
 *  Тохиолдол (шоо роботоос ХААНА байгаагаар):
 *    front_up_20   — УРД талын scroll-той тавцан 20см ДЭЭР
 *    front_down_20 — УРД талын scroll-той тавцан 20см ДООР (одоо Grab_Service-т
 * бий) left_up_20    — ЗҮҮН талын scroll 20см ДЭЭР left_down_20  — ЗҮҮН талын
 * scroll 20см ДООР right_up_20   — БАРУУН талын scroll 20см ДЭЭР right_down_20
 * — БАРУУН талын scroll 20см ДООР front_up_40   — ТУСГАЙ: 1/3-р тавцан, 40см
 * ДЭЭР (2 шоо авна) Facing (sun gear 180° эргэнэ):  _f = УРД харсан,  _b = АРД
 * харсан. Тус бүр non-blocking: нэг алхам хийж, ДУУСВАЛ 1 буцаана. ⚠ Одоохондоо
 * БҮГД ХООСОН (return 1 = шууд дуусав) — дараа бөглөнө. Эхлээд FRONT (урд)-ыг
 * бичиж турших.
 * =============================================================================
 */
uint8_t grab_front_up_20_f(void) { /* TODO */ return 1; }
uint8_t grab_front_up_20_b(void) { /* TODO */ return 1; }
/* grab_front_down_20_f — УРД талын scroll 20см ДООР, sun УРД харсан.
 *   sun−90 → sol1+5 ON → moon 130 → ЗҮҮН strafe хайх (val1&val2==1) →
 *   moon 140 + sol1 OFF (атгах) → moon буцах (0) + sol5 OFF (буцах явцад) →
 * дуусав. Strafe-ийг PCB1 хийнэ (0xB7). _reset()-ээр эхлүүлж non-blocking;
 * дуусвал 1.  */
static uint8_t gfd20f_st =
    0; // 0=idle 1=sun 2=moon130 3=strafe 4=strafe+500 5=moon145 6=sol1off+1s
       // 7=wait1s 8=moon0→sun+90 9=sol5off 10=дуусав
static int gfd20f_sun = 0;
static int gfd20f_moon = 0;
static uint32_t gfd20f_t0 = 0; // strafe timeout таймер
static uint32_t gfd20f_tx = 0; // strafe команд давтан илгээх таймер

void grab_front_down_20_f_reset(void) {
  controlSolenoid(1, false);
  controlSolenoid(5, false);
  gfd20f_sun = sun_deg_to_cnt(GRAB_SUN_DEG); // −90
  gfd20f_moon = counter[1]; // одоо байрлалдаа (130 хүртэл барина)
  gfd20f_st = 1;
}

uint8_t grab_front_down_20_f(void) {
  switch (gfd20f_st) {
  case 1: // sun −90 хүрэх → sol1+5 ON, moon 130
    if (sun_reached(gfd20f_sun)) {
      controlSolenoid(1, true);
      controlSolenoid(5, true);
      gfd20f_moon = GRAB_MOON_A; // 130
      gfd20f_st = 2;
    }
    break;

  case 2: // moon 130 тогтсон → ЗҮҮН strafe эхлүүл
    if (grab_moon_settled(gfd20f_moon)) {
      gfd20f_t0 = HAL_GetTick();
      gfd20f_tx = 0; // эхний командыг шууд илгээх
      gfd20f_st = 3;
    }
    break;

  case 3: // ЗҮҮН strafe (PCB1) — val1 && val2 хоёул 1 болтол ХАЙХ
    if (HAL_GetTick() - gfd20f_tx >= GRAB_STRAFE_TX_MS) {
      send_strafe_cmd(GRAB_STRAFE_LEFT); // давтан илгээх
      gfd20f_tx = HAL_GetTick();
    }
    if ((val1 == 1 && val2 == 1) ||
        HAL_GetTick() - gfd20f_t0 >= GRAB_STRAFE_TIMEOUT) {
      gfd20f_t0 = HAL_GetTick(); // val хангагдлаа → НЭМЖ 500ms зүүн явна
      gfd20f_tx = 0;
      gfd20f_st = 4;
    }
    break;

  case 4: // val хангагдсан ч 500ms НЭМЖ зүүн яваад зогс → moon 140
    if (HAL_GetTick() - gfd20f_tx >= GRAB_STRAFE_TX_MS) {
      send_strafe_cmd(GRAB_STRAFE_LEFT);
      gfd20f_tx = HAL_GetTick();
    }
    if (HAL_GetTick() - gfd20f_t0 >= GRAB_STRAFE_EXTRA_MS) {
      send_strafe_cmd(GRAB_STRAFE_STOP); // зогс (2 удаа — найдвартай)
      send_strafe_cmd(GRAB_STRAFE_STOP);
      gfd20f_moon = 145; // moon 145 (атгах байрлал)
      gfd20f_st = 5;
    }
    break;

  case 5: // moon 140 тогтсон → 500ms хүлээ
    if (grab_moon_settled(gfd20f_moon)) {
      gfd20f_t0 = HAL_GetTick();
      gfd20f_st = 6;
    }
    break;

  case 6: // 500ms → sol1 OFF → 1сек хүлээ (механик хөдөлгөөнд цаг). sol5 хэвээр
          // ON
    if (HAL_GetTick() - gfd20f_t0 >= GRAB_MOON_B_WAIT_MS) {
      controlSolenoid(1, false); // sol1 OFF
      gfd20f_t0 = HAL_GetTick(); // 1сек таймер эхлүүлэх
      gfd20f_st = 7;
    }
    break;

  case 7: // sol1 OFF-ийн дараа 1сек хүлээ → moon буцах (0)
    if (HAL_GetTick() - gfd20f_t0 >= GRAB_SOL_OFF_WAIT_MS) {
      gfd20f_moon = GRAB_MOON_HOME; // moon 0 (буцаж дээшлэх)
      gfd20f_st = 8;
    }
    break;

  case 8: // moon 0 тогтсон → sun +90 руу (180° эргэж дараагийн scroll-д
          // бэлдэнэ)
    if (grab_moon_settled(gfd20f_moon)) {
      gfd20f_sun = sun_deg_to_cnt(90); // +90 (дараагийн scroll руу эргэх)
      gfd20f_st = 9;
    }
    break;

  case 9: // sun +90 тогтсон → sol5 OFF, дуусав
    if (sun_reached(gfd20f_sun)) {
      controlSolenoid(5, false); // sol5 OFF (moon 0 + sun +90-ийн ДАРАА)
      gfd20f_st = 10;
    }
    break;

  default:
    break; // 0=idle, 10=дуусав
  }
  if (gfd20f_st != 0) { // эхэлсэн бол sun/moon-ыг P-hold-оор барих
    int sp = sun_hold(gfd20f_sun) * SUN_DIR;
    int mp = moon_hold(gfd20f_moon) * MOON_DIR;
    motor_control(5, sp);
    motor_control(6, mp);
    sun_last_pwm = sp;
    moon_last_pwm = mp;

    /* --- UART4 (115200) телеметр: moon/sun хөдөлгөөнийг ажиглах (PCB2-д LPMS
       алга) --- баганууд: GF st<төлөв> Mt<moon зорилт> Mp<moon одоо> Mw<moon
       PWM> St<sun зорилт> Sp<sun одоо> v<val1><val2>                    */
    static uint32_t gfd_ts = 0;
    if (HAL_GetTick() - gfd_ts >= 50) {
      gfd_ts = HAL_GetTick();
      char line[100];
      int n = snprintf(line, sizeof(line),
                       "GF\tst%d\tMt%d\tMp%d\tMw%d\tSt%d\tSp%d\tv%d%d\n",
                       gfd20f_st, gfd20f_moon, counter[1], mp, gfd20f_sun,
                       counter[0], (int)val1, (int)val2);
      HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)n, 20);
    }
  }
  return (gfd20f_st == 10) ? 1 : 0;
}
/* grab_front_down_20_b — УРД шоо авсны ДАРАА back gripper-ээр авах (sun +90 аль
 *   хэдийн эргэсэн — _f-ийн төгсгөлд). sun ХӨДЛӨХГҮЙ. sol5+sol2 (sol1 БИШ),
 *   moon −130 → БАРУУН strafe (val3&val4==1) → moon −145 + sol2 OFF (атгах) →
 *   moon 0 → sol5 OFF → дуусав.  Strafe-ийг PCB1 хийнэ. Дуусвал 1. */
static uint8_t gfd20b_st = 0; // 0=idle 1=sol/moon-130 2=moon-130 3=strafe-R
                              // 4=moon-145 5=moon0 6=дуусав
static int gfd20b_sun = 0;
static int gfd20b_moon = 0;
static uint32_t gfd20b_t0 = 0;
static uint32_t gfd20b_tx = 0;

void grab_front_down_20_b_reset(void) {
  gfd20b_sun = sun_deg_to_cnt(90); // +90 (_f-ээс аль хэдийн тэнд; эс бол хүрнэ)
  gfd20b_moon = counter[1];        // одоо байрлалдаа (−130 хүртэл барина)
  gfd20b_st = 1;
}

uint8_t grab_front_down_20_b(void) {
  switch (gfd20b_st) {
  case 1: // sun +90 (хөдлөхгүй) → sol5 ON, sol2 ON, moon −130
    if (sun_reached(gfd20b_sun)) {
      controlSolenoid(5, true);
      controlSolenoid(2, true);
      gfd20b_moon = -130;
      gfd20b_st = 2;
    }
    break;

  case 2: // moon −130 тогтсон → БАРУУН strafe эхлүүл
    if (grab_moon_settled(gfd20b_moon)) {
      gfd20b_t0 = HAL_GetTick();
      gfd20b_tx = 0;
      gfd20b_st = 3;
    }
    break;

  case 3: // БАРУУН strafe (PCB1) — val3(PNP тул УРВУУ)==0 && val4==1 болтол ХАЙХ
    if (HAL_GetTick() - gfd20b_tx >= GRAB_STRAFE_TX_MS) {
      send_strafe_cmd(GRAB_STRAFE_RIGHT);
      gfd20b_tx = HAL_GetTick();
    }
    if ((val3 == 0 && val4 == 1) || // ⚠ val3 PNP — 0=мэдэрсэн (бусад мэдрэгчийн эсрэг)
        HAL_GetTick() - gfd20b_t0 >= GRAB_STRAFE_TIMEOUT) {
      send_strafe_cmd(GRAB_STRAFE_STOP);
      send_strafe_cmd(GRAB_STRAFE_STOP);
      gfd20b_moon = -145; // грип байрлал
      gfd20b_st = 4;
    }
    break;

  case 4: // moon −145 тогтсон → sol2 OFF (атгах), moon 0
    if (grab_moon_settled(gfd20b_moon)) {
      controlSolenoid(2, false);    // sol2 OFF — атгах
      gfd20b_moon = GRAB_MOON_HOME; // moon 0 (буцах)
      gfd20b_st = 5;
    }
    break;

  case 5: // moon 0 тогтсон → sol5 OFF, дуусав
    if (grab_moon_settled(gfd20b_moon)) {
      controlSolenoid(5, false);
      gfd20b_st = 6;
    }
    break;

  default:
    break; // 0=idle, 6=дуусав
  }
  if (gfd20b_st != 0) {
    int sp = sun_hold(gfd20b_sun) * SUN_DIR;
    int mp = moon_hold(gfd20b_moon) * MOON_DIR;
    motor_control(5, sp);
    motor_control(6, mp);
    sun_last_pwm = sp;
    moon_last_pwm = mp;

    static uint32_t gb_ts = 0;
    if (HAL_GetTick() - gb_ts >= 50) {
      gb_ts = HAL_GetTick();
      char line[100];
      int n = snprintf(line, sizeof(line),
                       "GB\tst%d\tMt%d\tMp%d\tMw%d\tSt%d\tSp%d\tv%d%d\n",
                       gfd20b_st, gfd20b_moon, counter[1], mp, gfd20b_sun,
                       counter[0], (int)val3, (int)val4);
      HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)n, 20);
    }
  }
  return (gfd20b_st == 6) ? 1 : 0;
}
uint8_t grab_left_up_20_f(void) { /* TODO */ return 1; }
uint8_t grab_left_up_20_b(void) { /* TODO */ return 1; }
uint8_t grab_left_down_20_f(void) { /* TODO */ return 1; }
uint8_t grab_left_down_20_b(void) { /* TODO */ return 1; }
uint8_t grab_right_up_20_f(void) { /* TODO */ return 1; }
uint8_t grab_right_up_20_b(void) { /* TODO */ return 1; }
uint8_t grab_right_down_20_f(void) { /* TODO */ return 1; }
uint8_t grab_right_down_20_b(void) { /* TODO */ return 1; }
uint8_t grab_front_up_40_f(void) { /* TODO */ return 1; }
uint8_t grab_front_up_40_b(void) { /* TODO */ return 1; }
