/* =============================================================================
 *  general.c — Хөдөлгөөний өндөр түвшний удирдлага
 *              (differential (tank) жолоодлого, ADC, USB CDC)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "general.h"
#include <stdlib.h> // abs() — runner() дахь differential нормчлолд хэрэглэнэ

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
extern int counter[4];         // encoder тоолуурууд
extern int control_data[5][4]; // джойстик/товчлуурын өгөгдөл
extern int timer;              // TIM7-оос нэмэгддэг зөөлөн тоолуур

extern uint8_t usb_received_value;
extern uint8_t usb_new_data_flag;

/* =============================================================================
 *  ЖОЛООДЛОГО — DIFFERENTIAL (tank / skid-steer)
 *
 *  Энэ робот MECANUM БИШ. Хажуу тийш (strafe) хөдөлгөөн БАЙХГҮЙ.
 *  Зүүн тал (мотор 1, 3) ба баруун тал (мотор 2, 4) тус бүр нэг хурдаар эргэнэ:
 *    урагш = хоёр тал ижил чиглэл,  эргэлт = хоёр тал эсрэг чиглэл.
 *
 *  ТЭМДЭГЛЭЛ: 4 хөдөлгүүрт (тал бүрт 2) гэж үзсэн — mecanum-ийн үеийн адил
 *  1..4 суваг жолоодлогынх. Хэрэв энэ робот ЗӨВХӨН 2 хөдөлгүүртэй бол
 *  мотор 3, 4 руу өгсөн нь хоосон явна (хор хөнөөлгүй); эсвэл доорх
 *  motor_control(3/4, ...) мөрүүдийг устгаж болно.
 * =============================================================================
 */
#define DEADZONE 10   // joystick үхмэл бүс
#define SPEED_GAIN 10 // урагш/хойш (V): стик 100 → 1000 PWM (бүрэн хурд)
#define TURN_GAIN                                                              \
  11 // эргэлт (W):    стик 100 → 1100 PWM  ← эргэлтийг ИХЭСГЭХэд энэ утгыг
     // нэмэгдүүл
#define PWM_MAX 1000 // моторын PWM дээд хязгаар (нормчлол энэ руу хийнэ)

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
 *  runner — Джойстикоор differential (tank) жолоодлого
 *    Зүүн стик Y   → урагш/хойш (V)
 *    Баруун стик X → эргэлт (W)
 *    Зүүн тал  = V - W   (мотор 1, 3)
 *    Баруун тал = V + W  (мотор 2, 4)
 *  (Mecanum-ийн strafe гишүүн Vx-ийг л хассан тул шулуун явах/эргэх тэмдэг
 *   өмнөх кодтой ижил хэвээр.)
 * -----------------------------------------------------------------------------
 */
void runner(void) {
  // --- joystick унших (-100..100) ---
  int V = applyDeadzone(control_data[0][1]); // зүүн стик Y → урагш/хойш
  int W = applyDeadzone(control_data[0][2]); // баруун стик X → эргэлт

  // joystick Y нь дээш түлхэхэд сөрөг тул урагш = эерэг болгож эргүүлнэ
  V = -V;

  // --- Differential mixing (шууд PWM орон зайд; V ба W тус тусын gain-тай) ---
  int v_pwm = V * SPEED_GAIN; // урагш/хойш:  ±800
  int w_pwm =
      W * TURN_GAIN; // эргэлт:      ±800 (TURN_GAIN ихэсгэвэл эргэлт хүчтэй)
  int left = v_pwm - w_pwm;  // Зүүн тал
  int right = v_pwm + w_pwm; // Баруун тал

  // --- Нормчлол: аль нэг тал ±PWM_MAX хэтэрвэл харьцаагаа хадгалж багасгана
  // ---
  //     (100 нэгжид биш, PWM орон зайд хийснээр явж байхад эргэлтийн хүч
  //     хадгалагдана)
  int m = abs(left);
  if (abs(right) > m)
    m = abs(right);
  if (m > PWM_MAX) {
    left = left * PWM_MAX / m;
    right = right * PWM_MAX / m;
  }

  // --- Моторт өгөх (утга аль хэдийн PWM) ---
  motor_control(1, right); // Урд-Зүүн
  motor_control(2, right); // Хойд-Зүүн
  motor_control(3, left);  // Урд-Баруун
  motor_control(4, left);  // Хойд-Баруун
}

/* =============================================================================
 *  СОЛЕНОИД УДИРДЛАГА — товчоор TOGGLE (нэг даралт = нэг сэлгэлт)
 *
 *    L1 (control_data[3][0]) → соленоид 1
 *    R1 (control_data[3][1]) → соленоид 3 + 4 ХАМТ (нэг зэрэг)
 *    L2 (control_data[3][2]) → соленоид 2
 *    R2 (control_data[3][3]) → СУЛ (одоохондоо ашиглагдаагүй)
 *
 *  Товч дарах бүрд тухайн соленоид(ууд) HIGH ↔ LOW сэлгэнэ. Дараад БАРЬЖ
 *  байхад давтахгүй (зөвхөн 0→1 ирмэг). Bounce-ыг BTN_DEBOUNCE_MS-ээр шүүнэ.
 * =============================================================================
 */
#define BTN_DEBOUNCE_MS 25 // товчийг тогтвортой гэж үзэх хугацаа (мс)
#define SOLENOID_COUNT 4   // control_data[3] дэх товчны тоо (btn/state массивт)

typedef struct {
  uint8_t stable;     // debounce хийсэн төлөв
  uint8_t raw_last;   // сүүлийн түүхий уншилт
  uint32_t change_ms; // түүхий төлөв сүүлд өөрчлөгдсөн агшин
} Btn_t;

/* -----------------------------------------------------------------------------
 *  btn_pressed — Debounce хийсэн ЦЭВЭР ШИНЭ даралт (rising edge) таних
 *    return: 1 = дөнгөж дарагдлаа (0→1). Дараад барьж байхад дахин 1 БУЦААХГҮЙ.
 * -----------------------------------------------------------------------------
 */
static uint8_t btn_pressed(Btn_t *b, uint8_t raw) {
  uint32_t now = HAL_GetTick();

  if (raw != b->raw_last) { // түүхий төлөв өөрчлөгдвөл цагийг эхлүүлнэ
    b->raw_last = raw;
    b->change_ms = now;
  }

  // BTN_DEBOUNCE_MS турш тогтвортой байж байж л debounced төлвийг шинэчилнэ
  if ((now - b->change_ms) >= BTN_DEBOUNCE_MS && b->stable != raw) {
    b->stable = raw;
    if (raw)
      return 1; // 0 → 1 шилжилт = шинэ даралт
  }

  return 0;
}

/* -----------------------------------------------------------------------------
 *  solenoidControl — L1/R1/L2/R2 тус бүрээр соленоид 1..4-ийг toggle хийх
 *  Main loop-оос давталт бүрд дуудна (блоклохгүй).
 * -----------------------------------------------------------------------------
 */
void solenoidControl(void) {
  static Btn_t btn[SOLENOID_COUNT] = {0};
  static bool state[SOLENOID_COUNT] = {false};
  static bool inited = false;

  // Эхний дуудалтад бүх соленоидыг OFF болгож, программын төлөвтэй нийцүүлнэ
  if (!inited) {
    controlSolenoid(1, false);
    controlSolenoid(2, false);
    controlSolenoid(3, false);
    controlSolenoid(4, false);
    inited = true;
  }

  /* L1 → соленоид 1 */
  if (btn_pressed(&btn[0], (uint8_t)control_data[3][0])) {
    state[0] = !state[0];
    controlSolenoid(1, state[0]);
  }
  /* L2 → соленоид 2 */
  if (btn_pressed(&btn[2], (uint8_t)control_data[3][2])) {
    state[2] = !state[2];
    controlSolenoid(2, state[2]);
  }
  /* R1 → соленоид 3 + 4 ХАМТ (нэг зэрэг toggle) */
  if (btn_pressed(&btn[1], (uint8_t)control_data[3][1])) {
    state[1] = !state[1];
    controlSolenoid(3, state[1]);
    controlSolenoid(4, state[1]);
  }
  /* R2 (control_data[3][3]) — СУЛ (дараа хагас-автоматад ашиглана) */
}

/* =============================================================================
 *  ROBOT1 — 2 PCB, НЭГ project, 2 функц (main.c-д R1_PCB define-аар сонгож
 * build)
 *
 *  Гар робот бас 2 PCB-тэй ч код бага тул тусдаа project биш, нэг дотор 2
 * функц. Build хийхдээ main.c-ийн R1_PCB-ыг (1/2) сольж, тухайн PCB-ийн
 * firmware гаргаад flash хийнэ. Функц бүр main loop-ийн БИЕ (while(1) дотор
 * давталт бүрд дуудна).
 * =============================================================================
 */

/* --- 1-р PCB: differential жолоо + соленоид + мотор 5/6 (одоогийн удирдлага)
 * --- */
void robot1_pcb1(void) {
  static uint8_t sol7_init = 0;
  if (!sol7_init) {
    controlSolenoid(7, true); // solenoid 7 БАЙНГА асаалттай (нэг удаа асаана)
    sol7_init = 1;
  }

  runner();
  solenoidControl(); // L1→sol1, R1→sol3+4, L2→sol2

  /* D-Left → solenoid 6: debounce-ТАЙ — нэг даралтаар 2 СЕК асаад автоматаар унтарна.
     D-Right → solenoid 5: debounce-ГҮЙ momentary (дарвал 1, тавьвал 0).
     control_data[2]: [1]=D-Right [3]=D-Left */
  static Btn_t dl_btn = {0};
  static uint32_t sol6_t0 = 0;
  static bool sol6_on = false;
  if (btn_pressed(&dl_btn, (uint8_t)control_data[2][3])) { // D-Left rising → ON + таймер
    controlSolenoid(6, true);
    sol6_on = true;
    sol6_t0 = HAL_GetTick();
  }
  if (sol6_on && HAL_GetTick() - sol6_t0 >= 2000) { // 2 сек өнгөрөв → OFF
    controlSolenoid(6, false);
    sol6_on = false;
  }
  controlSolenoid(5, control_data[2][1] != 0); // D-Right → sol5 (momentary)

  /* R2 = shift давхарга (control_data[3][3]) — доорх бүх R2-давхаргад ашиглана */
  uint8_t r2 = (uint8_t)control_data[3][3];

  /* D-Up/D-Down:  R2 СУЛ → мотор 6 (PCB1, ±800) ; R2 ДАРСАН → PCB2 мотор 1 (линкээр p3).
     control_data[2]: [0]=D-Down [2]=D-Up */
  int m6 = 0, m1 = 0;
  if (r2) {                    // R2 дарсан → PCB2 мотор 1
    if (control_data[2][2] == 1) // D-Up   → +600
      m1 = 600;
    else if (control_data[2][0] == 1) // D-Down → −600
      m1 = -600;
  } else {                     // R2 сул → PCB1 мотор 6
    if (control_data[2][0] == 1) // D-Down → +800
      m6 = 800;
    else if (control_data[2][2] == 1) // D-Up → −800
      m6 = -800;
  }
  motor_control(6, m6);

  /* Triangle/Cross:  R2 СУЛ → мотор 5 (±400 тогтмол).
     R2 ДАРСАН → мотор 3 (PCB2 линкээр p2):
        Triangle → +600 (тогтмол).
        Cross    → −600.  sol1 OFF үед val1==0 болтол л явна (дараа 0/нейтрал);
                   sol1 ON үед тогтмол −600.
     control_data[1]: [0]=Cross [2]=Triangle */
  int tc = 0; // Triangle = +, Cross = −
  if (control_data[1][2] == 1)
    tc = +1; // Triangle
  else if (control_data[1][0] == 1)
    tc = -1; // Cross

  int m3 = 0, m5 = 0;
  if (r2) {       // R2 дарсан → мотор 3 (линк)
    if (tc > 0) { // Triangle → +600
      m3 = 600;
    } else if (tc < 0) { // Cross → −600
      uint8_t sol1_off =
          (HAL_GPIO_ReadPin(OP1_GPIO_Port, OP1_Pin) == GPIO_PIN_RESET);
      if (sol1_off)
        m3 = (val1 != 0) ? -600 : 0; // sol1 OFF → val1==0 болтол −600, дараа 0
      else
        m3 = -600; // sol1 ON → тогтмол −600
    }
  } else { // R2 сул → мотор 5 (тогтмол ±400)
    m5 = tc * 400;
  }
  motor_control(5, m5);

  /* PCB2 руу UART4-ээр 50Hz илгээнэ (кодчилол: p = 100 + pwm/10, нейтрал=100).
     p1 = МОТОР 2 (тулгуур, Square/Circle) ; p2 = МОТОР 3 (Triangle/Cross, R2) ;
     p3 = МОТОР 1 (D-Up/D-Down, R2). */
  static uint32_t link_t = 0;
  if (HAL_GetTick() - link_t >= 20) {
    link_t = HAL_GetTick();
    int m2 = 0; // мотор 2 = тулгуур
    if (control_data[1][1])
      m2 = 600; // Square → тулгуур гаргах
    else if (control_data[1][3])
      m2 = -600; // Circle → тулгуур татах
    r1_link_send3((uint8_t)(m2 / 10 + 100), (uint8_t)(m3 / 10 + 100),
                  (uint8_t)(m1 / 10 + 100)); // p3 = мотор 1 (R2 + D-Up/D-Down)
  }

  /* --- DEBUG дэлгэц: loop ажиллаж байгаа (hb) + PS5 өгөгдөл ирж байгаа эсэх ---
   *   hb өсөж байвал → loop ажиллаж байна (гацаагүй).
   *   стик хөдөлгөхөд LY/RX өөрчлөгдвөл → PS5 ирж байна (асуудал өөр талд).
   *   hb өсөж байгаа ч LY/RX/товч ҮРГЭЛЖ 0 бол → PS5 холбогдоогүй (control_data 0).
   *   ⚠ Оношилгооны дараа энэ блокийг устгаж болно.                            */
  static uint32_t dbg_t = 0;
  static uint32_t hb = 0;
  if (HAL_GetTick() - dbg_t >= 100) {
    dbg_t = HAL_GetTick();
    hb++;
    colorFill(Black);
    setCursor(2, 2);
    printStr("R1P1 hb%d", (int)hb);
    setCursor(2, 22);
    printStr("LY%d RX%d", control_data[0][1], control_data[0][2]);
    setCursor(2, 42);
    printStr("X%d Sq%d L1%d", control_data[1][0], control_data[1][1],
             control_data[3][0]);
    setScreen();
  }
}

/* --- 2-р PCB: p1→МОТОР 2 (тулгуур), p2→МОТОР 3, p3→МОТОР 1-ыг жолоодно --- */
void robot1_pcb2(void) {
  static uint32_t last = 0;
  if (r1_link_new) {
    r1_link_new = 0;
    last = HAL_GetTick();
  }

  int m1, m2, m3;
  if (HAL_GetTick() - last <= 300) {   // сүүлийн 300мс дотор багц ирсэн бол
    m1 = ((int)r1_link_p3 - 100) * 10; // мотор 1 (R2 + D-Up/D-Down)
    m2 = ((int)r1_link_p1 - 100) * 10; // тулгуур (Square/Circle)
    m3 = ((int)r1_link_p2 - 100) * 10; // мотор 3 (Triangle/Cross, R2)
  } else {
    m1 = 0;
    m2 = 0;
    m3 = 0; // холбоо тасарвал ЗОГС (аюулгүй)
  }
  motor_control(1, m1); // мотор 1
  motor_control(2, m2); // мотор 2 = тулгуур
  motor_control(3, m3); // мотор 3
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
