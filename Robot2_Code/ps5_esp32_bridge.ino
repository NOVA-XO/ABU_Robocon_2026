/*
 * ============================================================================
 *  PS5 КОНТРОЛЛЕР → UART ГҮҮР (ESP32)
 * ----------------------------------------------------------------------------
 *  Үүрэг:
 *    PS5 контроллерийг Bluetooth-оор холбож, джойстик ба товчнуудын утгыг
 *    уншаад, тогтмол 100 Hz давтамжтайгаар Serial2 (UART)-аар 23 байтын
 *    багц (packet) болгож STM32 руу илгээнэ.
 *
 *  Холболт:
 *    Serial  (USB)  — 115200 бод, дибаг (debug) мессеж хэвлэхэд
 *    Serial2 (UART) — 115200 бод, RX=GPIO16, TX=GPIO17, дата илгээхэд
 *
 *  ТЭМДЭГЛЭЛ: Энэ бол Arduino IDE-гийн ESP32 код. STM32-ийн CMake build-д
 *            ОРОХГҮЙ (зөвхөн лавлагаа/баримт болгож энд хадгалав).
 *
 *  Шаардлагатай номын сан:
 *    ps5Controller.h  →  https://github.com/rodneybakiskan/ps5-esp32
 * ============================================================================
 */

#include <ps5Controller.h>

// ───────────────────────── Тохиргоо (Configuration) ─────────────────────────
#define RXD2              16        // Serial2 RX гарын дугаар (GPIO)
#define TXD2              17        // Serial2 TX гарын дугаар (GPIO)
#define LED_BUILTIN       2         // Дотоод LED — холболтын төлөв харуулна

#define UART_BAUD         115200    // Serial болон Serial2-ийн дамжуулах хурд (бод)
#define SEND_INTERVAL_MS  10        // Дата илгээх давтамж: 10 мс → 100 Hz
#define BLINK_INTERVAL_MS 500       // Холбогдоогүй үед LED анивчих хугацаа (мс)

#define PS5_MAC           "A0:FA:9C:20:5C:D4"   // ←── Таны контроллерийн MAC хаяг

// ─────────────────── Багцын бүтэц (Packet layout) — 23 байт ──────────────────
//
//  ЗҮҮН баганад packet-ийн индекс, ДУНД баганад STM32 тал дээр очих
//  control_data[мөр][багана]-ыг, БАРУУН баганад товчны утгыг харуулав.
//  (STM32 задлалт: HAL_UART_RxCpltCallback дотор
//     control_data[0][i]         = information[1+i] - 100;   // джойстик -100..100
//     control_data[1+(i/4)][i%4] = information[5+i];         // товч 0/1  )
//
//   packet │ STM32 control_data │ Агуулга
//   ───────┼────────────────────┼──────────────────────────────────────────
//    [0]   │  —                 │ Эхлэлийн байт (START_BYTE = 0xAA)
//    [1]   │  [0][0]            │ Зүүн джойстик   X   ┐
//    [2]   │  [0][1]            │ Зүүн джойстик   Y   │ 0–200, төв = 100
//    [3]   │  [0][2]            │ Баруун джойстик X   │ (STM32-д −100..100 болно)
//    [4]   │  [0][3]            │ Баруун джойстик Y   ┘
//    [5]   │  [1][0]            │ Cross    (✕)        ┐
//    [6]   │  [1][1]            │ Square   (□)        │
//    [7]   │  [1][2]            │ Triangle (△)        │
//    [8]   │  [1][3]            │ Circle   (○)        │
//    [9]   │  [2][0]            │ D-pad Down          │
//   [10]   │  [2][1]            │ D-pad Right         │  Товч бүр:
//   [11]   │  [2][2]            │ D-pad Up            │    0 = дарагдаагүй
//   [12]   │  [2][3]            │ D-pad Left          │    1 = дарагдсан
//   [13]   │  [3][0]            │ L1                  │
//   [14]   │  [3][1]            │ R1                  │
//   [15]   │  [3][2]            │ L2 (товч)           │
//   [16]   │  [3][3]            │ R2 (товч)           │
//   [17]   │  [4][0]            │ Share               │
//   [18]   │  [4][1]            │ Options             │
//   [19]   │  [4][2]            │ L3 (зүүн стик дарах) │
//   [20]   │  [4][3]            │ R3 (баруун стик дарах)┘
//   [21]   │  —                 │ Холболтын флаг (1 = холбогдсон, 0 = үгүй)
//   [22]   │  —                 │ Төгсгөлийн байт (END_BYTE = 0x0A)
//
//  ⚠ АНХААР: control_data[3] = { L1, R1, L2, R2 } — L1 эхэнд байгааг март.
// ─────────────────────────────────────────────────────────────────────────────
#define PACKET_SIZE       23
#define START_BYTE        0xAA
#define END_BYTE          0x0A

bool isConnected = false;     // Контроллер яг одоо холбогдсон эсэх (loop бүрт шинэчлэгдэнэ)


// ─────────────────────────────────────────────────────────────────────────────
//  mapStick() — джойстикийн утгыг нэг байтад (uint8_t) багтаах
//
//  PS5 номын сан джойстикийн утгыг -128..127 хооронд буцаадаг.
//  Үүнийг 0..200 болгож хувиргаснаар:
//    • сөрөг тоог нэг байтад хадгалах боломжтой болно
//    • төв (хөдөлгөөнгүй) байрлал нь яг 100 болно
//  (STM32 тал дээр 100-г хасаж −100..100 болгоно.)
// ─────────────────────────────────────────────────────────────────────────────
uint8_t mapStick(int val) {
  if (val >= 0)
    return map(val,  0,  127, 100, 200);   // Эерэг тал:  төв(100) → дээд(200)
  else
    return map(val, -1, -128, 100,   0);   // Сөрөг тал:  төв(100) → доод(0)
}


// ─────────────────────────────────────────────────────────────────────────────
//  sendControllerData() — нэг багц цуглуулж Serial2-аар илгээх
//
//    isConnected = true  → бодит утгуудыг,
//    isConnected = false → бүх товч/стикийг 0-оор дүүргэж илгээнэ.
//  Ингэснээр хүлээн авагч тал тасралтгүй, тогтмол 100 Hz-ийн урсгал авна.
//  (packet[] мөр бүрийн ард → STM32 control_data-гийн очих байрлалыг тэмдэглэв.)
// ─────────────────────────────────────────────────────────────────────────────
void sendControllerData() {
  uint8_t packet[PACKET_SIZE];
  packet[0] = START_BYTE;

  if (isConnected) {
    // — Джойстикууд (→ control_data[0][0..3]) —
    packet[1] = mapStick(ps5.LStickX());   // → control_data[0][0]
    packet[2] = mapStick(ps5.LStickY());   // → control_data[0][1]
    packet[3] = mapStick(ps5.RStickX());   // → control_data[0][2]
    packet[4] = mapStick(ps5.RStickY());   // → control_data[0][3]

    // — Нүүрний товчнууд (→ control_data[1][0..3]) —
    packet[5]  = ps5.Cross();      // → control_data[1][0]
    packet[6]  = ps5.Square();     // → control_data[1][1]
    packet[7]  = ps5.Triangle();   // → control_data[1][2]
    packet[8]  = ps5.Circle();     // → control_data[1][3]

    // — Чиглэлийн товч / D-pad (→ control_data[2][0..3]) —
    packet[9]  = ps5.Down();       // → control_data[2][0]
    packet[10] = ps5.Right();      // → control_data[2][1]
    packet[11] = ps5.Up();         // → control_data[2][2]
    packet[12] = ps5.Left();       // → control_data[2][3]

    // — Мөр (shoulder) товчнууд (→ control_data[3][0..3]) —
    packet[13] = ps5.L1();   // → control_data[3][0]
    packet[14] = ps5.R1();   // → control_data[3][1]
    packet[15] = ps5.L2();   // → control_data[3][2]  (0/1; аналог бол ps5.L2Value())
    packet[16] = ps5.R2();   // → control_data[3][3]  (0/1; аналог бол ps5.R2Value())

    // — Бусад товч (→ control_data[4][0..3]) —
    packet[17] = ps5.Share();      // → control_data[4][0]
    packet[18] = ps5.Options();    // → control_data[4][1]
    packet[19] = ps5.L3();         // → control_data[4][2]
    packet[20] = ps5.R3();         // → control_data[4][3]

    packet[21] = 1;          // Холбогдсон флаг
  } else {
    for (int i = 1; i <= 20; i++) packet[i] = 0;   // Бүх товч/стикийг тэглэх
    packet[21] = 0;          // Холбогдоогүй флаг
  }

  packet[22] = END_BYTE;

  Serial2.write(packet, PACKET_SIZE);   // 23 байтыг нэг дор илгээнэ (23 удаа биш)
}


// ─────────────────────────────────────────────────────────────────────────────
//  setup() — анхны тохиргоо (нэг удаа ажиллана)
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(UART_BAUD);                          // Дибаг (USB) порт
  Serial2.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2); // Дата илгээх порт

  ps5.begin(PS5_MAC);                               // Заасан MAC-тай контроллер хайж эхлэх
  Serial.println("Waiting for PS5 controller...");
}


// ─────────────────────────────────────────────────────────────────────────────
//  loop() — үндсэн давталт (millis() ашигладаг тул блоклохгүй)
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  static unsigned long lastSend     = 0;     // Сүүлд дата илгээсэн хугацаа
  static unsigned long lastBlink    = 0;     // Сүүлд LED анивчуулсан хугацаа
  static bool          wasConnected = false; // Өмнөх давталтын холболтын төлөв

  isConnected = ps5.isConnected();

  // ── Холболтын төлөв ӨӨРЧЛӨГДСӨН агшинд л мессеж хэвлэх ──
  if (isConnected && !wasConnected) {
    Serial.println("Connected!");
    digitalWrite(LED_BUILTIN, HIGH);         // Холбогдвол LED тогтмол асна
  }
  if (!isConnected && wasConnected) {
    Serial.println("Disconnected!");
  }
  wasConnected = isConnected;

  // ── Холбогдоогүй үед LED анивчина (блоклохгүйгээр) ──
  if (!isConnected && millis() - lastBlink >= BLINK_INTERVAL_MS) {
    lastBlink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // ── Тогтмол 100 Hz-ээр хамгийн сүүлийн датаг илгээх ──
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    sendControllerData();
  }
}
