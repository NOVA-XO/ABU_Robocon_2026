# ABU Robocon 2026 — Robot2 · PCB1 (Автомат робот, их бие)

STM32F407 дээр суурилсан **автомат** роботын **PCB1** (их бие) firmware. Mecanum жолоо,
encoder-т суурилсан rack PID, LPMS-BE2 gyro-гоор чиг баримжаа, автомат авиралт (climb),
цэнхэр талын weapon дараалал зэргийг агуулна. Шоо-авах механик (sun/moon араа, соленоид)
нь **PCB2** дээр — [`../Robot2_PCB2_Code`](../Robot2_PCB2_Code).

> 🎮 Гар удирдлагатай робот → [`../Robot1_Code`](../Robot1_Code) ·
> монорепо тайлбар → [../README.md](../README.md)

## Онцлог (Features)

- **Mecanum жолоо** — 4 моторын inverse kinematics (`runner`); урагш = СӨРӨГ PWM.
  Gyro-гоор шулуун явах (`Drive_Straight`), gyro-strafe (`Strafe_Gyro`)
- **Rack PID** — front/back рак (Мотор 5, 6)-г encoder-аар тогтсон байрлалд PID-ээр барих.
  TIM7 ISR (`Rack_Service`) дотор тасралтгүй ажиллаж, зогсож байхад ч барина
- **Gyro (LPMS-BE2)** — UART stream, харьцангуй өнцгөөр эргэх (`Gyro_TurnAngle`),
  anchor чиг баримжаа (`Set_Yaw_Anchor` / `Get_Yaw_Offset_From_Anchor`)
- **Зайн PID** — хойд 2 дугуйн encoder-аар тодорхой зай явах (`Drive_Distance`),
  gyro + дугуй-balance-тай
- **Автомат авиралт (`auto_climb`)** — PCB2-оос ирсэн route (багана)-аар минхуа ойн
  тавцангуудыг авирах: алхам бүр `MOVE → QUERY → GRAB → FINISH`. Робот тухайн блок дээрээс
  ДАРААГИЙН (дээд) блокийн scroll-ыг авна; grab-ийн төрөл (up/down) scroll-ийн блокоос
  тодорхойлогдоно. Төгсгөлд нь минхуагаас гарах маневр (exit)
- **`tictactoe`** — exit-ийн дараах үргэлжлэл: gyro+дугуй-balance шулуун урагш → `val5==0`
  → strafe → ухрах → рак сунах
- **weapon (blue)** — цэнхэр талын автомат зэвсгийн дараалал (`weapon_blue`):
  урагш → рак → val3/val8 align → grab → 180° эргэх → рак
- **Тест горимууд** — `Drive_Distance_Test`, `Blue_Go_Test`, `Exit_Test`, `Tic_Tac_Toe_Test`
- **PS5 контроллер** — ESP32 гүүр 23 байтын packet-ийг 100 Hz-ээр UART-аар илгээнэ

## PCB хоорондын линк (USART2, PCB2-той)

PCB1 нь **PCB2** (авирах самбар)-той **USART2** дээр хоёр чиг холбогдоно. Мөн PS5-ийг
PCB2 руу энэ шугамаар дамжуулна (magic байтаар ялгана):

- **PCB1 → PCB2:** `QUERY [0xC3][block][0x0A]` (энэ блокт grab уу?),
  `GRAB [0xC1][0xC2][type][0x0A]` (grab эхлүүл; type 0=down, 1=up)
- **PCB2 → PCB1:** `[0xB3]` SET(route), `[0xB6]` grab-ans, `[0xB5]` grab-done,
  `[0xB7]` strafe(0/1/2)

## Тоног төхөөрөмж (Hardware)

| Зүйл | Тодорхойлолт |
|------|--------------|
| MCU | STM32F407 |
| Gyro/IMU | LPMS-BE2 (UART4) |
| Дэлгэц | SSD1306 OLED (I2C2) |
| Контроллер | PS5 → ESP32 → UART (USART3) |
| PCB2-той линк | USART2 |
| Мотор | 4 × mecanum (encoder-тэй) + front/back rack (M5/M6) |

## Файлын бүтэц (Core/Src)

| Файл | Үүрэг |
|------|-------|
| `main.c` | Entry point, periphery init, PS5 задлах, PCB2 линк (`Link_Send_Grab` г.м) |
| `default.c` | Горим сонголт (`selectMode`) + тест горимууд |
| `general.c` | Mecanum, rack PID, gyro эргэлт/strafe, `Drive_Distance`, `TTT_Drive` |
| `lpms.c` | LPMS-BE2 IMU драйвер + gyro |
| `sequence.c` | Non-blocking блокууд (`up_20/down_20/up_40/down_40`), `auto_climb`, `tictactoe`, тестүүд |
| `blue.c` | Цэнхэр талын автомат зэвсэг (`weapon_blue`) |
| `red.c` | Улаан талын автомат код |
| `pca9685.c`, `ssd1306*.c` | Гуравдагч этгээдийн сангууд |

## Контроллерын буулгалт (control_data)

PS5 packet-ийг STM32 тал дээр `control_data[5][4]`-д задалдаг:

- `[0][0..3]` — джойстик: LStickX, LStickY, RStickX, RStickY (−100..100)
- `[1][0..3]` — Cross, Square, Triangle, Circle
- `[2][0..3]` — D-pad: Down, Right, Up, Left
- `[3][0..3]` — **L1, R1, L2, R2**
- `[4][0..3]` — Share, Options, L3, R3

## Build

STM32CubeIDE (CMake) төсөл:

```
cmake --preset Debug
cmake --build --preset Debug
```

Хэрэглэгчийн эх файлуудыг `CMakeLists.txt`-ийн `target_sources`-д нэмдэг.
Шинэ файл нэмсэн бол **Clean + Rebuild** (CMake reconfigure) хийнэ.

> ⚠️ `auto_climb`, `Drive_Straight`, `tictactoe` зэрэг нь LPMS gyro-г ашигладаг тул
> LPMS идэвхтэй байх ёстой. Тэдгээр горимд UART4 serial telemetry LPMS-тэй зөрчилддөг.

## ESP32 PS5 гүүр

Хоёр роботод нийтлэг ESP32 гүүр репо root-д: [`../ps5_esp32_bridge/`](../ps5_esp32_bridge).
ESP32 дээр Arduino IDE-гээр ачаална (STM32 build-д ороогүй). PS5 контроллерийг
Bluetooth-оор холбож, 23 байтын packet-ийг UART-аар STM32 руу илгээнэ.
