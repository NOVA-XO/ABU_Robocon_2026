# ABU Robocon 2026 — Robot2 (Автомат робот)

STM32F407 дээр суурилсан **автомат** роботын firmware. PS5 контроллероор (ESP32
гүүрээр дамжуулан) удирдах, mecanum жолоо, encoder-т суурилсан rack PID,
LPMS-BE2 gyro-гоор чиг баримжаа барих, OLED дэлгэц зэргийг агуулна.

> 🎮 Гар удирдлагатай робот → [`../Robot1_Code`](../Robot1_Code) ·
> хоёул нэг ижил PCB дээр ажиллана → [../README.md](../README.md)

## Онцлог (Features)

- **Mecanum жолоо** — 4 моторын inverse kinematics (`runner`)
- **Rack PID** — Мотор 5, 6-г encoder-аар тогтсон байрлалд PID-ээр барих.
  TIM7 ISR (`Rack_Service`) дотор тасралтгүй ажиллаж, зогсож байхад ч байрлалыг барина.
- **Gyro (LPMS-BE2)** — UART stream, харьцангуй өнцгөөр эргэх (`Gyro_TurnAngle`),
  шулуун явах (`Drive_Straight`), anchor чиг баримжаа
- **Автомат дараалал** — non-blocking sequence блокууд (`up_20/down_20/up_40/down_40`),
  red/blue талын угсралт
- **PS5 контроллер** — ESP32 гүүр 23 байтын packet-ийг 100 Hz-ээр UART-аар илгээнэ
- **Периферал** — SSD1306 OLED, PCA9685, соленоид, серво, BLDC

## Тоног төхөөрөмж (Hardware)

| Зүйл | Тодорхойлолт |
|------|--------------|
| MCU | STM32F407 |
| Gyro/IMU | LPMS-BE2 (UART4) |
| Дэлгэц | SSD1306 OLED (I2C2) |
| Контроллер | PS5 → ESP32 → UART (USART3) |
| Мотор | 6 × DC (encoder-тэй), серво, BLDC, brush |

## Файлын бүтэц (Core/Src)

| Файл | Үүрэг |
|------|-------|
| `main.c` | Entry point, periphery init, PS5 packet задлах |
| `default.c` | Суурь драйвер (мотор, соленоид, серво, UART) |
| `general.c` | Өндөр түвшний хөдөлгөөн (mecanum, rack PID, gyro эргэлт) |
| `lpms.c` | LPMS-BE2 IMU драйвер + gyro PID |
| `sequence.c` | Нийтлэг non-blocking дарааллын блокууд |
| `red.c` / `blue.c` | Улаан / цэнхэр талын автомат код |
| `test.c` | Тоног төхөөрөмжийн тест функцууд |
| `bno055.c`, `pca9685.c`, `ssd1306*.c` | Гуравдагч этгээдийн сангууд |

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
cmake -S . -B build -G Ninja
cmake --build build
```

Хэрэглэгчийн эх файлуудыг `CMakeLists.txt`-ийн `target_sources`-д нэмдэг.
Шинэ файл нэмсэн бол **Clean + Rebuild** (CMake reconfigure) хийнэ.

## ESP32 PS5 гүүр

Хоёр роботод нийтлэг ESP32 гүүр репо root-д: [`../ps5_esp32_bridge/`](../ps5_esp32_bridge).
ESP32 дээр Arduino IDE-гээр ачаална (STM32 build-д ороогүй). PS5 контроллерийг
Bluetooth-оор холбож, 23 байтын packet-ийг UART-аар STM32 руу илгээнэ.
