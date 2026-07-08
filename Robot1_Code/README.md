# ABU Robocon 2026 — Robot1 (Гар удирдлагатай робот)

STM32F407 дээр суурилсан **гар удирдлагатай (manual)** роботын firmware. PS5
контроллероор (ESP32 гүүрээр дамжуулан) mecanum жолоодлого хийнэ. Автомат
хэсэг (gyro, sequence, red/blue) энэ роботод **байхгүй**.

> 🤖 Автомат робот → [`../Robot2_Code`](../Robot2_Code) ·
> хоёул нэг ижил PCB дээр ажиллана → [../README.md](../README.md)

## Онцлог (Features)

- **Mecanum жолоо** — 4 моторын inverse kinematics (`runner`)
- **Rack** — Мотор 5, 6-г encoder-аар байрлалд барих PID (гараар ашиглаж болно)
- **PS5 контроллер** — ESP32 гүүр 23 байтын packet-ийг 100 Hz-ээр UART-аар илгээнэ
- **Периферал** — SSD1306 OLED, PCA9685, соленоид, серво, brush

## Тоног төхөөрөмж (Hardware)

| Зүйл | Тодорхойлолт |
|------|--------------|
| MCU | STM32F407 |
| Дэлгэц | SSD1306 OLED (I2C2) |
| Контроллер | PS5 → ESP32 → UART (USART3) |
| Мотор | 6 × DC (encoder-тэй), серво, brush, соленоид |

## Файлын бүтэц (Core/Src)

| Файл | Үүрэг |
|------|-------|
| `main.c` | Entry point, periphery init, PS5 packet задлах, гар удирдлагын гогцоо |
| `default.c` | Суурь драйвер (мотор, соленоид, серво, UART) |
| `general.c` | Mecanum (`runner`) + rack PID |
| `test.c` | Тоног төхөөрөмжийн тест функцууд |
| `pca9685.c`, `ssd1306*.c` | Гуравдагч этгээдийн сангууд |

## Контроллерын буулгалт (control_data)

PS5 packet-ийг STM32 тал дээр `control_data[5][4]`-д задалдаг:

- `[0][0..3]` — джойстик: LStickX, LStickY, RStickX, RStickY (−100..100)
- `[1][0..3]` — Cross, Square, Triangle, Circle
- `[2][0..3]` — D-pad: Down, Right, Up, Left
- `[3][0..3]` — **L1, R1, L2, R2**
- `[4][0..3]` — Share, Options, L3, R3

## Build

CMake төсөл (STM32 VS Code extension / CMake preset):

```
cmake --preset Debug
cmake --build --preset Debug
```

Шинэ эх файл нэмсэн бол `CMakeLists.txt`-ийн `target_sources`-д нэмж, **Clean + Rebuild**.

## ESP32 PS5 гүүр

Хоёр роботод нийтлэг ESP32 гүүр репо root-д байна:
[`../ps5_esp32_bridge/`](../ps5_esp32_bridge). Arduino IDE-гээр ESP32-д ачаална
(STM32 build-д ороогүй).
