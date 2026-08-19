# ABU Robocon 2026 — Robot1 (Гар удирдлагатай робот)

STM32F407 дээр суурилсан **гар удирдлагатай (manual)** роботын firmware. PS5
контроллероор (ESP32 гүүрээр дамжуулан) **differential (tank)** жолоодлого, соленоид,
мотор удирдлага хийнэ. Автомат хэсэг (gyro, sequence, red/blue) энэ роботод **байхгүй**.

> 🤖 Автомат робот → [`../Robot2_Code`](../Robot2_Code) ·
> монорепо тайлбар → [../README.md](../README.md)

## 2 PCB (нэг төсөл, `R1_PCB` сонголт)

Робот 1 нь **2 PCB**-тэй ч код бага тул нэг төсөлд 2 функцээр багтсан. `main.c`-ийн
`#define R1_PCB 1/2`-аар аль PCB-ийн firmware болохыг **build-ийн өмнө** сонгоно:

| `R1_PCB` | Функц | Үүрэг |
|----------|-------|-------|
| **1** | `robot1_pcb1()` | Жолоо (differential) + соленоид + мотор 5/6. PS5-ийг хүлээж авна |
| **2** | `robot1_pcb2()` | Мотор 1/2/3-ыг PCB1-ээс UART4 линкээр ирсэн командаар эргүүлнэ |

**PCB1 → PCB2 линк (UART4):** `[0x0A][p1][p2][p3][0x0D]`, кодчилол `p = 100 + pwm/10`
(нейтрал = 100). **p1→M2** (тулгуур), **p2→M3**, **p3→M1**. 300мс дотор багц ирэхгүй
бол PCB2 моторуудаа зогсооно (аюулгүй).

## Онцлог (Features)

- **Differential (tank) жолоо** — зүүн/баруун талын mixing (`runner`); урагш ба эргэлт
  тус тусын коэффициенттэй (`SPEED_GAIN`, `TURN_GAIN`, `PWM_MAX`)
- **Соленоид** — L1/L2/R1 toggle (debounce); D-Left/D-Right тусгай (доор үз); sol7 байнга ON
- **R2 shift давхарга** — Triangle/Cross ба D-Up/D-Down-ийн үйлдлийг R2 дарсан эсэхээр солино
- **PCB хоорондын линк** — PCB1 мотор командыг UART4-өөр PCB2 руу 50 Hz-ээр илгээнэ

## Удирдлага (PCB1)

| Оролт | R2 сул | R2 дарсан |
|-------|--------|-----------|
| **Зүүн/баруун стик** | differential жолоо (`runner`) | — |
| **Triangle / Cross** | мотор **5** (±400) | **M3** (PCB2, ±600). Cross + sol1 OFF → M3 нь `val1==0` болтол −600, дараа зогсоно; sol1 ON → тогтмол −600 |
| **D-Up / D-Down** | мотор **6** (∓800) | **M1** (PCB2, ±600) |
| **Square / Circle** | **M2** тулгуур (PCB2, ±600) | ← адил |
| **L1 / L2 / R1** | соленоид 1 / 2 / 3+4 (toggle, debounce) | ← адил |
| **D-Left** | соленоид **6** — нэг даралт → **2 сек** асаад авто унтарна | ← адил |
| **D-Right** | соленоид **5** — momentary (дарвал ON, тавьвал OFF) | ← адил |
| *(эхлэхэд)* | соленоид **7** байнга ON | ← адил |

## Тоног төхөөрөмж (Hardware)

| Зүйл | Тодорхойлолт |
|------|--------------|
| MCU | STM32F407 (2 PCB-д ижил) |
| Дэлгэц | SSD1306 OLED (I2C2) |
| Контроллер | PS5 → ESP32 → UART (USART3) |
| PCB-хоорондын линк | UART4 (PCB1 TX → PCB2 RX) |
| Мотор | PCB1: жолоо 4×DC + M5/M6 ; PCB2: M1/M2/M3 |

## Файлын бүтэц (Core/Src)

| Файл | Үүрэг |
|------|-------|
| `main.c` | Entry point, `R1_PCB` сонголт, PS5 задлах, UART4 линк RX/TX, гол гогцоо |
| `general.c` | `runner` (жолоо), `solenoidControl`, `robot1_pcb1` / `robot1_pcb2` |
| `default.c` | Суурь драйвер (мотор, соленоид, серво, UART) |
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

> **PCB сонгох:** build-ийн өмнө `main.c`-ийн `#define R1_PCB`-ыг **1** эсвэл **2**
> болгож, тус бүрд нь дахин build хийж холбогдох самбарт флаш хийнэ.

Шинэ эх файл нэмсэн бол `CMakeLists.txt`-ийн `target_sources`-д нэмж, **Clean + Rebuild**.

## ESP32 PS5 гүүр

Хоёр роботод нийтлэг ESP32 гүүр репо root-д байна:
[`../ps5_esp32_bridge/`](../ps5_esp32_bridge). Arduino IDE-гээр ESP32-д ачаална
(STM32 build-д ороогүй).
