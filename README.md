# ABU Robocon 2026 — Robot Firmware

Багийн **хоёр роботын** STM32F407 firmware-ийг нэг репод (monorepo) агуулна.
Робот бүр **2 PCB**-тэй бөгөөд самбарууд хоорондоо UART линкээр холбогдоно.
Хоёр роботыг PS5 контроллероор (ESP32 Bluetooth гүүрээр дамжуулан) удирдана.

## Бүтэц

| Фолдер | Робот / PCB | Тайлбар |
|--------|-------------|---------|
| [`Robot1_Code/`](Robot1_Code) | **Робот 1** — гар удирдлагатай (2 PCB) | Нэг төсөл; `#define R1_PCB 1/2`-аар PCB сонгож build. Differential (tank) жолоо + соленоид + мотор |
| [`Robot2_Code/`](Robot2_Code) | **Робот 2 — PCB1** (их бие) | Mecanum жолоо, rack PID, LPMS gyro, climb/exit/tictactoe, weapon (blue) |
| [`Robot2_PCB2_Code/`](Robot2_PCB2_Code) | **Робот 2 — PCB2** (авирах/шоо-авах) | Sun/moon араа, соленоид, tic-tac-toe тактик, шоо-авах дараалал |
| [`ps5_esp32_bridge/`](ps5_esp32_bridge) | *(нийтлэг)* | ESP32 PS5→UART гүүр (Arduino) |
| [`hardware/`](hardware) | *(нийтлэг)* | PCB (EasyEDA) схем / зураг |

## Систем (архитектур)

```
PS5 pad ──BT──▶ ESP32 (ps5_esp32_bridge) ──23 байт / 100 Hz UART──▶ STM32 (PCB1)
                                                                        │
                                                  PCB хоорондын UART линк│
                                                                        ▼
                                                                     STM32 (PCB2)
```

ESP32 гүүр PS5-ийн джойстик/товчийг 23 байтын packet болгож STM32 руу илгээнэ.
STM32 тал `control_data[5][4]`-д задалж, өөрийн удирдлагадаа ашиглана.

### PCB хоорондын линк (робот бүрд өөр)

- **Робот 1** — PCB1 нь PS5-ийг хүлээн авч, мотор командыг **UART4**-өөр PCB2 руу
  дамжуулна (нэг чиг): `[0x0A][p1][p2][p3][0x0D]`, кодчилол `p = 100 + pwm/10`
  (нейтрал = 100). p1→M2, p2→M3, p3→M1.
- **Робот 2** — PCB1 (их бие) ↔ PCB2 (авирах) **USART2** дээр хоёр чиг линк. PCB1 нь
  PS5-ийг PCB2 руу мөн энэ шугамаар дамжуулна (magic байтаар ялгана):
  PCB1→PCB2 `QUERY [0xC3][block][0x0A]`, `GRAB [0xC1][0xC2][type][0x0A]`;
  PCB2→PCB1 `SET/ans/done/strafe [0xB_][…][0x0A]`.

## Тоног төхөөрөмж

- **MCU:** STM32F407 (бүх самбарт ижил)
- **Контроллер:** PS5 → ESP32 (Bluetooth) → UART (USART3)
- **Дэлгэц:** SSD1306 OLED (I2C2)
- **IMU (зөвхөн Robot2 PCB1):** LPMS-BE2 (UART4)
- **Мотор:** DC (encoder-тэй), серво, соленоид; Robot2 PCB2-т sun/moon араа

## PCB

Самбаруудыг EasyEDA дээр зурсан. Схем/зургийг [`hardware/`](hardware) дотор
хадгална (зураг нэмэгдэхэд доор харагдана):

![PCB](hardware/pcb.png)

## Build

Төсөл бүр тусдаа CMake. Тухайн фолдерт нь ороод:

```
cmake --preset Debug
cmake --build --preset Debug
```

> **Робот 1:** нэг төсөл, хоёр PCB. `Robot1_Code/Core/Src/main.c`-ийн
> `#define R1_PCB` -ыг **1** (жолоо/актуатор) эсвэл **2** (нэмэлт мотор) болгож,
> тус бүрд нь дахин build хийж флаш хийнэ.

VS Code дээр **бүх төслийг зэрэг нээх:** `ABU_Robocon_2026.code-workspace`
(File → Open Workspace from File…). Status bar-аас идэвхтэй project-оо
сонгож build/debug хийнэ.

## ESP32 гүүр

[`ps5_esp32_bridge/`](ps5_esp32_bridge) — Arduino IDE-гээр ESP32-д ачаална
(STM32 build-д ороогүй). Packet-ийн бүтцийг файл дотор дэлгэрэнгүй тайлбарласан.
