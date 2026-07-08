# Hardware — PCB (EasyEDA)

**Robot1 (гар)** ба **Robot2 (автомат)** хоёул **НЭГ ижил PCB** дээр угсрагдсан.
Үндсэн MCU: **STM32F407VET6**. PS5 гүүрийн **ESP32** залгаастай. EasyEDA, REV 1.0.

## PCB layout

![PCB layout](pcb.png)

## Функциональ блокууд (schematic)

| Sheet | Блок | Тайлбар |
|-------|------|---------|
| 1 | **Main / MCU** | STM32F407VET6 (U4), ESP32-DevKit (U2), SSD1306 OLED (I2C2), BNO055 талбай (I2C), buzzer, UART/I2C KF2510 холбогч |
| 2 | **Sensors** | SIN1–8 дижитал сенсор, 10k pull-up → **74HC245** буфер (U17) → SD1–8; SEN1–8 (KF2510-3A) холбогч |
| 3 | **Power** | XT60 (U15) → унтраалт → **DC-DC 5V** (U3) + **AMS1117-3.3** (U21/U18); power LED |
| 4 | **Encoders** | 4× encoder (KF2510-4A); 10k pull-up + 470Ω + 100pF → EN1–4_IN / EN1–4_EXT (EXTI) |
| 5 | **Solenoid out** | Opp1–8 → **PC817** opto (U42/U5) → **ULN2803APG** (U34) → Opp_Out1–8 (+12V, 4pin + LED) |
| 6 | **Motor drivers** | 3× **Dual VNH5019** shield (U19/U7/U12) → 6 мотор (M1–6: PWM + INA/INB + EN), brush |

> STM32-ийн бүрэн пин хуваарь → [`../Robot2_Code/Core/Inc/main.h`](../Robot2_Code/Core/Inc/main.h)
> доторх `#define ..._Pin` (Robot1 мөн адил).

### Sheet 1 — Main / MCU
![Sheet 1 — Main](Sheet1_main.png)

### Sheet 2 — Sensors
![Sheet 2 — Sensors](Sheet2_sensors.png)

### Sheet 3 — Power
![Sheet 3 — Power](Sheet3_power.png)

### Sheet 4 — Encoders
![Sheet 4 — Encoders](Sheet4_encoders.png)

### Sheet 5 — Solenoid out
![Sheet 5 — Solenoid](Sheet5_solenoid.png)

### Sheet 6 — Motor drivers
![Sheet 6 — Motor drivers](Sheet6_motor.png)

## Гироскопын тэмдэглэл

PCB дээр **BNO055** (I2C) талбай бий ч, одоогийн Robot2 firmware **LPMS-BE2**-ийг
**UART4** (UART KF2510 холбогч)-ээр ашиглаж байгаа. BNO055 нь хуучин / нөөц.

## 3D загвар (Git LFS)

`pcb_3d.obj` (21 MB) нь **Git LFS**-ээр хадгалагдсан. Clone хийхэд бодит файлыг
татахын тулд **git-lfs суусан** байх ёстой:

```
git lfs install
git clone https://github.com/NOVA-XO/ABU_Robocon_2026.git
```

git-lfs байхгүй бол зөвхөн заагч (pointer) файл татагдана. GitHub 3D-г README-д
шууд харуулахгүй — татаж аваад `.mtl`-тэй хамт 3D viewer-т нээж үзнэ.
