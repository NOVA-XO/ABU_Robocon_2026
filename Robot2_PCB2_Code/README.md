# ABU Robocon 2026 — Robot2 · PCB2 (Авирах / шоо-авах)

STM32F407 дээр суурилсан **автомат** роботын **PCB2** firmware. Энэ самбар нь шоо-авах
механикийг (sun/moon араа, соленоид) удирдаж, **tic-tac-toe тактик**-аар аль scroll-ыг
хэзээ авахыг шийддэг. Жолоо/авиралт/gyro нь **PCB1** дээр —
[`../Robot2_Code`](../Robot2_Code). Хоёр самбар USART2 линкээр уялдана.

> 🎮 Гар удирдлагатай робот → [`../Robot1_Code`](../Robot1_Code) ·
> монорепо тайлбар → [../README.md](../README.md)

## Онцлог (Features)

- **Sun / Moon араа** — Мотор 5 (sun, encoder0) ба Мотор 6 (moon, encoder1)-ийг тогтсон
  байрлалд PID-ээр барих (`sun_hold` / `moon_hold`). `moon_hold` нь тэвчих бүсэд зөөлөн
  P-барилттай — робот хөдлөх үед moon чөлөөт дрифт хийхээс сэргийлнэ
- **Соленоид** — `controlSolenoid(1..8)`; grab дараалалд sol1/sol2/sol5 ашиглана
- **Шоо-авах дараалал** — `grab_front_{up,down}_20_{f,b}`: sun эргүүлэх → moon грип →
  strafe (PCB1 хийнэ) → соленоид атгах. `f` = урд гар, `b` = ард гар. up-д `val5` limit
  switch-ээр эрт зогсоох
- **Tic-tac-toe тактик** — 4×3 grid дээр scroll (ST_R2)-ыг тэмдэглэн, хамгийн сайн
  баганаг (route) сонгож PCB1 руу илгээнэ. `query_grab_decision` нь робот тухайн блок
  дээрээ ДАРААГИЙН (дээд) блокийн scroll-ыг авахаар шийднэ. Grab-ийн ээлж: 1-р шоо урд
  гар, 2-р шоо ард гар (`grab_n`). Шинэ QUERY(yes)-ээр л grab зэвсэглэнэ (`grab_armed`)
- **`PCB2_Manual` (мод 0)** — гараар турших: зүүн стик Y→sun, баруун стик Y→moon;
  ▭→sol1, ○→sol2, △→sol5 (toggle)
- **`Grab_Test` (мод 2)** — grab дараалал бүрийг тусад нь турших (△/○/□/D-Down/D-Right)
- **`tictactoe` (PCB2 тал)** — PCB1-ийн tictactoe маневрын үед sol5 асаах

## PCB хоорондын линк (USART2, PCB1-тэй)

PCB2 нь **PCB1** (их бие)-тэй **USART2** дээр холбогдоно. PS5 өгөгдөл ч энэ шугамаар
PCB1-ээс дамжиж ирдэг (magic байтаар ялгана):

- **PCB1 → PCB2:** `QUERY [0xC3][block][0x0A]`, `GRAB [0xC1][0xC2][type][0x0A]`
  (type 0=down, 1=up)
- **PCB2 → PCB1:** `[0xB2]` preview, `[0xB3]` SET(route), `[0xB6]` grab-ans,
  `[0xB5]` grab-done, `[0xB7]` strafe (0=зогс/1=зүүн/2=баруун)

## Тоног төхөөрөмж (Hardware)

| Зүйл | Тодорхойлолт |
|------|--------------|
| MCU | STM32F407 |
| Дэлгэц | SSD1306 OLED (I2C2) |
| PCB1-тэй линк + PS5 | USART2 |
| Serial telemetry | UART4 (115200) — moon/grab оношилгоо |
| Мотор | Мотор 5 = sun араа, Мотор 6 = moon араа (encoder-тэй) + соленоид |

## Файлын бүтэц (Core/Src)

| Файл | Үүрэг |
|------|-------|
| `main.c` | Entry point, periphery init, USART2 линк RX (GRAB/QUERY задлах) |
| `default.c` | Горим сонголт (`selectMode`), `controlSolenoid` |
| `general.c` | Sun/moon PID, `grab_front_*` дараалал, `PCB2_Manual`, `Grab_Test`, `tictactoe` |
| `tactic.c` | Tic-tac-toe grid, route төлөвлөлт, grab шийдвэр (`query_grab_decision`) |
| `ssd1306*.c` | Гуравдагч этгээдийн сангууд |

## Build

STM32CubeIDE (CMake) төсөл:

```
cmake --preset Debug
cmake --build --preset Debug
```

Хэрэглэгчийн эх файлуудыг `CMakeLists.txt`-ийн `target_sources`-д нэмдэг.
Шинэ файл нэмсэн бол **Clean + Rebuild** (CMake reconfigure) хийнэ.

## ESP32 PS5 гүүр

PS5 нь ESP32 гүүрээр [`../ps5_esp32_bridge/`](../ps5_esp32_bridge) PCB1 руу ирж, тэндээс
USART2-оор PCB2 руу дамждаг. ESP32 кодыг Arduino IDE-гээр ачаална (STM32 build-д ороогүй).
