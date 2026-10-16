# AP_Periph для боковой платы гироскутера — готовый набор

Здесь лежат файлы, которые нужно перенести в **новый** репозиторий, чтобы GitHub
Actions собрал прошивку DroneCAN-узла и CAN-загрузчик. Разбор и обоснования —
в [`../ap-periph-sideboard.md`](../ap-periph-sideboard.md).

## Состав

| Файл | Куда в новом репозитории |
|---|---|
| `hwdef.dat` | `hwdef/HoverboardSB-F103/hwdef.dat` |
| `hwdef-bl.dat` | `hwdef/HoverboardSB-F103/hwdef-bl.dat` |
| `ardupilot-sideboard.patch` | `patches/ardupilot-sideboard.patch` |
| `build.yml` | `.github/workflows/build.yml` |

Имя платы (`HoverboardSB-F103`) должно совпадать в трёх местах: имя каталога с
hwdef, переменная `BOARD` в `build.yml`.

## Порядок действий

1. Создайте репозиторий (например `hoverboard-sideboard-AP_Periph`), скопируйте
   4 файла по таблице выше. Локально ничего собирать не нужно — всё делает CI.
2. `Actions → Build AP_Periph (hoverboard sideboard) → Run workflow`.
3. В артефактах появятся:
   * `AP_Periph.bin` — сама прошивка,
   * `AP_Periph.apj` — она же для загрузки через DroneCAN GUI Tool / Mission Planner,
   * `<board>_bl.bin` — загрузчик,
   * `AP_Periph.elf` — для отладки.
4. Смотрите шаг **«Report sizes»** в логе: если `AP_Periph.bin` больше ~103 КБ —
   на вашем чипе 64 КБ флеша, и этот путь не подойдёт (нужен 128 КБ).

## Прошивка платы (первый раз — только ST-Link)

```
st-flash write <board>_bl.bin      0x08000000     # загрузчик
st-flash write AP_Periph.bin       0x08006400     # приложение (25 КБ резерва)
```

В STM32CubeProgrammer / ST-Link Utility — то же самое: загрузчик по адресу
`0x08000000`, приложение по `0x08006400`.

Дальше обновлять прошивку можно **по CAN** (DroneCAN GUI Tool или Mission
Planner) — если раскомментирован ремап и CAN подключён к тем же пинам, что
использует загрузчик.

## Подключение CAN

```
Боковая плата                    SN65HVD230 (3.3 В)
  PA12 (CAN_TX) ───────────────►  D  (TXD)
  PA11 (CAN_RX) ◄───────────────  R  (RXD)
  3V3 с платы   ───────────────►  VCC
  GND           ───────────────►  GND
                                  CANH/CANL ──► шина, 120 Ω на концах
```

Если паять к выводам MCU не хочется — перенесите CAN на `PB8/PB9`
(LED-разъёмы J3/J4), но тогда:

* снимите базовые резисторы (или транзисторы) каналов LED2/LED3 — иначе
  база-эмиттерный диод зажмёт линию CANRX на ~0.7 В и приём не заработает;
* раскомментируйте `AFIO_MAPR_CAN_REMAP_REMAP2` в `ardupilot-sideboard.patch`
  (в **обоих** местах — приложение и загрузчик) и строки `PB8/PB9` в `hwdef.dat`
  и `hwdef-bl.dat`.

## Настройка узла после первого запуска

* Node ID — автоматически (DroneCAN dynamic allocation) либо вручную.
* 5 ключей = реле DroneCAN: на узле `RELAYx_FUNCTION = 10…25` (это
  `DroneCAN_HARDPOINT_0…15`), `RELAYx_PIN` = номер пина из `hwdef.dat`
  (`GPIO(81)`…`GPIO(85)`).
* На стороне автопилота: `RELAYx_FUNCTION = 10…15` (hardpoint 0…5), дальше —
  `MAV_CMD_DO_SET_RELAY` из миссии/Lua/GCS.

## Что учесть

* Кварца на плате нет → тактирование от HSI (патч). CAN на HSI работает, но
  точность ±1 % близка к пределу — при ошибках шины добавьте кварц 8 МГц.
* `PB4` (LED5) освобождается только потому, что патч отключает JTAG (SWD
  остаётся рабочим) — без этого вывод недоступен как GPIO.
* Если раскомментирован IMU (`AP_PERIPH_IMU_ENABLED`), помните: `ahrs.RawIMU`
  потребляет PX4, а ArduPilot — нет.
