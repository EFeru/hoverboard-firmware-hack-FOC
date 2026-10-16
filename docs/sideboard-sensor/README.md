# Боковая плата как автономный датчик положения (SBSP v1)

Готовый набор для варианта `VARIANT_SENSOR`: плата GD32F130C6T6 отдаёт по UART
roll/pitch (у трёх плат — 6 углов: 3 элемента × roll/pitch), принимает команды
светодиодов, читает оптические датчики, умеет менять частоту потока (1…200 Гц,
по умолчанию 200) и хранит настройку в своей флеш-странице. С основной платой
гироскутера не связана никак.

Описание протокола — [`PROTOCOL.md`](PROTOCOL.md).

## Что в папке

| Файл | Что это |
|---|---|
| `PROTOCOL.md` | Спецификация SBSP v1: кадры, команды, частоты, хранение настроек |
| `sensor_proto.patch` | Патч к `EFeru/hoverboard-sideboard-hack-GD` (7 файлов + 2 новых); проверен `git apply --check` на свежем клоне |
| `files/Inc/sensor_proto.h` | Новый заголовок протокола (копия, если удобнее копировать руками) |
| `files/Src/sensor_proto.c` | Реализация протокола, флеш-настроек и светодиодов |

## Как применить

```bash
git clone https://github.com/EFeru/hoverboard-sideboard-hack-GD.git sideboard-sensor
cd sideboard-sensor
git apply /путь/до/sensor_proto.patch      # или скопировать files/* в Inc/ и Src/
pio run -e VARIANT_SENSOR                  # VS Code + PlatformIO
```

Патч меняет:

| Файл | Что именно |
|---|---|
| `Inc/config.h` | блок `VARIANT_SENSOR` (`SERIAL_SENSOR`, `SENSORS_LED_INDICATION_OFF`, `SBSP_USE_FLASH`), `MPU_DEFAULT_HZ = 200` для этого варианта |
| `Src/sensor_proto.c/.h` | **новые файлы** — протокол, CRC, флеш-настройки, светодиоды |
| `Src/util.c` | DMA TX/RX для USART_MAIN, насос байтов в приёмник, `sensor_proto_leds()` вместо штатной логики, `sensor_proto_status_bits()` (оптика + состояние IMU) |
| `Src/main.c` | вызов `sensor_proto_poll()` в главном цикле |
| `Src/mpu6050.c` | флаг «пришёл новый отсчёт DMP» (в существующей заглушке `if (new_data)`) |
| `GD32F130C6T_FLASH.ld` | FLASH 32K → **31K** (последняя страница 0x08007C00 — под настройки) |
| `platformio.ini` | окружение `VARIANT_SENSOR` |

Прошивка на плату — как обычно: ST-Link по SWD (`GND`, `SWDIO`, `SWCLK`, `3V3`),
адрес `0x08000000`; если плата ещё залочена — сначала снять защиту
(`docs/sideboards.md` §6).

## Первый пуск: чек-лист

1. Подать на 4-пиновый кабель **12–15 В** (не 5 В) и GND; USART — 3.3 В, к
   ESP32/USB-UART подключается напрямую.
2. Ожидаемое поведение в локальном режиме: LED1 ровно (IMU жив), LED4 ровно
   (поток идёт), LED5 ровно, если настройки взяты из флеша (первый пуск — по
   умолчанию, LED5 погашен; после `SET_RATE` c `save=1` — загорится).
3. Байтовый поток должен начинаться с `A5 5A 01 0A …` и идти с частотой
   **200 Гц** (период 5 мс), 15 байт на кадр.
4. Команды: `GET_INFO` → `INFO`; `SET_RATE(50, save=1)` → `ACK 0` и поток
   станет 20 мс; `SET_MODE(led_slave)` + `LED(mask, blink)` → светодиоды под
   управлением хоста.
5. Проверить знаки углов: наклон платы вокруг продольной оси даёт `roll` со
   знаком, вокруг поперечной — `pitch`. Если знаки/оси не совпадают с монтажом —
   поправить `MPU_ORIENTATION` в `Inc/config.h`.

## Заготовка приёмника для ESP32 (ESP-IDF, эскиз)

```c
#define SBSP_SOF0 0xA5
#define SBSP_SOF1 0x5A
#define SBSP_MAX_PAYLOAD 24

typedef struct {                       /* распакованный DATA/DATA_EXT */
    uint8_t  type, seq, status;
    uint16_t t_ms;
    int16_t  roll, pitch, gz;          /* °×100, °×100, raw gyro */
    int16_t  yaw, gx, gy, ax, ay, az;  /* только для DATA_EXT */
    uint32_t last_ms;                  /* local receive time */
} sbsp_sample_t;

static uint8_t sbsp_crc8(const uint8_t *p, int n)
{
    uint8_t crc = 0;
    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* вызывать из задачи, читающей uart: возвращает true, когда кадр разобран */
static bool sbsp_parse(const uint8_t *buf, int len, uint8_t type, uint8_t plen,
                       const uint8_t *pl, uint8_t crc_rx)
{
    uint8_t t[3 + SBSP_MAX_PAYLOAD];
    t[0] = type; t[1] = plen;
    for (int i = 0; i < plen; i++) t[2 + i] = pl[i];
    return sbsp_crc8(t, 2 + plen) == crc_rx;
}

/* отправка команды (пример: частота + сохранение) */
static void sbsp_send_set_rate(uart_port_t u, uint16_t hz, uint8_t save)
{
    uint8_t f[8] = { SBSP_SOF0, SBSP_SOF1, 0x10, 3, (uint8_t)(hz & 0xFF),
                     (uint8_t)(hz >> 8), save, 0 };
    uint8_t t[5] = { f[2], f[3], f[4], f[5], f[6] };
    f[7] = sbsp_crc8(t, 5);
    uart_write_bytes(u, (const char *)f, sizeof(f));
}
```

Остальные команды собираются так же: `A5 5A | type | len | payload | CRC`.
Разбор лучше делать автоматом состояний по байтам (как в `sensor_proto.c`), а не
поиском по буферу: так потери и склейки кадров не ломают поток.

## Что стоит проверить и о чём сообщить

* размер сборки (`pio run -e VARIANT_SENSOR` покажет Flash/RAM): влезает ли
  в 31 КБ, или нужно убирать `SBSP_USE_FLASH`;
* что `SET_RATE(50, save=1)` переживает выключение питания (LED5 должен
  загореться после перезапуска — это признак, что настройки прочитаны из флеша);
* реальная частота кадров на 200 Гц (осциллограф/логический анализатор или
  счётчик по `seq` на ESP32) — DMP должен держать 5 мс;
* стабильность на конце кабеля луча: длина, помехи от моторов, нужен ли экран;
* знаки и ориентация осей на конкретном монтаже платы.
