# VARIANT_UARTBUS: RemoteUartBus and CANSimple over UART

`VARIANT_UARTBUS` makes this board a node on the serial bus of the split-board firmwares
[Hoverboard-Firmware-Hack-Gen2.x-GD32](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32)
(`REMOTE_UARTBUS`, `REMOTE_CANSIMPLE`) and Hoverboard-Firmware-Hack-Gen2.x-MM32. Two framings share the bus. Each has
its own option with the same name as on the GD32 (see [Building](#building)), and by default the board reads both at
the same time:

- **v1 frames** (RemoteUartBus): start byte `/`, answered with a 15 byte telemetry frame
- **CANSimple over UART**: ODrive's [CANSimple](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html)
  messages, one per frame: `0xA5 | node id | cmd id (bit 7 = request) | length | payload | CRC-16`

Every multi-byte field is little endian, including the CRC. CRC-16/XMODEM (poly 0x1021, init 0) covers the whole v1
frame without its CRC, and for CANSimple the bytes from the node id to the end of the payload.

## Two ids per board

A split board drives one motor and has one id. This board drives two, so it has **two ids**, one per motor:

| config.h | default | motor |
|---|---|---|
| `UARTBUS_ID_LEFT`  | 0 | left motor (ODrive "axis") |
| `UARTBUS_ID_RIGHT` | 1 | right motor |

Each id is an axis of its own, with its own arm state, control mode, set point, watchdog and limits. To a master the
board looks like two split boards. With several boards on one bus, give every motor its own id (for example
0/1, 2/3, ...). The ids are 0..62. The CANSimple broadcast id 63 reaches both motors, and nothing answers on it.

## Configuration (Inc/config.h, section VARIANT_UARTBUS)

| setting | default | |
|---|---|---|
| `UARTBUS_SERIAL_USART3` / `UARTBUS_SERIAL_USART2` | USART3 | right sensor cable (PB10 TX, PB11 RX, 5 V tolerant) / left cable (PA2 TX, PA3 RX, **3.3 V only**) |
| `UARTBUS_BAUD` | 115200 | 8N1 |
| `UARTBUS_TX_OPEN_DRAIN` | on | TX is open drain, so several boards can share the master's RX line |
| `UARTBUS_TIMEOUT_MS` | 500 | watchdog of an armed motor (CANSimple endpoint `watchdog_timeout`) |
| `UARTBUS_HALL_STEPS` | 90 | hall steps per turn (6 per pole pair, 15 pole pairs); CANSimple positions are in turns |
| `UARTBUS_TORQUE_CONST` | 0.34 Nm/A | converts the CANSimple torques to motor current |
| `UARTBUS_VEL_LIMIT` | 3 turns/s | `vel_limit`: top speed in position mode and for Set_Input_Vel |
| `UARTBUS_POS_GAIN` | 5 (turns/s)/turn | `pos_gain` of the position mode |

`CTRL_TYP_SEL` and `CTRL_MOD_REQ` set the start mode of both motors (default FOC, voltage). `I_MOT_MAX` and
`N_MOT_MAX` set the scale of the torque and speed commands, as in the other variants.

## Building

Only `make` and the Arm GCC toolchain are needed:

```
make -e VARIANT=VARIANT_UARTBUS                            # both protocols (default)
make -e VARIANT=VARIANT_UARTBUS REMOTE=REMOTE_UARTBUS      # v1 frames only
make -e VARIANT=VARIANT_UARTBUS REMOTE=REMOTE_CANSIMPLE    # CANSimple only
```

Run `make clean` when you switch `VARIANT` or `REMOTE`: the objects do not depend on the command line. Instead of
`REMOTE=` you can also comment out `REMOTE_UARTBUS` or `REMOTE_CANSIMPLE` in config.h. The PlatformIO environment
`VARIANT_UARTBUS` builds both protocols.

## Wiring

- Master TX goes to RX on every board. TX of every board goes to master RX. Connect GND too, and **never** the
  cable's 12/15 V pin.
- With `UARTBUS_TX_OPEN_DRAIN` the **master RX needs a pull-up**. An FT232R has one built in. On an ESP32 call
  `pinMode(RX, INPUT_PULLUP)` after `Serial.begin()`, or fit 4.7–10 kΩ to the master's supply. Comment the setting
  out for push-pull TX when this board is the only node.
- Logic levels: 3.3 V. USART3 accepts 5 V on RX and on the open-drain TX pull-up; USART2 does not.

## Arming and safety

- Both motors **start disarmed**: gates off, the motor coasts. This is ODrive's IDLE at startup.
- A motor is armed by CANSimple Set_Axis_State 8 (CLOSED_LOOP_CONTROL), or by a v1 command frame (type 0 / 1) with
  state bit 64 clear. Set_Axis_State 1, or a v1 frame with state bit 64 set, disarms it. Arming starts from set
  point 0, and position mode starts from the current position.
- **Watchdog**: if an armed motor gets no set point for `watchdog_timeout`, its set point becomes 0, or in position
  mode it holds where it is. It stays armed. The board beeps 3 times while that lasts.
- **Estop** disarms with ESTOP_REQUESTED. A motor error disarms that motor: hall sensors with MISSING_ESTIMATE, a
  blocked motor with SPINOUT_DETECTED. The reason stays latched until Clear_Errors, and arming is refused until then.
  As in the other variants, any motor error also switches both motors off while it lasts.
- v1 state bit 128, or the endpoint `power_off`, switches the board off.
- While a master keeps an armed motor fed, the inactivity power-off (`INACTIVITY_TIMEOUT`) does not run.

## Directions and units

"Positive" means the direction in which a positive `cmdL` / `cmdR` drives that wheel, with `INVERT_x_DIRECTION`
applied. This is the same for commands and for measurements (speed, position, current), so a motor driven forward
counts up on both sides.

Commands go to the motor controller without the input filter, rate limiter or mixer of the other variants (the v1
type 1 frame is the exception). Command 1000 is full scale: maximum voltage, `N_MOT_MAX` rpm or `I_MOT_MAX` A
(`current_soft_max`).

## v1 frames

| type | bytes | here |
|---|---|---|
| 0 speed, state | 8 | the addressed motor: state bits (64 disarm, 128 power off), then the speed field |
| 1 speed, steer, state, stateSlave | 11 | **both motors** through the firmware's mixer (`SPEED_COEFFICIENT`, `STEER_COEFFICIENT`, ±1000): `state` is the left motor, `stateSlave` the right one |
| 2 config | 15 | drive mode of the addressed motor. Battery limits and slaveNew are not applied, nothing is saved |

Drive mode = commutation × 4 + control, as on the MM32: commutation 0 block, 1 sine, 2 FOC; control 0 pwm, 1 speed,
2 torque, 3 position. Speed, torque and position need FOC (`SPD_MODE` / `TRQ_MODE` of this firmware), so only
0 (block pwm), 4 (sine pwm) and 8–11 (FOC) are taken. Anything else leaves the mode unchanged; a GD32 master that
sends mode 1 for speed has to send 9 instead. The commutation can change only while the motor is disarmed or stands
still.

| control | speed field (MM32 units) |
|---|---|
| pwm | ±1000 = ±100 % voltage |
| speed | rpm (clamped to `N_MOT_MAX`) |
| torque | 0.01 A |
| position | target odometer, hall steps |

LED state bits are ignored. Every valid frame with one of the board's ids gets a reply for that motor:

| byte | field | |
|---|---|---|
| 0–1 | `CD AB` | |
| 2 | slave | the motor's id |
| 3–4 | speed i16 | 0.1 rpm |
| 5–6 | voltage u16 | 0.01 V, battery |
| 7–8 | current i16 | 0.01 A, DC link current of this motor |
| 9–12 | odometer i32 | hall steps since boot |
| 13–14 | CRC | |

## CANSimple

Messages, payloads and units are ODrive's: float32 in turns, turns/s, Nm, A and V.

| id | message | here |
|---|---|---|
| 0x00 | Get_Version | protocol 2, firmware 2.0.0 (the protocol level of the GD32 / MM32 implementations) |
| 0x01 | Heartbeat | errors \| disarm reason, axis state in effect (1 / 8), trajectory done (position within 1 hall step) |
| 0x02 | Estop | disarm, ESTOP_REQUESTED |
| 0x03 | Get_Error | active errors, disarm reason |
| 0x04 / 0x05 | RxSdo / TxSdo | endpoints below |
| 0x07 | Set_Axis_State | 1 IDLE, 8 CLOSED_LOOP_CONTROL |
| 0x09 | Get_Encoder_Estimates | position (turns, from the hall steps, resolution 1/90 turn), velocity |
| 0x0B | Set_Controller_Mode | 0 voltage, 1 torque, 2 velocity, 3 position. Torque, velocity and position switch the motor to FOC. Input mode: passthrough only |
| 0x0C | Set_Input_Pos | turns; the feed-forward terms are ignored |
| 0x0D | Set_Input_Vel | turns/s, clamped to `vel_limit`; torque feed-forward ignored |
| 0x0E | Set_Input_Torque | Nm, ÷ torque constant = motor current, clamped to `current_soft_max` |
| 0x0F | Set_Limits | `vel_limit`, current limit (1–40 A; also the torque scale) |
| 0x14 | Get_Iq | set point (torque mode, else NaN), measured q current (FOC, else NaN) |
| 0x15 | Get_Temperature | board temperature (the MCU's sensor, `TEMP_CAL_*`), motor NaN |
| 0x17 | Get_Bus_Voltage_Current | battery voltage, DC link current of this motor (the two ids add up to the board's) |
| 0x18 | Clear_Errors | clears the disarm reason |
| 0x19 | Set_Absolute_Position | re-zeroes the position; the target follows, no jump |
| 0x1A | Set_Pos_Gain | (turns/s)/turn |
| 0x1C | Get_Torques | Get_Iq × torque constant |
| 0x1D | Get_Powers | electrical V × I; mechanical from Iq (FOC, else NaN) |
| 0x1E | Set_Input_Voltage | extension of the GD32 / MM32 firmwares: fraction −1…1 of the maximum voltage |

Set_Vel_Gains, Set_Traj_*, Reboot and Set_Linear_Count are ignored. Set_ messages are not answered. Requests
(bit 7, no payload) and RxSdo are answered with the same framing.

Position mode is a P loop in the main loop (200 Hz) on top of the FOC speed controller: speed set point =
`pos_gain` × error, clamped to `vel_limit`, and 0 within 1 hall step of the target.

### Endpoints (RxSdo / TxSdo)

The ids and names come from the MM32 firmware's `endpoints.json`; this list is the part that exists here. Values are
RAM only, and `save_configuration` is not supported. Request: opcode u8 (0 read, 1 write), endpoint u16, 0,
value (4 bytes). A read is answered with the value, a write is confirmed without one. Unknown endpoints get no answer.

| id | name | type | |
|---|---|---|---|
| 0x100 | vbus_voltage | float r | V |
| 0x101 | ibus | float r | A, this motor |
| 0x102 | axis0.pos_estimate | float rw | turns; writing re-zeroes the position |
| 0x103 | axis0.vel_estimate | float r | turns/s |
| 0x104 | axis0.motor.foc.Iq_measured | float r | A (FOC, else NaN) |
| 0x105 | axis0.motor.foc.Iq_setpoint | float r | A (torque mode, else NaN) |
| 0x106 | axis0.current_state | uint8 rw | 1 / 8 |
| 0x107 | axis0.active_errors | uint32 r | |
| 0x108 | axis0.disarm_reason | uint32 r | |
| 0x109 | hoverboard.odometer | int32 r | hall steps since boot |
| 0x10A | axis0.controller.config.control_mode | uint8 rw | |
| 0x10B | axis0.controller.config.input_mode | uint8 r | 1 passthrough |
| 0x10C | hoverboard.commutation | uint8 rw | 0 block, 1 sine, 2 FOC |
| 0x10D–0x110 | input_pos, input_vel, input_torque, hoverboard.input_voltage | float rw | |
| 0x111 | hoverboard.torque_constant_in_use | float r | Nm/A |
| 0x120 | axis0.controller.config.vel_limit | float rw | turns/s |
| 0x121 | axis0.controller.config.pos_gain | float rw | (turns/s)/turn |
| 0x124 | axis0.config.motor.current_soft_max | float rw | A, 1–40 |
| 0x126 | axis0.config.motor.torque_constant | float rw | Nm/A |
| 0x127 | axis0.config.motor.pole_pairs | uint32 r | `UARTBUS_HALL_STEPS` / 6 |
| 0x129 | axis0.config.watchdog_timeout | float rw | s, 0.02–60 |
| 0x12C | axis0.config.can.node_id | uint32 r | config.h |
| 0x12D | config.uart0_baudrate | uint32 r | |
| 0x135 | axis0.controller.pos_setpoint | float r | turns |
| 0x136 | axis0.controller.vel_setpoint | float r | turns/s (position loop output) |
| 0x201 | clear_errors | function | |
| 0x202 | hoverboard.power_off | function | |

## Example: left motor (id 0) at 2 turns/s, then read its estimates

```
a5 00 0b 08 02 00 00 00 01 00 00 00 ..  Set_Controller_Mode VELOCITY, PASSTHROUGH
a5 00 07 04 08 00 00 00 ..              Set_Axis_State CLOSED_LOOP_CONTROL
a5 00 0d 08 00 00 00 40 00 00 00 00 ..  Set_Input_Vel 2.0   (repeat faster than watchdog_timeout)
a5 00 89 00 ..                          request Get_Encoder_Estimates
a5 00 09 08 <f32 pos> <f32 vel> ..      answer
```

(`..` = the two CRC bytes.) Poll one id at a time and wait for its answer, because the boards share the TX line.
Host libraries: `protocol.js` of the RemoteUartBus web console (browser / node), and `tools/cansimple_uart.py` of the
MM32 firmware (Python).
