# hoverboard-firmware-hack-FOC
[![Build Status](https://travis-ci.com/EmanuelFeru/hoverboard-firmware-hack-FOC.svg?branch=master)](https://travis-ci.com/EmanuelFeru/hoverboard-firmware-hack-FOC)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donate_SM.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=CU2SWN2XV9SCY&currency_code=EUR&source=url)

This repository implements Field Oriented Control (FOC) for stock hoverboards. Compared to the commutation method, this new FOC control method offers superior performance featuring:
 - reduced noise and vibrations 	
 - smooth torque output and improved motor efficiency. Thus, lower energy consumption
 - field weakening to increase maximum speed range
 
#### For the hoverboard sideboard firmware, see the following repositories:
 - [hoverboard-sideboard-hack-GD](https://github.com/EmanuelFeru/hoverboard-sideboard-hack-GD)
 - [hoverboard-sideboard-hack-STM](https://github.com/EmanuelFeru/hoverboard-sideboard-hack-STM)
 
#### For the FOC controller design, see the following repository:
 - [bldc-motor-control-FOC](https://github.com/EmanuelFeru/bldc-motor-control-FOC)
 
 
---
## Hardware
 
![mainboard_pinout](/docs/pictures/mainboard_pinout.png)

The original Hardware supports two 4-pin cables that originally were connected to the two sideboards. They break out GND, 12/15V and USART2&3 of the Hoverboard mainboard. Both USART2&3 support UART, PWM, PPM, and iBUS input. Additionally, the USART2 can be used as 12bit ADC, while USART3 can be used for I2C. Note that while USART3 (right sideboard cable) is 5V tolerant, USART2 (left sideboard cable) is **not** 5V tolerant.

Typically, the mainboard brain is an [STM32F103RCT6](/docs/literature/[10]_STM32F103xC_datasheet.pdf), however some mainboards feature a [GD32F103RCT6](/docs/literature/[11]_GD32F103xx-Datasheet-Rev-2.7.pdf) which is also supported by this firmware.

For the reverse-engineered schematics of the mainboard, see [20150722_hoverboard_sch.pdf](/docs/20150722_hoverboard_sch.pdf)

 
---
## FOC Firmware
 
In this firmware 3 control types are available:
- Commutation
- SIN (Sinusoidal)
- FOC (Field Oriented Control) with the following 3 control modes:
  - **VOLTAGE MODE**: in this mode the controller applies a constant Voltage to the motors. Recommended for robotics applications or applications where a fast motor response is required.
  - **SPEED MODE**: in this mode a closed-loop controller realizes the input speed target by rejecting any of the disturbance (resistive load) applied to the motor. Recommended for robotics applications or constant speed applications.
  - **TORQUE MODE**: in this mode the input torque target is realized. This mode enables motor "freewheeling" when the torque target is `0`. Recommended for most applications with a sitting human driver.
  
#### Comparison between different control methods

|Control method| Complexity | Efficiency | Smoothness | Field Weakening | Freewheeling | Standstill hold |
|--|--|--|--|--|--|--|
|Commutation| - | - | ++ | n.a. | n.a. | + |
|Sinusoidal| + | ++ | ++ | +++ | n.a. | + |
|FOC VOLTAGE| ++ | +++ | ++ | ++ | n.a. | +<sup>(2)</sup> |
|FOC SPEED| +++ | +++ | + | ++ | n.a. | +++ |
|FOC TORQUE| +++ | +++ | +++ | ++ | +++<sup>(1)</sup> | n.a<sup>(2)</sup> |

<sup>(1)</sup> By enabling `ELECTRIC_BRAKE_ENABLE` in `config.h`, the freewheeling amount can be adjusted using the `ELECTRIC_BRAKE_MAX` parameter.

<sup>(2)</sup> The standstill hold functionality can be forced by enabling `STANDSTILL_HOLD_ENABLE` in `config.h`. 


In all FOC control modes, the controller features maximum motor speed and maximum motor current protection. This brings great advantages to fulfil the needs of many robotic applications while maintaining safe operation.


### Field Weakening / Phase Advance

 - By default the Field weakening is disabled. You can enable it in config.h file by setting the FIELD_WEAK_ENA = 1 
 - The Field Weakening is a linear interpolation from 0 to FIELD_WEAK_MAX or PHASE_ADV_MAX (depeding if FOC or SIN is selected, respectively)
 - The Field Weakening starts engaging at FIELD_WEAK_LO and reaches the maximum value at FIELD_WEAK_HI
 - The figure below shows different possible calibrations for Field Weakening / Phase Advance
 ![Field Weakening](/docs/pictures/FieldWeakening.png) 
 - If you re-calibrate the Field Weakening please take all the safety measures! The motors can spin very fast!


### Parameters 
 - All the calibratable motor parameters can be found in the 'BLDC_controller_data.c'. I provided you with an already calibrated controller, but if you feel like fine tuning it feel free to do so 
 - The parameters are represented in Fixed-point data type for a more efficient code execution
 - For calibrating the fixed-point parameters use the [Fixed-Point Viewer](https://github.com/EmanuelFeru/FixedPointViewer) tool
 - The parameters data Fixed-point types are given in the following table:

![Parameters table](/docs/pictures/paramTable.png)


### Diagnostics
Each motor is constantly monitored for errors. These errors are:
- **Error 001**: Hall sensor not connected
- **Error 002**: Hall sensor short circuit
- **Error 004**: Motor NOT able to spin (Possible causes: motor phase disconnected, MOSFET defective, operational Amplifier defective, motor blocked)

The error codes above are reported for each motor in the variables **rtY_Left.z_errCode** and **rtY_Right.z_errCode** for Left motor (long wired motor) and Right motor (short wired motor), respectively. In case of error, the motor power is reduced to 0, while an audible (fast beep) can be heard to notify the user.


### Demo Videos

[►Video: HOVERCAR](https://www.youtube.com/watch?v=IgHCcj0NgWQ&t=)

[►Video: Commutation vs Advanced control (constant speed)](https://drive.google.com/open?id=1vC_kEkp2LE2lAaMCJcmK4z2m3jrPUoBD)

[►Video: Commutation vs Advanced control (variable speed)](https://drive.google.com/open?id=1rrQ4k5VLhhAWXQzDSCar_SmEdsbM-hq2)

[►Video: Reliable Serial Communication demo](https://drive.google.com/open?id=1mUM-p7SE6gmyTH7zhDHy5DUyczXvmy5d)


---
## Example Variants 

This firmware offers currently these variants (selectable in [platformio.ini](/platformio.ini) or [config.h](/Inc/config.h)):
- **VARIANT_ADC**: In this variant the motors are controlled by two potentiometers connected to the Left sensor cable (long wired)
- **VARIANT_USART**: In this variant the motors are controlled via serial protocol (e.g. on USART3 right sensor cable, the short wired cable). The commands can be sent from an Arduino. Check out the [hoverserial.ino](/Arduino/hoverserial) as an example sketch.
- **VARIANT_NUNCHUK**: Wii Nunchuk offers one hand control for throttle, braking and steering. This was one of the first input device used for electric armchairs or bottle crates.
- **VARIANT_PPM**: This is when you want to use an RC remote control with PPM Sum signal.
- **VARIANT_PWM**: This is when you want to use an RC remote control with PWM signal.
- **VARIANT_IBUS**: This is when you want to use an RC remote control with Flysky IBUS protocol connected to the Left sensor cable.
- **VARIANT_HOVERCAR**: In this variant the motors are controlled by two pedals brake and throttle. Reverse is engaged by double tapping on the brake pedal at standstill. See [HOVERCAR video](https://www.youtube.com/watch?v=IgHCcj0NgWQ&t=).
- **VARIANT_HOVERBOARD**: In this variant the mainboard reads the sideboards data. The sideboards need to be flashed with the hacked version. Only balancing controller is still to be implemented.
- **VARIANT_TRANSPOTTER**: This build is for transpotter which is a hoverboard based transportation system. For more details on how to build it check [here](https://github.com/NiklasFauth/hoverboard-firmware-hack/wiki/Build-Instruction:-TranspOtter) and [here](https://hackaday.io/project/161891-transpotter-ng).
- **VARIANT_SKATEBOARD**: This is for skateboard build, controlled using an RC remote with PWM signal connected to the right sensor cable.

Of course the firmware can be further customized for other needs or projects.


---
## Flashing

Right to the STM32, there is a debugging header with GND, 3V3, SWDIO and SWCLK. Connect GND, SWDIO and SWCLK to your SWD programmer, like the ST-Link found on many STM devboards.

If you have never flashed your sideboard before, the MCU is probably locked. To unlock the flash, check-out the wiki page [How to Unlock MCU flash](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/wiki/How-to-Unlock-MCU-flash).

Do not power the mainboard from the 3.3V of your programmer! This has already killed multiple mainboards.

Make sure you hold the powerbutton or connect a jumper to the power button pins while flashing the firmware, as the STM might release the power latch and switches itself off during flashing. Battery > 36V have to be connected while flashing.

To build and flash choose one of the following methods:

### Method 1: Using Platformio IDE

- open the folder in the IDE of choice (vscode or Atom)
- press the 'PlatformIO:Build' or the 'PlatformIO:Upload' button (bottom left in vscode).

### Method 2: Using Keil uVision

- in [Keil uVision](https://www.keil.com/download/product/), open the [mainboard-hack.uvproj](/MDK-ARM/)
- if you are asked to install missing packages, click Yes
- click Build Target (or press F7) to build the firmware
- click Load Code (or press F8) to flash the firmware.

### Method 3: Using Linux CLI

- prerequisites: install [ST-Flash utility](https://github.com/texane/stlink).
- open a terminal in the repo check-out folder and if you have definded the variant in [config.h](/Inc/config.h) type:
```
make
```
or you can set the variant like this
```
make -e VARIANT=VARIANT_####
```
- flash the firmware by typing:
```
make flash
```
- or
```
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c flash "write_image erase build/hover.bin 0x8000000"
```

### Method 4: MacOS CLI
- prerequisites:  first get brew https://brew.sh
- then install stlink ST-Flash utility

#### Using Make
```
brew install stlink
```
- open a terminal in the repo check-out folder and if you have definded the variant in [config.h](/Inc/config.h) type:
```
make
```
or you can set the variant like this
```
make -e VARIANT=VARIANT_####
```
If compiling fails because something is missing just install it with brew AND leave a comment to improve this howto or pull request ;-)

- flash the firmware by typing:
```
make flash
```
- if unlock is needed
```
make unlock
```

#### Using platformio CLI

```
brew install platformio
platformio run -e VARIANT_####
platformio run –target upload -e VARIANT_####
```
If you have set default_envs in [platformio.ini](/platformio.ini) you can ommit -e parameter

---
## Troubleshooting
First, check that power is connected and voltage is >36V while flashing.
If the board draws more than 100mA in idle, it's probably broken.

If the motors do something, but don't rotate smooth and quietly, try to use an alternative phase mapping. Usually, color-correct mapping (blue to blue, green to green, yellow to yellow) works fine. However, some hoverboards have a different layout then others, and this might be the reason your motor isn't spinning.

Nunchuk not working: Use the right one of the 2 types of nunchuks. Use i2c pullups.

Nunchuk or PPM working bad: The i2c bus and PPM signal are very sensitive to emv distortions of the motor controller. They get stronger the faster you are. Keep cables short, use shielded cable, use ferrits, stabilize voltage in nunchuk or reviever, add i2c pullups. To many errors leads to very high accelerations which triggers the protection board within the battery to shut everything down.

Recommendation: Nunchuk Breakout Board https://github.com/Jan--Henrik/hoverboard-breakout

Most robust way for input is to use the ADC and potis. It works well even on 1m unshielded cable. Solder ~100k Ohm resistors between ADC-inputs and gnd directly on the mainboard. Use potis as pullups to 3.3V.


---
## Projects and Links

- **Original firmware:** [https://github.com/NiklasFauth/hoverboard-firmware-hack](https://github.com/NiklasFauth/hoverboard-firmware-hack)
- **[RoboDurden's](https://github.com/RoboDurden) online compiler:** [https://pionierland.de/hoverhack/](https://pionierland.de/hoverhack/) 
- **Hoverboard hack for AT32F403RCT6 mainboards:** [https://github.com/cloidnerux/hoverboard-firmware-hack](https://github.com/cloidnerux/hoverboard-firmware-hack)
- **Hoverboard hack for split mainboards:** [https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2](https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2)
- **Hoverboard hack from BiPropellant:** [https://github.com/bipropellant](https://github.com/bipropellant)
- **Hoverboard breakout boards:** [https://github.com/Jan--Henrik/hoverboard-breakout](https://github.com/Jan--Henrik/hoverboard-breakout)

<a/>

- **Bobbycar** [https://github.com/larsmm/hoverboard-firmware-hack-bbcar](https://github.com/larsmm/hoverboard-firmware-hack-bbcar)
- **Wheel chair:** [https://github.com/Lahorde/steer_speed_ctrl](https://github.com/Lahorde/steer_speed_ctrl)
- **TranspOtterNG:** [https://github.com/Jan--Henrik/transpOtterNG](https://github.com/Jan--Henrik/transpOtterNG)
- **ST Community:** [Custom FOC motor control](https://community.st.com/s/question/0D50X0000B28qTDSQY/custom-foc-control-current-measurement-dma-timer-interrupt-needs-review)

<a/>

- **Telegram Community:** If you are an enthusiast join our [Hooover Telegram Group](https://t.me/joinchat/BHWO_RKu2LT5ZxEkvUB8uw)


---
## Contributions

Every contribution to this repository is highly appreciated! Feel free to create pull requests to improve this firmware as ultimately you are going to help everyone. 

If you want to donate to keep this firmware updated, please use the link below:

[![paypal](https://www.paypalobjects.com/en_US/NL/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=CU2SWN2XV9SCY&currency_code=EUR&source=url)


---

