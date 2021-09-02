## hoverboard-firmware-hack-FOC - bobby car edition

This is a 4-wheel-drive-bobbycar-optimized version of the famous [hoverboard mainboard FOC firmware by EmanuelFeru](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC). This an improved version of my [old bbcar firmware](https://github.com/larsmm/hoverboard-firmware-hack-bbcar).

![bobbycar pic](https://raw.githubusercontent.com/larsmm/hoverboard-firmware-hack-bbcar/master/pic1.jpg)

### Features
* Controlled by 2 potis on the steering wheel: 1. forward, 2. break or backward or turbo mode
* 4 driving modes with different speeds, accelerations and features
* Acceleration and breaking ramps, turbo ramp
* Everything else is identical to: https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
* Backwards beep for safety :D (can be disabled in config.h)

### Safety features
* Poti-out-of-range detection: controlled breaking and poweroff (fast di-de-di-de-di-de-di-de sound)
* Low battery beep and poweroff (calibrate battery and use a good BMS!)
* Motor fault detection
* See [beep codes](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC#diagnostics)

### Hardware
* This firmware is only compatible with [Single Mainboards with STM32F103RCT6 and GD32F103RCT6 chips](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/wiki/Firmware-Compatibility).
* Schematic for [connecting potis](https://larsm.org/wp-content/uploads/2019/08/connecting-potis-v2.png) ([full guide](https://larsm.org/allrad-e-bobby-car/))
* Use 1 power button and connect it to both boards

### Driving modes
You can activate them by holding one or more of the potis while poweron. (km/h @ full 12s battery):
* Mode 1 - Child: left poti, max speed ~3 km/h, very slow backwards, no turbo
* Mode 2 - STVO: no poti, max speed ~<6 km/h (verify it), slow backwards, no turbo
* Mode 3 - Fun: right poti, max speed ~12 km/h, no turbo
* Mode 4 - Power: both potis, max speed ~22 km/h, with turbo ~39 km/h

After poweron it beeps the mode in 1 to 4 fast beeps. Default mode is nr. 2.

### Turbo
Field weakening is only availible in mode 4. It can only be activated if you are already at 80% of top speed. To activate it, press the backwards poti to get additional 40% more power. :) Please be very careful, this speed is dangarous.

### Power and battery
Peak power on full throttle+turbo on dry sand is around 50A = ~2300W at 12 lithium battery cells. More power does not make much sense. The wheels are not able to get the power onto the ground. Cooling of the board is no problem, the wheels will get too hot earlier. You will get a range of ~20km out of a 10Ah 12s battery on mode 4 without turbo. Use a good BMS!!!

### Build and flash
On new boards the chip is write-protected and have to be [unlocked first](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/wiki/How-to-Unlock-MCU-flash).
* Install Visual Studio Code
* Install PlatformIO IDE within VSCode extension system
* File --> open folder: hoverboard-firmware-hack-FOC-bbcar
* Connect st-link v2 adapter to the board (do not connect 3.3V) [pic](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC#hardware)
* Connect battery or power supply to the board
* Press and hold poweron button, press "Upload" button, wait for upload to finish, release poweron button

### Calibrate ADCs (potis)
If the ADCs are not calibrated, the board will make a fast di-de-di-de-di-de-di-de sound and will poweroff emiedetly after poweron
* Disconnect serial and st-link adapters
* Make sure you calibrate both boards together!
* Poweron and hold the button until a beep sound (after ~10s)
* Now you have 20s to move both potis to min and max positions. Release in the mid position (which equals min position in this case). Wait for the 20s to end or short press poweron button.

### Fine tune ADCs (potis)
* You have a dead zone at the start and end of the poti range.
* Edit config.h ### VARIANT BBCAR ### section:
* Make ADC_MARGIN smaller to make the dead zone smaller. It depends on the quality of your potis how small you can make it. If it is too small, the car sometimes starts driving slowly without pressing the potis or you will not reach max speed. I recommand to use hall-potis instead of mechanical ones.
* Re-upload

### Calibrate battery voltage
* Edit config.h ### BATTERY ### section:
* Connect usb-serial converter (GND, TX, RX, NOT 3.3V!) to the right sideboard cable. [pic](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC#hardware)
* Press "Serial Monitor" button to get debug feedback from the board
* Connect a known voltage to the board, verify with multimeter. For 45.00V for example write 4500 to BAT_CALIB_REAL_VOLTAGE. If board powers off emiedetly, either the ADCs are not calibrated (calibrate them first) or undervoltage protection jumps in (use higher voltage up to 51V)
* Write BatADC from serial output to BAT_CALIB_ADC in config.h
* Make sure BAT_CELLS matches your cell count
* Re-upload

### Trouble shooting
* Sometimes 1 board is on and the other off. This is a rare case. Press power button a few times or disconnect the battery.

### ToDo
* At the moment it runs on sinus mode, not foc. This is because field weakening does not work on foc. I get strange noise from the motors on foc+fw and only a very little bit more power. On sinus this works much better. Even on sinus it is much better than my [old firmware](https://github.com/larsmm/hoverboard-firmware-hack-bbcar).

### More info
* https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
* https://larsm.org/allrad-e-bobby-car/
* https://figch.de/index.php?nav=bobbycar