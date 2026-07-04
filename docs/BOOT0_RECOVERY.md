# BOOT0 Pin Recovery Guide

## When to Use This Method

If you cannot flash the board via ST-Link because:
- The firmware is blocking debug access
- ST-Link shows "unable to connect to target" or "init mode failed"
- You previously flashed firmware that prevents reprogramming
- You need to recover from a bad flash

## What is BOOT0?

BOOT0 is a special pin on the STM32F103RC microcontroller that determines boot source:
- **BOOT0 = LOW (GND)**: Normal operation - runs firmware from flash memory
- **BOOT0 = HIGH (3.3V)**: Bootloader mode - STM32 enters DFU/programming mode

## Locating BOOT0 Pin

### STM32F103RC Pin Location
- **Pin 60** on the 64-pin LQFP package
- Located on the top edge, 5 pins away from Pin 1 (corner with dot marker)

### Finding BOOT0 on Your Board
Look for:
1. A test pad labeled "BOOT0", "B0", or "BOOT"
2. A solder jumper near the STM32 chip
3. A small resistor (usually 10kΩ) connected from pin 60 to GND

## Step-by-Step Recovery Process

### Method 1: Using BOOT0 Pin (Recommended)

1. **Power OFF the main board**
   - Disconnect battery/power source

2. **Connect BOOT0 to 3.3V**
   - Use a jumper wire to connect BOOT0 pin/pad to a 3.3V source
   - Sources: ST-Link 3.3V pin, board's 3.3V rail, or any 3.3V test pad
   - ⚠️ **IMPORTANT**: Use 3.3V, NOT 5V!

3. **Power ON the board**
   - With BOOT0 still connected to 3.3V, power on the board
   - The STM32 will now enter bootloader mode
   - You can release/disconnect BOOT0 after power-on (it only matters during startup)

4. **Flash the firmware**
   ```bash
   pio run --target upload
   ```
   Or use the flash script:
   ```bash
   ./flash_board.ps1
   ```

5. **Restore normal operation**
   - Power OFF the board
   - Disconnect BOOT0 from 3.3V (let it return to GND via pull-down resistor)
   - Power ON the board - it will now run your new firmware

### Method 2: Using ST-Link Utility (Alternative)

If you have ST-Link Utility software:

1. Open ST-Link Utility
2. Go to **Target** → **Settings**
3. Enable **"Connect Under Reset"** mode
4. Click OK
5. Power cycle the board and immediately click **Target** → **Connect**
6. Once connected, go to **Target** → **Erase Chip**
7. After erasing, flash your new firmware normally

### Method 3: Power Cycle Timing

This method requires precise timing:

1. In ST-Link Utility, enable "Connect Under Reset"
2. Disconnect board power
3. Click Connect in ST-Link Utility (it will wait for connection)
4. Immediately reconnect power to the board
5. ST-Link should catch the chip during the brief boot window

## Prevention: Firmware Startup Delay

As of the latest firmware update, all variants include a 2-second startup delay. This gives debuggers time to connect before the firmware fully initializes, preventing debug lockout.

If you're using older firmware, you can add this delay manually in `Src/main.c`:

```c
int main(void) {
  HAL_Init();
  
  // Startup delay to allow debugger connection
  for(volatile uint32_t i = 0; i < 2000000; i++) {
    __NOP(); // 2 second delay
  }
  
  // Rest of initialization...
}
```

## Common Pin Labels

- **BOOT0**: Most common label
- **B0**: Abbreviated version
- **BOOT**: Generic boot pin
- May be unlabeled - look for a 10kΩ resistor to ground from pin 60

## Important Notes

⚠️ **Safety Warnings:**
- Always use **3.3V** for BOOT0, never 5V
- Do not confuse BOOT0 with other pins:
  - PF0/PF1 are crystal oscillator pins - do not touch!
  - PC12 is a GPIO pin, not BOOT0
- BOOT0 only needs to be HIGH during power-on/reset

💡 **Tips:**
- BOOT0 only matters during chip startup/reset
- Once in bootloader mode, you can release BOOT0
- The pull-down resistor automatically returns BOOT0 to LOW when disconnected
- If your board has a BOOT0 button, just hold it while powering on

## Troubleshooting

**ST-Link still can't connect in bootloader mode:**
- Verify BOOT0 is actually at 3.3V (use multimeter)
- Check ST-Link connections (SWDIO, SWCLK, GND)
- Ensure board is powered
- Try different USB port or ST-Link cable

**Board won't run firmware after flashing:**
- Make sure BOOT0 is disconnected from 3.3V
- Power cycle the board
- Check that BOOT0 has returned to GND (via pull-down resistor)

**Can't find BOOT0 pin:**
- Search for STM32F103RC datasheet for pin 60 location
- Trace from pin 60 on the chip to find test pad
- Look for resistors near the STM32 chip going to ground

## Additional Resources

- [STM32F103RC Datasheet](https://www.st.com/resource/en/datasheet/stm32f103rc.pdf)
- [AN2606: STM32 Bootloader Documentation](https://www.st.com/resource/en/application_note/cd00167594.pdf)
- [Project Wiki](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki)

---

**Last Updated:** March 2026  
**Applies to:** All firmware variants (ADC, USART, NUNCHUK, PPM, PWM, IBUS, HOVERCAR, HOVERBOARD, TRANSPOTTER, SKATEBOARD)
