# Brain Badge Wakeup Module

This directory contains the wakeup module implementation for the Brain Badge RP2040 board, ported from the MicroPython Badger2040W firmware.

## Overview

The wakeup module provides functionality to:
1. Capture GPIO state at boot time for button press detection
2. Configure the PCF85063A RTC to disable clock output (power saving)
3. Allow Python code to determine which button was pressed or if RTC alarm triggered during wakeup

This is particularly useful for implementing deep sleep functionality where the board wakes on button press or RTC alarm.

## Architecture

The implementation is fully C-based to capture GPIO state as early as possible in the boot process:

### MicroPython Approach (Original)
- C++ module with constructor that runs before main()
- Directly registered as a Python module
- Uses MicroPython's C++ module system

### CircuitPython Approach (This Port)
- C-based initialization called from `board_init()` - runs before Python VM starts
- Native C module (`_wakeup_native`) exposes C functions to Python
- Python wrapper module (`wakeup`) provides user-friendly API
- Follows CircuitPython's module registration pattern

## Files

- `wakeup.c` - Core wakeup initialization and GPIO state capture (called from board_init)
- `wakeup.h` - Header file with function declarations
- `wakeup_native.c` - Native Python module bindings (_wakeup_native module)
- `wakeup.py` - Python wrapper providing user-friendly API (frozen into firmware)

## Implementation Details

### GPIO State Capture

The wakeup system captures GPIO state in two phases:

1. **C Initialization (wakeup_init)**: Called from `board_init()` before the Python VM starts
   - Configures all button pins with pull-downs (buttons are active-HIGH)
   - Configures RTC alarm pin (GPIO8) with pull-down (alarm is active-HIGH via MOSFET inversion)
   - Immediately reads all GPIO pins using `gpio_get_all()`
   - Waits 5ms and reads again, OR-ing the results
   - This two-phase capture ensures brief button presses are detected
   - RTC clock output is disabled to save power

2. **Python Access**: The captured state is exposed via the `_wakeup_native` module
   - `get_gpio_state()` returns the 32-bit captured state
   - Button and RTC alarm states are active-HIGH (bit set = active)

### Key Differences from MicroPython

1. **No C++ constructors**: CircuitPython doesn't support C++ priority constructors
2. **Explicit initialization**: `wakeup_init()` is called from `board_init()`
3. **Module registration**: Uses `MP_REGISTER_MODULE` for the native module
4. **Two-tier design**: Native C module + Python wrapper (vs. single C++ module)

### Hardware Signal Characteristics

**Buttons** (GPIO11-15):
- Active-HIGH with pull-downs
- Button pressed: GPIO reads HIGH (1)
- Button released: GPIO reads LOW (0)

**RTC Alarm** (GPIO8):
- Active-HIGH due to MOSFET inversion in hardware
- PCF85063A INT pin is open-drain active-LOW
- Board circuit inverts this to provide active-HIGH signal on GPIO8
- Normal state: LOW (0)
- Alarm triggered: HIGH (1)
- Signal persists until AF flag cleared in software

## RTC Alarm Configuration

The PCF85063A requires specific configuration for alarm interrupts to work:

### Critical Requirements

1. **AIE (Alarm Interrupt Enable) must be set**: `rtc.alarm_interrupt = True`
2. **AF (Alarm Flag) must be cleared after use**: `rtc.alarm_status = False`
3. **Alarm time registers must be set with appropriate AEN bits**
4. **INT pin is open-drain active-LOW** - do not drive it in firmware

### Helper Functions

The wakeup module provides convenience functions:

```python
import wakeup

# Enable RTC alarm interrupt output
wakeup.configure_rtc_alarm_interrupt(rtc, True)

# Check if alarm triggered at boot
if wakeup.rtc_alarm_triggered():
    print("Woke from RTC alarm!")
    # MUST clear the alarm flag
    wakeup.clear_rtc_alarm(rtc)
```

## Native Alarm Module Integration

The wakeup module **fully integrates with CircuitPython's native `alarm` module**. You can use the standard `alarm.exit_and_deep_sleep_until_alarms()` and `alarm.light_sleep_until_alarms()` functions.

### Convenience Functions for alarm Module

To make deep sleep configuration easier and prevent polarity/pin errors, the wakeup module provides helper functions:

#### `create_rtc_alarm()`
Creates a properly configured `PinAlarm` for RTC wake:
```python
import alarm
import wakeup

rtc_alarm = wakeup.create_rtc_alarm()  # Automatically sets GPIO8, value=True, pull=False
alarm.exit_and_deep_sleep_until_alarms(rtc_alarm)
```

#### `create_button_alarm(button_name)`
Creates a properly configured `PinAlarm` for a specific button:
```python
import alarm
import wakeup

button_alarm = wakeup.create_button_alarm('A')  # Buttons: 'A', 'B', 'C', 'UP', 'DOWN'
alarm.exit_and_deep_sleep_until_alarms(button_alarm)
```

#### `create_all_button_alarms()`
Creates `PinAlarm` objects for all five buttons:
```python
import alarm
import wakeup

button_alarms = wakeup.create_all_button_alarms()
rtc_alarm = wakeup.create_rtc_alarm()

# Wake on any button or RTC alarm
alarm.exit_and_deep_sleep_until_alarms(rtc_alarm, *button_alarms)
```

#### `check_alarm_wake()`
Examines `alarm.wake_alarm` and returns a friendly description:
```python
import alarm
import wakeup

wake_source = wakeup.check_alarm_wake()
if wake_source:
    print(f"Woke from: {wake_source}")  # e.g., "button A", "RTC alarm"
else:
    print("Power on / hard reset")
```

#### `wakeup_reason()`
Checks **both** `alarm.wake_alarm` (if available) **and** GPIO state capture:
```python
import wakeup

# Works after alarm wake OR after any boot/reset
reason = wakeup.wakeup_reason()
print(f"Wake reason: {reason}")
```

### Why Use the Helper Functions?

1. **Automatic polarity configuration**: Ensures `value=True` for active-HIGH signals
2. **No pull configuration mistakes**: Pull resistors already configured in C code
3. **Correct pin mapping**: Uses the right GPIO pins automatically
4. **Type safety**: Prevents typos in button names
5. **Less verbose**: Shorter, clearer code

## Complete Usage Examples

### Example 1: Basic Wakeup Detection

```python
import wakeup
import pcf85063a
import busio
import board

# Check wakeup reason (works with or without alarm module)
print(f"Wakeup reason: {wakeup.wakeup_reason()}")

if wakeup.rtc_alarm_triggered():
    print("Woke from RTC alarm!")
    # Initialize RTC and clear alarm
    i2c = busio.I2C(board.SCL, board.SDA)
    rtc = pcf85063a.PCF85063A(i2c)
    wakeup.clear_rtc_alarm(rtc)

elif wakeup.button_a():
    print("Button A pressed at wakeup")

elif wakeup.pressed_to_wake():
    print("Some button was pressed")
```

### Example 2: Deep Sleep with Native alarm Module (Simple)

```python
import time
import alarm
import busio
import board
import pcf85063a
import wakeup

# Initialize RTC
i2c = busio.I2C(board.SCL, board.SDA)
rtc = pcf85063a.PCF85063A(i2c)

# Enable RTC alarm interrupt
wakeup.configure_rtc_alarm_interrupt(rtc, True)
wakeup.clear_rtc_alarm(rtc)

# Set alarm for 2 minutes from now
current = rtc.datetime
alarm_time = time.struct_time((
    current.tm_year, current.tm_mon, current.tm_mday,
    current.tm_hour, current.tm_min + 2, 0,
    0, 0, -1
))
rtc.alarm = alarm_time

print(f"Alarm set for: {alarm_time}")

# Use helper functions for correct configuration
rtc_alarm = wakeup.create_rtc_alarm()
button_a_alarm = wakeup.create_button_alarm('A')

print("Entering deep sleep...")
alarm.exit_and_deep_sleep_until_alarms(rtc_alarm, button_a_alarm)
```

### Example 2b: Deep Sleep - Wake on Any Button

```python
import alarm
import wakeup

# Create alarms for all buttons and RTC
button_alarms = wakeup.create_all_button_alarms()
rtc_alarm = wakeup.create_rtc_alarm()

print("Entering deep sleep...")
print("Wake with any button or RTC alarm")

# Enter deep sleep
alarm.exit_and_deep_sleep_until_alarms(rtc_alarm, *button_alarms)
```

### Example 3: Check Wake Source After alarm Module Wake

```python
import alarm
import wakeup

print("=== Brain Badge Wake Test ===")

# Check alarm.wake_alarm first (preferred method after deep sleep)
wake_source = wakeup.check_alarm_wake()
if wake_source:
    print(f"Woke from alarm: {wake_source}")
else:
    # No alarm.wake_alarm - check GPIO state capture
    print(f"Wake reason (GPIO): {wakeup.wakeup_reason()}")

# Or just use wakeup_reason() which checks both
print(f"Combined check: {wakeup.wakeup_reason()}")
```

### Example 4: Runtime Pin Test (Verify Hardware)

Test that the MCU can see RTC_ALARM pin changes while running:

```python
import board
import digitalio
import time
import pcf85063a
import busio
import wakeup

# Initialize RTC
i2c = busio.I2C(board.SCL, board.SDA)
rtc = pcf85063a.PCF85063A(i2c)

# Configure alarm interrupt
wakeup.configure_rtc_alarm_interrupt(rtc, True)
wakeup.clear_rtc_alarm(rtc)

# Set alarm for 30 seconds from now
current = rtc.datetime
alarm_time = time.struct_time((
    current.tm_year, current.tm_mon, current.tm_mday,
    current.tm_hour, current.tm_min, current.tm_sec + 30,
    0, 0, -1
))
rtc.alarm = alarm_time

# Watch RTC_ALARM pin
rtc_alarm_pin = digitalio.DigitalInOut(board.GP8)
rtc_alarm_pin.direction = digitalio.Direction.INPUT
rtc_alarm_pin.pull = digitalio.Pull.DOWN

print(f"Alarm set for 30 seconds from now")
print(f"Watching RTC_ALARM pin (GP8)...")

prev_state = rtc_alarm_pin.value
while True:
    current_state = rtc_alarm_pin.value
    if current_state != prev_state:
        print(f"RTC_ALARM changed: {prev_state} -> {current_state}")
        if current_state:
            print("ALARM TRIGGERED!")
            # Clear the alarm
            wakeup.clear_rtc_alarm(rtc)
            print("Alarm cleared")
            break
        prev_state = current_state
    time.sleep(0.1)
```

## Troubleshooting Guide

### Hardware Verification Steps

1. **Manual wake test**:
   - Connect a pull-up resistor (10kΩ) from GPIO8 to 3.3V
   - Verify board powers on (simulates RTC alarm)
   - If this fails, check power switching circuit

2. **RTC oscillator test**:
   - Read RTC seconds register repeatedly
   - Verify it increments (oscillator running)
   - If stuck, check crystal Y1 and load capacitors

3. **RTC INT pin test** (requires oscilloscope):
   - Set an alarm and monitor PCF85063A INT pin (before MOSFET)
   - Verify it goes LOW when alarm triggers
   - If it doesn't, check RTC registers (AIE, AF, alarm time)

4. **RTC_ALARM pin test** (after MOSFET inversion):
   - Monitor GPIO8 (RTC_ALARM) with multimeter or scope
   - When alarm triggers, should go from 0V to 3.3V
   - If INT works but RTC_ALARM doesn't, check MOSFET circuit

### Software Debugging

1. **Check captured GPIO state**:
   ```python
   import wakeup
   state = wakeup.get_gpio_state()
   print(f"GPIO state: 0x{state:08X}")
   print(f"GPIO8 (RTC_ALARM): {bool(state & (1 << 8))}")
   ```

2. **Verify RTC registers**:
   ```python
   import pcf85063a
   import busio
   import board

   i2c = busio.I2C(board.SCL, board.SDA)
   rtc = pcf85063a.PCF85063A(i2c)

   print(f"AIE (alarm interrupt enable): {rtc.alarm_interrupt}")
   print(f"AF (alarm flag): {rtc.alarm_status}")
   print(f"Current time: {rtc.datetime}")
   print(f"Alarm time: {rtc.alarm}")
   ```

3. **Test runtime detection** (see Example 3 above)

4. **Deep sleep polarity check**:
   - Ensure PinAlarm uses `value=True` (active-HIGH)
   - Test with USB disconnected (battery mode)

### Common Issues

| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| Button wake works, alarm doesn't | AIE not set or AF not cleared | Use `wakeup.configure_rtc_alarm_interrupt(rtc, True)` |
| INT goes low but RTC_ALARM stays low | MOSFET circuit issue | Check Q1, R2, R3 on schematic |
| Alarm triggers but wakeup doesn't detect | Polarity wrong in PinAlarm | Use `value=True` not `False` |
| AF flag never sets | Alarm time wrong or AEN bits wrong | Check alarm registers, ensure time will match |
| Board doesn't wake from deep sleep | VSW not powering VSYS | Check power switching circuit |
| wake_alarm is None after deep sleep | Platform-specific issue | Try light_sleep_until_alarms instead |

## Testing Procedure

Follow these steps in order:

1. ✅ **Manual hardware wake**: Pull GPIO8 high externally → board powers on
2. ✅ **RTC oscillator**: Read seconds register → increments every second
3. ✅ **RTC INT assertion**: Set alarm, scope INT pin → goes LOW at alarm time
4. ✅ **RTC_ALARM signal**: Monitor GPIO8 → goes HIGH when alarm triggers
5. ✅ **Runtime pin detection**: Run Example 3 → script sees GPIO8 change
6. ✅ **Deep sleep wake**: Run Example 2 → board wakes and reboots on alarm

## Build Integration

The module is integrated into the board build via `mpconfigboard.mk`:

```makefile
# Board-specific wakeup module (Python wrapper)
FROZEN_MPY_DIRS += $(TOP)/ports/raspberrypi/boards/bhawks_brainbadge_rp2040

# Board-specific C sources for wakeup functionality
SRC_C += boards/$(BOARD)/wakeup.c           # Core initialization
SRC_C += boards/$(BOARD)/wakeup_native.c    # Native module bindings
```

## Pin Reference

| Pin | Name | Function | Pull | Active State |
|-----|------|----------|------|--------------|
| GPIO8 | RTC_ALARM | RTC interrupt (inverted) | DOWN | HIGH |
| GPIO11 | SW_DOWN | Down button | DOWN | HIGH |
| GPIO12 | SW_A | A button | DOWN | HIGH |
| GPIO13 | SW_B | B button | DOWN | HIGH |
| GPIO14 | SW_C | C button | DOWN | HIGH |
| GPIO15 | SW_UP | Up button | DOWN | HIGH |

All signals are active-HIGH with pull-down resistors configured in `wakeup_init()`.
