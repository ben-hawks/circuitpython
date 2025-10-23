"""Wakeup module for Brain Badge - detects button presses and manages RTC wakeup.

This module provides functionality to detect which button was pressed to wake
the board from sleep, or if the RTC alarm was triggered. It reads button states
and the RTC alarm pin at boot time (captured in C code), before they might be released.

The RTC clock output is disabled during board initialization (in wakeup.c) to
save power, and the RTC alarm pin (GPIO8) is configured to detect alarm signals.

The C implementation captures GPIO state very early in the boot process,
ensuring even brief button presses are detected.

IMPORTANT: RTC_ALARM on this board is active-HIGH due to MOSFET inversion.
The PCF85063A INT pin is open-drain active-LOW, but the board circuit inverts
this to provide an active-HIGH signal on GPIO8 (RTC_ALARM).
"""

import _wakeup_native
from micropython import const

try:
    import alarm
    import board

    _ALARM_AVAILABLE = True
except ImportError:
    _ALARM_AVAILABLE = False

# GPIO pin assignments for button detection
_SW_DOWN = const(11)
_SW_A = const(12)
_SW_B = const(13)
_SW_C = const(14)
_SW_UP = const(15)
_RTC_ALARM = const(8)


def get_gpio_state() -> int:
    """Get the GPIO state captured at boot time.

    Returns the 32-bit GPIO state that was captured when the board was
    initialized. Bits corresponding to button pins and the RTC alarm pin
    will be set if those inputs were active-HIGH.

    Returns:
        int: 32-bit GPIO state value
    """
    return _wakeup_native.get_gpio_state()


def reset_gpio_state() -> None:
    """Reset the captured GPIO state to 0.

    This is primarily useful for testing purposes.
    """
    _wakeup_native.reset_gpio_state()


def rtc_alarm_triggered() -> bool:
    """Check if the RTC alarm was triggered at wakeup.

    The PCF85063A RTC alarm output is active-HIGH on this board due to
    MOSFET inversion in the hardware. When an alarm triggers, GPIO8 goes HIGH.
    The pin has a pull-down resistor, so it's normally LOW (bit clear in state).

    With the OR operation capturing HIGH states:
    - Normal state: pin is LOW (0), bit is NOT set
    - Alarm triggered: pin goes HIGH (1), bit IS set

    The alarm signal persists until cleared in software via rtc.alarm_status = False.

    Returns:
        bool: True if the RTC alarm was triggered
    """
    state = get_gpio_state()
    # RTC alarm is active-HIGH with pull-down
    # If bit is set (1), alarm was triggered
    return bool(state & (1 << _RTC_ALARM))


def clear_rtc_alarm(rtc) -> None:
    """Clear the RTC alarm flag.

    This must be called after detecting an RTC alarm wake to clear the AF flag
    and allow the alarm to trigger again. The PCF85063A keeps the alarm flag
    set until explicitly cleared by software.

    Args:
        rtc: PCF85063A RTC object (from pcf85063a module)

    Example:
        import pcf85063a
        import busio
        import board
        import wakeup

        i2c = busio.I2C(board.SCL, board.SDA)
        rtc = pcf85063a.PCF85063A(i2c)

        if wakeup.rtc_alarm_triggered():
            print("Woke from RTC alarm!")
            wakeup.clear_rtc_alarm(rtc)  # Clear the alarm flag
    """
    rtc.alarm_status = False


def configure_rtc_alarm_interrupt(rtc, enable: bool = True) -> None:
    """Enable or disable RTC alarm interrupt output.

    The PCF85063A requires the AIE (Alarm Interrupt Enable) bit to be set
    for the INT pin to assert when an alarm occurs. This function sets or
    clears that bit.

    Args:
        rtc: PCF85063A RTC object
        enable: True to enable alarm interrupts, False to disable

    Example:
        import pcf85063a
        import busio
        import board
        import wakeup

        i2c = busio.I2C(board.SCL, board.SDA)
        rtc = pcf85063a.PCF85063A(i2c)

        # Enable alarm interrupt output
        wakeup.configure_rtc_alarm_interrupt(rtc, True)

        # Set alarm time
        import time
        current = rtc.datetime
        alarm_time = time.struct_time((
            current.tm_year, current.tm_mon, current.tm_mday,
            current.tm_hour, current.tm_min + 1, 0,  # 1 minute from now
            0, 0, -1
        ))
        rtc.alarm = alarm_time
    """
    rtc.alarm_interrupt = enable


# --- New helpers for diagnostics ---


def _describe_object(obj):
    """Return a short description of an object (type, repr, key attrs)."""
    if obj is None:
        return "<None>"
    try:
        t = type(obj)
        name = getattr(t, "__name__", str(t))
        # collect some useful attributes if present
        attrs = []
        for a in ("direction", "pull", "value", "deinit", "init", "drive_mode", "id"):
            if hasattr(obj, a):
                try:
                    v = getattr(obj, a)
                    # avoid calling callables
                    if not callable(v):
                        attrs.append(f"{a}={v}")
                    else:
                        attrs.append(f"{a}=<callable>")
                except Exception:
                    attrs.append(f"{a}=<error>")
        short = f"type={name} repr={repr(obj)}"
        if attrs:
            short += " " + ",".join(attrs)
        return short
    except Exception:
        try:
            return repr(obj)
        except Exception:
            return "<unprintable object>"


def describe_pin_owner(pin_obj):
    """Return diagnostic info about the object currently exposed on `board` for a pin.

    Attempts to provide useful information about the object so callers can
    identify what driver or code has claimed the pin (e.g. digitalio.DigitalInOut,
    a driver object, or a runtime board pin wrapper).

    Args:
        pin_obj: The object obtained from `getattr(board, 'SW_A')` or similar.

    Returns:
        str: Diagnostic string summarizing the pin owner, or a fallback note.
    """
    try:
        # If a Pin object from board is provided, describe it
        desc = _describe_object(pin_obj)

        # If it's a DigitalInOut-like object, try to find the underlying pin
        try:
            underlying = getattr(pin_obj, "pin", None)
            if underlying is not None:
                desc += f"; underlying_pin={_describe_object(underlying)}"
        except Exception:
            pass

        return desc
    except Exception as e:
        return f"(describe_pin_owner error: {e})"


# --- Existing alarm creation helpers, now with robust diagnostics ---


def create_rtc_alarm():
    """Create a PinAlarm for RTC wake with correct configuration.

    Returns an alarm.pin.PinAlarm configured for the RTC_ALARM pin (GPIO8)
    with the correct polarity (active-HIGH) and pull configuration.

    This is a convenience function that ensures the alarm is configured correctly.

    Returns:
        alarm.pin.PinAlarm or None: Configured alarm for RTC wake or None if pin is claimed

    Raises:
        ImportError: If alarm module is not available

    """
    if not _ALARM_AVAILABLE:
        raise ImportError("alarm module not available")

    # Prefer board.GP8 if present, otherwise try to locate RTC alarm in board
    pin = getattr(board, "GP8", None)
    if pin is None:
        # try common alternate names
        pin = getattr(board, "RTC_ALARM", None)
    if pin is None:
        # Best effort: try to find an attribute that references GPIO8 by number
        # Fall back to raising informative error
        raise RuntimeError("board has no attribute GP8/RTC_ALARM - cannot create RTC PinAlarm")

    # If the board pin object already exposes .value, it's already configured/claimed
    if hasattr(pin, "value"):
        # Caller should rely on the frozen wakeup GPIO snapshot instead
        return None

    # If board provides a DigitalInOut-like wrapper, extract the underlying pin
    raw_pin = getattr(pin, "pin", pin)

    try:
        # GPIO8 (RTC_ALARM) is active-HIGH, use value=True, no pull needed (configured in C)
        return alarm.pin.PinAlarm(raw_pin, value=True, pull=False)
    except Exception as e:
        owner_info = describe_pin_owner(pin)
        raise RuntimeError(f"Could not create RTC PinAlarm: {e}; pin_owner={owner_info}")


def create_button_alarm(button_name: str):
    """Create a PinAlarm for a button with correct configuration.

    Args:
        button_name: Button name - one of 'A', 'B', 'C', 'UP', 'DOWN'

    Returns:
        alarm.pin.PinAlarm or None: Configured alarm for the specified button or None if pin is claimed

    Raises:
        ValueError: If button_name is not recognized
        ImportError: If alarm module is not available

    """
    if not _ALARM_AVAILABLE:
        raise ImportError("alarm module not available")

    # Map to board attributes if present
    button_map = {
        "A": getattr(board, "SW_A", None),
        "B": getattr(board, "SW_B", None),
        "C": getattr(board, "SW_C", None),
        "UP": getattr(board, "SW_UP", None),
        "DOWN": getattr(board, "SW_DOWN", None),
    }

    button_name = button_name.upper()
    if button_name not in button_map:
        raise ValueError(
            f"Unknown button: {button_name}. Must be one of: {', '.join(button_map.keys())}"
        )

    pin = button_map[button_name]
    if pin is None:
        raise RuntimeError(f"board has no attribute for button SW_{button_name}")

    # If the board pin object already exposes .value, it is configured/claimed by other code
    if hasattr(pin, "value"):
        return None

    # If a wrapper object (DigitalInOut) was returned, extract the underlying pin
    raw_pin = getattr(pin, "pin", pin)

    try:
        # Buttons are active-HIGH, use value=True, no pull needed (configured in C)
        return alarm.pin.PinAlarm(raw_pin, value=True, pull=False)
    except Exception as e:
        owner_info = describe_pin_owner(pin)
        raise RuntimeError(
            f"Could not create PinAlarm for button {button_name}: {e}; pin_owner={owner_info}"
        )


def create_all_button_alarms():
    """Create PinAlarms for all buttons.

    Returns a list of PinAlarms configured for all five buttons.

    Returns:
        list: List of alarm.pin.PinAlarm objects for all buttons (may be empty)

    Raises:
        ImportError: If alarm module is not available

    """
    if not _ALARM_AVAILABLE:
        raise ImportError("alarm module not available")

    alarms = []
    errors = []
    for name in ("DOWN", "A", "B", "C", "UP"):
        try:
            pin = getattr(board, f"SW_{name}", None)
            # Skip pins already configured (DigitalInOut exposing .value)
            if pin is not None and hasattr(pin, "value"):
                # already claimed; rely on frozen wakeup GPIO snapshot
                continue
            pa = create_button_alarm(name)
            if pa is not None:
                alarms.append(pa)
        except Exception as e:
            # Collect error but continue creating other alarms
            errors.append(str(e))
    # Do not raise if no alarms created; just return empty list and let caller decide
    return alarms


def check_alarm_wake():
    """Check what caused the most recent wake from alarm.

    Examines alarm.wake_alarm and returns a descriptive string.
    This works with CircuitPython's native alarm module.

    Returns:
        str: Description of wake source, or None if not woken by alarm

    Example:
        import alarm
        import wakeup

        # Check if we woke from an alarm
        wake_source = wakeup.check_alarm_wake()
        if wake_source:
            print(f"Woke from: {wake_source}")
        else:
            print("Power on / hard reset")
    """
    if not _ALARM_AVAILABLE:
        return None

    if alarm.wake_alarm is None:
        return None

    # Check if it was a pin alarm
    if isinstance(alarm.wake_alarm, alarm.pin.PinAlarm):
        pin = alarm.wake_alarm.pin

        # Map pins to friendly names
        if pin == board.RTC_ALARM:
            return "RTC alarm"
        elif pin == board.SW_A:
            return "button A"
        elif pin == board.SW_B:
            return "button B"
        elif pin == board.SW_C:
            return "button C"
        elif pin == board.SW_UP:
            return "button UP"
        elif pin == board.SW_DOWN:
            return "button DOWN"
        else:
            return f"pin {pin}"

    # Time alarm
    elif isinstance(alarm.wake_alarm, alarm.time.TimeAlarm):
        return "time alarm"

    return "unknown alarm"


def button_pressed(pin_number: int) -> bool:
    """Check if a specific button was pressed at wakeup.

    Args:
        pin_number: GPIO pin number to check (e.g., 11 for SW_DOWN)

    Returns:
        bool: True if the button was pressed (pin was HIGH) at boot time
    """
    state = get_gpio_state()
    # Check if the bit for this pin is set (button was pressed)
    return bool(state & (1 << pin_number))


def button_down() -> bool:
    """Check if DOWN button was pressed at wakeup."""
    return button_pressed(_SW_DOWN)


def button_up() -> bool:
    """Check if UP button was pressed at wakeup."""
    return button_pressed(_SW_UP)


def button_a() -> bool:
    """Check if A button was pressed at wakeup."""
    return button_pressed(_SW_A)


def button_b() -> bool:
    """Check if B button was pressed at wakeup."""
    return button_pressed(_SW_B)


def button_c() -> bool:
    """Check if C button was pressed at wakeup."""
    return button_pressed(_SW_C)


def pressed_to_wake() -> bool:
    """Check if any button was pressed to wake the board."""
    return button_down() or button_up() or button_a() or button_b() or button_c()


def wakeup_reason() -> str:
    """Get a string describing the wakeup reason.

    Checks both the native alarm module (if available) and the GPIO capture
    to determine what caused the wakeup.

    Returns:
        str: Description of what triggered the wakeup (e.g., "RTC alarm", "button A", etc.)
    """
    # First check alarm.wake_alarm if available
    alarm_source = check_alarm_wake()
    if alarm_source:
        return alarm_source

    # Fall back to GPIO state capture
    if rtc_alarm_triggered():
        return "RTC alarm"
    elif button_a():
        return "button A"
    elif button_b():
        return "button B"
    elif button_c():
        return "button C"
    elif button_down():
        return "button DOWN"
    elif button_up():
        return "button UP"
    else:
        return "power on/reset"


def is_pin_claimed(pin):
    """Return True if `pin` appears to be claimed by another driver (DigitalInOut or similar).

    We attempt to create a short-lived DigitalInOut to test claimability. If creation
    raises an exception, the pin is likely already in use. If digitalio is not
    available, conservatively return False.
    """
    try:
        import digitalio
    except Exception:
        # Can't test without digitalio; assume not claimed
        return False

    try:
        d = digitalio.DigitalInOut(pin)
        # If creation succeeded, immediately deinit to release the pin
        try:
            d.deinit()
        except Exception:
            pass
        return False
    except Exception:
        return True
