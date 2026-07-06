#!/usr/bin/env python3
"""
Smart Safety & Accident Detection System — Raspberry Pi 5  (BioEV)
===================================================================
Sensors  : HC-SR04 (ultrasonic), MPU9250 (gyro+accel, I2C), Pulse sensor (SPI/ADC)
Outputs  : LED, Buzzer, SSD1306 OLED (I2C)
Camera   : Pi Camera via rpicam-still  →  HTTP POST to server
Server   : sends back "Pothole has been detected." message

ACCIDENT DETECTION (MPU9250-based):
  Mode A — Sustained tilt : tilt > GYRO_FALL_DEG for > FALL_SUSTAIN_S seconds
  Mode B — Impact + tilt  : accel spike then tilt within IMPACT_FALL_WINDOW_S
  Mode C — Full accident  : all four original conditions simultaneously

RESET (three ways):
  1. Physical button on RESET_PIN  — press any time after accident
  2. Auto-reset timer              — fires after RESET_TIMEOUT_S (0 = disabled)
  3. Keyboard 'r' + Enter          — for testing on a terminal
"""

import io
import math
import subprocess
import threading
import time

import requests
import smbus2
import spidev
from gpiozero import Buzzer, Button, DistanceSensor, LED
from luma.core.interface.serial import i2c as luma_i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import Image, ImageFont

# ──────────────────────────────────────────────────────────────────
#  CONFIGURATION
# ──────────────────────────────────────────────────────────────────

SERVER_IP   = "192.168.81.223"
SERVER_PORT = 5000

# GPIO pins
LED_PIN    = 17
BUZZER_PIN = 26
TRIG_PIN   = 23
ECHO_PIN   = 24
RESET_PIN  = 22      # ← push-button between GPIO22 and GND; internal pull-up used

# I2C
MPU9250_ADDR  = 0x68
I2C_BUS       = 1
OLED_I2C_PORT = 1
OLED_I2C_ADDR = 0x3C

# SPI (pulse sensor via MCP3008 ADC)
SPI_BUS    = 0
SPI_DEVICE = 0
PULSE_CH   = 0

# Distance thresholds (cm)
DIST_CLOSE     = 60
DIST_STOP      = 40
DIST_COLLISION = 15

# ── Fall / accident detection ──────────────────────────────────────
GYRO_FALL_DEG         = 60
FALL_SUSTAIN_S        = 2.0
ACCEL_IMPACT_G        = 1.8
ACCEL_REST_G          = 0.15
ACCEL_REST_TICKS_NEED = 20
IMPACT_FALL_WINDOW_S  = 3.0
PULSE_LOSS_TIMEOUT_S  = 5

# ── Post-accident timeouts ─────────────────────────────────────────
BUZZER_ACCIDENT_TIMEOUT_S = 30    # buzzer off after this many seconds (0 = forever)
LED_ACCIDENT_TIMEOUT_S    = 120   # LED off after this many seconds   (0 = forever)

# ── Reset ──────────────────────────────────────────────────────────
# Auto-reset fires this many seconds after the accident is confirmed.
# Set to 0 to disable auto-reset (button or keyboard only).
# Must be > BUZZER_ACCIDENT_TIMEOUT_S so the buzzer has already stopped.
RESET_TIMEOUT_S = 5   # 5 minutes after accident → auto-reset
                         # set to 0 to require manual button press

# ── ntfy.sh ────────────────────────────────────────────────────────
NTFY_TOPIC  = "bioe_accident_alert_xyz123"   # ← change this
NTFY_SERVER = "https://ntfy.sh"

# Camera
CAPTURE_INTERVAL_S = 2

# ──────────────────────────────────────────────────────────────────
#  SHARED STATE
# ──────────────────────────────────────────────────────────────────

# Holds the initial clean values so _reset_system() can restore them.
_INITIAL_STATE = {
    "distance_cm"        : 999.0,
    "gyro_tilt_deg"      : 0.0,
    "accel_mag_g"        : 1.0,
    "pulse_bpm"          : 0.0,
    "last_pulse_time"    : 0.0,     # reset thread will set to time.time()

    "accel_impact_seen"  : False,
    "accel_rest_seen"    : False,
    "impact_timestamp"   : 0.0,

    "tilt_exceed_start"  : None,

    "accident_confirmed" : False,
    "accident_type"      : "",
    "accident_timestamp" : 0.0,
    "buzzer_active"      : False,

    "server_message"     : "",
    "led_state"          : "off",
}

state = dict(_INITIAL_STATE)
state["last_pulse_time"] = time.time()
state_lock = threading.Lock()

# Timers are kept so they can be cancelled on reset
_buzzer_timer = None
_led_timer    = None
_reset_timer  = None
_timers_lock  = threading.Lock()

# ──────────────────────────────────────────────────────────────────
#  HARDWARE INIT
# ──────────────────────────────────────────────────────────────────

led          = LED(LED_PIN)
buzzer       = Buzzer(BUZZER_PIN)
reset_button = Button(RESET_PIN, pull_up=True, bounce_time=0.05)
ultrasonic   = DistanceSensor(echo=ECHO_PIN, trigger=TRIG_PIN, max_distance=3)

oled_serial = luma_i2c(port=OLED_I2C_PORT, address=OLED_I2C_ADDR)
oled        = ssd1306(oled_serial)

i2c_bus = smbus2.SMBus(I2C_BUS)

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = 1_350_000

# ──────────────────────────────────────────────────────────────────
#  MPU9250 HELPERS
# ──────────────────────────────────────────────────────────────────

_PWR_MGMT_1  = 0x6B
_ACCEL_CFG   = 0x1C
_GYRO_CFG    = 0x1B
_ACCEL_XOUT  = 0x3B
_GYRO_XOUT   = 0x43
_ACCEL_SCALE = 16384.0
_GYRO_SCALE  = 131.0

def _mpu_init():
    i2c_bus.write_byte_data(MPU9250_ADDR, _PWR_MGMT_1, 0x00)
    i2c_bus.write_byte_data(MPU9250_ADDR, _ACCEL_CFG,  0x00)
    i2c_bus.write_byte_data(MPU9250_ADDR, _GYRO_CFG,   0x00)
    time.sleep(0.1)

def _read_word_2c(reg: int) -> int:
    hi  = i2c_bus.read_byte_data(MPU9250_ADDR, reg)
    lo  = i2c_bus.read_byte_data(MPU9250_ADDR, reg + 1)
    val = (hi << 8) | lo
    return val - 0x10000 if val >= 0x8000 else val

def _read_mpu9250():
    ax = _read_word_2c(_ACCEL_XOUT)     / _ACCEL_SCALE
    ay = _read_word_2c(_ACCEL_XOUT + 2) / _ACCEL_SCALE
    az = _read_word_2c(_ACCEL_XOUT + 4) / _ACCEL_SCALE
    gx = _read_word_2c(_GYRO_XOUT)      / _GYRO_SCALE
    gy = _read_word_2c(_GYRO_XOUT + 2)  / _GYRO_SCALE
    gz = _read_word_2c(_GYRO_XOUT + 4)  / _GYRO_SCALE
    return (ax, ay, az), (gx, gy, gz)

# ──────────────────────────────────────────────────────────────────
#  MCP3008 SPI ADC HELPER
# ──────────────────────────────────────────────────────────────────

def _read_adc(channel: int) -> int:
    assert 0 <= channel <= 7
    reply = spi.xfer2([1, (8 + channel) << 4, 0])
    return ((reply[1] & 3) << 8) | reply[2]

# ──────────────────────────────────────────────────────────────────
#  OLED DISPLAY HELPER
# ──────────────────────────────────────────────────────────────────

_FONT = ImageFont.load_default()

def oled_show(text: str):
    lines = []
    for raw_line in text.split("\n"):
        words, cur = raw_line.split(), ""
        for w in words:
            candidate = (cur + " " + w).strip()
            if len(candidate) * 6 < oled.width - 4:
                cur = candidate
            else:
                if cur:
                    lines.append(cur)
                cur = w
        if cur:
            lines.append(cur)
    lines = lines[:4]
    with canvas(oled) as draw:
        draw.rectangle(oled.bounding_box, fill="black")
        for i, line in enumerate(lines):
            draw.text((2, 2 + i * 15), line, fill="white", font=_FONT)

# ──────────────────────────────────────────────────────────────────
#  SYSTEM RESET
#  Cancels all pending timers, wipes every accident-related state
#  field back to initial values, silences hardware, shows confirmation.
#  Safe to call from any thread (button callback, Timer, keyboard).
# ──────────────────────────────────────────────────────────────────

def _reset_system(source: str = "manual"):
    """
    Resets the system to normal monitoring after an accident.

    What gets cleared
    ─────────────────
    • accident_confirmed / accident_type / accident_timestamp
    • accel_impact_seen / accel_rest_seen / impact_timestamp
    • tilt_exceed_start  (so fall-timer starts fresh)
    • buzzer_active / led_state
    • server_message

    What does NOT change
    ────────────────────
    • Live sensor readings (distance_cm, gyro_tilt_deg, etc.)
      — these are updated by their own threads continuously.
    • last_pulse_time — reset to now so pulse-loss flag is cleared.
    """
    global _buzzer_timer, _led_timer, _reset_timer

    print(f"\n[RESET] System reset triggered — source: {source}")

    # Cancel any pending timers so they don't fire after reset
    with _timers_lock:
        for t in (_buzzer_timer, _led_timer, _reset_timer):
            if t is not None:
                t.cancel()
        _buzzer_timer = None
        _led_timer    = None
        _reset_timer  = None

    # Silence hardware immediately
    buzzer.off()
    led.off()

    # Restore all accident-related state fields
    with state_lock:
        state["accident_confirmed"] = False
        state["accident_type"]      = ""
        state["accident_timestamp"] = 0.0
        state["buzzer_active"]      = False

        state["accel_impact_seen"]  = False
        state["accel_rest_seen"]    = False
        state["impact_timestamp"]   = 0.0

        state["tilt_exceed_start"]  = None

        state["last_pulse_time"]    = time.time()   # clear pulse-loss flag
        state["server_message"]     = ""
        state["led_state"]          = "off"

    print("[RESET] All accident flags cleared — resuming normal monitoring\n")

    # Visual confirmation: brief blink + OLED message
    oled_show(f"System Reset\n({source})\nMonitoring...")
    led.on();  time.sleep(0.2)
    led.off(); time.sleep(0.2)
    led.on();  time.sleep(0.2)
    led.off()

# ──────────────────────────────────────────────────────────────────
#  BUZZER / LED STOP  (called by threading.Timer)
# ──────────────────────────────────────────────────────────────────

def _stop_buzzer():
    buzzer.off()
    with state_lock:
        state["buzzer_active"] = False
    print(f"[BUZZER] Stopped after {BUZZER_ACCIDENT_TIMEOUT_S}s")

def _stop_led():
    with state_lock:
        state["led_state"] = "off"
    print(f"[LED] Stopped after {LED_ACCIDENT_TIMEOUT_S}s")

# ──────────────────────────────────────────────────────────────────
#  PUSH NOTIFICATION — ntfy.sh
# ──────────────────────────────────────────────────────────────────

def _send_notification(accident_type: str):
    messages = {
        "fall"        : ("BioEV: Bicycle Fallen",
                         "Bicycle has fallen sideways. Rider may need help!"),
        "impact_fall" : ("BioEV: Crash Detected",
                         "Hard impact followed by a fall. Check on the rider!"),
        "full"        : ("BioEV: Serious Accident",
                         "All conditions triggered. Emergency likely!"),
    }
    title, body = messages.get(accident_type, ("BioEV Alert", "Accident detected."))

    def _post():
        try:
            requests.post(
                f"{NTFY_SERVER}/{NTFY_TOPIC}",
                data=body.encode("utf-8"),
                headers={
                    "Title"   : title,
                    "Priority": "urgent",
                    "Tags"    : "rotating_light,sos",
                },
                timeout=10,
            )
            print(f"[NOTIFY] Sent: {title}")
        except Exception as e:
            print(f"[NOTIFY] ntfy.sh failed: {e}")

    threading.Thread(target=_post, daemon=True, name="ntfy_post").start()

# ──────────────────────────────────────────────────────────────────
#  ACCIDENT TRIGGER
# ──────────────────────────────────────────────────────────────────

def _trigger_accident(accident_type: str):
    global _buzzer_timer, _led_timer, _reset_timer

    with state_lock:
        if state["accident_confirmed"]:
            return
        state["accident_confirmed"] = True
        state["accident_type"]      = accident_type
        state["accident_timestamp"] = time.time()
        state["buzzer_active"]      = True
        state["led_state"]          = "blink_fast"

    print(f"\n[!!!] ACCIDENT CONFIRMED — type: '{accident_type}'\n")
    buzzer.on()
    oled_show(f"ACCIDENT!\n{accident_type.upper()}\nPress RESET btn\nor wait {RESET_TIMEOUT_S}s")
    _send_notification(accident_type)

    with _timers_lock:
        # Buzzer off timer
        if BUZZER_ACCIDENT_TIMEOUT_S > 0:
            _buzzer_timer = threading.Timer(BUZZER_ACCIDENT_TIMEOUT_S, _stop_buzzer)
            _buzzer_timer.daemon = True
            _buzzer_timer.start()

        # LED off timer
        if LED_ACCIDENT_TIMEOUT_S > 0:
            _led_timer = threading.Timer(LED_ACCIDENT_TIMEOUT_S, _stop_led)
            _led_timer.daemon = True
            _led_timer.start()

        # Auto-reset timer — fires after RESET_TIMEOUT_S
        if RESET_TIMEOUT_S > 0:
            _reset_timer = threading.Timer(
                RESET_TIMEOUT_S,
                _reset_system,
                kwargs={"source": "auto-timer"},
            )
            _reset_timer.daemon = True
            _reset_timer.start()
            print(f"[RESET] Auto-reset scheduled in {RESET_TIMEOUT_S}s")

# ──────────────────────────────────────────────────────────────────
#  LED MANAGER THREAD
# ──────────────────────────────────────────────────────────────────

def led_manager():
    current    = "off"
    blink_tick = False
    while True:
        with state_lock:
            desired = state["led_state"]
        if desired != current:
            led.off()
            current    = desired
            blink_tick = False
        if current == "off":
            led.off()
        elif current == "on":
            led.on()
        elif current == "blink_slow":
            blink_tick = not blink_tick
            led.on() if blink_tick else led.off()
            time.sleep(0.5)
            continue
        elif current == "blink_fast":
            blink_tick = not blink_tick
            led.on() if blink_tick else led.off()
            time.sleep(0.15)
            continue
        time.sleep(0.05)

# ──────────────────────────────────────────────────────────────────
#  RESET BUTTON THREAD
#  Watches RESET_PIN via gpiozero Button.
#  Only resets when an accident is currently confirmed — pressing it
#  during normal monitoring does nothing.
# ──────────────────────────────────────────────────────────────────

def reset_button_thread():
    print(f"[INIT] Reset button listening on GPIO{RESET_PIN}")

    def _on_press():
        with state_lock:
            accident_active = state["accident_confirmed"]
        if accident_active:
            _reset_system(source="button")
        else:
            # Button pressed during normal operation — just show a message
            print("[RESET] Button pressed but no accident active — ignored")

    reset_button.when_pressed = _on_press

    # Keep thread alive (gpiozero handles the callback internally)
    while True:
        time.sleep(1)

# ──────────────────────────────────────────────────────────────────
#  KEYBOARD RESET THREAD  (useful for testing without a button)
#  Type  r  + Enter in the terminal to reset.
# ──────────────────────────────────────────────────────────────────

def keyboard_reset_thread():
    print("[INIT] Keyboard reset: type 'r' + Enter to reset after accident")
    while True:
        try:
            key = input()
            if key.strip().lower() == "r":
                with state_lock:
                    accident_active = state["accident_confirmed"]
                if accident_active:
                    _reset_system(source="keyboard")
                else:
                    print("[RESET] No accident active — nothing to reset")
        except EOFError:
            break   # stdin closed (e.g. running as a service)

# ──────────────────────────────────────────────────────────────────
#  SENSOR THREADS
# ──────────────────────────────────────────────────────────────────

def ultrasonic_thread():
    while True:
        try:
            dist_cm = (ultrasonic.distance or 3.0) * 100.0
        except Exception:
            dist_cm = 999.0
        with state_lock:
            state["distance_cm"] = dist_cm
        time.sleep(0.05)


def mpu9250_thread():
    prev_mag   = 1.0
    rest_ticks = 0
    while True:
        try:
            (ax, ay, az), _ = _read_mpu9250()
        except Exception as e:
            print(f"[MPU9250] read error: {e}")
            time.sleep(0.1)
            continue

        tilt = math.degrees(math.atan2(math.sqrt(ax**2 + ay**2), abs(az)))
        mag  = math.sqrt(ax**2 + ay**2 + az**2)
        now  = time.time()

        with state_lock:
            state["gyro_tilt_deg"] = tilt
            state["accel_mag_g"]   = mag

            if (mag - prev_mag) > ACCEL_IMPACT_G:
                state["accel_impact_seen"] = True
                state["impact_timestamp"]  = now
                rest_ticks = 0
                print(f"[MPU] Impact: Δg={mag - prev_mag:.2f}  tilt={tilt:.1f}°")

            if state["accel_impact_seen"]:
                if abs(mag - 1.0) < ACCEL_REST_G:
                    rest_ticks += 1
                    if rest_ticks >= ACCEL_REST_TICKS_NEED:
                        state["accel_rest_seen"] = True
                else:
                    rest_ticks = 0

            if tilt >= GYRO_FALL_DEG:
                if state["tilt_exceed_start"] is None:
                    state["tilt_exceed_start"] = now
                    print(f"[MPU] Tilt threshold crossed: {tilt:.1f}° — timer started")
            else:
                if state["tilt_exceed_start"] is not None:
                    print(f"[MPU] Tilt recovered ({tilt:.1f}°) — timer reset")
                state["tilt_exceed_start"] = None

        prev_mag = mag
        time.sleep(0.05)


def pulse_thread():
    SAMPLE_RATE = 20
    BUF_SECS    = 10
    buf         = []
    while True:
        val = _read_adc(PULSE_CH)
        buf.append(val)
        if len(buf) > SAMPLE_RATE * BUF_SECS:
            buf.pop(0)
        bpm = 0.0
        if len(buf) >= SAMPLE_RATE * 2:
            mean  = sum(buf) / len(buf)
            peaks = sum(
                1 for i in range(1, len(buf) - 1)
                if buf[i] > mean * 1.2
                and buf[i] > buf[i - 1]
                and buf[i] > buf[i + 1]
            )
            bpm_raw = (peaks / (len(buf) / SAMPLE_RATE)) * 60.0
            if 40 <= bpm_raw <= 200:
                bpm = bpm_raw
        with state_lock:
            state["pulse_bpm"] = bpm
            if bpm > 0:
                state["last_pulse_time"] = time.time()
        time.sleep(1.0 / SAMPLE_RATE)


def camera_thread():
    IMG_PATH = "/tmp/pi_capture.jpg"
    while True:
        try:
            result = subprocess.run(
                ["rpicam-still", "--output", IMG_PATH, "--nopreview",
                 "--timeout", "200", "--width", "640", "--height", "480"],
                capture_output=True, timeout=15,
            )
            if result.returncode != 0:
                print(f"[Camera] rpicam-still failed: {result.stderr.decode()}")
                time.sleep(CAPTURE_INTERVAL_S)
                continue

            img = Image.open(IMG_PATH).convert("RGB")
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=75)
            buf.seek(0)

            resp = requests.post(
                f"http://{SERVER_IP}:{SERVER_PORT}/upload",
                files={"image": ("frame.jpg", buf, "image/jpeg")},
                timeout=6,
            )
            if resp.ok:
                msg = resp.json().get("message", "")
                if msg:
                    with state_lock:
                        state["server_message"] = msg
                    print(f"[Camera] Server: {msg}")

        except requests.exceptions.ConnectionError:
            print("[Camera] Server unreachable — retrying")
        except Exception as e:
            print(f"[Camera] Error: {e}")
        time.sleep(CAPTURE_INTERVAL_S)

# ──────────────────────────────────────────────────────────────────
#  ACCIDENT WATCH THREAD
# ──────────────────────────────────────────────────────────────────

def accident_watch_thread():
    while True:
        now = time.time()
        with state_lock:
            already   = state["accident_confirmed"]
            tilt      = state["gyro_tilt_deg"]
            t_start   = state["tilt_exceed_start"]
            impact    = state["accel_impact_seen"]
            imp_time  = state["impact_timestamp"]
            rest      = state["accel_rest_seen"]
            dist      = state["distance_cm"]
            pulse_age = now - state["last_pulse_time"]

        if already:
            time.sleep(0.05)
            continue

        # Mode A — sustained tilt
        if t_start is not None:
            if (now - t_start) >= FALL_SUSTAIN_S:
                print(f"[ACCIDENT] Mode A — tilt {tilt:.1f}° for {now - t_start:.1f}s")
                _trigger_accident("fall")
                continue

        # Mode B — impact then tilt
        if impact and t_start is not None:
            if (now - imp_time) <= IMPACT_FALL_WINDOW_S:
                print(f"[ACCIDENT] Mode B — impact {now - imp_time:.1f}s ago + tilt {tilt:.1f}°")
                _trigger_accident("impact_fall")
                continue

        # Mode C — all four conditions
        if (dist <= DIST_COLLISION and
                tilt >= GYRO_FALL_DEG and
                pulse_age > PULSE_LOSS_TIMEOUT_S and
                impact and rest):
            print("[ACCIDENT] Mode C — all four conditions")
            _trigger_accident("full")

        time.sleep(0.05)

# ──────────────────────────────────────────────────────────────────
#  MAIN LOOP
# ──────────────────────────────────────────────────────────────────

def main():
    print("=" * 55)
    print("  BioEV Safety System — Raspberry Pi 5")
    print("=" * 55)

    try:
        _mpu_init()
        print("[INIT] MPU9250 ready")
    except Exception as e:
        print(f"[INIT] MPU9250 failed: {e}")

    for fn in [
        led_manager,
        ultrasonic_thread,
        mpu9250_thread,
        pulse_thread,
        camera_thread,
        accident_watch_thread,
        reset_button_thread,
        keyboard_reset_thread,
    ]:
        t = threading.Thread(target=fn, daemon=True, name=fn.__name__)
        t.start()
        print(f"[INIT] Thread started: {fn.__name__}")

    oled_show("System Ready")
    with state_lock:
        state["led_state"] = "on"
    time.sleep(1)
    with state_lock:
        state["led_state"] = "off"

    print(f"\n[INIT] All systems up.\n"
          f"       ntfy topic     : {NTFY_TOPIC}\n"
          f"       Fall threshold : {GYRO_FALL_DEG}°  sustain: {FALL_SUSTAIN_S}s\n"
          f"       Buzzer stops   : {BUZZER_ACCIDENT_TIMEOUT_S}s after accident\n"
          f"       Auto-reset     : {'disabled' if RESET_TIMEOUT_S == 0 else str(RESET_TIMEOUT_S) + 's after accident'}\n"
          f"       Reset button   : GPIO{RESET_PIN}\n"
          f"       Keyboard reset : type 'r' + Enter\n")

    try:
        while True:
            with state_lock:
                dist      = state["distance_cm"]
                tilt      = state["gyro_tilt_deg"]
                pulse     = state["pulse_bpm"]
                srv_msg   = state["server_message"]
                accident  = state["accident_confirmed"]
                acc_type  = state["accident_type"]
                acc_time  = state["accident_timestamp"]
                buzzer_on = state["buzzer_active"]

            # ── (1) Accident confirmed — show status, never touch buzzer ──────
            if accident:
                elapsed = int(time.time() - acc_time)
                reset_in = max(0, RESET_TIMEOUT_S - elapsed) if RESET_TIMEOUT_S > 0 else 0

                if buzzer_on:
                    status = "Buzzer ON"
                else:
                    status = "Buzzer OFF"

                if RESET_TIMEOUT_S > 0 and reset_in > 0:
                    oled_show(f"ACCIDENT!\n{status}\nAuto-reset in\n{reset_in}s")
                else:
                    oled_show(f"ACCIDENT!\n{acc_type.upper()}\n{status}\nPress RESET btn")

                time.sleep(0.5)
                continue

            # ── (2) Pothole from server ────────────────────────────────────────
            if "Pothole has been detected" in srv_msg:
                print(f"[SERVER] {srv_msg}")
                oled_show(srv_msg)
                with state_lock:
                    state["led_state"]      = "blink_slow"
                    state["server_message"] = ""
                time.sleep(5)
                with state_lock:
                    state["led_state"] = "off"
                continue

            # ── (3) Distance alerts ────────────────────────────────────────────
            if dist <= DIST_COLLISION:
                oled_show("COLLISION\nOCCURRED")
                buzzer.on()
                with state_lock:
                    state["led_state"] = "blink_fast"

            elif dist <= DIST_STOP:
                oled_show("STOP")
                buzzer.on()
                with state_lock:
                    state["led_state"] = "blink_slow"

            elif dist <= DIST_CLOSE:
                oled_show("Object close")
                buzzer.off()
                with state_lock:
                    state["led_state"] = "on"

            else:
                oled_show(f"D:{dist:.0f}cm T:{tilt:.0f}deg\nPulse:{pulse:.0f}bpm")
                buzzer.off()
                with state_lock:
                    state["led_state"] = "off"

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C — shutting down gracefully...")

    finally:
        buzzer.off()
        led.off()
        oled_show("System Off")
        time.sleep(0.5)
        oled.cleanup()
        spi.close()
        i2c_bus.close()
        print("[EXIT] Done.")


if __name__ == "__main__":
    main()
