import utime
import sys
import machine
from machine import Pin, ADC, PWM
from Mdriver import MotorDriver
from tcs34725 import *
from hcsr04 import * 

# ─── Hardware setup ──────────────────────────────────────────────────────────
motors = MotorDriver()
button = Pin(10, Pin.IN, Pin.PULL_DOWN)

servo  = PWM(Pin(0))
servo.freq(50)
kicker = PWM(Pin(2))
kicker.freq(50)

def set_servo_us(us):
    servo.duty_u16(int(us * 65535 / 20000))

def set_kicker_us(us):
    kicker.duty_u16(int(us * 65535 / 20000))

REST         = 1480
WINDUP       = 2400
KICK_BACK    = 900
KICK_FORWARD = 2000

print("Initializing servos...")
set_servo_us(REST)
utime.sleep(1)
set_kicker_us(KICK_BACK)
utime.sleep(1)

# ─── IR setup ────────────────────────────────────────────────────────────────
right_adc = machine.ADC(0)
left_adc  = machine.ADC(1)

RIGHT_OFFSET  = 1500 #if the right adc is higher for some reason increase this 
SAMPLES       = 2 #ir sensors are already presampled within the hardware but just in case add this - only ~5 is best 
DETECT_THRESH = 40250 #decrease in order to see further, increase if triggering for no reason.
WIDE_MARGIN   = 3000 #if the two IR sensors are within 3000 adc values of each other 

SEARCH_SPEED  = 45
TURN_SPEED    = 55
FORWARD_SPEED = 50
TURN_MS       = 600
FORWARD_MS    = 2000
SETTLE_MS     = 450

COLOR_DRIVE_SPEED = 50 #placeholders, can be removed. They were here mainly because once the ball is obtained there isn't a way to know what to do 
COLOR_DRIVE_MS    = 1000
COLOR_PAUSE_MS    = 3000

# ─── Color sensor setup ──────────────────────────────────────────────────────
tcs = TCS34725(scl=Pin(5), sda=Pin(4))
if not tcs.isconnected:
    print("Color sensor not found — terminating") #color sensor has to be connected at all times 
    sys.exit()#delete maybe just so if the color sensor gets dced it can maybe come back 

tcs.gain     = TCSGAIN_LOW
tcs.integ    = TCSINTEG_HIGH

consecutive    = 0
last_color     = "-"
color_interrupt = None   # set by wait_ms() when a color is locked mid-maneuver


#   OVERCURRENT SETUP

SenseA = Pin(14, Pin.IN, Pin.PULL_UP)
SenseB = Pin(15, Pin.IN, Pin.PULL_UP)
overcurrent_interrupt = False

# OVERCURRENT

def check_overcurrent():
    global overcurrent_interrupt
    if SenseA.value() == 1 or SenseB.value() == 1:
        print("overcurrent stop!")
        motors.Stop(0)
        utime.sleep(2)
        overcurrent_interrupt = True
        return True
    return False
    
# ════════════════════════════════════════════════════════════════════════════
# COLOR SENSOR
# ════════════════════════════════════════════════════════════════════════════

def rgb_to_hsv(r, g, b):
    r_f, g_f, b_f = r/255.0, g/255.0, b/255.0
    cmax  = max(r_f, g_f, b_f)
    cmin  = min(r_f, g_f, b_f)
    delta = cmax - cmin
    if delta == 0:
        h = 0
    elif cmax == r_f:
        h = 60 * (((g_f - b_f) / delta) % 6)
    elif cmax == g_f:
        h = 60 * (((b_f - r_f) / delta) + 2)
    else:
        h = 60 * (((r_f - g_f) / delta) + 4)
    s = 0 if cmax == 0 else (delta / cmax) * 100
    v = cmax * 100
    return int(h), int(s), int(v)

def hsv_to_color_name(h, s, v):
    if s < 45 or v < 20: #values are like this due to random green bias in the lab room. 
        return "-"
    if h < 20 or h >= 340:
        return "Red"
    elif h < 160:
        return "Green"
    elif h < 270:
        return "Blue"
    return "-"

def scale_to_255(count, gain_factor, overflow_count):
    if count < 0:
        return 255
    return min(255, int(count * 255 // (overflow_count // gain_factor)))

def check_color():
    """Single color sensor read. Returns locked color name or '-'."""
    global consecutive, last_color
    c, r, g, b = tcs.colors
    gf  = tcs.gain_factor
    oc  = tcs.overflow_count
    r8  = scale_to_255(r, gf, oc)
    g8  = scale_to_255(g, gf, oc)
    b8  = scale_to_255(b, gf, oc)
    h, s, v = rgb_to_hsv(r8, g8, b8)
    color   = hsv_to_color_name(h, s, v)
    print("  color → R:{} G:{} B:{} H:{} S:{}% V:{}% → {}".format(
          r8, g8, b8, h, s, v, color))
    if color == last_color and color != "-":
        consecutive += 1
    else:
        consecutive = 1
        last_color  = color
    if consecutive >= 3:
        locked      = last_color
        consecutive = 0
        last_color  = "-"
        return locked
    return "-"

# ════════════════════════════════════════════════════════════════════════════
# NON-BLOCKING WAIT — the key to continuous color checking
# ════════════════════════════════════════════════════════════════════════════

def wait_ms(duration):
    """
    Waits for duration ms in 100ms chunks.
    Polls color sensor each chunk.
    Returns True and sets color_interrupt if a color is locked mid-wait.
    Returns False if the full duration elapsed with nothing detected.
    """
    global color_interrupt
    elapsed = 0
    while elapsed < duration:
        chunk = min(100, duration - elapsed)  # don't overshoot
        utime.sleep_ms(chunk)
        elapsed += chunk
        locked = check_color()
        if locked != "-":
            color_interrupt = locked
            return True   # interrupted — caller should stop and bail
    return False           # completed normally

# ════════════════════════════════════════════════════════════════════════════
# IR SENSORS
# ════════════════════════════════════════════════════════════════════════════

def read_ir():
    r_total = 0
    l_total = 0
    for _ in range(SAMPLES):
        r_total += right_adc.read_u16()
        l_total += left_adc.read_u16()
        utime.sleep_ms(50)
    r = (r_total // SAMPLES) - RIGHT_OFFSET
    l =  l_total // SAMPLES
    return r, l

def ir_detected():
    r, l = read_ir()
    return r > DETECT_THRESH or l > DETECT_THRESH

def obj_in_front():
    r, l = read_ir()
    if r > DETECT_THRESH and l > DETECT_THRESH:
        return abs(r - l) < WIDE_MARGIN
    return False

def forward_check(speed, duration):
    """Drive forward, checking IR and color every 100ms."""
    global color_interrupt
    step    = 100
    elapsed = 0
    motors.Forward(speed)
    while elapsed < duration:
        utime.sleep_ms(step)
        elapsed += step
        if check_overcurrent():
            return False
        if obj_in_front():
            motors.Stop(0)
            return True
        locked = check_color()
        if locked != "-":
            motors.Stop(0)
            color_interrupt = locked
            return False   # not an IR hit, but caller checks color_interrupt
    motors.Stop(0)
    return False

# ─── IR evasion — all use wait_ms() so color is checked throughout ────────

def both_trig():
    global color_interrupt
    print("WIDE OBJECT — reversing and turning away")
    motors.Reverse(SEARCH_SPEED)
    if wait_ms(800): return
    motors.TurnLeft(TURN_SPEED)
    if wait_ms(800): return
    motors.Stop(0)
    wait_ms(SETTLE_MS)

def both_trig_right():
    global color_interrupt
    print("Both triggered, right stronger — turning right")
    motors.TurnRight(TURN_SPEED)
    if wait_ms(TURN_MS): return
    if obj_in_front():
        motors.Stop(0)
        if wait_ms(SETTLE_MS): return
        both_trig()
        return
    if forward_check(FORWARD_SPEED, FORWARD_MS): 
        both_trig()
        return
    if color_interrupt: return
    wait_ms(SETTLE_MS)

def both_trig_left():
    global color_interrupt
    print("Both triggered, left stronger — turning left")
    motors.TurnLeft(TURN_SPEED)
    if wait_ms(TURN_MS): return
    if obj_in_front():
        motors.Stop(0)
        if wait_ms(SETTLE_MS): return
        both_trig()
        return
    if forward_check(FORWARD_SPEED, FORWARD_MS):
        both_trig()
        return
    if color_interrupt: return
    wait_ms(SETTLE_MS)

def right_trig():
    global color_interrupt
    print("RIGHT triggered — turning right then forward")
    motors.TurnRight(TURN_SPEED)
    if wait_ms(TURN_MS): return
    if obj_in_front():
        motors.Stop(0)
        if wait_ms(SETTLE_MS): return
        both_trig()
        return
    if forward_check(FORWARD_SPEED, FORWARD_MS):
        both_trig()
        return
    if color_interrupt: return
    wait_ms(SETTLE_MS)

def left_trig():
    global color_interrupt
    print("LEFT triggered — turning left then forward")
    motors.TurnLeft(TURN_SPEED)
    if wait_ms(TURN_MS): return
    if obj_in_front():
        motors.Stop(0)
        if wait_ms(SETTLE_MS): return
        both_trig()
        return
    if forward_check(FORWARD_SPEED, FORWARD_MS):
        both_trig()
        return
    if color_interrupt: return
    wait_ms(SETTLE_MS)

# ════════════════════════════════════════════════════════════════════════════
# COLOR RESPONSE
# ════════════════════════════════════════════════════════════════════════════

def kick_ball():
    print("[KICKER] Firing")
    for pos in range(KICK_BACK, KICK_FORWARD + 1, 60):
        set_kicker_us(pos)
        utime.sleep(0.005)
    set_kicker_us(KICK_FORWARD)
    utime.sleep(0.5)
    for pos in range(KICK_FORWARD, KICK_BACK - 1, -40):
        set_kicker_us(pos)
        utime.sleep(0.008)
    set_kicker_us(KICK_BACK)
    print("[KICKER] Reset complete")

def on_color_locked(color):
    global color_interrupt
    print(f"Color locked: {color} — closing claw")

    for pos in range(REST, WINDUP, 20):
        set_servo_us(pos)
        utime.sleep(0.02)
    utime.sleep(1.2)

    print("Driving forward...")
    motors.Forward(COLOR_DRIVE_SPEED)
    utime.sleep_ms(COLOR_DRIVE_MS)
    motors.Stop(0)

    print("Pausing...")
    utime.sleep_ms(COLOR_PAUSE_MS)

    if color == "Red":
        print("RED BALL — kicking!")
        for pos in range(WINDUP, REST, -20):
            set_servo_us(pos)
            utime.sleep(0.02)
        set_servo_us(REST)
        kick_ball()
        color_interrupt = None
        return 

    elif color in ("Blue", "Green"):
        motors.Forward(FORWARD_SPEED)
        utime.sleep(1)
        motors.Stop(0)

    print("Opening claw — resuming search")
    for pos in range(WINDUP, REST, -20):
        set_servo_us(pos)
        utime.sleep(0.02)
    set_servo_us(REST)

    # Clear interrupt so main loop doesn't double-fire
    color_interrupt = None

# ════════════════════════════════════════════════════════════════════════════
# MAIN LOOP
# ════════════════════════════════════════════════════════════════════════════

print("Starting — taking initial reading...")
utime.sleep_ms(1000)

while True: #the while loop here is done by priority of tasks to accomplish
    
    #ideal
    '''
    1. Overcurrent
    2. Ultrasonic Distance
    3. Color Sensing
    3.5? Tentative , where to go once a ball is confirmed 
    4. IR Search function



    '''
    
    if overcurrent_interrupt:
        print("Recovering from overcurrent — waiting 2s")
        utime.sleep_ms(2000)
        overcurrent_interrupt = False
        continue
    
    # If a color was locked mid-maneuver by wait_ms(), handle it now
    if color_interrupt is not None:
        motors.Stop(0)
        on_color_locked(color_interrupt)
        color_interrupt = None
        continue

    # Normal color check at top of loop
    locked = check_color()
    if locked != "-":
        motors.Stop(0)
        on_color_locked(locked)
        continue

    # IR logic
    r, l  = read_ir()
    r_det = r > DETECT_THRESH
    l_det = l > DETECT_THRESH
    print(f"IR — right={r}  left={l}")

    if r_det and l_det:
        diff = abs(r - l)
        if diff < WIDE_MARGIN:
            both_trig()
        elif r > l:
            both_trig_right()
        else:
            both_trig_left()

    elif r_det:
        right_trig()

    elif l_det:
        left_trig()

    else:
        print("Nothing detected — searching")
        motors.Forward(SEARCH_SPEED)
        if check_overcurrent(): continue
        if wait_ms(500): continue
        motors.Stop(0)
        if ir_detected(): continue
        if wait_ms(200): continue
        motors.TurnRight(TURN_SPEED)
        if check_overcurrent(): continue
        if wait_ms(400): continue
        motors.Stop(0)
        if ir_detected(): continue
        if wait_ms(200): continue
        motors.TurnLeft(TURN_SPEED)
        if check_overcurrent(): continue
        if wait_ms(500): continue
        motors.Stop(0)
        if ir_detected(): continue
        wait_ms(SETTLE_MS)
