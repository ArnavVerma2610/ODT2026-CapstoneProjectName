from machine import I2C, Pin, SPI
from ssd1306 import SSD1306_SPI
import framebuf, uos
import time

# ══════════════════════════════════════════════════════════════════════════════
# PCA9685
# ══════════════════════════════════════════════════════════════════════════════
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400_000)
PCA_ADDR  = 0x40
LED0_ON_L = 0x06

def write_reg(reg, val):
    i2c.writeto_mem(PCA_ADDR, reg, bytes([val]))
def read_reg(reg):
    return i2c.readfrom_mem(PCA_ADDR, reg, 1)[0]
def pca_init():
    write_reg(0x00, 0x00)
    time.sleep_ms(10)
    pre = round(25_000_000 / (4096 * 50)) - 1
    old = read_reg(0x00)
    write_reg(0x00, (old & 0x7F) | 0x10)
    write_reg(0xFE, pre)
    write_reg(0x00, old)
    time.sleep_ms(5)
    write_reg(0x00, old | 0xA1)
def angle_to_tick(angle):
    pulse_us = 500 + (2400 - 500) * angle / 180.0
    return int(pulse_us * 4096 / 20000)

# ══════════════════════════════════════════════════════════════════════════════
# CHANNEL CONFIG
#   ch0 = FR hip   ch1 = FL hip   ch2 = BL hip   ch3 = BR hip
#   ch4 = FR knee  ch5 = FL knee  ch6 = BL knee  ch7 = BR knee   <- back knees
# ══════════════════════════════════════════════════════════════════════════════
CH_MAX = [90, 90, 90, 90, 150, 150, 150, 130]
CH_REV = [False, True, False, True, False, True, False, True]

# Final resting pose — matches kame_v2 HOME physically
FINAL_POSE = [0, 0, 0, 0, 0, 0, 0, 0]

# ══════════════════════════════════════════════════════════════════════════════
# OLED
# ══════════════════════════════════════════════════════════════════════════════
_spi_oled = SPI(1, baudrate=20_000_000, sck=Pin(18), mosi=Pin(23))
oled = SSD1306_SPI(128, 64, _spi_oled, dc=Pin(2), res=Pin(4), cs=Pin(5))

_OLED_FRAME_SIZE = 1024
_OLED_BUF = bytearray(_OLED_FRAME_SIZE)
_OLED_FB  = framebuf.FrameBuffer(_OLED_BUF, 128, 64, framebuf.MONO_VLSB)

_OLED_FPATH    = "comp4.bin"
_OLED_TOTAL    = uos.stat(_OLED_FPATH)[6] // _OLED_FRAME_SIZE
_OLED_FILE     = open(_OLED_FPATH, "rb")
_OLED_FRAME_MS = 128
_OLED_IDX      = 0
_OLED_LAST     = 0
_OLED_RUNNING  = False

def oled_start():
    global _OLED_IDX, _OLED_LAST, _OLED_RUNNING
    _OLED_IDX = 0
    _OLED_LAST = time.ticks_ms()
    _OLED_RUNNING = True
    _OLED_FILE.seek(0)
    _OLED_FILE.readinto(_OLED_BUF)
    oled.blit(_OLED_FB, 0, 0)
    oled.show()

def oled_stop():
    global _OLED_RUNNING
    _OLED_RUNNING = False
    oled.fill(0)
    oled.show()

def oled_tick():
    global _OLED_IDX, _OLED_LAST
    if not _OLED_RUNNING:
        return
    now = time.ticks_ms()
    if time.ticks_diff(now, _OLED_LAST) < _OLED_FRAME_MS:
        return
    _OLED_IDX += 1
    if _OLED_IDX >= _OLED_TOTAL:
        _OLED_IDX = 0
    _OLED_FILE.seek(_OLED_IDX * _OLED_FRAME_SIZE)
    _OLED_FILE.readinto(_OLED_BUF)
    oled.blit(_OLED_FB, 0, 0)
    oled.show()
    _OLED_LAST = now

def wait_ms(ms):
    end = time.ticks_add(time.ticks_ms(), ms)
    while True:
        oled_tick()
        r = time.ticks_diff(end, time.ticks_ms())
        if r <= 0:
            return
        time.sleep_ms(2 if r > 2 else r)

# ══════════════════════════════════════════════════════════════════════════════
# SERVO CORE
# ══════════════════════════════════════════════════════════════════════════════
def validate(pose, label=""):
    for i in range(8):
        if pose[i] < 0 or pose[i] > CH_MAX[i]:
            raise ValueError("STOP! {} Ch{}={}° max={}°".format(label, i, pose[i], CH_MAX[i]))

def set_all(pose):
    buf = bytearray(32)
    for i in range(8):
        a = max(0.0, min(float(CH_MAX[i]), float(pose[i])))
        if CH_REV[i]:
            a = CH_MAX[i] - a
        tick = angle_to_tick(a)
        idx = i * 4
        buf[idx + 2] = tick & 0xFF
        buf[idx + 3] = tick >> 8
    i2c.writeto_mem(PCA_ADDR, LED0_ON_L, buf)

def lerp(start, end, ms=500):
    validate(end, "lerp")
    steps = max(1, ms // 20)
    for s in range(1, steps + 1):
        t = s / steps
        frame = [start[i] + (end[i] - start[i]) * t for i in range(8)]
        set_all(frame)
        wait_ms(20)
    return list(end)

# ══════════════════════════════════════════════════════════════════════════════
# WALK  — trot gait, symmetrical back-lean for front-heavy chassis
#
#   Pair A (diagonal) = FR + BL     (ch0,ch4 + ch2,ch6)
#   Pair B (diagonal) = FL + BR     (ch1,ch5 + ch3,ch7)
#
#   BL back-lean is applied ONLY to the two back knees (ch6 and ch7)
#   equally, tilting the body nose-up without creating a side bias.
# ══════════════════════════════════════════════════════════════════════════════
H   = 35   # hip swing (longer stride)
K_F = 22   # front knee lift
K_B = 35   # back  knee lift
BL  = 12   # back-lean on both back knees (ch6 & ch7)

#                   ch0  ch1   ch2  ch3   ch4   ch5     ch6       ch7
W_STAND   = [        0,   0,    0,   0,    0,   0,       BL,       BL ]
W_LIFT_A  = [        0,   0,    0,   0,  K_F,   0,       BL+K_B,   BL ]     # lift FR + BL knees
W_FWD_A   = [        H,   0,    H,   0,  K_F,   0,       BL+K_B,   BL ]     # swing pair A hips fwd
W_DOWN_A  = [        H,   0,    H,   0,    0,   0,       BL,       BL ]     # plant pair A knees
W_LIFT_B  = [        H,   0,    H,   0,    0,   K_F,     BL,       BL+K_B]  # lift FL + BR knees
W_FWD_B   = [        0,   H,    0,   H,    0,   K_F,     BL,       BL+K_B]  # pair A pushes back, B fwd
W_DOWN_B  = [        0,   H,    0,   H,    0,   0,       BL,       BL ]     # plant pair B knees

WALK_CYCLE = [W_LIFT_A, W_FWD_A, W_DOWN_A, W_LIFT_B, W_FWD_B, W_DOWN_B, W_STAND]

ALL_POSES = [FINAL_POSE, W_STAND] + WALK_CYCLE

def validate_all():
    for i, p in enumerate(ALL_POSES):
        validate(p, "pose#{}".format(i))
    print("[OK] All {} poses validated".format(len(ALL_POSES)))

def walk(cycles=15, step_ms=220):
    print("[walk] {} cycles, {} ms per phase".format(cycles, step_ms))
    pos = list(FINAL_POSE)
    set_all(pos)
    time.sleep_ms(500)

    pos = lerp(pos, W_STAND, 700)
    wait_ms(200)

    for c in range(cycles):
        print("  cycle {}/{}".format(c + 1, cycles))
        for p in WALK_CYCLE:
            pos = lerp(pos, p, step_ms)

    print("[walk] return to final pose")
    pos = lerp(pos, FINAL_POSE, 800)
    return pos

# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════
pca_init()
print("PCA9685 ready")

print("OLED ready — comp4.bin:", _OLED_TOTAL, "frames @", _OLED_FRAME_MS, "ms")

validate_all()

set_all(FINAL_POSE)
time.sleep_ms(800)

oled_start()
walk(cycles=15, step_ms=220)
oled_stop()

set_all(FINAL_POSE)
print("Done.")
