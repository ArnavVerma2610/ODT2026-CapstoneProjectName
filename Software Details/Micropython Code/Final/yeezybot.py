from machine import I2C, Pin, UART, SPI
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
# ══════════════════════════════════════════════════════════════════════════════
CH_MAX = [90, 90, 90, 90, 150, 150, 150, 130]
CH_REV = [False, True, False, True, False, True, False, True]
HOME   = [0, 90, 0, 90, 0, 150, 0, 130]

# ══════════════════════════════════════════════════════════════════════════════
# DFPLAYER
# ══════════════════════════════════════════════════════════════════════════════
_uart = UART(2, baudrate=9600, tx=17, rx=16)
def df_send(cmd, p1=0, p2=0):
    total = 0xFF + 0x06 + cmd + 0x00 + p1 + p2
    cs = -total & 0xFFFF
    _uart.write(bytes([0x7E, 0xFF, 0x06, cmd, 0x00, p1, p2,
                       (cs >> 8) & 0xFF, cs & 0xFF, 0xEF]))
    time.sleep_ms(30)
def df_init(vol=25):
    df_send(0x0C)
    time.sleep_ms(1500)
    df_send(0x06, 0, vol)
    time.sleep_ms(100)
def df_play(track=1):
    df_send(0x03, 0, track)

# ══════════════════════════════════════════════════════════════════════════════
# OLED  (SSD1309 128x64 SPI, same wiring as before)
#   CS=5, DC=2, RST=4, MOSI=23, SCK=18
# Streams comp4.bin (1210 frames, MONO_VLSB) in the background, advancing
# one frame every _OLED_FRAME_MS. All servo waits go through wait_ms() so
# the animation stays in lockstep with the music + servo timeline.
# ══════════════════════════════════════════════════════════════════════════════
_spi_oled = SPI(1, baudrate=20_000_000, sck=Pin(18), mosi=Pin(23))
oled = SSD1306_SPI(128, 64, _spi_oled, dc=Pin(2), res=Pin(4), cs=Pin(5))

_OLED_FRAME_SIZE = 1024                              # 128 * 64 / 8
_OLED_BUF = bytearray(_OLED_FRAME_SIZE)
_OLED_FB  = framebuf.FrameBuffer(_OLED_BUF, 128, 64, framebuf.MONO_VLSB)

_OLED_FPATH    = "comp4.bin"
_OLED_TOTAL    = uos.stat(_OLED_FPATH)[6] // _OLED_FRAME_SIZE   # 1210
_OLED_FILE     = open(_OLED_FPATH, "rb")
_OLED_FRAME_MS = 128      # 155 000 ms / 1210 frames ≈ 128.1 ms per frame
_OLED_IDX      = 0
_OLED_LAST     = 0
_OLED_RUNNING  = False

def oled_start():
    """Reset OLED playback to frame 0 and let wait_ms() drive it."""
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
    """Push the next OLED frame if _OLED_FRAME_MS has elapsed. Cheap; safe
    to call from inside tight wait loops."""
    global _OLED_IDX, _OLED_LAST
    if not _OLED_RUNNING:
        return
    now = time.ticks_ms()
    if time.ticks_diff(now, _OLED_LAST) < _OLED_FRAME_MS:
        return
    _OLED_IDX += 1
    if _OLED_IDX >= _OLED_TOTAL:
        # Hold last frame and stop ticking so we don't loop past the servo.
        _OLED_IDX = _OLED_TOTAL - 1
        return
    _OLED_FILE.seek(_OLED_IDX * _OLED_FRAME_SIZE)
    _OLED_FILE.readinto(_OLED_BUF)
    oled.blit(_OLED_FB, 0, 0)
    oled.show()
    _OLED_LAST = now

def wait_ms(ms):
    """Drop-in replacement for time.sleep_ms that also pushes OLED frames.
    Accuracy is servo-grade (sub-millisecond tail), OLED frames are
    pushed on the 128 ms cadence regardless of how short each wait is."""
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
        wait_ms(20)          # <-- OLED advances during every servo step
    return list(end)

# ══════════════════════════════════════════════════════════════════════════════
# ALL POSES
# ══════════════════════════════════════════════════════════════════════════════
#                          ch0  ch1   ch2  ch3   ch4   ch5    ch6   ch7
H = HOME

# ── Starting low (crouched) ───────────────────────────────────────────────
LOW       = [  0,  90,    0,  90,   80,   70,   80,   50]

# ── Bobs ──────────────────────────────────────────────────────────────────
# Gentle idle bob
BOB_DN    = [  0,  90,    0,  90,   15,  135,   15,  115]

# Excited bob — bigger movement
EX_DN     = [  0,  90,    0,  90,   45,  105,   45,   85]

# Slow relaxed bob — very small
RELAX_DN  = [  0,  90,    0,  90,   10,  140,   10,  120]

# All-legs bounce — medium
BOUNCE_DN = [  0,  90,    0,  90,   35,  115,   35,   95]

# ── Wave poses ────────────────────────────────────────────────────────────
# Wave FL leg — sit slightly back first
W_SIT     = [  0,  90,    0,  90,    0,  130,   40,   90]
W_UP      = [  0,  50,    0,  90,    0,   90,   40,   90]
W_DN      = [  0,  85,    0,  90,    0,  140,   40,   90]

# Wave FR leg
WR_SIT    = [  0,  90,    0,  90,    0,  130,   40,   90]
WR_UP     = [ 40,  90,    0,  90,   60,  130,   40,   90]
WR_DN     = [  5,  90,    0,  90,   10,  130,   40,   90]

# ── Dance poses (mellow) ─────────────────────────────────────────────────
SQ        = [  0,  90,    0,  90,   40,  110,   40,  100]
MED_SQ    = [  0,  90,    0,  90,   60,   90,   60,   80]
TALL      = [  0,  90,    0,  90,    0,  150,    0,  130]

TW_R      = [ 30,  60,   30,  60,    0,  150,    0,  130]

LN_F      = [  0,  90,    0,  90,   40,  110,    0,  130]
LN_B      = [  0,  90,    0,  90,    0,  150,   40,   90]

TI_R      = [  0,  90,    0,  90,    0,  110,   35,  130]
TI_L      = [  0,  90,    0,  90,   35,  150,    0,   95]

CRS_A     = [ 25,  65,   25,  65,   20,  130,    0,  130]
CRS_B     = [  0,  90,    0,  90,    0,  150,   20,  110]

P_FR      = [ 20,  90,    0,  90,   50,  150,    0,  130]
P_FL      = [  0,  70,    0,  90,    0,  100,    0,  130]
P_BL      = [  0,  90,   20,  90,    0,  150,   50,  130]
P_BR      = [  0,  90,    0,  70,    0,  150,    0,   80]

SH_A      = [ 20,  70,   20,  70,    0,  150,    0,  130]
SH_B      = [  0,  90,    0,  90,    0,  150,    0,  130]

# Wave-like body ripple — front then back
RIPPLE_F  = [  0,  90,    0,  90,   40,  110,    0,  130]
RIPPLE_B  = [  0,  90,    0,  90,    0,  150,   40,   90]

# Side wave — alternating diagonal pairs
SIDE_A    = [  0,  90,    0,  90,   30,  150,    0,   95]
SIDE_B    = [  0,  90,    0,  90,    0,  120,   30,  130]


# ══════════════════════════════════════════════════════════════════════════════
# VALIDATE ALL POSES UPFRONT
# ══════════════════════════════════════════════════════════════════════════════
ALL_POSES = [
    H, LOW, BOB_DN, EX_DN, RELAX_DN, BOUNCE_DN,
    W_SIT, W_UP, W_DN, WR_SIT, WR_UP, WR_DN,
    SQ, MED_SQ, TALL, TW_R, LN_F, LN_B,
    TI_R, TI_L, CRS_A, CRS_B,
    P_FR, P_FL, P_BL, P_BR, SH_A, SH_B,
    RIPPLE_F, RIPPLE_B, SIDE_A, SIDE_B,
]

def validate_all():
    for i, p in enumerate(ALL_POSES):
        validate(p, "pose#{}".format(i))
    print("[OK] All {} poses validated".format(len(ALL_POSES)))


# ══════════════════════════════════════════════════════════════════════════════
# FINAL ANIMATION — 2 minutes 35 seconds (155s total)
# ══════════════════════════════════════════════════════════════════════════════
def final_animation():

    print("=" * 50)
    print("  FINAL ANIMATION — 2:35")
    print("=" * 50)

    # Start crouched
    pos = list(LOW)
    set_all(pos)

    # ── 1 second delay ──
    print("[0:00] delay")
    time.sleep_ms(1000)

    # ── START MUSIC + OLED ──
    print("[0:00] music + oled start")
    df_play(1)
    oled_start()

    # ══════════════════════════════════════════════════════════════════
    # 0:01–0:08  RISE FROM LOW TO HOME (7 seconds)
    # ══════════════════════════════════════════════════════════════════
    print("[0:01] rising...")
    pos = lerp(pos, MED_SQ, 2000)
    pos = lerp(pos, SQ,     1500)
    pos = lerp(pos, BOB_DN, 1500)
    pos = lerp(pos, H,      2000)

    # ══════════════════════════════════════════════════════════════════
    # 0:08–0:12  HOLD + TINY BOBS (4 seconds)
    # ══════════════════════════════════════════════════════════════════
    print("[0:08] settling")
    for _ in range(3):
        pos = lerp(pos, RELAX_DN, 600)
        pos = lerp(pos, H,        600)
    wait_ms(400)

    # ══════════════════════════════════════════════════════════════════
    # 0:12–0:14  WAVE WITH FL LEG (2 seconds)
    # ══════════════════════════════════════════════════════════════════
    print("[0:12] wave")
    pos = lerp(pos, W_SIT, 400)
    pos = lerp(pos, W_UP,  300)
    pos = lerp(pos, W_DN,  300)
    pos = lerp(pos, W_UP,  300)
    pos = lerp(pos, W_DN,  300)
    pos = lerp(pos, H,     400)

    # ══════════════════════════════════════════════════════════════════
    # 0:14–0:18  BOB UP AND DOWN (4 seconds)
    # ══════════════════════════════════════════════════════════════════
    print("[0:14] bob")
    for _ in range(4):
        pos = lerp(pos, BOB_DN, 500)
        pos = lerp(pos, H,      500)

    # ══════════════════════════════════════════════════════════════════
    # 0:19–0:51  DANCE ROUTINE (32 seconds)
    # ══════════════════════════════════════════════════════════════════
    print("[0:19] dance start")
    wait_ms(500)
    B = 500  # beat timing

    # -- 0:19–0:23 Bounces (4s) --
    print("  bounces")
    pos = lerp(pos, SQ,   B)
    pos = lerp(pos, TALL, B)
    pos = lerp(pos, SQ,   B)
    pos = lerp(pos, TALL, B)
    pos = lerp(pos, BOUNCE_DN, B)
    pos = lerp(pos, H,         B)
    pos = lerp(pos, BOUNCE_DN, B)
    pos = lerp(pos, H,         B)

    # -- 0:23–0:27 Twists + tilts (4s) --
    print("  twists")
    pos = lerp(pos, TW_R, B)
    pos = lerp(pos, H,    B)
    pos = lerp(pos, TW_R, B)
    pos = lerp(pos, H,    B)
    pos = lerp(pos, TI_R, B)
    pos = lerp(pos, TI_L, B)
    pos = lerp(pos, TI_R, B)
    pos = lerp(pos, H,    B)

    # -- 0:27–0:31 Body ripple / wave motion (4s) --
    print("  ripple")
    pos = lerp(pos, RIPPLE_F, B)
    pos = lerp(pos, RIPPLE_B, B)
    pos = lerp(pos, RIPPLE_F, B)
    pos = lerp(pos, H,        B)
    pos = lerp(pos, SIDE_A,   B)
    pos = lerp(pos, SIDE_B,   B)
    pos = lerp(pos, SIDE_A,   B)
    pos = lerp(pos, H,        B)

    # -- 0:31–0:35 Rocks + cross (4s) --
    print("  rocks")
    pos = lerp(pos, LN_F,  B)
    pos = lerp(pos, LN_B,  B)
    pos = lerp(pos, LN_F,  B)
    pos = lerp(pos, H,     B)
    pos = lerp(pos, CRS_A, B)
    pos = lerp(pos, CRS_B, B)
    pos = lerp(pos, CRS_A, B)
    pos = lerp(pos, H,     B)

    # -- 0:35–0:39 Leg pops (4s) --
    print("  pops")
    pos = lerp(pos, P_FR, B)
    pos = lerp(pos, H,    B)
    pos = lerp(pos, P_FL, B)
    pos = lerp(pos, H,    B)
    pos = lerp(pos, P_BL, B)
    pos = lerp(pos, H,    B)
    pos = lerp(pos, P_BR, B)
    pos = lerp(pos, H,    B)

    # -- 0:39–0:43 Shimmy (4s) --
    print("  shimmy")
    for _ in range(4):
        pos = lerp(pos, SH_A, 250)
        pos = lerp(pos, SH_B, 250)
    pos = lerp(pos, H, B)
    wait_ms(500)

    # -- 0:43–0:47 All-legs bounce wave (4s) --
    print("  bounce wave")
    pos = lerp(pos, BOUNCE_DN, 400)
    pos = lerp(pos, TALL,      400)
    pos = lerp(pos, BOUNCE_DN, 400)
    pos = lerp(pos, TALL,      400)
    pos = lerp(pos, SQ,        400)
    pos = lerp(pos, TALL,      400)
    pos = lerp(pos, SQ,        400)
    pos = lerp(pos, H,         400)
    wait_ms(200)

    # -- 0:47–0:51 Wave goodbye + wind down (4s) --
    print("  wave bye")
    pos = lerp(pos, W_SIT, 400)
    pos = lerp(pos, W_UP,  350)
    pos = lerp(pos, W_DN,  350)
    pos = lerp(pos, W_UP,  350)
    pos = lerp(pos, W_DN,  350)
    pos = lerp(pos, H,     500)
    wait_ms(700)

    # ══════════════════════════════════════════════════════════════════
    # 0:52–0:56  EXCITED BOBS (4 seconds)
    # ══════════════════════════════════════════════════════════════════
    print("[0:52] excited bobs")
    for _ in range(5):
        pos = lerp(pos, EX_DN, 350)
        pos = lerp(pos, H,     350)
    wait_ms(500)

    # ══════════════════════════════════════════════════════════════════
    # 0:58–2:35  SLOW RELAXED BOBS (97 seconds)
    # ══════════════════════════════════════════════════════════════════
    print("[0:58] slow bobs — 97s")
    for c in range(27):
        if c % 9 == 0:
            elapsed = 58 + c * 3.6
            m = int(elapsed) // 60
            s = int(elapsed) % 60
            print("  {}:{:02d}  cycle {}/27".format(m, s, c + 1))
        pos = lerp(pos, RELAX_DN, 1600)
        wait_ms(200)
        pos = lerp(pos, H,        1600)
        wait_ms(200)

    # ══════════════════════════════════════════════════════════════════
    # END — return to HOME
    # ══════════════════════════════════════════════════════════════════
    print("[2:35] final home")
    pos = lerp(pos, HOME, 1000)
    set_all(HOME)
    oled_stop()
    print()
    print("=" * 50)
    print("  ANIMATION COMPLETE")
    print("  Final position: HOME = {}".format(HOME))
    print("=" * 50)
    return list(HOME)


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════
pca_init()
print("PCA9685 ready")

df_init(vol=25)
print("DFPlayer ready")

print("OLED ready — comp4.bin:", _OLED_TOTAL, "frames @", _OLED_FRAME_MS, "ms")
print()

validate_all()
print()

# Start at LOW (crouched)
set_all(LOW)
time.sleep_ms(500)

final_animation()
