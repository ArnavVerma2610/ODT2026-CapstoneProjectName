from machine import I2C, Pin, SPI
from ssd1306 import SSD1306_SPI
import framebuf, uos
import time

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

CH_MAX = [90, 90, 90, 90, 150, 150, 150, 130]
CH_REV = [False, True, False, True, False, True, False, True]
HOME   = [0, 90, 0, 90, 0, 150, 0, 130]

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
        _OLED_IDX = _OLED_TOTAL - 1
        return
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

H = HOME

LOW       = [  0,  90,    0,  90,   80,   70,   80,   50]
BOB_DN    = [  0,  90,    0,  90,   15,  135,   15,  115]
EX_DN     = [  0,  90,    0,  90,   45,  105,   45,   85]
RELAX_DN  = [  0,  90,    0,  90,   10,  140,   10,  120]
BOUNCE_DN = [  0,  90,    0,  90,   35,  115,   35,   95]

W_SIT     = [  0,  90,    0,  90,    0,  130,   40,   90]
W_UP      = [  0,  50,    0,  90,    0,   90,   40,   90]
W_DN      = [  0,  85,    0,  90,    0,  140,   40,   90]
WR_SIT    = [  0,  90,    0,  90,    0,  130,   40,   90]
WR_UP     = [ 40,  90,    0,  90,   60,  130,   40,   90]
WR_DN     = [  5,  90,    0,  90,   10,  130,   40,   90]

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
RIPPLE_F  = [  0,  90,    0,  90,   40,  110,    0,  130]
RIPPLE_B  = [  0,  90,    0,  90,    0,  150,   40,   90]
SIDE_A    = [  0,  90,    0,  90,   30,  150,    0,   95]
SIDE_B    = [  0,  90,    0,  90,    0,  120,   30,  130]

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


def final_animation():

    print("=" * 50)
    print("  FINAL ANIMATION — 2:35")
    print("=" * 50)

    pos = list(LOW)
    set_all(pos)

    print("[0:00] delay")
    time.sleep_ms(1000)

    print("[0:00] oled start")
    oled_start()

    print("[0:01] rising...")
    pos = lerp(pos, MED_SQ, 2000)
    pos = lerp(pos, SQ,     1500)
    pos = lerp(pos, BOB_DN, 1500)
    pos = lerp(pos, H,      2000)

    print("[0:08] settling")
    for _ in range(3):
        pos = lerp(pos, RELAX_DN, 600)
        pos = lerp(pos, H,        600)
    wait_ms(400)

    print("[0:12] wave")
    pos = lerp(pos, W_SIT, 400)
    pos = lerp(pos, W_UP,  300)
    pos = lerp(pos, W_DN,  300)
    pos = lerp(pos, W_UP,  300)
    pos = lerp(pos, W_DN,  300)
    pos = lerp(pos, H,     400)

    print("[0:14] bob")
    for _ in range(4):
        pos = lerp(pos, BOB_DN, 500)
        pos = lerp(pos, H,      500)

    print("[0:19] dance start")
    wait_ms(500)
    B = 500

    print("  bounces")
    pos = lerp(pos, SQ,   B)
    pos = lerp(pos, TALL, B)
    pos = lerp(pos, SQ,   B)
    pos = lerp(pos, TALL, B)
    pos = lerp(pos, BOUNCE_DN, B)
    pos = lerp(pos, H,         B)
    pos = lerp(pos, BOUNCE_DN, B)
    pos = lerp(pos, H,         B)

    print("  twists")
    pos = lerp(pos, TW_R, B)
    pos = lerp(pos, H,    B)
    pos = lerp(pos, TW_R, B)
    pos = lerp(pos, H,    B)
    pos = lerp(pos, TI_R, B)
    pos = lerp(pos, TI_L, B)
    pos = lerp(pos, TI_R, B)
    pos = lerp(pos, H,    B)

    print("  ripple")
    pos = lerp(pos, RIPPLE_F, B)
    pos = lerp(pos, RIPPLE_B, B)
    pos = lerp(pos, RIPPLE_F, B)
    pos = lerp(pos, H,        B)
    pos = lerp(pos, SIDE_A,   B)
    pos = lerp(pos, SIDE_B,   B)
    pos = lerp(pos, SIDE_A,   B)
    pos = lerp(pos, H,        B)

    print("  rocks")
    pos = lerp(pos, LN_F,  B)
    pos = lerp(pos, LN_B,  B)
    pos = lerp(pos, LN_F,  B)
    pos = lerp(pos, H,     B)
    pos = lerp(pos, CRS_A, B)
    pos = lerp(pos, CRS_B, B)
    pos = lerp(pos, CRS_A, B)
    pos = lerp(pos, H,     B)

    print("  pops")
    pos = lerp(pos, P_FR, B)
    pos = lerp(pos, H,    B)
    pos = lerp(pos, P_FL, B)
    pos = lerp(pos, H,    B)
    pos = lerp(pos, P_BL, B)
    pos = lerp(pos, H,    B)
    pos = lerp(pos, P_BR, B)
    pos = lerp(pos, H,    B)

    print("  shimmy")
    for _ in range(4):
        pos = lerp(pos, SH_A, 250)
        pos = lerp(pos, SH_B, 250)
    pos = lerp(pos, H, B)
    wait_ms(500)

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

    print("  wave bye")
    pos = lerp(pos, W_SIT, 400)
    pos = lerp(pos, W_UP,  350)
    pos = lerp(pos, W_DN,  350)
    pos = lerp(pos, W_UP,  350)
    pos = lerp(pos, W_DN,  350)
    pos = lerp(pos, H,     500)
    wait_ms(700)

    print("[0:52] excited bobs")
    for _ in range(5):
        pos = lerp(pos, EX_DN, 350)
        pos = lerp(pos, H,     350)
    wait_ms(500)

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


pca_init()
print("P
