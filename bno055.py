import time
import ustruct

from drivers.i2cutils import I2cRegDevice

# pylint: disable=undefined-variable

AXIS_MAP_CONFIG = const(0x41)
AXIS_MAP_SIGN = const(0x42)


OP_MODE = const(0x3D)
OP_MODE_CONFIG = const(0b0000)
OP_MODE_ACC_ONLY = const(0b0001)
OP_MODE_MAG_ONLY = const(0b0010)
OP_MODE_GYRO_ONLY = const(0b0011)
OP_MODE_ACC_MAG = const(0b0100)
OP_MODE_ACC_GYRO = const(0b0101)
OP_MODE_MAG_GYRO = const(0b0110)
OP_MODE_AMG = const(0b0111)
OP_MODE_IMU = const(0b1000)
OP_MODE_COMPASS = const(0b1001)
OP_MODE_M4G = const(0b1010)
OP_MODE_NDOF_FMC_OFF = const(0b1011)
OP_MODE_NDOF = const(0b1100)

SYS_STATUS = const(0x39)

SYS_STATUS_IDLE = const(0)
SYS_STATUS_ERROR = const(1)
SYS_STATUS_FUSION_RUNNING = const(5)

SYS_TRIGGER = const(0x3F)
SYS_TRIGGER_RST_SYS = const(1 << 5)

EUL_ORIENT_START = const(0x1A)
EUL_ORIENT_LEN = const(6)


class SystemError(Exception):
    pass


class Bno055:

    AXIS_MAP_P0 = const((0x21 << 3) + 0x04)
    AXIS_MAP_P1 = const((0x24 << 3) + 0x00)
    AXIS_MAP_P2 = const((0x24 << 3) + 0x06)
    AXIS_MAP_P3 = const((0x21 << 3) + 0x02)
    AXIS_MAP_P4 = const((0x24 << 3) + 0x03)
    AXIS_MAP_P5 = const((0x21 << 3) + 0x01)
    AXIS_MAP_P6 = const((0x21 << 3) + 0x07)
    AXIS_MAP_P7 = const((0x24 << 3) + 0x05)

    def __init__(self, i2c, axises=AXIS_MAP_P1, address=0x28):
        self.dev = I2cRegDevice(i2c, address)
        self.eul_buf = bytearray(EUL_ORIENT_LEN)
        self.init(axises)

    def init(self, axises=AXIS_MAP_P1):
        self.reset()

        try:
            status = self.dev[SYS_STATUS]
        except OSError:
            status = -1

        while status != SYS_STATUS_IDLE:
            if status == SYS_STATUS_ERROR:
                raise SystemError()

            time.sleep_ms(5)
            try:
                status = self.dev[SYS_STATUS]
            except OSError:
                status = -1

        self._set_axises(axises)

        self.mode(OP_MODE_NDOF)

        status = self.dev[SYS_STATUS]

        while status != SYS_STATUS_FUSION_RUNNING:
            print("re-reading! - wait for fusion")
            if status == 1:
                raise SystemError()

            time.sleep_ms(5)
            status = self.dev[SYS_STATUS]

    def deinit(self):
        self.reset()

    def reset(self):
        self.dev[SYS_TRIGGER] = SYS_TRIGGER_RST_SYS

    def _set_axises(self, axises):
        print(axises >> 3)
        print(axises & 0b111)
        self.dev[AXIS_MAP_CONFIG] = axises >> 3
        self.dev[AXIS_MAP_SIGN] = axises & 0b111

    def mode(self, mode=None):
        if mode is None:
            return self.dev[OP_MODE]

        self.dev[OP_MODE] = mode

        # Just sleep the full 19 ms for switching to config mode.
        time.sleep_ms(19)

    def get_pos(self):
        self.dev.readfrom_mem_into(EUL_ORIENT_START, self.eul_buf)
        vals = ustruct.unpack("<hhh", self.eul_buf)

        return vals[0]/16.0, vals[1]/16.0, vals[2]/16.0
