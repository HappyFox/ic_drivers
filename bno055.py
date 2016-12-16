import time
import ustruct

from drivers.i2cutils import I2cRegDevice

# pylint: disable=undefined-variable

_AXIS_MAP_CONFIG = const(0x41)
_AXIS_MAP_SIGN = const(0x42)

_CALIB_STAT = const(0x35)
_CALIB_START = const(0x55)
_CALIB_LEN = const(21)

_GRAV_START = const(0x2E)
_LIA_START = const(0x28)

_OP_MODE = const(0x3D)
_OP_MODE_CONFIG = const(0b0000)
_OP_MODE_NDOF = const(0b1100)

_SYS_STATUS = const(0x39)

_SYS_STATUS_IDLE = const(0)
_SYS_STATUS_ERROR = const(1)
_SYS_STATUS_FUSION_RUNNING = const(5)

_SYS_TRIGGER = const(0x3F)
_SYS_TRIGGER_RST_SYS = const(1 << 5)

_EUL_ORIENT_START = const(0x1A)
_EUL_ORIENT_LEN = const(6)


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

    def __init__(self, i2c, axises=AXIS_MAP_P1, calib_data=None, address=0x28):
        self.dev = I2cRegDevice(i2c, address)
        self.eul_buf = bytearray(_EUL_ORIENT_LEN)
        self.init(axises, calib_data)

    def init(self, axises=AXIS_MAP_P1, calib_data=None):
        self.reset()

        try:
            status = self.dev[_SYS_STATUS]
        except OSError:
            status = -1

        while status != _SYS_STATUS_IDLE:
            if status == _SYS_STATUS_ERROR:
                raise SystemError()

            time.sleep_ms(5)
            try:
                status = self.dev[_SYS_STATUS]
            except OSError:
                status = -1

        if calib_data:
            if len(calib_data) != _CALIB_LEN:
                raise SystemError("Invalid calibrate data.")
            self.dev.writeto_mem(_CALIB_START, calib_data)

        self._set_axises(axises)

        self.mode(_OP_MODE_NDOF)

        status = self.dev[_SYS_STATUS]

        while status != _SYS_STATUS_FUSION_RUNNING:
            if status == 1:
                raise SystemError()

            time.sleep_ms(5)
            status = self.dev[_SYS_STATUS]

    def deinit(self):
        self.reset()

    def reset(self):
        self.dev[_SYS_TRIGGER] = _SYS_TRIGGER_RST_SYS

    def _set_axises(self, axises):
        self.dev[_AXIS_MAP_CONFIG] = axises >> 3
        self.dev[_AXIS_MAP_SIGN] = axises & 0b111

    def mode(self, mode=None):
        if mode is None:
            return self.dev[_OP_MODE]

        self.dev[_OP_MODE] = mode

        # Just sleep the full 19 ms for switching to config mode.
        time.sleep_ms(19)

    def _get_3_int16(self, addr, scale):
        self.dev.readfrom_mem_into(addr, self.eul_buf)
        vals = ustruct.unpack("<hhh", self.eul_buf)

        return vals[0]/scale, vals[1]/scale, vals[2]/scale

    def get_orient(self):
        return self._get_3_int16(_EUL_ORIENT_START, 16.0)

    def get_lin_acc(self):
        return self._get_3_int16(_LIA_START, 100)

    def get_grav(self):
        return self._get_3_int16(_GRAV_START, 100)

    def calib_state(self):
        state = self.dev[_CALIB_STAT]
        sys_calib = bool(state & (3 << 6))
        gyr_calib = bool(state & (3 << 4))
        acc_calib = bool(state & (3 << 2))
        mag_calib = bool(state & 3)

        return sys_calib, gyr_calib, acc_calib, mag_calib

    def get_calib_data(self):
        self.mode(_OP_MODE_CONFIG)

        calib_data = self.dev.readfrom_mem(_CALIB_START, _CALIB_LEN)

        self.mode(_OP_MODE_NDOF)

        return calib_data
