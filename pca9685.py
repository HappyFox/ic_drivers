import math
import ustruct
import utime

from .i2cutils import I2cRegDevice


MODE1 = 0x00
PRESCALE = 0xFE

MAX_PRESCALE = 0xFF
MIN_PRESCALE = 0x03

LED_BASE_ADDR = 0x06
LED_SIZE = 0x04

_ALL_LED_START = const(0xFA)

ALL_BIT = 12

_25_MHZ = const(25000000)


class PreScaleChangeError(Exception):
    pass


class InvalidPrescale(Exception):
    pass


class PinWrongTypeError(Exception):
    pass


class Pwm:

    MAX_DUTY = 4095

    def __init__(self, pca, pin, freq=None, duty=None):
        self.pca = pca
        self.pin = pin
        self.pin_addr = LED_BASE_ADDR + LED_SIZE * self.pin
        self.legacy = False
        self.reset()

        if freq:
            pca.freq(freq)
        if duty:
            self.duty(duty)

    def reset(self):
        self.pca.dev.writeto_mem(self.pin_addr, '\x00' * LED_SIZE)

    def freq(self, freq_hz=None):
        return self.pca.freq(freq_hz)

    def duty(self, duty=None):
        if duty is None:
            on, off = self.pwm()

            if on == 4096:
                if self.legacy:
                    return 1023
                return 4095
            else:
                if self.legacy:
                    off = off / 4
                return int(off)

        if self.legacy:
            if duty == 1023:
                duty = 4095
            elif 0 <= duty <= 1023:
                raise ValueError()
            else:
                duty = duty * 4

        if not 0 <= duty <= 4095:
            raise ValueError()

        if duty == 4095:
            self.pwm(4096, 0)
        else:
            self.pwm(0, duty)

    def pwm(self, on=None, off=None):
        if on is None and off is None:
            payload = self.pca.dev.readfrom_mem(self.pin_addr, LED_SIZE)
            return ustruct.unpack("<HH", payload)

        payload = ustruct.pack("<HH", on, off)
        self.pca.dev.writeto_mem(self.pin_addr, payload)


class Pin:

    def __init__(self, pca, pin):
        self.pca = pca
        self.pin = pin
        self.pin_addr = LED_BASE_ADDR + LED_SIZE * self.pin
        self.reset()

    def reset(self):
        self.pca.dev.writeto_mem(self.pin_addr, '\x00' * LED_SIZE)

    @property
    def value(self):
        payload = self.pca.dev.readfrom_mem(self.pin_addr, 2)
        led_on = ustruct.unpack("<H", payload)[0]
        return bool(led_on & 1 << ALL_BIT)

    @value.setter
    def value(self, value):
        payload = ustruct.pack("<H", bool(value) << ALL_BIT)
        self.pca.dev.writeto_mem(self.pin_addr, payload)

    @property
    def high(self):
        return self.value

    @high.setter
    def high(self, value):
        self.value = value

    @property
    def low(self):
        return not self.value

    @low.setter
    def low(self, value):
        self.value = not value


class Pca9685:

    def __init__(self, i2c, address, freq=50.0, clock=_25_MHZ):
        self.dev = I2cRegDevice(i2c, address)


        self.pins = []

        for _ in range(16):
            self.pins.append(None)

        self.init(freq, clock)

    def init(self, freq=50.0, clock=_25_MHZ):
        self.dev[MODE1] = 0x00
        self._prescale = 0
        self.clock = clock
        self.freq(freq)
        self.dev.writeto_mem(_ALL_LED_START, b'\x00' * 4)

    def deinit(self):
        self.dev.writeto_mem(_ALL_LED_START, b'\x00' * 4)
        self.dev[MODE1] = 0x00

    def freq(self, freq_hz=None):

        if freq_hz is None:
            if not self._freq:
                return 0

            # if no-one has changed what the prescale was set to, return the
            # freq we set to get that prescale.
            if self.dev[PRESCALE] == self._prescale:
                return self._freq

            raise PreScaleChangeError("The prescale was changed")

        prescale = self.clock
        prescale /= 4096.0       # 12-bit
        prescale /= float(freq_hz)
        prescale -= 0.5

        prescale = int(math.floor(prescale))

        if MIN_PRESCALE > prescale or prescale > MAX_PRESCALE:
            raise InvalidPrescale(
                "Attempting prescale of {}".format(prescale))

        self._prescale = prescale
        self._freq = freq_hz

        orig_mode = self.dev[MODE1]

        sleep_mode = (orig_mode & 0x7F) | 0x10

        self.dev[MODE1] = sleep_mode
        self.dev[PRESCALE] = self._prescale
        self.dev[MODE1] = orig_mode

        utime.sleep_us(5)

        self.dev[MODE1] = orig_mode | 0xa1

    def _init_pin(self, idx, _type):
        if self.pins[idx]:
            pin = self.pins[idx]
            if isinstance(pin, _type):
                return pin
            else:
                raise PinWrongTypeError()

        self.pins[idx] = _type(self, idx)
        return self.pins[idx]

    def init_pwm(self, pin):
        return self._init_pin(pin, Pwm)

    def init_pin(self, pin):
        return self._init_pin(pin, Pin)

    def deinit_pin(self, pin):
        idx = self.pins.index(pin)
        if pin:
            self.pins[idx] = None
            pin.reset()
            del(pin)
