import math

from ..pca9685 import Pca9685



class InvalidDirectionError(Exception):
    pass


class InvalidMotorError(Exception):
    pass


class InvalidSpeedError(Exception):
    pass


motor_pins = [(8, 9, 10), (13, 12, 11), (2, 3, 4), (7, 6, 5)]


def get_motor_shield(i2c, address=0x60):
    pca = Pca9685(i2c, address)
    return MotorShield(pca)


class Motor:

    def __init__(self, pwm, in_1, in_2):
        self.pwm = pwm

        self.in_1 = in_1
        self.in_2 = in_2

        self.invert = False
        self.speed = 0

    @property
    def speed(self):
        if self.in_1.value == self.in_2.value:
            return 0 # No speed

        return self.pwm.duty() / self.pwm.MAX_DUTY

    @speed.setter
    def speed(self, value):

        if not -1.0 <= value <= 1.0:
            raise InvalidSpeedError()

        if self.invert:
            value = value * -1

        if value > 0:
            self.in_1.high = True
            self.in_2.low = True
        elif value < 0:
            self.in_1.low = True
            self.in_2.high = True
        else:
            self.in_1.low = True
            self.in_2.low = True
            self.pwm.duty(0)
            return

        duty = self.pwm.MAX_DUTY * abs(value)
        self.pwm.duty(int(duty))

    @property
    def brake(self):
        if self.in_1.high and self.in_2.high:
            return self.pwm.duty() / self.pwm.MAX_DUTY

        return 0

    @brake.setter
    def brake(self, value):

        if not 0 <= value <= 1.0:
            raise InvalidSpeedError()

        self.in_1.high = True
        self.in_2.high = True
        duty = self.pwm.MAX_DUTY * value

        self.pwm.duty(int(duty))


class MotorShield:

    def __init__(self, pca):
        self.pca = pca
        self.pca.freq(1600)

        self.motors = [None, None, None, None]

    def get_motor(self, idx):
        if not 0 <= idx <= 3:
            raise InvalidMotorError()

        if not self.motors[idx]:
            pwm_idx, in_1_idx, in_2_idx = motor_pins[idx]
            pwm = self.pca.init_pwm(pwm_idx)
            in_1 = self.pca.init_pin(in_1_idx)
            in_2 = self.pca.init_pin(in_2_idx)

            self.motors[idx] = Motor(pwm, in_1, in_2)

        return self.motors[idx]
