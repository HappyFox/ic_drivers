import math

from ..pca9685 import Pca9685



class InvalidDirectionError(Exception):
    pass


class InvalidMotorError(Exception):
    pass


motor_pins = [(8, 9, 10), (13, 12, 11), (2, 3, 4), (7, 6, 5)]


def get_motor_shield(i2c, address=0x60):
    pca = Pca9685(i2c, address)
    return MotorShield(pca)


class Motor:

    FREEWHEEL = 0
    FORWARD = 1
    BRAKE = 3
    BACKWARD = 2

    def __init__(self, pwm, in_1, in_2):
        self.pwm = pwm
        self.in_1 = in_1
        self.in_2 = in_2

        self.invert = False
        self.dir(self.FREEWHEEL)

    def speed(self, speed=None):
        if speed is None:
            return self.pwm.duty()

        self.pwm.duty(speed)

    def dir(self, direction=None):
        if direction is None:
            direction = self.in_1.value()
            direction += self.in_2.value() << 1

            if self.invert and 0b01 <= direction <= 0b10:
                direction = direction ^ 0b11  # Binary invert.

            return direction

        if not 0 <= direction <= 0b11:
            raise InvalidDirectionError()

        if self.invert and 0b01 <= direction <= 0b10:
            direction = direction ^ 0b11

        self.in_1.value(direction & 0b01)
        self.in_2.value(direction & 0b10)


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
