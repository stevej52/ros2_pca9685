#!/usr/bin/env python
import time

import Adafruit_PCA9685
from pca9685.models.config import PCA9685Config

class PCA9685:
    def __init__(
        self,
        config: PCA9685Config,
        init_delay: float = 0.1,
    ):
        default_freq = 60
        frequency = config.frequency
        self.pwm_scale = frequency / default_freq
        self.pwm = Adafruit_PCA9685.PCA9685(
            address=config.address, busnum=config.busnum
        )
        self.pwm.set_pwm_freq(frequency)
        time.sleep(init_delay)  # Sometimes servos make a little leap otherwise

    def set_pulse(self, channel: int, pulse: int) -> None:
        scaled_pwm = int(pulse * self.pwm_scale)
        self.pwm.set_pwm(channel, 0, scaled_pwm)

