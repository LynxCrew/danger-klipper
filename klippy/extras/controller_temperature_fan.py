# Support fans that are enabled when temperature exceeds a set threshold
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from . import fan, temperature_fan, controller_fan

KELVIN_TO_CELSIUS = -273.15
MAX_FAN_TIME = 5.0
AMBIENT_TEMP = 25.0
PID_PARAM_BASE = 255.0
PIN_MIN_TIME = 0.100


class ControllerTemperatureFan:
    def __init__(self, config):
        self.config = config
        self.full_name = config.get_name()
        self.name = self.full_name.split()[-1]
        self.printer = config.get_printer()
        self.fan = fan.Fan(config, default_shutdown_speed=1.0)
        self.temperature_fan = temperature_fan.TemperatureFan(
            config, self.fan, self
        )
        self.controller_fan = controller_fan.ControllerFan(config, self.fan)

    def get_mcu(self):
        return self.fan.get_mcu()

    def set_speed(self, value, read_time):
        self.temperature_fan.set_tf_speed(
            read_time, max(value, self.controller_fan.get_speed(read_time))
        )

    def temperature_callback(self, read_time, temp):
        self.temperature_fan.temperature_callback(read_time, temp)

    def get_temp(self, eventtime):
        return self.temperature_fan.get_temp(eventtime)

    def get_min_speed(self):
        return self.temperature_fan.min_speed

    def get_max_speed(self):
        return self.temperature_fan.get_max_speed()

    def get_status(self, eventtime):
        return self.temperature_fan.get_status(eventtime)

    def is_adc_faulty(self):
        return self.temperature_fan.is_adc_faulty()


def load_config_prefix(config):
    return ControllerTemperatureFan(config)
