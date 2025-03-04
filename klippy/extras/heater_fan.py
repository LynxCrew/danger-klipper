# Support fans that are enabled when a heater is on
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import fan

PIN_MIN_TIME = 0.100


class PrinterHeaterFan:
    def __init__(self, config):
        self.full_name = config.get_name()
        self.name = self.full_name.split()[-1]
        self.printer = config.get_printer()
        self.printer.load_object(config, "heaters")
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.heater_names = config.getlist("heater", ("extruder",))
        self.heater_temp = config.getfloat("heater_temp", 50.0)
        self.fan_off_hysteresis = config.getfloat("fan_off_hysteresis", 0.0)
        self.heaters = []
        self.fan = fan.Fan(config, default_shutdown_speed=1.0)
        self.config_fan_speed = config.getfloat(
            "fan_speed", 1.0, minval=0.0, maxval=1.0
        )
        self.fan_speed = self.config_fan_speed
        self.last_speed = 0.0
        self.enabled = 1
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_HEATER_FAN",
            "HEATER_FAN",
            self.name,
            self.cmd_SET_HEATER_FAN,
            desc=self.cmd_SET_HEATER_FAN_help,
        )

    def get_mcu(self):
        return self.fan.get_mcu()

    def _handle_ready(self):
        pheaters = self.printer.lookup_object("heaters")
        self.heaters = [pheaters.lookup_heater(n) for n in self.heater_names]
        reactor = self.printer.get_reactor()
        reactor.register_timer(
            self.callback, reactor.monotonic() + PIN_MIN_TIME
        )

    def get_status(self, eventtime):
        status = self.fan.get_status(eventtime)
        status["enabled"] = self.enabled
        return status

    def callback(self, eventtime):
        speed = 0.0
        for heater in self.heaters:
            current_temp, target_temp = heater.get_temp(eventtime)
            if (target_temp or current_temp > self.heater_temp) or (
                self.last_speed > 0
                and (
                    target_temp
                    or current_temp > self.heater_temp - self.fan_off_hysteresis
                )
            ):
                speed = self.fan_speed

        if self.enabled and speed != self.last_speed:
            self.last_speed = speed
            self.fan.set_speed(speed)
        return eventtime + 1.0

    cmd_SET_HEATER_FAN_help = "Enable or Disable a heater_fan"

    def cmd_SET_HEATER_FAN(self, gcmd):
        self.enabled = gcmd.get_int("ENABLE", self.enabled, minval=0, maxval=1)
        self.fan_speed = gcmd.get_float(
            "FAN_SPEED", self.fan_speed, minval=0, maxval=1
        )
        if self.enabled:
            self.fan.set_speed(self.fan_speed)
        if not self.enabled:
            self.fan.set_speed(0.0)


def load_config_prefix(config):
    return PrinterHeaterFan(config)
