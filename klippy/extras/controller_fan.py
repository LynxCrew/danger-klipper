# Support a fan for cooling the MCU whenever a stepper or heater is on
#
# Copyright (C) 2019  Nils Friedchen <nils.friedchen@googlemail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from . import fan

PIN_MIN_TIME = 0.100


class ControllerFan:
    def __init__(self, config, defined_fan=None):
        self.full_name = config.get_name()
        self.name = self.full_name.split()[-1]
        self.printer = config.get_printer()
        self.klipper_threads = self.printer.get_klipper_threads()
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        if defined_fan is None:
            self.printer.register_event_handler(
                "klippy:ready", self._handle_ready
            )
            self.fan = fan.Fan(config)
        else:
            self.fan = defined_fan
        self.stepper_names = config.getlist("stepper", None)
        self.stepper_enable = self.printer.load_object(config, "stepper_enable")
        self.printer.load_object(config, "heaters")
        self.heaters = []
        self.fan_speed = config.getfloat(
            "fan_speed", default=1.0, minval=0.0, maxval=1.0
        )
        self.idle_speed = config.getfloat(
            "idle_speed", default=self.fan_speed, minval=0.0, maxval=1.0
        )
        self.idle_timeout = config.getint("idle_timeout", default=30, minval=-1)
        self.heater_names = config.getlist("heater", None)
        self.pheaters = self.printer.load_object(config, "heaters")
        self.last_on = self.idle_timeout
        self.last_speed = 0.0
        self.enabled = True
        self.temperature_sample_thread = self.klipper_threads.register_job(
            target=self.callback
        )
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_CONTROLLER_FAN",
            "CONTROLLER_FAN",
            self.name,
            self.cmd_SET_CONTROLLER_FAN,
            desc=self.cmd_SET_CONTROLLER_FAN_help,
        )

    def get_mcu(self):
        return self.fan.get_mcu()

    def _handle_connect(self):
        # Heater lookup
        all_heaters = self.pheaters.available_heaters
        if self.heater_names is None:
            self.heater_names = all_heaters
        if not all(x in all_heaters for x in self.heater_names):
            raise self.printer.config_error(
                "One or more of these heaters are unknown: "
                "%s (valid heaters are: %s)"
                % (self.heater_names, ", ".join(all_heaters))
            )
        # Stepper lookup
        all_steppers = self.stepper_enable.get_steppers()
        if self.stepper_names is None:
            self.stepper_names = all_steppers
            return
        if not all(x in all_steppers for x in self.stepper_names):
            raise self.printer.config_error(
                "One or more of these steppers are unknown: "
                "%s (valid steppers are: %s)"
                % (self.stepper_names, ", ".join(all_steppers))
            )

    def _handle_ready(self):
        self.temperature_sample_thread.start()

    def get_status(self, eventtime):
        return self.fan.get_status(eventtime)

    def get_speed(self, eventtime):
        speed = self.idle_speed
        active = False
        for name in self.stepper_names:
            active |= self.stepper_enable.lookup_enable(name).is_motor_enabled()
        for name in self.heater_names:
            _, target_temp = self.pheaters.lookup_heater(name).get_temp(eventtime)
            if target_temp:
                active = True
        if active:
            self.last_on = 0
            speed = self.fan_speed
        elif self.idle_timeout != -1:
            if self.last_on >= self.idle_timeout:
                speed = 0.0
            else:
                self.last_on += 1
        return speed

    def callback(self):
        curtime = self.printer.get_reactor().monotonic()
        speed = self.get_speed(curtime)
        if self.enabled and speed != self.last_speed:
            self.last_speed = speed
            self.fan.set_speed(speed)
        return 1.0

    cmd_SET_CONTROLLER_FAN_help = "Enable or Disable a controller_fan"

    def cmd_SET_CONTROLLER_FAN(self, gcmd):
        self.enabled = gcmd.get_int("ENABLE", self.enabled, minval=0, maxval=1)
        if not self.enabled:
            self.fan.set_speed_from_command(0.0)


def load_config_prefix(config):
    return ControllerFan(config)
