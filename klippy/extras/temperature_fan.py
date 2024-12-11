# Support fans that are enabled when temperature exceeds a set threshold
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import numpy as np

from . import fan

KELVIN_TO_CELSIUS = -273.15
MAX_FAN_TIME = 5.0
AMBIENT_TEMP = 25.0
PID_PARAM_BASE = 255.0


class TemperatureFan:
    def __init__(self, config, defined_fan=None, super_fan=None):
        self.full_name = config.get_name()
        self.name = self.full_name.split()[-1]
        self.printer = config.get_printer()
        if defined_fan is None:
            self.fan = fan.Fan(config, default_shutdown_speed=1.0)
        else:
            self.fan = defined_fan
        self.min_temp = config.getfloat("min_temp", minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat("max_temp", above=self.min_temp)
        pheaters = self.printer.load_object(config, "heaters")
        self.sensor = pheaters.setup_sensor(config)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        pheaters.register_sensor(config, self)
        self.speed_delay = self.sensor.get_report_time_delta()
        self.max_speed_conf = config.getfloat(
            "max_speed", 1.0, above=0.0, maxval=1.0
        )
        self.max_speed = self.max_speed_conf
        self.min_speed_conf = config.getfloat(
            "min_speed", 0.3, minval=0.0, maxval=1.0
        )
        self.min_speed = self.min_speed_conf
        self.mode = "automatic"
        self.manual_speed = None
        self.last_temp = 0.0
        self.measured_min = 99999999.0
        self.measured_max = -99999999.0
        self.last_temp_time = 0.0
        self.target_temp_conf = config.getfloat(
            "target_temp",
            40.0 if self.max_temp > 40.0 else self.max_temp,
            minval=self.min_temp,
            maxval=self.max_temp,
        )
        self.target_temp = self.target_temp_conf
        algos = {
            "watermark": ControlBangBang,
            "pid": ControlPID,
            "curve": ControlCurve,
        }
        algo = config.getchoice("control", algos)
        self.control = algo(self, config, super_fan)
        self.next_speed_time = 0.0
        self.last_speed_value = 0.0
        self.enabled = True
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_TEMPERATURE_FAN_TARGET",
            "TEMPERATURE_FAN",
            self.name,
            self.cmd_SET_TEMPERATURE_FAN,
            desc=self.cmd_SET_TEMPERATURE_FAN_help,
        )
        gcode.register_mux_command(
            "SET_TEMPERATURE_FAN",
            "TEMPERATURE_FAN",
            self.name,
            self.cmd_SET_TEMPERATURE_FAN,
            desc=self.cmd_SET_TEMPERATURE_FAN_help,
        )

    def get_mcu(self):
        return self.fan.get_mcu()

    def set_speed(self, value, read_time=None):
        if read_time is None:
            systime = self.printer.get_reactor().monotonic()
            read_time = self.get_mcu().estimated_print_time(
                systime + fan.output_pin.PIN_MIN_TIME
            )
        return self.set_tf_speed(read_time, value)

    def set_tf_speed(self, read_time, value):
        if value <= 0.0:
            value = 0.0
        elif value < self.min_speed:
            value = self.min_speed
        if self.target_temp <= 0.0:
            value = 0.0
        if (
            read_time < self.next_speed_time or not self.last_speed_value
        ) and abs(value - self.last_speed_value) < 0.05:
            # No significant change in value - can suppress update
            return
        speed_time = read_time + self.speed_delay
        self.next_speed_time = speed_time + 0.75 * MAX_FAN_TIME
        self.last_speed_value = value
        self.fan.set_speed(value, speed_time)

    def temperature_callback(self, read_time, temp):
        self.last_temp = temp
        if self.mode == "manual":
            self.set_speed(self.manual_speed, read_time)
        else:
            self.control.temperature_callback(read_time, temp)
        if temp:
            self.measured_min = min(self.measured_min, temp)
            self.measured_max = max(self.measured_max, temp)

    def get_temp(self, eventtime):
        return self.last_temp, self.target_temp

    def get_min_speed(self):
        return self.min_speed

    def get_max_speed(self):
        return self.max_speed

    def get_status(self, eventtime):
        status = self.fan.get_status(eventtime)
        status["temperature"] = round(self.last_temp, 2)
        status["measured_min_temp"] = round(self.measured_min, 2)
        status["measured_max_temp"] = round(self.measured_max, 2)
        status["target"] = self.target_temp
        status["control"] = self.control.get_type()
        return status

    def is_adc_faulty(self):
        if self.last_temp > self.max_temp or self.last_temp < self.min_temp:
            return True
        return False

    cmd_SET_TEMPERATURE_FAN_help = "Sets a temperature fan target and fan speed limits and enable or disable it"

    def cmd_SET_TEMPERATURE_FAN(self, gcmd):
        target = gcmd.get_float("TARGET", None)
        min_speed = gcmd.get_float("MIN_SPEED", self.min_speed)
        max_speed = gcmd.get_float("MAX_SPEED", self.max_speed)
        speed = gcmd.get_float("SPEED", None)
        mode = gcmd.get("MODE", self.mode if speed is None else "manual")
        if target is not None and self.control.get_type() == "curve":
            raise gcmd.error("Setting Target not supported for control curve")
        if min_speed > max_speed:
            raise self.printer.command_error(
                "Requested min speed (%.1f) is greater than max speed (%.1f)"
                % (min_speed, max_speed)
            )
        self.set_min_speed(min_speed)
        self.set_max_speed(max_speed)
        self.set_manual_speed(speed)
        self.set_mode(mode)
        self.set_temp(self.target_temp_conf if target is None else target)
        self.enabled = gcmd.get_int("ENABLE", self.enabled, minval=0, maxval=1)
        if not self.enabled:
            curtime = self.printer.get_reactor().monotonic()
            print_time = self.fan.get_mcu().estimated_print_time(curtime)
            self.fan.set_speed(0.0, print_time)

    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp)
            )
        self.target_temp = degrees

    def set_min_speed(self, speed):
        if speed and (speed < 0.0 or speed > 1.0):
            raise self.printer.command_error(
                "Requested min speed (%.1f) out of range (0.0 : 1.0)" % (speed)
            )
        self.min_speed = speed

    def set_max_speed(self, speed):
        if speed and (speed < 0.0 or speed > 1.0):
            raise self.printer.command_error(
                "Requested max speed (%.1f) out of range (0.0 : 1.0)" % (speed)
            )
        self.max_speed = speed

    def set_manual_speed(self, speed):
        if speed and (speed < self.min_speed or speed > self.max_speed):
            raise self.printer.command_error(
                "Requested max speed (%.1f) out of range (%.1f : %.1f)"
                % (speed, self.min_speed, self.max_speed)
            )
        self.manual_speed = speed

    def set_mode(self, mode):
        if mode not in ("automatic", "manual"):
            raise self.printer.command_error(
                "Requested mode not valid, use 'manual' or 'automatic'"
            )
        self.mode = mode


######################################################################
# Bang-bang control algo
######################################################################


class ControlBangBang:
    def __init__(self, temperature_fan, config, controlled_fan=None):
        self.temperature_fan = temperature_fan
        self.controlled_fan = (
            temperature_fan if controlled_fan is None else controlled_fan
        )
        self.reverse = config.getboolean("reverse", False)
        self.max_delta = config.getfloat("max_delta", 2.0, above=0.0)
        self.heating = False

    def temperature_callback(self, read_time, temp):
        current_temp, target_temp = self.temperature_fan.get_temp(read_time)
        if (
            self.heating != self.reverse
            and temp >= target_temp + self.max_delta
        ):
            self.heating = self.reverse
        elif (
            self.heating == self.reverse
            and temp <= target_temp - self.max_delta
        ):
            self.heating = not self.reverse
        if self.heating:
            self.controlled_fan.set_speed(0.0, read_time)
        else:
            self.controlled_fan.set_speed(
                self.temperature_fan.get_max_speed(), read_time
            )

    def get_type(self):
        return "watermark"


######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.0
PID_SETTLE_SLOPE = 0.1


class ControlPID:
    def __init__(self, temperature_fan, config, controlled_fan=None):
        self.temperature_fan = temperature_fan
        self.controlled_fan = (
            temperature_fan if controlled_fan is None else controlled_fan
        )
        self.reverse = config.getboolean("reverse", False)
        self.Kp = config.getfloat("pid_Kp") / PID_PARAM_BASE
        self.Ki = config.getfloat("pid_Ki") / PID_PARAM_BASE
        self.Kd = config.getfloat("pid_Kd") / PID_PARAM_BASE
        self.min_deriv_time = config.getfloat("pid_deriv_time", 2.0, above=0.0)
        imax = config.getfloat(
            "pid_integral_max", self.temperature_fan.get_max_speed(), minval=0.0
        )
        self.temp_integ_max = imax / self.Ki
        self.prev_temp = AMBIENT_TEMP
        self.prev_temp_time = 0.0
        self.prev_temp_deriv = 0.0
        self.prev_temp_integ = 0.0

    def temperature_callback(self, read_time, temp):
        current_temp, target_temp = self.temperature_fan.get_temp(read_time)
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (
                self.prev_temp_deriv * (self.min_deriv_time - time_diff)
                + temp_diff
            ) / self.min_deriv_time
        # Calculate accumulated temperature "error"
        temp_err = target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0.0, min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self.Kp * temp_err + self.Ki * temp_integ - self.Kd * temp_deriv
        bounded_co = max(0.0, min(self.temperature_fan.get_max_speed(), co))
        if not self.reverse:
            self.controlled_fan.set_speed(
                max(
                    self.controlled_fan.get_min_speed(),
                    self.controlled_fan.get_max_speed() - bounded_co,
                ),
                read_time,
            )
        else:
            self.controlled_fan.set_speed(
                max(self.temperature_fan.get_min_speed(), bounded_co), read_time
            )
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ

    def get_type(self):
        return "pid"


class ControlCurve:
    def __init__(self, temperature_fan, config, controlled_fan=None):
        self.temperature_fan = temperature_fan
        self.controlled_fan = (
            temperature_fan if controlled_fan is None else controlled_fan
        )

        points = list(
            config.getlists("points", seps=(",", "\n"), parser=float, count=2)
        )
        points.sort(key=lambda x: x[0])

        if len(points) < 2:
            raise temperature_fan.printer.config_error(
                "At least two points need to be defined for curve in "
                "temperature_fan."
            )
        if any(len(point) != 2 for point in points):
            raise temperature_fan.printer.config_error(
                "A point must have exactly one temperature and one pwm setting "
                "value."
            )
        if not all(points[i] <= points[i + 1] for i in range(len(points) - 1)):
            raise temperature_fan.printer.config_error(
                "The fan curve must be monotonically increasing."
            )

        temp_values, pwm_values = zip(*points)

        if len(temp_values) > len(set(temp_values)):
            raise temperature_fan.printer.config_error(
                "Temperature may not exist twice in curve table."
            )
        if temp_values[-1] > temperature_fan.target_temp:
            raise temperature_fan.printer.config_error(
                "Temperature in point may not exceed target_temp."
            )
        if temp_values[0] < temperature_fan.min_temp:
            raise temperature_fan.printer.config_error(
                "Temperature in point may not fall below min_temp."
            )
        if pwm_values[-1] > temperature_fan.get_max_speed():
            raise temperature_fan.printer.config_error(
                "Speed in point may not exceed max_speed."
            )
        if pwm_values[0] < temperature_fan.get_min_speed():
            raise temperature_fan.printer.config_error(
                "Speed in point may not fall below min_speed."
            )

        if points[0][0] > temperature_fan.min_temp:
            points.insert(0, (temperature_fan.min_temp, points[0][1]))
        if points[-1][0] < temperature_fan.max_temp:
            points.append((temperature_fan.max_temp, points[-1][1]))

        self.smooth_readings = config.getint(
            "smooth_readings", default=None, minval=0
        )
        if self.smooth_readings is not None:
            config.deprecate("smooth_readings")
        self.cooling_hysteresis = config.getfloat("cooling_hysteresis", 0.0)
        self.heating_hysteresis = config.getfloat("heating_hysteresis", 0.0)
        self.curve_standard = np.array([*points]).transpose()
        self.curve_heating = np.copy(self.curve_standard)
        self.curve_cooling = np.copy(self.curve_standard)
        self.curve_heating[0, :] += self.heating_hysteresis
        self.curve_cooling[0, :] -= self.cooling_hysteresis

    def temperature_callback(self, read_time, temp):
        current_speed = self.controlled_fan.last_speed_value
        upper_temp = np.interp(
            current_speed, self.curve_heating[1], self.curve_heating[0]
        )
        lower_temp = np.interp(
            current_speed, self.curve_cooling[1], self.curve_cooling[0]
        )

        if temp < lower_temp:
            next_speed = np.interp(
                temp, self.curve_cooling[0], self.curve_cooling[1]
            )
        elif temp > upper_temp:
            next_speed = np.interp(
                temp, self.curve_heating[0], self.curve_heating[1]
            )
        else:
            next_speed = current_speed

        self.controlled_fan.set_speed(read_time, next_speed)

    def get_type(self):
        return "curve"


def load_config_prefix(config):
    return TemperatureFan(config)
