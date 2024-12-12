# Printer cooling fan
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import pulse_counter, output_pin

SAFETY_CHECK_INIT_TIME = 3.0


class Fan:
    def __init__(
        self,
        config,
        default_shutdown_speed=0.0,
        cmd_set_fan=None,
        cmd_set_fan_description=None,
    ):
        self.printer = config.get_printer()
        self.full_name = config.get_name()
        self.name = self.full_name.split()[-1]
        self.gcode = self.printer.lookup_object("gcode")
        self.reactor = self.printer.get_reactor()
        self.estimated_print_time = None
        self.kalico_threads = self.printer.get_kalico_threads()
        self.last_pwm_value = self.last_req_pwm_value = 0.0
        self.last_fan_value = self.last_req_value = 0.0
        # Read config
        self.kick_start_time = config.getfloat(
            "kick_start_time", 0.1, minval=0.0
        )
        self.kick_start_threshold = (
            config.getfloat("kick_start_threshold", 0.5, minval=0.0, maxval=1.0)
            if self.kick_start_time
            else 0.5
        )
        self.kick_start_power = (
            config.getfloat("kick_start_power", 1.0, minval=0.0, maxval=1.0)
            if self.kick_start_time
            else 1.0
        )

        self.min_power = config.getfloat(
            "min_power", default=None, minval=0.0, maxval=1.0
        )
        self.off_below = config.getfloat(
            "off_below", default=None, minval=0.0, maxval=1.0
        )
        if self.off_below is not None:
            config.deprecate("off_below")
        self.initial_speed = config.getfloat(
            "initial_speed", default=None, minval=0.0, maxval=1.0
        )

        # handles switchover of variable
        # if new var is not set, and old var is, set new var to old var
        # if new var is not set and neither is old var, set new var to default of 0.0
        # if new var is set, use new var
        if self.min_power is not None and self.off_below is not None:
            raise config.error(
                "min_power and off_below are both set. Remove one!"
            )
        if self.min_power is None:
            if self.off_below is None:
                # both unset, set to 0.0
                self.min_power = 0.0
            else:
                self.min_power = self.off_below

        self.max_power = config.getfloat(
            "max_power", 1.0, above=0.0, maxval=1.0
        )
        if self.min_power > self.max_power:
            raise config.error(
                "min_power=%f can't be larger than max_power=%f"
                % (self.min_power, self.max_power)
            )
        self.full_speed_max_power = config.getboolean(
            "full_speed_max_power", False
        )

        cycle_time = config.getfloat("cycle_time", 0.010, above=0.0)
        hardware_pwm = config.getboolean("hardware_pwm", False)
        shutdown_speed = config.getfloat(
            "shutdown_speed", default_shutdown_speed, minval=0.0, maxval=1.0
        )
        # Setup pwm object
        ppins = self.printer.lookup_object("pins")
        self.mcu_fan = ppins.setup_pin("pwm", config.get("pin"))
        self.mcu_fan.setup_max_duration(0.0)
        self.mcu_fan.setup_cycle_time(cycle_time, hardware_pwm)

        if hardware_pwm:
            self.shutdown_power = max(0.0, min(self.max_power, shutdown_speed))
        else:
            # the config allows shutdown_power to be > 0 and < 1, but it is validated
            # in MCU_pwm._build_config().
            self.shutdown_power = max(0.0, shutdown_speed)

        self.mcu_fan.setup_start_value(0.0, self.shutdown_power)
        self.enable_pin = None
        enable_pin = config.get("enable_pin", None)
        if enable_pin is not None:
            self.enable_pin = ppins.setup_pin("digital_out", enable_pin)
            self.enable_pin.setup_max_duration(0.0)

        # Create gcode request queue
        self.gcrq = output_pin.GCodeRequestQueue(
            config, self.mcu_fan.get_mcu(), self._apply_speed
        )

        self.tachometer = FanTachometer(config)

        self.name = config.get_name().split()[-1]
        self.num_err = 0
        self.min_rpm = config.getint("min_rpm", None, minval=0)
        self.max_err = config.getint("max_error", None, minval=0)
        self.on_error_gcode = config.get("on_error_gcode", None)
        self.startup_check = config.getboolean("startup_check", None)
        self.startup_check_delay = config.getfloat("startup_check_delay", None)
        self.startup_check_rpm = config.getfloat(
            "startup_check_rpm", None, minval=0
        )
        if (
            self.min_rpm is not None
            and self.min_rpm > 0
            and self.tachometer._freq_counter is None
        ):
            raise config.error(
                "'tachometer_pin' must be specified before enabling 'min_rpm'"
            )
        if self.max_err is not None and self.min_rpm is None:
            raise config.error(
                "'min_rpm' must be specified before enabling 'max_error'"
            )
        if self.on_error_gcode is not None and self.min_rpm is None:
            raise config.error(
                "'min_rpm' must be specified before enabling 'on_error_gcode'"
            )
        if self.startup_check is not None and self.min_rpm is None:
            raise config.error(
                "'min_rpm' must be specified before enabling 'startup_check'"
            )
        if self.startup_check_delay is not None and self.startup_check is None:
            raise config.error(
                "'startup_check' must be enabled before enabling "
                "'startup_check_delay'"
            )
        if self.startup_check_rpm is not None and self.startup_check is None:
            raise config.error(
                "'startup_check' must be enabled before enabling 'startup_check_rpm'"
            )

        self.min_rpm = 0 if self.min_rpm is None else self.min_rpm
        self.max_err = 3 if self.max_err is None else self.max_err
        self.fan_check_thread = None

        self.startup_check = (
            False if self.startup_check is None else self.startup_check
        )
        self.startup_check_delay = (
            SAFETY_CHECK_INIT_TIME
            if self.startup_check_delay is None
            else self.startup_check_delay
        )
        self.startup_check_rpm = (
            self.min_rpm
            if self.startup_check_rpm is None
            else self.startup_check_rpm
        )
        self.self_checking = False

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        # Register callbacks
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_FAN",
            "FAN",
            self.name,
            self.cmd_SET_FAN if cmd_set_fan is None else cmd_set_fan,
            desc=(
                self.cmd_SET_FAN_help
                if cmd_set_fan_description is None
                else cmd_set_fan_description
            ),
        )

    def _handle_ready(self):
        self.estimated_print_time = self.get_mcu().estimated_print_time
        if self.startup_check:
            self.self_checking = True
            toolhead = self.printer.lookup_object("toolhead")
            toolhead.register_lookahead_callback(
                (lambda pt: self.set_startup_fan_speed(pt))
            )
        if self.initial_speed:
            self.set_speed_from_command(self.initial_speed)

    def set_startup_fan_speed(self, print_time):
        self.mcu_fan.set_pwm(print_time, self.max_power)
        self.reactor.register_callback(
            self.startup_self_check,
            self.reactor.monotonic() + self.startup_check_delay,
        )

    def startup_self_check(self, eventtime=None):
        rpm = self.tachometer.get_status()["rpm"]
        if rpm < self.startup_check_rpm:
            msg = (
                "Fan startup check!\n"
                "'%s' spinning below minimum safe speed.\n"
                "expected: %d rev/min\n"
                "actual: %d rev/min" % (self.name, self.startup_check_rpm, rpm)
            )
            logging.error(msg)
            self.printer.invoke_shutdown(msg)
        self.printer.lookup_object("toolhead").register_lookahead_callback(
            (lambda pt: self.set_speed(self.last_req_value, pt, force=True))
        )
        self.self_checking = False

    def get_mcu(self):
        return self.mcu_fan.get_mcu()

    def _apply_speed(self, print_time, value, force=False):
        if value > 0:
            if value == 1.0 and self.full_speed_max_power:
                pwm_value = 1.0
            else:
                # Scale value between min_power and max_power
                pwm_value = (
                    value * (self.max_power - self.min_power) + self.min_power
                )
                pwm_value = max(self.min_power, min(self.max_power, pwm_value))
        else:
            pwm_value = 0

        if (
            value == self.last_fan_value
            and pwm_value == self.last_pwm_value
            and not force
        ):
            return "discard", 0.0

        if force or not self.self_checking:
            if self.enable_pin:
                if value > 0 and self.last_pwm_value == 0:
                    self.enable_pin.set_digital(print_time, 1)
                elif value == 0 and self.last_pwm_value > 0:
                    self.enable_pin.set_digital(print_time, 0)
            if (
                pwm_value
                and self.kick_start_time
                and (
                    not self.last_pwm_value
                    or pwm_value - self.last_pwm_value
                    > self.kick_start_threshold
                )
            ):
                # Run fan at full speed for specified kick_start_time
                self.last_req_value = value
                self.last_req_pwm_value = pwm_value

                self.last_fan_value = self.max_power
                self.last_pwm_value = self.max_power

                self.mcu_fan.set_pwm(print_time, self.max_power)
                return "delay", self.kick_start_time
            self.mcu_fan.set_pwm(print_time, pwm_value)
        self.last_fan_value = self.last_req_value = value
        self.last_pwm_value = self.last_req_pwm_value = pwm_value

        if self.min_rpm > 0 and (force or not self.self_checking):
            if pwm_value > 0:
                if self.fan_check_thread is None:
                    self.fan_check_thread = self.kalico_threads.register_job(
                        target=self.fan_check
                    )
                    self.fan_check_thread.start()
            else:
                if self.fan_check_thread is not None:
                    self.fan_check_thread.unregister()
                    self.fan_check_thread = None

    def set_speed(self, value, print_time=None, force=False):
        self.gcrq.send_async_request(value, print_time, force)

    def set_speed_from_command(self, value, force=False):
        self.gcrq.queue_gcode_request(value, force)

    def _handle_request_restart(self, print_time):
        self.set_speed(0.0, print_time)

    def get_status(self, eventtime):
        tachometer_status = self.tachometer.get_status(eventtime)
        return {
            "speed": self.last_req_value,
            "pwm_value": self.last_req_pwm_value,
            "rpm": tachometer_status["rpm"],
        }

    def fan_check(self):
        rpm = self.tachometer.get_status()["rpm"]
        if self.last_pwm_value and rpm is not None and rpm < self.min_rpm:
            self.num_err += 1
            if self.num_err > self.max_err:
                msg = (
                    "'%s' spinning below minimum safe speed.\n"
                    "expected: %d rev/min\n"
                    "actual: %d rev/min" % (self.name, self.min_rpm, rpm)
                )
                logging.error(msg)
                if self.on_error_gcode is not None:
                    self.gcode.respond_info(msg)
                    self.gcode.run_script_from_command(self.on_error_gcode)
                else:
                    self.printer.invoke_shutdown(msg)
                    return 0
        else:
            self.num_err = 0
        if self.last_fan_value:
            return 1.5
        return 0

    cmd_SET_FAN_help = "Change settings for a fan"

    def cmd_SET_FAN(self, gcmd):
        self.min_power = gcmd.get_float(
            "MIN_POWER", self.min_power, minval=0.0, maxval=1.0
        )
        self.max_power = gcmd.get_float(
            "MAX_POWER", self.max_power, above=self.min_power, maxval=1.0
        )
        self.min_rpm = gcmd.get_float("MIN_RPM", self.min_rpm, minval=0.0)
        self.set_speed_from_command(self.last_fan_value, force=True)


class FanTachometer:
    def __init__(self, config):
        printer = config.get_printer()
        self._freq_counter = None

        pin = config.get("tachometer_pin", None)
        if pin is not None:
            self.ppr = config.getint("tachometer_ppr", 2, minval=1)
            poll_time = config.getfloat(
                "tachometer_poll_interval", 0.0015, above=0.0
            )
            sample_time = 1.0
            self._freq_counter = pulse_counter.FrequencyCounter(
                printer, pin, sample_time, poll_time
            )

    def get_status(self, eventtime=None):
        if self._freq_counter is not None:
            rpm = self._freq_counter.get_frequency() * 30.0 / self.ppr
        else:
            rpm = None
        return {"rpm": rpm}


class PrinterFan:
    def __init__(self, config):
        self.fan = Fan(config)
        # Register commands
        gcode = config.get_printer().lookup_object("gcode")
        gcode.register_command("M106", self.cmd_M106)
        gcode.register_command("M107", self.cmd_M107)

    def get_status(self, eventtime):
        return self.fan.get_status(eventtime)

    def cmd_M106(self, gcmd):
        # Set fan speed
        value = gcmd.get_float("S", 255.0, minval=0.0, maxval=255.0) / 255.0
        force = gcmd.get_int("F", 0, minval=0, maxval=1)
        self.fan.set_speed_from_command(value, force)

    def cmd_M107(self, gcmd):
        # Turn fan off
        force = gcmd.get_int("F", 0, minval=0, maxval=1)
        self.fan.set_speed_from_command(0.0, force)


def load_config(config):
    return PrinterFan(config)
