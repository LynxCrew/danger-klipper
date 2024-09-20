# Printer cooling fan
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import pulse_counter

FAN_MIN_TIME = 0.100
SAFETY_CHECK_INIT_TIME = 3.0


class Fan:
    def __init__(self, config, default_shutdown_speed=0.0):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.gcode = self.printer.lookup_object("gcode")
        self.reactor = self.printer.get_reactor()
        self.estimated_print_time = None
        self.klipper_threads = self.printer.get_klipper_threads()
        self.last_fan_value = 0.0
        self.last_pwm_value = 0.0
        self.last_fan_time = 0.0
        self.queued_value = None
        self.queued_pwm_value = None
        self.queued_force = False
        self.locking = False
        self.unlock_timer = self.reactor.register_timer(self._unlock_lock)
        # Read config
        self.kick_start_time = config.getfloat("kick_start_time", 0.1, minval=0.0)
        self.kick_start_threshold = config.getfloat(
            "kick_start_threshold", 0.5, minval=0.0, maxval=1.0
        )
        self.min_power = config.getfloat(
            "min_power", default=None, minval=0.0, maxval=1.0
        )
        self.off_below = config.getfloat(
            "off_below", default=None, minval=0.0, maxval=1.0
        )

        # handles switchover of variable
        # if new var is not set, and old var is, set new var to old var
        # if new var is not set and neither is old var, set new var to default of 0.0
        # if new var is set, use new var
        if self.min_power is not None and self.off_below is not None:
            raise config.error("min_power and off_below are both set. Remove one!")
        if self.min_power is None:
            if self.off_below is None:
                # both unset, set to 0.0
                self.min_power = 0.0
            else:
                self.min_power = self.off_below

        self.max_power = config.getfloat("max_power", 1.0, above=0.0, maxval=1.0)
        if self.min_power > self.max_power:
            raise config.error(
                "min_power=%f can't be larger than max_power=%f"
                % (self.min_power, self.max_power)
            )
        self.full_speed_max_power = config.getboolean("full_speed_max_power", False)

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

        self.tachometer = FanTachometer(config)

        self.name = config.get_name().split()[-1]
        self.num_err = 0
        self.min_rpm = config.getint("min_rpm", None, minval=0)
        self.max_err = config.getint("max_error", None, minval=0)
        self.on_error_gcode = config.get("on_error_gcode", None)
        self.startup_check = config.getboolean("startup_check", None)
        self.startup_check_delay = config.getfloat("startup_check_delay", None)
        self.startup_check_rpm = config.getfloat("startup_check_rpm", None, minval=0)
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

        self.startup_check = False if self.startup_check is None else self.startup_check
        self.startup_check_delay = (
            SAFETY_CHECK_INIT_TIME
            if self.startup_check_delay is None
            else self.startup_check_delay
        )
        self.startup_check_rpm = (
            self.min_rpm if self.startup_check_rpm is None else self.startup_check_rpm
        )
        self.self_checking = False

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        # Register callbacks
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_FAN",
            "FAN",
            self.name,
            self.cmd_SET_FAN,
            desc=self.cmd_SET_FAN_help,
        )

    def handle_ready(self):
        self.estimated_print_time = self.get_mcu().estimated_print_time
        if self.startup_check:
            self.self_checking = True
            toolhead = self.printer.lookup_object("toolhead")
            toolhead.register_lookahead_callback(
                (lambda pt: self.set_startup_fan_speed(pt))
            )

    def set_startup_fan_speed(self, print_time):
        self.mcu_fan.set_pwm(print_time, self.max_power)
        self.reactor.register_callback(
            self.startup_self_check,
            self.reactor.monotonic() + self.startup_check_delay,
        )

    def startup_self_check(self, eventtime):
        rpm = self.tachometer.get_status(eventtime)["rpm"]
        if rpm < self.startup_check_rpm:
            msg = (
                "'%s' spinning below minimum safe speed.\n"
                "expected: %d rev/min\n"
                "actual: %d rev/min" % (self.name, self.startup_check_rpm, rpm)
            )
            logging.error(msg)
            self.printer.invoke_shutdown(msg)
        self.printer.lookup_object("toolhead").register_lookahead_callback(
            (lambda pt: self.set_speed(pt, self.last_pwm_value, force=True))
        )
        self.self_checking = False

    def get_mcu(self):
        return self.mcu_fan.get_mcu()

    def set_speed(self, print_time, value, force=False):
        if value > 0:
            if self.full_speed_max_power and value == 1.0:
                pwm_value = 1.0
            else:
                # Scale value between min_power and max_power
                pwm_value = value * (self.max_power - self.min_power) + self.min_power
                pwm_value = max(self.min_power, min(self.max_power, pwm_value))
        else:
            pwm_value = 0
        if self.locking:
            self.queued_value = value
            self.queued_pwm_value = pwm_value
            self.queued_force = force
            return
        self._set_speed(
            print_time=print_time, value=value, pwm_value=pwm_value, force=force
        )

    def _set_speed(
        self, print_time, value, pwm_value, force=False, resend=False, eventtime=None
    ):
        if eventtime is None:
            eventtime = self.reactor.monotonic()

        if (
            value == self.last_fan_value
            and pwm_value == self.last_pwm_value
            and not force
        ):
            return eventtime + FAN_MIN_TIME
        if force or not self.self_checking:
            self.locking = True
            if self.enable_pin:
                if value > 0 and self.last_pwm_value == 0:
                    self.enable_pin.set_digital(print_time, 1)
                elif value == 0 and self.last_pwm_value > 0:
                    self.enable_pin.set_digital(print_time, 0)
            if (
                pwm_value
                and pwm_value < self.max_power
                and self.kick_start_time
                and (
                    not self.last_fan_value
                    or value - self.last_fan_value > self.kick_start_threshold
                )
            ):
                # Run fan at full speed for specified kick_start_time
                self.mcu_fan.set_pwm(print_time, self.max_power)
                print_time += self.kick_start_time
                eventtime += self.kick_start_time
            self.mcu_fan.set_pwm(print_time, pwm_value)
            if not resend:
                self.reactor.update_timer(self.unlock_timer, eventtime + FAN_MIN_TIME)
        self.last_fan_value = value
        self.last_pwm_value = pwm_value
        self.last_fan_time = print_time

        if self.min_rpm > 0 and (force or not self.self_checking):
            if pwm_value > 0:
                if self.fan_check_thread is None:
                    self.fan_check_thread = self.klipper_threads.register_job(
                        target=self.fan_check
                    )
                    self.fan_check_thread.start()
            else:
                if self.fan_check_thread is not None:
                    self.fan_check_thread.unregister()
                    self.fan_check_thread = None
        return eventtime + FAN_MIN_TIME

    def _unlock_lock(self, eventtime):
        if self.queued_value is not None or self.queued_pwm_value is not None:
            value = self.queued_value
            pwm_value = self.queued_pwm_value
            force = self.queued_force
            self.queued_value = None
            self.queued_pwm_value = None
            self.queued_force = False
            if (
                self.queued_value != self.last_fan_value
                or self.queued_pwm_value != self.last_pwm_value
                or not self.queued_force
            ):
                return self._set_speed(
                    print_time=self.last_fan_time + FAN_MIN_TIME,
                    value=value,
                    pwm_value=pwm_value,
                    force=force,
                    resend=True,
                    eventtime=eventtime,
                )
        self.locking = False
        return self.reactor.NEVER

    def set_speed_from_command(self, value, force=False):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.register_lookahead_callback(
            (lambda pt: self.set_speed(pt, value, force))
        )

    def _handle_request_restart(self, print_time):
        self.reactor.update_timer(self.unlock_timer, self.reactor.NEVER)
        self.queued_value = None
        self.mcu_fan.set_pwm(print_time, self.shutdown_power)

    def get_status(self, eventtime):
        tachometer_status = self.tachometer.get_status(eventtime)
        return {
            "speed": self.last_pwm_value,
            "normalized_speed": self.last_fan_value,
            "rpm": tachometer_status["rpm"],
        }

    def fan_check(self):
        measured_time = self.reactor.monotonic()
        eventtime = self.printer.lookup_object("mcu").estimated_print_time(
            measured_time
        )
        rpm = self.tachometer.get_status(eventtime)["rpm"]
        if self.last_fan_value and rpm is not None and rpm < self.min_rpm:
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
        curtime = self.reactor.monotonic()
        print_time = self.get_mcu().estimated_print_time(curtime)
        self.set_speed(print_time, self.last_fan_value, force=True)


class FanTachometer:
    def __init__(self, config):
        printer = config.get_printer()
        self._freq_counter = None

        pin = config.get("tachometer_pin", None)
        if pin is not None:
            self.ppr = config.getint("tachometer_ppr", 2, minval=1)
            poll_time = config.getfloat("tachometer_poll_interval", 0.0015, above=0.0)
            sample_time = 1.0
            self._freq_counter = pulse_counter.FrequencyCounter(
                printer, pin, sample_time, poll_time
            )

    def get_status(self, eventtime):
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
