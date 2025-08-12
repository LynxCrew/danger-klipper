
import logging

from . import pulse_counter, output_pin, manual_stepper


SAFETY_CHECK_INIT_TIME = 3.0

class CFlap:
    def __init__(self, config):
        self.config = config
        self.full_name = config.get_name()
        self.name = self.full_name.split()[-1]
        self.printer = config.get_printer()
        self.fan = CFlapFan(config, default_shutdown_speed=1.0)
        self.stepper = manual_stepper.ManualStepper(config)
        self.toolhead = None

        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )

        self.homing_speed = self.config.getfloat("homing_speed", 30, above=0.0)
        self.ignore_trigger = self.config.getboolean("ignore_trigger", False)

        gcode = config.get_printer().lookup_object("gcode")
        gcode.register_command("M106", self.cmd_M106)
        gcode.register_command("M107", self.cmd_M107)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")

    def set_speed(self, value, read_time=None, force=False):
        self.move_stepper(value * 255.0)

    def enable_stepper(self, enable):
        if enable != self.stepper.steppers[0].is_motor_enabled():
            if enable:
                self.stepper.do_enable(True)
                self.toolhead.wait_moves()
                self.stepper.do_homing_move(
                    -255, self.homing_speed, self.stepper.accel, True, not self.ignore_trigger
                )
                self.toolhead.wait_moves()
                self.stepper.do_set_position(0)
                self.toolhead.wait_moves()
            else:
                self.stepper.do_move(0, self.stepper.velocity, self.stepper.accel, 1)
                self.toolhead.wait_moves()
                self.stepper.do_enable(False)
                self.toolhead.wait_moves()

    def move_stepper(self, s):
        self.enable_stepper(True)
        if s is not None:
            self.stepper.do_move(s, self.stepper.velocity, self.stepper.accel, 0)
        else:
            self.stepper.do_move(255, self.stepper.velocity, self.stepper.accel, 0)

    def cmd_M106(self, gcmd):
        p = gcmd.get_int("P", None)
        s = gcmd.get_float("S", None, minval=0.0, maxval=255.0)
        if p is not None:
            if p == 3:
                speed = 1
                if s is not None:
                    if s == 255:
                        speed = 1
                    elif s == 0:
                        speed = 0
                    else:
                        speed = round(
                            s / 255.0, 10
                        )
                self.fan.set_speed_from_command(speed)
            elif p == 1:
                self.move_stepper(s)
            else:
                if s is None:
                    s = 255.0

                value = round(
                    s / 255.0, 10
                )
                self.fan.set_speed_from_command(value)
        else:
            self.move_stepper(s)

    def cmd_M107(self, gcmd):
        p = gcmd.get_int("P", None)
        if p is not None:
            if p == 1:
                self.move_stepper(0)
            else:
                self.fan.set_speed_from_command(0.0)
        else:
            self.fan.set_speed_from_command(0.0)
            self.move_stepper(0)
            self.enable_stepper(False)


FAN_MIN_TIME = 0.100

class CFlapFan:
    def __init__(self, config, default_shutdown_speed=0.0):
        self.printer = config.get_printer()
        self.last_fan_value = 0.0
        self.last_fan_time = 0.0
        self.last_pwm_value = 0.0
        # Read config
        self.kick_start_time = config.getfloat(
            "kick_start_time", 0.1, minval=0.0
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
            shutdown_power = max(0.0, min(self.max_power, shutdown_speed))
        else:
            # the config allows shutdown_power to be > 0 and < 1, but it is validated
            # in MCU_pwm._build_config().
            shutdown_power = max(0.0, shutdown_speed)

        self.mcu_fan.setup_start_value(0.0, shutdown_power)
        self.enable_pin = None
        enable_pin = config.get("fan_enable_pin", None)
        if enable_pin is not None:
            self.enable_pin = ppins.setup_pin("digital_out", enable_pin)
            self.enable_pin.setup_max_duration(0.0)

        # Setup tachometer
        self.tachometer = FanTachometer(config)

        # Register callbacks
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def get_mcu(self):
        return self.mcu_fan.get_mcu()

    def set_speed(self, print_time, value):
        if value == self.last_fan_value:
            return
        if value > 0:
            # Scale value between min_power and max_power
            value = min(value, 1.0)
            pwm_value = (
                    value * (self.max_power - self.min_power) + self.min_power
            )
        else:
            pwm_value = 0
        print_time = max(self.last_fan_time + FAN_MIN_TIME, print_time)
        if self.enable_pin:
            if value > 0 and self.last_fan_value == 0:
                self.enable_pin.set_digital(print_time, 1)
            elif value == 0 and self.last_fan_value > 0:
                self.enable_pin.set_digital(print_time, 0)
        if (
                value
                and value < self.max_power
                and self.kick_start_time
                and (not self.last_fan_value or value - self.last_fan_value > 0.5)
        ):
            # Run fan at full speed for specified kick_start_time
            self.mcu_fan.set_pwm(print_time, self.max_power)
            print_time += self.kick_start_time
        self.mcu_fan.set_pwm(print_time, pwm_value)
        self.last_pwm_value = pwm_value
        self.last_fan_time = print_time
        self.last_fan_value = value

    def set_speed_from_command(self, value):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.register_lookahead_callback(
            (lambda pt: self.set_speed(pt, value))
        )

    def _handle_request_restart(self, print_time):
        self.set_speed(print_time, 0.0)

    def _handle_ready(self):
        if self.initial_speed:
            self.set_speed_from_command(self.initial_speed)

    def get_status(self, eventtime):
        tachometer_status = self.tachometer.get_status(eventtime)
        return {
            "power": self.last_pwm_value,
            "value": self.last_fan_value,
            "speed": self.last_fan_value * self.max_power,
            "rpm": tachometer_status["rpm"],
        }


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

    def get_status(self, eventtime):
        if self._freq_counter is not None:
            rpm = self._freq_counter.get_frequency() * 30.0 / self.ppr
        else:
            rpm = None
        return {"rpm": rpm}

def load_config(config):
    cflap = CFlap(config)
    config.get_printer().add_object("fan", cflap)
    return cflap
