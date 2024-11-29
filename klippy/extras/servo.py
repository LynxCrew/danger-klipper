# Support for servos
#
# Copyright (C) 2017-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import output_pin

SERVO_SIGNAL_PERIOD = 0.020


class PrinterServo:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.min_width = config.getfloat(
            "minimum_pulse_width", 0.001, above=0.0, below=SERVO_SIGNAL_PERIOD
        )
        self.max_width = config.getfloat(
            "maximum_pulse_width",
            0.002,
            above=self.min_width,
            below=SERVO_SIGNAL_PERIOD,
        )
        self.max_angle = config.getfloat("maximum_servo_angle", 180.0)
        self.angle_to_width = (self.max_width - self.min_width) / self.max_angle
        self.width_to_value = 1.0 / SERVO_SIGNAL_PERIOD
        self.last_value = 0.0
        initial_pwm = 0.0
        iangle = config.getfloat(
            "initial_angle", None, minval=0.0, maxval=360.0
        )
        if iangle is not None:
            initial_pwm = self._get_pwm_from_angle(iangle)
        else:
            iwidth = config.getfloat(
                "initial_pulse_width", 0.0, minval=0.0, maxval=self.max_width
            )
            initial_pwm = self._get_pwm_from_pulse_width(iwidth)
        # Setup mcu_servo pin
        ppins = self.printer.lookup_object("pins")
        self.mcu_servo = ppins.setup_pin("pwm", config.get("pin"))
        self.mcu_servo.setup_max_duration(0.0)
        self.mcu_servo.setup_cycle_time(SERVO_SIGNAL_PERIOD)
        self.mcu_servo.setup_start_value(initial_pwm, 0.0)
        # Create gcode request queue
        self.gcrq = output_pin.GCodeRequestQueue(
            config, self.mcu_servo.get_mcu(), self._set_pwm
        )
        self.template_eval = output_pin.lookup_template_eval(config)
        # Register commands
        servo_name = config.get_name().split()[1]
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_SERVO",
            "SERVO",
            servo_name,
            self.cmd_SET_SERVO,
            desc=self.cmd_SET_SERVO_help,
        )

    def get_status(self, eventtime=None):
        return {"value": self.last_value}

    def _template_update(self, text):
        try:
            type, value = [t.upper().strip() for t in text.split(":", 2)]
            if type == "ANGLE":
                # Transmit pending changes
                angle = float(value.strip())
                self.gcrq.send_async_request(self._get_pwm_from_angle(angle))
            elif type == "WIDTH":
                width = float(value.strip())
                self.gcrq.send_async_request(
                    self._get_pwm_from_pulse_width(width)
                )
        except ValueError as e:
            logging.exception("servo template render error")

    def _set_pwm(self, print_time, value):
        if value == self.last_value:
            return "discard", 0.0
        self.last_value = value
        self.mcu_servo.set_pwm(print_time, value)

    def _get_pwm_from_angle(self, angle):
        angle = max(0.0, min(self.max_angle, angle))
        width = self.min_width + angle * self.angle_to_width
        return width * self.width_to_value

    def _get_pwm_from_pulse_width(self, width):
        if width:
            width = max(self.min_width, min(self.max_width, width))
        return width * self.width_to_value

    cmd_SET_SERVO_help = "Set servo angle"

    def cmd_SET_SERVO(self, gcmd):
        width = gcmd.get_float("WIDTH", None)
        angle = gcmd.get_float("ANGLE", None)
        template = gcmd.get("TEMPLATE", None)
        if (width is None) == (angle is None) == (template is None):
            raise gcmd.error("SET_SERVO must specify WIDTH, ANGLE or TEMPLATE")
        if template is not None:
            self.template_eval.set_template(gcmd, self._template_update)
            return
        if width is not None:
            value = self._get_pwm_from_pulse_width(width)
        else:
            value = self._get_pwm_from_angle(angle)
        self.gcrq.queue_gcode_request(value)


def load_config_prefix(config):
    return PrinterServo(config)
