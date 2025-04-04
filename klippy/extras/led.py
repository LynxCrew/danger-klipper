# Support for PWM driven LEDs
#
# Copyright (C) 2019-2022  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import output_pin


# Time between each led template update
RENDER_TIME = 0.500


# Helper code for common LED initialization and control


class LEDHelper:
    def __init__(self, config, update_func, led_count=1):
        self.printer = config.get_printer()
        self.update_func = update_func
        self.led_count = led_count
        self.need_transmit = False
        # Initial color
        red = config.getfloat("initial_RED", 0.0, minval=0.0, maxval=1.0)
        green = config.getfloat("initial_GREEN", 0.0, minval=0.0, maxval=1.0)
        blue = config.getfloat("initial_BLUE", 0.0, minval=0.0, maxval=1.0)
        white = config.getfloat("initial_WHITE", 0.0, minval=0.0, maxval=1.0)
        self.led_state = [(red, green, blue, white)] * led_count
        self.active_template = None
        # Support setting an led template
        self.template_eval = output_pin.lookup_template_eval(config)
        self.tcallbacks = [
            (
                (lambda text, s=self, index=i: s._template_update(index, text)),
                self._check_transmit,
                (lambda color, index=i: self._set_color(index, color)),
            )
            for i in range(1, led_count + 1)
        ]
        # Register commands
        self.name = config.get_name().split()[-1]
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_LED",
            "LED",
            self.name,
            self.cmd_SET_LED,
            desc=self.cmd_SET_LED_help,
        )
        gcode.register_mux_command(
            "SET_LED_TEMPLATE",
            "LED",
            self.name,
            self.cmd_SET_LED_TEMPLATE,
            desc=self.cmd_SET_LED_TEMPLATE_help,
        )

    def check_index(self, index, gcmd, led_count):
        try:
            i = int(index)
        except ValueError:
            raise gcmd.error(
                "index '%s' is not a number, "
                "only numbers, ',', '-' and '|' are allowed." % index
            )
        if i < 1:
            raise gcmd.error("index can not be less than 1(was '%d')" % i)
        if i > led_count:
            raise gcmd.error(
                "index can not exceed amount of leds in chain(was '%d')" % i
            )
        return i

    def check_step(self, step, min, max, gcmd):
        try:
            i = int(step)
        except ValueError:
            raise gcmd.error(
                "step '%s' is not a number, only numbers are allowed." % step
            )
        if i < 1:
            raise gcmd.error("step can not be less than 1(was '%d')" % i)
        if i > max - min:
            raise gcmd.error(
                "Steps can not be bigger than range (was '%d')" % i
            )
        return i

    def get_indices(self, gcmd, led_count):
        given_indices = gcmd.get("INDEX", None)
        if given_indices is None:
            return range(1, (led_count + 1))
        indices = set()
        for index in given_indices.split(","):
            led_range = index.split("-")
            if len(led_range) > 2:
                raise gcmd.error(
                    "More than one '-' found in '%s', only one allowed" % index
                )
            elif len(led_range) == 1:
                if "|" in led_range[0]:
                    raise gcmd.error(
                        "'|' specified without preceding range in '%s'" % index
                    )
                indices.add(self.check_index(index, gcmd, led_count))
            else:
                step = 1
                min_val = led_range[0]
                max_val = led_range[1]
                range_steps = max_val.split("|")
                if len(range_steps) > 2:
                    raise gcmd.error(
                        "More than one '|' found in '%s', "
                        "only one allowed" % index
                    )
                elif len(range_steps) == 2:
                    step = range_steps[1]
                    max_val = range_steps[0]
                min = self.check_index(min_val, gcmd, led_count)
                max = self.check_index(max_val, gcmd, led_count)
                if max < min:
                    raise gcmd.error(
                        "Min value greater than max value in '%s'" % index
                    )
                for i in range(
                    min, (max + 1), self.check_step(step, min, max, gcmd)
                ):
                    indices.add(i)
        return indices

    def get_status(self, eventtime=None):
        return {
            "color_data": self.led_state,
            "active_template": self.active_template,
        }

    def _set_color(self, index, color):
        if index is None:
            new_led_state = [color] * self.led_count
            if self.led_state == new_led_state:
                return
        else:
            if self.led_state[index - 1] == color:
                return
            new_led_state = list(self.led_state)
            new_led_state[index - 1] = color
        self.led_state = new_led_state
        self.need_transmit = True

    def _template_update(self, index, text):
        try:
            parts = [max(0.0, min(1.0, float(f))) for f in text.split(",", 4)]
        except ValueError as e:
            logging.exception("led template render error")
            parts = []
        if len(parts) < 4:
            parts += [0.0] * (4 - len(parts))
        self._set_color(index, tuple(parts))

    def _check_transmit(self, print_time=None):
        if not self.need_transmit:
            return
        self.need_transmit = False
        try:
            self.update_func(self.led_state, print_time)
        except self.printer.command_error as e:
            logging.exception("led update transmit error")

    cmd_SET_LED_help = "Set the color of an LED"

    def cmd_SET_LED(self, gcmd):
        # Parse parameters
        red = gcmd.get_float("RED", 0.0, minval=0.0, maxval=1.0)
        green = gcmd.get_float("GREEN", 0.0, minval=0.0, maxval=1.0)
        blue = gcmd.get_float("BLUE", 0.0, minval=0.0, maxval=1.0)
        white = gcmd.get_float("WHITE", 0.0, minval=0.0, maxval=1.0)
        disable_template = gcmd.get_int(
            "DISABLE_TEMPLATE", 0, minval=0, maxval=1
        )
        transmit = gcmd.get_int("TRANSMIT", 1)
        sync = gcmd.get_int("SYNC", 1)
        color = (red, green, blue, white)

        # Update and transmit data

        indices = self.get_indices(gcmd, self.led_count)

        if disable_template:
            color_callbacks = set()
            flush_callbacks = set()
            callback_delay = 0

            def lookahead_bgfunc(print_time):
                for cb in color_callbacks:
                    cb(color)
                if transmit:
                    for cb in flush_callbacks:
                        # noinspection PyArgumentList
                        cb(print_time)

            for index in indices:
                callback, flush_callback, set_color = self.tcallbacks[index - 1]
                if callback in self.template_eval.active_templates:
                    del self.template_eval.active_templates[callback]
                    callback_delay = output_pin.RENDER_TIME
                color_callbacks.add(set_color)
                flush_callbacks.add(flush_callback)

            if sync:
                reactor = self.printer.get_reactor()
                toolhead = self.printer.lookup_object("toolhead")
                reactor.register_callback(
                    lambda _: toolhead.register_lookahead_callback(
                        (lambda pt: lookahead_bgfunc(pt))
                    ),
                    reactor.monotonic() + callback_delay,
                )
            else:
                lookahead_bgfunc(None)
            return

        def lookahead_bgfunc(print_time):
            for index in indices:
                self._set_color(index, color)
            if transmit:
                self._check_transmit(print_time)

        if sync:
            # Sync LED Update with print time and send
            toolhead = self.printer.lookup_object("toolhead")
            toolhead.register_lookahead_callback(lookahead_bgfunc)
        else:
            # Send update now (so as not to wake toolhead and reset idle_timeout)
            lookahead_bgfunc(None)

    cmd_SET_LED_TEMPLATE_help = "Assign a display_template to an LED"

    def cmd_SET_LED_TEMPLATE(self, gcmd):
        def lookahead_bgfunc(print_time, callbacks):
            for cb in callbacks:
                cb(print_time)

        sync = gcmd.get_int("SYNC", 1)

        flush_callbacks = set()
        set_template = self.template_eval._set_template
        tpl_name = None
        for index in self.get_indices(gcmd, self.led_count):
            callback, flush_callback, set_color = self.tcallbacks[index - 1]
            tpl_name = set_template(gcmd, callback, flush_callback)
            if tpl_name == "":
                set_color((0, 0, 0, 0))
                flush_callbacks.add(flush_callback)
        self.template_eval._activate_timer()
        self.active_template = tpl_name
        if sync:
            toolhead = self.printer.lookup_object("toolhead")
            toolhead.register_lookahead_callback(
                (lambda pt: lookahead_bgfunc(pt, flush_callbacks))
            )
        else:
            lookahead_bgfunc(None, flush_callbacks)


PIN_MIN_TIME = 0.100
MAX_SCHEDULE_TIME = 5.0


# Handler for PWM controlled LEDs


class PrinterPWMLED:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        # Configure pwm pins
        ppins = printer.lookup_object("pins")
        cycle_time = config.getfloat(
            "cycle_time", 0.010, above=0.0, maxval=MAX_SCHEDULE_TIME
        )
        hardware_pwm = config.getboolean("hardware_pwm", False)
        self.pins = []
        for i, name in enumerate(("red", "green", "blue", "white")):
            pin_name = config.get(name + "_pin", None)
            if pin_name is None:
                continue
            mcu_pin = ppins.setup_pin("pwm", pin_name)
            mcu_pin.setup_max_duration(0.0)
            mcu_pin.setup_cycle_time(cycle_time, hardware_pwm)
            self.pins.append((i, mcu_pin))
        if not self.pins:
            raise config.error(
                "No LED pin definitions found in '%s'" % (config.get_name(),)
            )
        self.last_print_time = 0.0
        # Initialize color data
        self.led_helper = LEDHelper(config, self.update_leds, 1)
        self.prev_color = color = self.led_helper.get_status()["color_data"][0]
        for idx, mcu_pin in self.pins:
            mcu_pin.setup_start_value(color[idx], 0.0)

    def update_leds(self, led_state, print_time):
        if print_time is None:
            eventtime = self.printer.get_reactor().monotonic()
            mcu = self.pins[0][1].get_mcu()
            print_time = mcu.estimated_print_time(eventtime) + PIN_MIN_TIME
        print_time = max(print_time, self.last_print_time + PIN_MIN_TIME)
        color = led_state[0]
        for idx, mcu_pin in self.pins:
            if self.prev_color[idx] != color[idx]:
                mcu_pin.set_pwm(print_time, color[idx])
                self.last_print_time = print_time
        self.prev_color = color

    def get_status(self, eventtime=None):
        return self.led_helper.get_status(eventtime)


def load_config_prefix(config):
    return PrinterPWMLED(config)
