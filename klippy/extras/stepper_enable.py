# Support for enable pins on stepper motor drivers
#
# Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import chelper

MIN_ENABLE_TIME = 2.0
MAX_ENABLE_TIME = 60.0
DISABLE_STALL_TIME = 0.100


# Tracking of shared stepper enable pins


class StepperEnablePin:
    def __init__(
        self,
        set_enable_pin,
        enable_count,
        printer,
    ):
        self.printer = printer
        self.reactor = self.printer.get_reactor()
        self.set_enable_pin = set_enable_pin
        self.enable_count = enable_count
        self.is_dedicated = True
        self.last_value = 0
        self.resend_timer = None
        self.last_print_time = 0.0

    def set_enable(self, print_time):
        if not self.enable_count and self.set_enable_pin is not None:
            self.set_enable_pin(print_time, 1)
        self.enable_count += 1

    def set_disable(self, print_time):
        self.enable_count -= 1
        if not self.enable_count and self.set_enable_pin is not None:
            toolhead = self.printer.lookup_object("toolhead")
            toolhead.wait_moves()
            toolhead.dwell(DISABLE_STALL_TIME)
            toolhead.register_lookahead_callback(lambda pt: self.set_enable_pin(pt, 0))


class error(Exception):
    pass


class StepperEnableOutputPin:
    def __init__(self, pin_params):
        self._mcu = pin_params["chip"]
        self._max_duration = 2.0
        self._oid = self._mcu.create_oid()
        ffi_main, ffi_lib = chelper.get_ffi()
        self._stepqueue = ffi_main.gc(
            ffi_lib.stepcompress_alloc(self._oid), ffi_lib.stepcompress_free
        )
        self._mcu.register_stepqueue(self._stepqueue)
        self._stepcompress_queue_mq_msg = ffi_lib.stepcompress_queue_mq_msg
        self._mcu.register_config_callback(self._build_config)
        self._pin = pin_params["pin"]
        self._invert = pin_params["invert"]
        self._last_clock = self._last_value = self._default_value = 0
        self._duration_ticks = 0
        self._set_cmd_tag = None
        self._toolhead = None
        self.is_dedicated = True
        printer = self._mcu.get_printer()
        printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        self._toolhead = self._mcu.get_printer().lookup_object("toolhead")

    def get_mcu(self):
        return self._mcu

    def setup_max_duration(self, max_duration):
        self._max_duration = max_duration

    def _build_config(self):
        config_error = self._mcu.get_printer().config_error
        self._default_value = self._last_value = 1.0 if self._invert else 0.0
        cmd_queue = self._mcu.alloc_command_queue()
        curtime = self._mcu.get_printer().get_reactor().monotonic()
        printtime = self._mcu.estimated_print_time(curtime)
        self._last_clock = self._mcu.print_time_to_clock(printtime + 0.200)
        self._duration_ticks = self._mcu.seconds_to_clock(self._max_duration)
        if self._duration_ticks >= 1 << 31:
            raise config_error("PWM pin max duration too large")
        if self._duration_ticks:
            self._mcu.register_flush_callback(self._flush_notification)
        self._mcu.add_config_cmd(
            "config_digital_out oid=%d pin=%s value=%d"
            " default_value=%d max_duration=%d"
            % (
                self._oid,
                self._pin,
                self._default_value,
                self._default_value,
                self._duration_ticks,
            )
        )
        self._mcu.add_config_cmd(
            "queue_digital_out oid=%d clock=%d on_ticks=%d"
            % (self._oid, self._last_clock, self._last_value),
            is_init=True,
        )
        self._set_cmd_tag = self._mcu.lookup_command(
            "queue_digital_out oid=%c clock=%u on_ticks=%u", cq=cmd_queue
        ).get_command_tag()

    def _send_update(self, clock, val):
        self._last_clock = clock = max(self._last_clock, clock)
        self._last_value = val
        data = (self._set_cmd_tag, self._oid, clock & 0xFFFFFFFF, val)
        ret = self._stepcompress_queue_mq_msg(self._stepqueue, clock, data, len(data))
        if ret:
            raise error("Internal error in stepcompress")
        # Notify toolhead so that it will flush this update
        wakeclock = clock
        if self._last_value != self._default_value:
            # Continue flushing to resend time
            wakeclock += self._duration_ticks
        wake_print_time = self._mcu.clock_to_print_time(wakeclock)
        self._toolhead.note_mcu_movequeue_activity(wake_print_time)

    def set_pin(self, print_time, value):
        clock = self._mcu.print_time_to_clock(print_time)
        if self._invert:
            value = 1.0 - value
        v = int(max(0.0, min(1.0, value)))
        self._send_update(clock, v)

    def _flush_notification(self, print_time, clock):
        if self._last_value != self._default_value:
            while clock >= self._last_clock + self._duration_ticks:
                self._send_update(
                    self._last_clock + self._duration_ticks, self._last_value
                )


def setup_enable_pin(printer, pin, max_enable_time=0.0):
    if pin is None:
        # No enable line (stepper always enabled)
        enable = StepperEnablePin(None, 9999, printer)
        enable.is_dedicated = False
        return enable
    ppins = printer.lookup_object("pins")
    pin_params = ppins.lookup_pin(pin, can_invert=True, share_type="stepper_enable")
    enable = pin_params.get("class")
    if enable is not None:
        # Shared enable line
        enable.is_dedicated = False
        return enable
    if max_enable_time:
        mcu_enable = StepperEnableOutputPin(pin_params)
        set_enable_pin = mcu_enable.set_pin
    else:
        mcu_enable = pin_params["chip"].setup_pin("digital_out", pin_params)
        set_enable_pin = mcu_enable.set_digital

    mcu_enable.setup_max_duration(max_enable_time)
    enable = pin_params["class"] = StepperEnablePin(set_enable_pin, 0, printer)
    return enable


# Enable line tracking for each stepper motor
class EnableTracking:
    def __init__(self, stepper, enable):
        self.stepper = stepper
        self.enable = enable
        self.callbacks = []
        self.is_enabled = False
        self.stepper.add_active_callback(self.motor_enable)

    def register_state_callback(self, callback):
        self.callbacks.append(callback)

    def motor_enable(self, print_time):
        if not self.is_enabled:
            for cb in self.callbacks:
                cb(print_time, True)
            self.enable.set_enable(print_time)
            self.is_enabled = True

    def motor_disable(self, print_time):
        if self.is_enabled:
            # Enable stepper on future stepper movement
            for cb in self.callbacks:
                cb(print_time, False)
            self.enable.set_disable(print_time)
            self.is_enabled = False
            self.stepper.add_active_callback(self.motor_enable)

    def is_motor_enabled(self):
        return self.is_enabled

    def has_dedicated_enable(self):
        return self.enable.is_dedicated


# Global stepper enable line tracking
class PrinterStepperEnable:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.enable_lines = {}
        self.kinematics = config.getsection("printer").get("kinematics")
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )
        # Register M18/M84 commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("M18", self.cmd_M18)
        gcode.register_command("M84", self.cmd_M18)
        gcode.register_command(
            "SET_STEPPER_ENABLE",
            self.cmd_SET_STEPPER_ENABLE,
            desc=self.cmd_SET_STEPPER_ENABLE_help,
        )

    def register_stepper(self, config, mcu_stepper):
        name = mcu_stepper.get_name()
        max_enable_time = config.getfloat(
            "max_enable_time", 0.0, minval=MIN_ENABLE_TIME, maxval=MAX_ENABLE_TIME
        )
        disable_on_error = config.getboolean("disable_on_error", False)
        if disable_on_error:
            config.deprecate("disable_on_error")
            if not max_enable_time:
                max_enable_time = 5.0
        enable = setup_enable_pin(
            self.printer,
            config.get("enable_pin", None),
            max_enable_time,
        )
        self.enable_lines[name] = EnableTracking(mcu_stepper, enable)

    def stepper_off(self, stepper_name, print_time, rail_name):
        el = self.enable_lines[stepper_name]
        el.motor_disable(print_time)
        if rail_name != "extruder":
            self.printer.send_event(
                "stepper_enable:disable_%s" % rail_name.lower(), print_time
            )

    def motor_off(self):
        self.axes_off()

    def axes_off(self, axes=None):
        if axes is None:
            axes = [0, 1, 2, 3]
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(DISABLE_STALL_TIME)
        print_time = toolhead.get_last_move_time()
        kin = toolhead.get_kinematics()
        if 3 in axes:
            if "extruder" in self.enable_lines:
                self.stepper_off("extruder", print_time, "extruder")
                i = 1
                extruder_name = f"extruder{i}"
                while extruder_name in self.enable_lines:
                    self.stepper_off(extruder_name, print_time, "extruder")
                    i += 1
                    extruder_name = f"extruder{i}"
        if hasattr(kin, "get_connected_rails"):
            for axis in axes:
                try:
                    rails = kin.get_connected_rails(axis)
                    for rail in rails:
                        steppers = rail.get_steppers()
                        rail_name = rail.mcu_stepper.get_name(True)
                        for stepper in steppers:
                            self.stepper_off(stepper.get_name(), print_time, rail_name)
                except IndexError:
                    continue
        else:
            if 0 in axes or 1 in axes or 2 in axes:
                for axis_name, el in self.enable_lines.items():
                    if not axis_name.startswith("extruder"):
                        el.motor_disable(print_time)
                self.printer.send_event("stepper_enable:motor_off", print_time)
        self.printer.send_event("stepper_enable:axes_off", print_time)
        toolhead.dwell(DISABLE_STALL_TIME)

    def motor_debug_enable(self, steppers, enable, notify=True):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(DISABLE_STALL_TIME)
        print_time = toolhead.get_last_move_time()
        kin = toolhead.get_kinematics()
        if not hasattr(kin, "get_rails"):
            notify = False
        for stepper_name in steppers:
            el = self.enable_lines[stepper_name]
            if enable:
                el.motor_enable(print_time)
            else:
                el.motor_disable(print_time)
                if notify:
                    for rail in kin.get_rails():
                        for stepper in rail.get_steppers():
                            if stepper.get_name() == stepper_name:
                                self.printer.send_event(
                                    "stepper_enable:disable_%s"
                                    % rail.mcu_stepper.get_name(True).lower(),
                                    print_time,
                                )
        logging.info(
            "%s have been manually %s",
            steppers,
            "enabled" if enable else "disabled",
        )
        toolhead.dwell(DISABLE_STALL_TIME)

    def get_status(self, eventtime):
        steppers = {
            name: et.is_motor_enabled() for (name, et) in self.enable_lines.items()
        }
        return {"steppers": steppers}

    def _handle_request_restart(self, print_time):
        self.motor_off()

    def cmd_M18(self, gcmd):
        axes = []
        for pos, axis in enumerate("XYZE"):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
        if not axes:
            axes = [0, 1, 2, 3]
        # Turn off motors
        self.axes_off(axes)

    cmd_SET_STEPPER_ENABLE_help = "Enable/disable individual stepper by name"

    def cmd_SET_STEPPER_ENABLE(self, gcmd):
        steppers_str = gcmd.get("STEPPERS", None)
        stepper_enable = gcmd.get_int("ENABLE", 1)
        notify = gcmd.get_int("NOTIFY", 1)
        if steppers_str is None:
            steppers = [None]
            old_stepper_str = gcmd.get("STEPPER", None)
            if old_stepper_str is not None:
                steppers = old_stepper_str.split(",")
                gcmd.respond_info('"STEPPER" parameter is deprecated')
        else:
            steppers = steppers_str.split(",")
        for stepper_name in steppers:
            if stepper_name not in self.enable_lines:
                gcmd.respond_info(
                    'SET_STEPPER_ENABLE: Invalid stepper "%s"' % stepper_name
                )
                return
        self.motor_debug_enable(steppers, stepper_enable, notify)

    def lookup_enable(self, name):
        if name not in self.enable_lines:
            raise self.printer.config_error("Unknown stepper '%s'" % (name,))
        return self.enable_lines[name]

    def get_steppers(self):
        return list(self.enable_lines.keys())


def load_config(config):
    return PrinterStepperEnable(config)
