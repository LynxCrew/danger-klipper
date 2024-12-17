# Filament Motion Sensor Module
#
# Copyright (C) 2021 Joshua Wherrett <thejoshw.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import filament_switch_sensor


class EncoderSensor:
    def __init__(self, config):
        # Read config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        switch_pin = config.get("switch_pin")
        self.extruder_name = config.get("extruder")
        self.detection_length = config.getfloat(
            "detection_length", 7.0, above=0.0
        )
        self.keep_enabled = config.getboolean("keep_enabled", False)
        # Configure pins
        buttons = self.printer.load_object(config, "buttons")
        buttons.register_buttons([switch_pin], self.encoder_event)
        # Get printer objects
        self.reactor = self.printer.get_reactor()
        self.runout_helper = filament_switch_sensor.RunoutHelper(config, self)
        self.get_status = self.runout_helper.get_status
        self.extruder = None
        self.estimated_print_time = None
        # Initialise internal state
        self.filament_runout_pos = None
        # Register commands and event handlers
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        self.printer.register_event_handler(
            "print_stats:start_printing", self._handle_printing_smart
        )
        self.printer.register_event_handler(
            "print_stats:complete_printing", self._handle_not_printing_smart
        )
        self.printer.register_event_handler(
            "print_stats:cancelled_printing",
            self._handle_not_printing_smart,
        )
        self.printer.register_event_handler(
            "print_stats:paused_printing", self._handle_not_printing_smart
        )

        self.printer.register_event_handler(
            "idle_timeout:printing", self._handle_printing
        )
        self.printer.register_event_handler(
            "idle_timeout:ready", self._handle_not_printing
        )
        self.printer.register_event_handler(
            "idle_timeout:idle", self._handle_not_printing
        )

    def _update_filament_runout_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        self.filament_runout_pos = (
            self.get_extruder_pos(eventtime) + self.detection_length
        )

    def _handle_ready(self):
        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.estimated_print_time = self.printer.lookup_object(
            "mcu"
        ).estimated_print_time
        self._update_filament_runout_pos()
        self._extruder_pos_update_timer = self.reactor.register_timer(
            self._extruder_pos_update_event
        )
        self.runout_helper.note_filament_present(True)

    def _handle_printing(self, *args):
        if not self.runout_helper.smart:
            self.reset()
            self.reactor.update_timer(
                self._extruder_pos_update_timer, self.reactor.NOW
            )

    def _handle_printing_smart(self, *args):
        if self.runout_helper.smart:
            self.reset()
            self.reactor.update_timer(
                self._extruder_pos_update_timer, self.reactor.NOW
            )

    def _handle_not_printing(self, *args):
        if not self.runout_helper.smart:
            self.reactor.update_timer(
                self._extruder_pos_update_timer, self.reactor.NEVER
            )

    def _handle_not_printing_smart(self, *args):
        if self.runout_helper.smart:
            self.reactor.update_timer(
                self._extruder_pos_update_timer, self.reactor.NEVER
            )

    def get_extruder_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        print_time = self.estimated_print_time(eventtime)
        return self.extruder.find_past_position(print_time)

    def _extruder_pos_update_event(self, eventtime):
        extruder_pos = self.get_extruder_pos(eventtime)
        # Check for filament runout
        self.runout_helper.note_filament_present(
            extruder_pos < self.filament_runout_pos
        )
        return eventtime + self.runout_helper.check_runout_timeout

    def encoder_event(self, eventtime, state):
        if self.extruder is not None:
            self._update_filament_runout_pos(eventtime)
            # Check for filament insertion
            # Filament is always assumed to be present on an encoder event
            self.runout_helper.note_filament_present(True)

    def get_sensor_status(self):
        return (
            "Filament Sensor %s: %s\n"
            "Filament Detected: %s\n"
            "Detection Length: %.2f\n"
            "Smart: %s\n"
            "Keep enabled: %s\n"
            "Always Fire Events: %s"
            % (
                self.runout_helper.name,
                "enabled" if self.runout_helper.sensor_enabled else "disabled",
                "true" if self.runout_helper.filament_present else "false",
                self.detection_length,
                "true" if self.runout_helper.smart else "false",
                "true" if self.keep_enabled else "false",
                "true" if self.runout_helper.always_fire_events else "false",
            )
        )

    def sensor_get_status(self, eventtime):
        return {"detection_length": float(self.detection_length)}

    def get_info(self, gcmd):
        keep_enabled = gcmd.get_int("KEEP_ENABLED", None, minval=0, maxval=1)
        detection_length = gcmd.get_float("DETECTION_LENGTH", None, minval=0.0)
        if keep_enabled is None and detection_length is None:
            gcmd.respond_info(self.get_sensor_status())
            return True
        return False

    def reset_needed(self, enable=None, always_fire_events=None):
        if enable and not self.runout_helper.sensor_enabled:
            return True
        return False

    def set_filament_sensor(self, gcmd):
        reset_needed = False
        keep_enabled = gcmd.get_int("KEEP_ENABLED", None, minval=0, maxval=1)
        detection_length = gcmd.get_float("DETECTION_LENGTH", None, minval=0.0)
        if detection_length is not None:
            if detection_length != self.detection_length:
                reset_needed = True
            self.detection_length = detection_length
        if keep_enabled is not None:
            if keep_enabled and not self.keep_enabled:
                reset_needed = True
            self.keep_enabled = keep_enabled
        return reset_needed

    def reset(self):
        self._update_filament_runout_pos()
        self.runout_helper.note_filament_present(True)


def load_config_prefix(config):
    return EncoderSensor(config)
