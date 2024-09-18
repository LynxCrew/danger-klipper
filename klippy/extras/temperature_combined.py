# -*- coding: utf-8 -*-
# Support for combination of temperature sensors
#
# Copyright (C) 2023  Michael Jäger <michael@mjaeger.eu>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import threading
import time

from extras.danger_options import get_danger_options

REPORT_TIME = 0.300


class PrinterSensorCombined:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.klipper_threads = self.printer.get_klipper_threads()
        self.name = config.get_name().split()[-1]
        # get sensor names
        self.sensor_names = config.getlist("sensor_list")
        # get maximum_deviation parameter from config
        self.max_deviation = config.getfloat(
            "maximum_deviation", default=None, above=0.0
        )
        self.ignore = (
            self.name in get_danger_options().temp_ignore_limits
        ) or self.max_deviation is None
        # ensure compatibility with itself
        self.sensor = self
        # get empty list for sensors, could be any sensor class or a heater
        self.sensors = []
        # get combination method to handle the different sensor values with
        algos = {"min": min, "max": max, "mean": mean}
        self.apply_mode = config.getchoice("combination_method", algos)
        # set default values
        self.last_temp = self.min_temp = self.max_temp = 0.0
        self.measured_min = 99999999.0
        self.measured_max = -99999999.0
        self.temperature_callback = None
        # add object
        self.printer.add_object("temperature_combined " + self.name, self)
        # time-controlled sensor update
        self.initialized = False

        self.temperature_sample_thread = self.klipper_threads.register_job(
            target=self._temperature_update_event
        )

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_connect(self):
        for sensor_name in self.sensor_names:
            sensor = self.printer.lookup_object(sensor_name)
            # check if sensor has get_status function and
            # get_status has a 'temperature' value
            if hasattr(sensor, "get_status") and "temperature" in sensor.get_status(
                self.reactor.monotonic()
            ):
                self.sensors.append(sensor)
            else:
                raise self.printer.config_error(
                    "'%s' does not report a temperature." % (sensor_name,)
                )

    def _handle_ready(self):
        # Start temperature update timer
        self.temperature_sample_thread.start()

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback

    def get_report_time_delta(self):
        return REPORT_TIME

    def update_temp(self, eventtime):
        if not self.initialized:
            initialized = True
            for sensor in self.sensors:
                if hasattr(sensor, "initialized") and not sensor.initialized:
                    initialized = False
            self.initialized = initialized
        values = []
        for sensor in self.sensors:
            if not hasattr(sensor, "initialized") or sensor.initialized:
                sensor_temperature = sensor.get_status(eventtime)["temperature"]
                if sensor_temperature is not None:
                    values.append(sensor_temperature)

        if values:
            # check if values are out of max_deviation range
            if not self.ignore and (max(values) - min(values)) > self.max_deviation:
                self.printer.invoke_shutdown(
                    "[%s]\n"
                    "Maximum deviation exceeded limit of %0.1f, "
                    "max sensor value %0.1f, min sensor value %0.1f."
                    % (
                        self.name,
                        self.max_deviation,
                        max(values),
                        min(values),
                    )
                )

            temp = self.apply_mode(values)
            if temp is not None:
                self.last_temp = temp
                self.measured_min = min(self.measured_min, temp)
                self.measured_max = max(self.measured_max, temp)

    def get_temp(self, eventtime):
        return self.last_temp, 0.0

    def get_status(self, eventtime):
        return {
            "temperature": round(self.last_temp, 2),
            "measured_min_temp": round(self.measured_min, 2),
            "measured_max_temp": round(self.measured_max, 2),
        }

    def _temperature_update_event(self):
        eventtime = self.reactor.monotonic()
        # update sensor value
        self.update_temp(eventtime)

        if not self.ignore:
            # check min / max temp values
            if self.initialized and self.last_temp < self.min_temp:
                self.printer.invoke_shutdown(
                    "COMBINED SENSOR temperature %0.1f "
                    "below minimum temperature of %0.1f."
                    % (
                        self.last_temp,
                        self.min_temp,
                    )
                )
            if self.initialized and self.last_temp > self.max_temp:
                self.printer.invoke_shutdown(
                    "COMBINED SENSOR temperature %0.1f "
                    "above maximum temperature of %0.1f."
                    % (
                        self.last_temp,
                        self.max_temp,
                    )
                )

        # this is copied from temperature_host to enable time triggered updates
        # get mcu and measured / current(?) time
        mcu = self.printer.lookup_object("mcu")
        # convert to print time?! for the callback???
        self.temperature_callback(mcu.estimated_print_time(eventtime), self.last_temp)
        # set next update time
        return REPORT_TIME


def mean(values):
    if not values:
        return None
    return sum(values) / len(values)


def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("temperature_combined", PrinterSensorCombined)
