# Support for Raspberry Pi temperature sensor
#
# Copyright (C) 2020  Al Crate <al3ph@users.noreply.github.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import time

from .danger_options import get_danger_options

HOST_REPORT_TIME = 1.0
RPI_PROC_TEMP_FILE = "/sys/class/thermal/thermal_zone0/temp"


class Temperature_HOST:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.kalico_threads = self.printer.get_kalico_threads()
        self.full_name = config.get_name()
        self.name = self.full_name.split()[-1]
        self.path = config.get("sensor_path", RPI_PROC_TEMP_FILE)

        self.temp = self.min_temp = self.max_temp = 0.0

        self.report_time = HOST_REPORT_TIME

        self.printer.add_object("temperature_host " + self.name, self)
        if self.printer.get_start_args().get("debugoutput") is not None:
            return

        self.temperature_sample_thread = self.kalico_threads.register_job(
            target=self._sample_pi_temperature
        )
        self.ignore = self.name in get_danger_options().temp_ignore_limits

        try:
            self.file_handle = open(self.path, "r")
        except:
            raise config.error(
                "Unable to open temperature file '%s'" % (self.path,)
            )

        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )

    def _run_sample_timer(self):
        wait_time = self._sample_pi_temperature()
        while wait_time > 0 and not self.printer.is_shutdown():
            time.sleep(wait_time)
            wait_time = self._sample_pi_temperature()

    def _handle_connect(self):
        self.temperature_sample_thread.start()

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def _sample_pi_temperature(self):
        try:
            self.file_handle.seek(0)
            self.temp = float(self.file_handle.read()) / 1000.0
        except Exception:
            logging.exception("temperature_host: Error reading data")
            self.temp = 0.0
            return 0

        if self.temp < self.min_temp or self.temp > self.max_temp:
            if not self.ignore:
                self.printer.invoke_shutdown(
                    "HOST: temperature %0.1f outside range of %0.1f-%.01f"
                    % (self.temp, self.min_temp, self.max_temp)
                )
            elif get_danger_options().echo_limits_to_console:
                gcode = self.printer.lookup_object("gcode")
                gcode.respond_error(
                    "HOST: temperature %0.1f outside range of %0.1f-%.01f"
                    % (self.temp, self.min_temp, self.max_temp)
                )

        mcu = self.printer.lookup_object("mcu")
        measured_time = self.reactor.monotonic()
        self._callback(mcu.estimated_print_time(measured_time), self.temp)
        return self.report_time

    def get_status(self, eventtime):
        return {
            "temperature": round(self.temp, 2),
        }

    def set_report_time(self, report_time):
        if report_time is None:
            return
        self.report_time = report_time


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("temperature_host", Temperature_HOST)
