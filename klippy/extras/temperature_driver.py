# Support for micro-controller chip based temperature sensors
#
# Copyright (C) 2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import threading

DRIVER_REPORT_TIME = 1.0


class PrinterTemperatureDriver:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get("sensor_driver")

        self.driver = None

        self.temp = self.min_temp = self.max_temp = 0.0

        self.report_time = DRIVER_REPORT_TIME

        self.reactor = self.printer.get_reactor()

        self.temperature_sample_thread = threading.Thread(
            target=self._start_sample_timer
        )
        self.temperature_sample_thread.start()

        self.printer.register_event_handler(
            "klippy:connect", self.handle_connect
        )

    def _start_sample_timer(self):
        self.sample_timer = self.reactor.register_timer(
            self._sample_driver_temperature
        )

    def handle_connect(self):
        self.driver = self.printer.lookup_object(self.name)
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def get_report_time_delta(self):
        return self.report_time

    def _sample_driver_temperature(self, eventtime):
        logging.info("TEMPERATURE_DRIVER_UPDATE")
        self.temp = self.driver.get_temperature()

        if self.temp is not None:
            if self.temp < self.min_temp or self.temp > self.max_temp:
                self.printer.invoke_shutdown(
                    "DRIVER [%s] temperature %0.1f outside range of %0.1f:%.01f"
                    % (self.name, self.temp, self.min_temp, self.max_temp)
                )

        measured_time = self.reactor.monotonic()

        if self.temp is None:
            self.temp = 0.0

        mcu = self.driver.get_mcu()
        self.temperature_callback(
            mcu.estimated_print_time(measured_time), self.temp
        )

        return measured_time + self.report_time

    def set_report_time(self, report_time):
        self.report_time = report_time


def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("temperature_driver", PrinterTemperatureDriver)
