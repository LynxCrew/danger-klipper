# Support generic temperature sensors
#
# Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

KELVIN_TO_CELSIUS = -273.15


class PrinterSensorGeneric:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.full_name = config.get_name()
        self.name = self.full_name.split()[-1]
        self.mcu = self._parse_mcu(config)
        pheaters = self.printer.load_object(config, "heaters")
        self.sensor = pheaters.setup_sensor(config)
        self.min_temp = config.getfloat(
            "min_temp", KELVIN_TO_CELSIUS, minval=KELVIN_TO_CELSIUS
        )
        self.max_temp = config.getfloat(
            "max_temp", 99999999.9, above=self.min_temp
        )
        if hasattr(self.sensor, "set_report_time"):
            report_time = config.getfloat("report_time", None)
            if report_time is not None:
                self.sensor.set_report_time(report_time)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        pheaters.register_sensor(config, self)
        self.last_temp = 0.0
        self.measured_min = 99999999.0
        self.measured_max = -99999999.0
        self.initialized = False

    def _parse_mcu(self, config):
        pin = config.get("pin", None)
        if pin is None:
            sensor_type = config.get("sensor_type")
            if sensor_type == "temperature_host":
                return DummyMCU()
            if sensor_type == "temperature_mcu":
                return self.printer.lookup_object(("mcu " + config.get("sensor_mcu", "")).strip())
            if sensor_type == "temperature_driver":
                return self.printer.lookup_object(config.get("sensor_driver")).get_mcu()
        if ":" not in pin:
            return self.printer.lookup_object("mcu")
        else:
            return self.printer.lookup_object(("mcu " + pin.split(":", 1)[0].strip()).strip())

    def get_mcu(self):
        return self.mcu

    def temperature_callback(self, read_time, temp):
        self.last_temp = temp
        self.initialized = True
        if temp:
            self.measured_min = min(self.measured_min, temp)
            self.measured_max = max(self.measured_max, temp)

    def get_temp(self, eventtime):
        return self.last_temp, 0.0

    def stats(self, eventtime):
        return False, "%s: temp=%.1f" % (self.name, self.last_temp)

    def get_status(self, eventtime):
        return {
            "temperature": round(self.last_temp, 2),
            "measured_min_temp": round(self.measured_min, 2),
            "measured_max_temp": round(self.measured_max, 2),
        }

    def is_adc_faulty(self):
        if self.last_temp > self.max_temp or self.last_temp < self.min_temp:
            return True
        return False

class DummyMCU:
    def __init__(self):
        self.non_critical_disconnected = False



def load_config_prefix(config):
    return PrinterSensorGeneric(config)
