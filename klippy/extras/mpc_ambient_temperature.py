from .danger_options import get_danger_options
from . import heaters

AMBIENT_REPORT_TIME = 1.0


class MPC_AMBIENT_TEMP_WRAPPER:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]

        self.heater = None

        self.heater_name = config.get("heater_name")

        self.temperature_callback = None

        self.report_time = AMBIENT_REPORT_TIME

        self.temp = self.min_temp = self.max_temp = 0.0

        self.ignore = self.name in get_danger_options().temp_ignore_limits

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        pheaters = self.printer.lookup_object("heaters")
        self.heater = pheaters.lookup_heater(self.heater_name)
        self.heater.add_mpc_sensor(self, "ambient_temperature")

    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def get_report_time_delta(self):
        return self.report_time

    def process_temp_update(self, temperature, read_time):
        self.temp = temperature

        if self.temp is not None:
            if not self.heater.mcu_pwm.get_mcu().non_critical_disconnected and (
                self.temp < self.min_temp or self.temp > self.max_temp
            ):
                if not self.ignore:
                    self.printer.invoke_shutdown(
                        "Ambient MPC %s\nTemperature %0.1f outside range of %0.1f-%.01f"
                        % (self.name, self.temp, self.min_temp, self.max_temp)
                    )
                elif get_danger_options().echo_limits_to_console:
                    gcode = self.printer.lookup_object("gcode")
                    gcode.respond_error(
                        "Ambient MPC %s\nTemperature %0.1f outside range of %0.1f-%.01f"
                        % (self.name, self.temp, self.min_temp, self.max_temp)
                    )
        else:
            self.temp = 0.0

        self.temperature_callback(
            self.printer.lookup_object("mcu").estimated_print_time(
                read_time
            ),
            self.temp,
        )

    def set_report_time(self, report_time):
        if report_time is None:
            return
        self.report_time = report_time


def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory(
        "mpc_ambient_temperature", MPC_AMBIENT_TEMP_WRAPPER
    )
