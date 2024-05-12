BLOCK_REPORT_TIME = 1.0


class MPC_BLOCK_TEMP_WRAPPER:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = "beacon_coil"

        self.heater = None

        self.heater_name = config.get("heater_name")

        self.temperature_callback = None

        self.report_time = BLOCK_REPORT_TIME

        self.temp = self.min_temp = self.max_temp = 0.0

        self.reactor = self.printer.get_reactor()
        self.sample_timer = self.reactor.register_timer(
            self._sample_block_temperature
        )

        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_ready(self):
        pheaters = self.printer.lookup_object("heaters")
        self.heater = pheaters.lookup_heater(self.heater_name)
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def get_report_time_delta(self):
        return self.report_time

    def _sample_block_temperature(self, eventtime):
        if self.heater.get_control().get_type() == "mpc":
            self.temp = self.heater.get_control().state_block_temp
        else:
            self.temp = None

        if self.temp is not None:
            if self.temp < self.min_temp or self.temp > self.max_temp:
                self.printer.invoke_shutdown(
                    "DRIVER [%s] temperature %0.1f outside range of %0.1f:%.01f"
                    % (self.name, self.temp, self.min_temp, self.max_temp)
                )
        else:
            self.temp = 0.0

        measured_time = self.reactor.monotonic()

        self.temperature_callback(
            self.printer.lookup_object("mcu").estimated_print_time(
                measured_time
            ),
            self.temp,
        )

        return measured_time + self.report_time

    def set_report_time(self, report_time):
        self.report_time = report_time

def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("block_temperature", MPC_BLOCK_TEMP_WRAPPER)
