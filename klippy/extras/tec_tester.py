import random


MAX_HEAT_TIME = 5.0

class TecTester:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.kalico_threads = self.printer.get_kalico_threads()
        self.name = config.get_name().split()[-1]
        self.min_temp_cold_side = config.get_float("min_temp_cold_side", default=20)
        self.min_temp_hot_side = config.get_float("min_temp_hot_side", default=20)
        self.max_temp_cold_side = config.get_float("max_temp_cold_side", default=80)
        self.max_temp_hot_side = config.get_float("max_temp_hot_side", default=80)
        self.max_deviation = config.get_float("max_deviation", default=60.0)
        self.dew_point_safety = config.get_float("dew_point_safety", default=5.0)
        self.dew_point_range = config.get_float("dew_point_range", default=10)
        self.dew_point_base = config.get_float("dew_point_base", default=30)
        self.target_temperature = config.get_float("target_temperature", default=50)
        self.sensor_cold_name = config.get("sensor_cold_name")
        self.sensor_hot_name = config.get("sensor_hot_name")
        self.peltier_pin = config.get("peltier_pin")
        self.sensor_cold = None
        self.sensor_hot = None

        ppins = self.printer.lookup_object("pins")
        self.mcu_pwm = ppins.setup_pin("pwm", self.peltier_pin)
        pwm_cycle_time = config.getfloat(
            "pwm_cycle_time", 0.100, above=0.0, maxval=0.25
        )
        self.mcu_pwm.setup_cycle_time(pwm_cycle_time)
        self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)

        self.temperature_sample_thread = self.kalico_threads.register_job(
            target=self.callback
        )

        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_TEC_TARGET",
            "TEC_TESTER",
            self.name,
            self.cmd_SET_TARGET_TEMP
        )

        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready
        )

    def _handle_connect(self):
        self.sensor_cold = self.printer.lookup_object(self.sensor_cold_name)
        self.sensor_hot = self.printer.lookup_object(self.sensor_hot_name)

    def _handle_ready(self):
        self.temperature_sample_thread.start()

    def callback(self):
        curtime = self.printer.get_reactor().monotonic()
        temp_cold = self.sensor_cold.get_status(curtime)["temperature"]
        temp_hot = self.sensor_hot.get_status(curtime)["temperature"]
        if temp_cold < self.min_temp_cold_side:
            self.printer.invoke_shutdown(
                "[%s]\n"
                "Cold side temp too low"
                % (
                    self.name,
                )
            )
        if temp_cold > self.max_temp_cold_side:
            self.printer.invoke_shutdown(
                "[%s]\n"
                "Cold side temp too high"
                % (
                    self.name,
                )
            )
        if temp_hot < self.min_temp_hot_side:
            self.printer.invoke_shutdown(
                "[%s]\n"
                "Hot side temp too low"
                % (
                    self.name,
                )
            )
        if temp_hot > self.max_temp_hot_side:
            self.printer.invoke_shutdown(
                "[%s]\n"
                "Hot side temp too high"
                % (
                    self.name,
                )
            )
        if abs(temp_cold - temp_hot) > self.max_deviation:
            if temp_cold < self.min_temp_cold_side:
                self.printer.invoke_shutdown(
                    "[%s]\n"
                    "Deviation between cold and hot too high"
                    % (
                        self.name,
                    )
                )
        dew_point = self.dew_point_base + random.randint(0, self.dew_point_range)
        dew_point = dew_point + self.dew_point_safety
        target_temp = self.target_temperature if self.target_temperature > dew_point else dew_point

        read_time = self.mcu_pwm.get_mcu().estimated_print_time(curtime)
        if (temp_cold < target_temp):
            self.mcu_pwm.set_pwm(read_time, 0)
        else:
            self.mcu_pwm.set_pwm(read_time, 1)
        return 0.25

    def cmd_SET_TARGET_TEMP(self, gcmd):
        self.target_temperature = gcmd.getfloat("TARGET", self.target_temperature)
        gcmd.respond_info(f"TARGET_TEMP={self.target_temperature}")

def load_config_prefix(config):
    return TecTester(config)
