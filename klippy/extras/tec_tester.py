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
        self.enable_delay = config.get_float("enable_delay", 120)
        self.max_pwm = config.get_float("max_pwm", 1, minval=0, maxval=1)
        self.min_deriv_time = config.get_float("smooth_time", 1.0, above=0.0)
        self.kp = config.get_float("pid_kp", 1.0, above=0.0)
        self.ki = config.get_float("pid_ki", 1.0, above=0.0)
        self.kd = config.get_float("pid_kd", 1.0, above=0.0)
        self.sensor_cold = None
        self.sensor_hot = None

        self.prev_temp_time = 0.0
        self.prev_temp_deriv = 0.0
        self.prev_temp_integ = 0.0
        self.prev_temp = 0.0

        self.temp_integ_max = 0.0
        if self.ki:
            self.temp_integ_max = self.max_pwm / self.ki

        pwm_cycle_time = config.getfloat(
            "pwm_cycle_time", 0.0004, above=0.0, maxval=0.25
        )
        hardware_pwm = config.getboolean("hardware_pwm", False)

        ppins = self.printer.lookup_object("pins")
        self.mcu_pwm = ppins.setup_pin("pwm", config.get("peltier_pin"))
        self.mcu_pwm.setup_cycle_time(pwm_cycle_time, hardware_pwm)
        self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)

        self.control = config.get("control", "watermark")

        self.callback_control = None
        if self.control == "watermark":
            self.callback_control = self.callback_watermark
        if self.control == "pid":
            self.callback_control = self.callback_pid

        self.temperature_sample_thread = self.kalico_threads.register_job(
            target=self.callback
        )

        self.last_value = 0
        self.last_enable_time = 0

        self.printer.add_object("heater_fan " + self.name, self)
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

        return self.callback_control(curtime, temp_cold, temp_hot)

    def callback_watermark(self, curtime, temp_cold, temp_hot):
        dew_point = self.dew_point_base + random.randint(0, self.dew_point_range)
        dew_point = dew_point + self.dew_point_safety
        target_temp = self.target_temperature if self.target_temperature > dew_point else dew_point

        read_time = self.mcu_pwm.get_mcu().estimated_print_time(curtime)
        if self.last_value == 0 and read_time < self.last_enable_time + self.enable_delay:
            return 0.25

        if temp_cold < target_temp:
            if self.last_value == 1:
                self.last_enable_time = read_time
            self.last_value = 0
            self.mcu_pwm.set_pwm(read_time, 0)
        else:
            self.last_value = self.max_pwm
            self.mcu_pwm.set_pwm(read_time, self.max_pwm)
        return 0.25

    def callback_pid(self, curtime, temp_cold, temp_hot):
        read_time = self.mcu_pwm.get_mcu().estimated_print_time(curtime)
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp_cold - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (
                self.prev_temp_deriv * (self.min_deriv_time - time_diff)
                + temp_diff
            ) / self.min_deriv_time
        # Calculate accumulated temperature "error"
        temp_err = self.target_temperature - temp_cold
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0.0, min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self.kp * temp_err + self.ki * temp_integ - self.kd * temp_deriv
        # logging.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%d",
        #    temp, read_time, temp_diff, temp_deriv, temp_err, temp_integ, co)
        bounded_co = max(0.0, min(self.max_pwm, co))
        self.mcu_pwm.set_pwm(read_time, bounded_co)
        # Store state for next measurement
        self.prev_temp = temp_cold
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ
        return 0.25


    def cmd_SET_TARGET_TEMP(self, gcmd):
        self.target_temperature = gcmd.getfloat("TARGET", self.target_temperature)
        gcmd.respond_info(f"TARGET_TEMP={self.target_temperature}")

    def get_status(self, eventtime):
        return {
            "speed": self.last_value,
            "pwm_value": self.last_value,
            "rpm": None,
        }

def load_config_prefix(config):
    return TecTester(config)
