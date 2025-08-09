import random


MAX_HEAT_TIME = 5.0

class TecTester:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.min_temp_cold_side = config.get_float("min_temp_cold_side", default=20)
        self.min_temp_hot_side = config.get_float("min_temp_hot_side", default=20)
        self.max_temp_cold_side = config.get_float("max_temp_cold_side", default=80)
        self.max_temp_hot_side = config.get_float("max_temp_hot_side", default=80)
        self.hot_side_safety = config.get_float("hot_side_safety", default=10)
        self.max_deviation = config.get_float("max_deviation", default=60.0)
        self.dew_point_safety = config.get_float("dew_point_safety", default=5.0)
        self.dew_point_range = config.get_float("dew_point_range", default=10)
        self.dew_point_base = config.get_float("dew_point_base", default=30)
        self.target_temperature = config.get_float("target_temperature", default=50)
        self.sensor_cold_name = config.get("sensor_cold_name")
        self.sensor_hot_name = config.get("sensor_hot_name")
        self.enable_delay = config.get_float("enable_delay", 120)
        self.max_pwm = config.get_float("max_pwm", 1, minval=0, maxval=1)
        self.smooth_time = config.get_float("smooth_time", 1.0, above=0.0)
        self.kp = config.get_float("pid_kp", 1.0, above=0.0)
        self.ki = config.get_float("pid_ki", 1.0, above=0.0)
        self.kd = config.get_float("pid_kd", 1.0, above=0.0)
        self.sensor_cold = None
        self.sensor_hot = None

        self.enable = 0

        self.prev_err = 0.0
        self.prev_der = 0.0
        self.int_sum = 0.0
        self.prev_temp_time = 0.0
        self.prev_temp = 25.0

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

        controls = {"watermark": self.callback_watermark, "pid": self.callback_pid}
        self.callback_control = self.config.getchoice("control", controls, default="watermark")

        self.temperature_sample_timer = self.reactor.register_timer(
            self.callback
        )

        self.last_value = 0
        self.last_enable_time = 0

        self.printer.add_object("heater_fan " + self.name, self)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_TEC_TESTER",
            "TEC_TESTER",
            self.name,
            self.cmd_SET_TEC_TESTER
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
        self.reactor.update_timer(
            self.temperature_sample_timer, self.reactor.monotonic() + 1.0
        )

    def callback(self, eventtime):
        temp_cold = self.sensor_cold.get_status(eventtime)["temperature"]
        temp_hot = self.sensor_hot.get_status(eventtime)["temperature"]
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

        if not self.enable:
            return self.callback_disabled()
        return self.callback_control(temp_cold, temp_hot)

    def callback_disabled(self):
        curtime = self.reactor.monotonic()
        read_time = self.mcu_pwm.get_mcu().estimated_print_time(curtime)
        self.mcu_pwm.set_pwm(read_time, 0)
        return curtime + 0.25

    def callback_watermark(self, temp_cold, temp_hot, enabled):
        curtime = self.reactor.monotonic()
        dew_point = self.dew_point_base + random.randint(0, self.dew_point_range)
        dew_point = dew_point + self.dew_point_safety
        target_temp = self.target_temperature if self.target_temperature > dew_point else dew_point

        read_time = self.mcu_pwm.get_mcu().estimated_print_time(curtime)
        if self.last_value == 0 and read_time < self.last_enable_time + self.enable_delay:
            return 0.25

        if temp_cold < target_temp or temp_hot >= (self.max_temp_cold_side - self.hot_side_safety) or not enabled:
            if self.last_value == 1:
                self.last_enable_time = read_time
            self.last_value = 0
            self.mcu_pwm.set_pwm(read_time, 0)
        else:
            self.last_value = self.max_pwm
            self.mcu_pwm.set_pwm(read_time, self.max_pwm)
        return curtime + 0.25

    def callback_pid(self, temp_cold, temp_hot, enabled):
        curtime = self.reactor.monotonic()
        read_time = self.mcu_pwm.get_mcu().estimated_print_time(curtime)

        # calculate the error
        err = self.target_temperature - temp_cold
        # calculate the time difference
        dt = read_time - self.prev_temp_time
        # calculate the current integral amount using the Trapezoidal rule
        ic = ((self.prev_err + err) / 2.0) * dt
        i = self.int_sum + ic

        # calculate the current derivative using derivative on measurement,
        # to account for derivative kick when the set point changes
        # smooth the derivatives using a modified moving average
        # that handles unevenly spaced data points
        n = max(1.0, self.smooth_time / dt)
        dc = -(temp_cold - self.prev_temp) / dt
        dc = ((n - 1.0) * self.prev_der + dc) / n

        # calculate the output
        o = self.kp * err + self.ki * i + self.kd * dc
        # calculate the saturated output
        so = max(0.0, min(self.max_pwm, o))

        pwm = self.max_pwm - so

        # update the heater
        if temp_hot >= (self.max_temp_cold_side - self.hot_side_safety) or not enabled:
            pwm = 0.0
        self.mcu_pwm.set_pwm(read_time, pwm)
        # update the previous values
        self.prev_temp = temp_cold
        self.prev_temp_time = read_time
        self.prev_der = dc
        if temp_hot < (self.max_temp_cold_side - self.hot_side_safety) and enabled:
            self.prev_err = err
            if o == so:
                # not saturated so an update is allowed
                self.int_sum = i
            else:
                # saturated, so conditionally integrate
                if (o > 0.0) - (o < 0.0) != (ic > 0.0) - (ic < 0.0):
                    # the signs are opposite so an update is allowed
                    self.int_sum = i
        else:
            self.prev_err = 0.0
            self.int_sum = 0.0

        return curtime + 0.25


    def cmd_SET_TEC_TESTER(self, gcmd):
        self.target_temperature = gcmd.getfloat("TARGET", self.target_temperature)
        self.enable = gcmd.getint("ENABLE", self.enable, minval=0, maxval=1)
        gcmd.respond_info(f"TARGET_TEMP={self.target_temperature}")

    def get_status(self, eventtime):
        return {
            "speed": self.last_value,
            "pwm_value": self.last_value,
            "rpm": None,
        }

def load_config_prefix(config):
    return TecTester(config)
