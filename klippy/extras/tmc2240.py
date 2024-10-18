# TMC2240 configuration
#
# Copyright (C) 2018-2023  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2023  Alex Voinea <voinea.dragos.alexandru@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
from . import tmc, tmc2130, tmc_uart
from configfile import PrinterConfig

TMC_FREQUENCY = 12500000.0

Registers = {
    "GCONF": 0x00,
    "GSTAT": 0x01,
    "IFCNT": 0x02,
    "NODECONF": 0x03,
    "IOIN": 0x04,
    "DRV_CONF": 0x0A,
    "GLOBALSCALER": 0x0B,
    "IHOLD_IRUN": 0x10,
    "TPOWERDOWN": 0x11,
    "TSTEP": 0x12,
    "TPWMTHRS": 0x13,
    "TCOOLTHRS": 0x14,
    "THIGH": 0x15,
    "DIRECT_MODE": 0x2D,
    "ENCMODE": 0x38,
    "X_ENC": 0x39,
    "ENC_CONST": 0x3A,
    "ENC_STATUS": 0x3B,
    "ENC_LATCH": 0x3C,
    "ADC_VSUPPLY_AIN": 0x50,
    "ADC_TEMP": 0x51,
    "OTW_OV_VTH": 0x52,
    "MSLUT0": 0x60,
    "MSLUT1": 0x61,
    "MSLUT2": 0x62,
    "MSLUT3": 0x63,
    "MSLUT4": 0x64,
    "MSLUT5": 0x65,
    "MSLUT6": 0x66,
    "MSLUT7": 0x67,
    "MSLUTSEL": 0x68,
    "MSLUTSTART": 0x69,
    "MSCNT": 0x6A,
    "MSCURACT": 0x6B,
    "CHOPCONF": 0x6C,
    "COOLCONF": 0x6D,
    "DRV_STATUS": 0x6F,
    "PWMCONF": 0x70,
    "PWM_SCALE": 0x71,
    "PWM_AUTO": 0x72,
    "SG4_THRS": 0x74,
    "SG4_RESULT": 0x75,
    "SG4_IND": 0x76,
}

ReadRegisters = [
    "GCONF",
    "GSTAT",
    "IOIN",
    "DRV_CONF",
    "GLOBALSCALER",
    "IHOLD_IRUN",
    "TPOWERDOWN",
    "TSTEP",
    "TPWMTHRS",
    "TCOOLTHRS",
    "THIGH",
    "ADC_VSUPPLY_AIN",
    "ADC_TEMP",
    "MSCNT",
    "MSCURACT",
    "CHOPCONF",
    "COOLCONF",
    "DRV_STATUS",
    "PWMCONF",
    "PWM_SCALE",
    "PWM_AUTO",
    "SG4_THRS",
    "SG4_RESULT",
    "SG4_IND",
]

Fields = {}
Fields["COOLCONF"] = {
    "semin": 0x0F << 0,
    "seup": 0x03 << 5,
    "semax": 0x0F << 8,
    "sedn": 0x03 << 13,
    "seimin": 0x01 << 15,
    "sgt": 0x7F << 16,
    "sfilt": 0x01 << 24,
}
Fields["CHOPCONF"] = {
    "toff": 0x0F << 0,
    "hstrt": 0x07 << 4,
    "hend": 0x0F << 7,
    "fd3": 0x01 << 11,
    "disfdcc": 0x01 << 12,
    "chm": 0x01 << 14,
    "tbl": 0x03 << 15,
    "vhighfs": 0x01 << 18,
    "vhighchm": 0x01 << 19,
    "tpfd": 0x0F << 20,  # midrange resonances
    "mres": 0x0F << 24,
    "intpol": 0x01 << 28,
    "dedge": 0x01 << 29,
    "diss2g": 0x01 << 30,
    "diss2vs": 0x01 << 31,
}
Fields["DRV_STATUS"] = {
    "sg_result": 0x3FF << 0,
    "s2vsa": 0x01 << 12,
    "s2vsb": 0x01 << 13,
    "stealth": 0x01 << 14,
    "fsactive": 0x01 << 15,
    "cs_actual": 0x1F << 16,
    "stallguard": 0x01 << 24,
    "ot": 0x01 << 25,
    "otpw": 0x01 << 26,
    "s2ga": 0x01 << 27,
    "s2gb": 0x01 << 28,
    "ola": 0x01 << 29,
    "olb": 0x01 << 30,
    "stst": 0x01 << 31,
}
Fields["GCONF"] = {
    "faststandstill": 0x01 << 1,
    "en_pwm_mode": 0x01 << 2,
    "multistep_filt": 0x01 << 3,
    "shaft": 0x01 << 4,
    "diag0_error": 0x01 << 5,
    "diag0_otpw": 0x01 << 6,
    "diag0_stall": 0x01 << 7,
    "diag1_stall": 0x01 << 8,
    "diag1_index": 0x01 << 9,
    "diag1_onstate": 0x01 << 10,
    "diag0_pushpull": 0x01 << 12,
    "diag1_pushpull": 0x01 << 13,
    "small_hysteresis": 0x01 << 14,
    "stop_enable": 0x01 << 15,
    "direct_mode": 0x01 << 16,
}
Fields["GSTAT"] = {
    "reset": 0x01 << 0,
    "drv_err": 0x01 << 1,
    "uv_cp": 0x01 << 2,
    "register_reset": 0x01 << 3,
    "vm_uvlo": 0x01 << 4,
}
Fields["GLOBALSCALER"] = {"globalscaler": 0xFF << 0}
Fields["IHOLD_IRUN"] = {
    "ihold": 0x1F << 0,
    "irun": 0x1F << 8,
    "iholddelay": 0x0F << 16,
    "irundelay": 0x0F << 24,
}
Fields["IOIN"] = {
    "step": 0x01 << 0,
    "dir": 0x01 << 1,
    "encb": 0x01 << 2,
    "enca": 0x01 << 3,
    "drv_enn": 0x01 << 4,
    "encn": 0x01 << 5,
    "uart_en": 0x01 << 6,
    "comp_a": 0x01 << 8,
    "comp_b": 0x01 << 9,
    "comp_a1_a2": 0x01 << 10,
    "comp_b1_b2": 0x01 << 11,
    "output": 0x01 << 12,
    "ext_res_det": 0x01 << 13,
    "ext_clk": 0x01 << 14,
    "adc_err": 0x01 << 15,
    "silicon_rv": 0x07 << 16,
    "version": 0xFF << 24,
}
Fields["MSLUT0"] = {"mslut0": 0xFFFFFFFF}
Fields["MSLUT1"] = {"mslut1": 0xFFFFFFFF}
Fields["MSLUT2"] = {"mslut2": 0xFFFFFFFF}
Fields["MSLUT3"] = {"mslut3": 0xFFFFFFFF}
Fields["MSLUT4"] = {"mslut4": 0xFFFFFFFF}
Fields["MSLUT5"] = {"mslut5": 0xFFFFFFFF}
Fields["MSLUT6"] = {"mslut6": 0xFFFFFFFF}
Fields["MSLUT7"] = {"mslut7": 0xFFFFFFFF}
Fields["MSLUTSEL"] = {
    "x3": 0xFF << 24,
    "x2": 0xFF << 16,
    "x1": 0xFF << 8,
    "w3": 0x03 << 6,
    "w2": 0x03 << 4,
    "w1": 0x03 << 2,
    "w0": 0x03 << 0,
}
Fields["MSLUTSTART"] = {
    "start_sin": 0xFF << 0,
    "start_sin90": 0xFF << 16,
    "offset_sin90": 0xFF << 24,
}
Fields["MSCNT"] = {"mscnt": 0x3FF << 0}
Fields["MSCURACT"] = {"cur_a": 0x1FF << 0, "cur_b": 0x1FF << 16}
Fields["PWM_AUTO"] = {"pwm_ofs_auto": 0xFF << 0, "pwm_grad_auto": 0xFF << 16}
Fields["PWMCONF"] = {
    "pwm_ofs": 0xFF << 0,
    "pwm_grad": 0xFF << 8,
    "pwm_freq": 0x03 << 16,
    "pwm_autoscale": 0x01 << 18,
    "pwm_autograd": 0x01 << 19,
    "freewheel": 0x03 << 20,
    "pwm_meas_sd_enable": 0x01 << 22,
    "pwm_dis_reg_stst": 0x01 << 23,
    "pwm_reg": 0x0F << 24,
    "pwm_lim": 0x0F << 28,
}
Fields["PWM_SCALE"] = {
    "pwm_scale_sum": 0x3FF << 0,
    "pwm_scale_auto": 0x1FF << 16,
}
Fields["TPOWERDOWN"] = {"tpowerdown": 0xFF << 0}
Fields["TPWMTHRS"] = {"tpwmthrs": 0xFFFFF << 0}
Fields["TCOOLTHRS"] = {"tcoolthrs": 0xFFFFF << 0}
Fields["TSTEP"] = {"tstep": 0xFFFFF << 0}
Fields["THIGH"] = {"thigh": 0xFFFFF << 0}
Fields["DRV_CONF"] = {"current_range": 0x03 << 0, "slope_control": 0x03 << 4}
Fields["ADC_VSUPPLY_AIN"] = {
    "adc_vsupply": 0x1FFF << 0,
    "adc_ain": 0x1FFF << 16,
}
Fields["ADC_TEMP"] = {"adc_temp": 0x1FFF << 0}
Fields["OTW_OV_VTH"] = {
    "overvoltage_vth": 0x1FFF << 0,
    "overtempprewarning_vth": 0x1FFF << 16,
}
Fields["SG4_THRS"] = {
    "sg4_thrs": 0xFF << 0,
    "sg4_filt_en": 0x01 << 8,
    "sg4_angle_offset": 0x01 << 9,
}
Fields["SG4_RESULT"] = {"sg4_result": 0x3FF << 0}
Fields["SG4_IND"] = {
    "sg4_ind_0": 0xFF << 0,
    "sg4_ind_1": 0xFF << 8,
    "sg4_ind_2": 0xFF << 16,
    "sg4_ind_3": 0xFF << 24,
}


SignedFields = ["cur_a", "cur_b", "sgt", "pwm_scale_auto", "offset_sin90"]

FieldFormatters = dict(tmc2130.FieldFormatters)
FieldFormatters.update(
    {
        "s2vsa": (lambda v: "1(ShortToSupply_A!)" if v else ""),
        "s2vsb": (lambda v: "1(ShortToSupply_B!)" if v else ""),
        "adc_temp": (lambda v: "0x%04x(%.1fC)" % (v, ((v - 2038) / 7.7))),
        "adc_vsupply": (lambda v: "0x%04x(%.3fV)" % (v, v * 0.009732)),
        "adc_ain": (lambda v: "0x%04x(%.3fmV)" % (v, v * 0.3052)),
    }
)


######################################################################
# TMC stepper current config helper
######################################################################

KIFS = [11750.0, 24000.0, 36000.0, 36000.0]
GLOBALSCALER_ERROR = (
    "[%s %s]\n"
    "GLOBALSCALER(%d) calculation out of bounds.\n"
    "The target current can't be achieved with the given RREF(%f) "
    "and CS(%d). Please adjust your configuration.\n"
    "Valid values are 0 and 32-255.\n"
    "Calculated current_range bit: %d\n"
    "Calculated KIFS: %f"
)


class TMC2240CurrentHelper(tmc.BaseTMCCurrentHelper):
    def __init__(self, config, mcu_tmc, tmc_type="tmc2240"):
        self.use_motor_peak_current = config.getboolean("use_motor_peak_current", False)
        self.printer = config.get_printer()
        pconfig: PrinterConfig = self.printer.lookup_object("configfile")
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.Rref = config.getfloat("rref", None, minval=12000.0, maxval=60000.0)
        if self.Rref is None:
            pconfig.warn(
                "config",
                f"""[{self.type} {self.name}] rref not specified; using default = 12000.
                If this value is wrong, it might burn your house down.
                This parameter will be mandatory in future versions.
                Specify the parameter to resolve this warning""",
                f"{self.type} {self.name}",
                "rref",
            )
            self.Rref = 12000.0

        super().__init__(config, mcu_tmc, self._get_ifs(3), tmc_type)

        current_range = self._calc_current_range(self.actual_current)
        self.fields.set_field("current_range", current_range)

        self.cs = config.getint("driver_cs", 31, maxval=31, minval=0)
        self.cap_global_scaler = config.getboolean("cap_global_scaler", False)

        gscaler = self._calc_globalscaler(self.req_run_current)
        # if 1 <= gscaler <= 31 or gscaler > 256:
        #     raise config.error(
        #         GLOBALSCALER_ERROR
        #         % (
        #             self.type,
        #             self.name,
        #             gscaler,
        #             self.Rref,
        #             self.cs,
        #             f"{current_range:02b}",
        #             f"{(KIFS[current_range]/1000):.2f}",
        #         )
        #    )

        ihold = (
            self.cs
            if self.req_hold_current is None
            else self._calc_current_bits(
                min(self.req_run_current, self.req_hold_current), gscaler
            )
        )

        self.fields.set_field("globalscaler", gscaler)
        self.fields.set_field("ihold", ihold)
        self.fields.set_field("irun", self.cs)

    def _get_ifs(self, current_range=None):
        if current_range is None:
            current_range = self.fields.get_field("current_range")
        ifs = KIFS[current_range] / self.Rref
        if self.use_motor_peak_current:
            return ifs
        return ifs / math.sqrt(2)

    def _calc_current_range(self, current):
        current_range = 0
        for current_range in range(4):
            if current <= self._get_ifs(current_range):
                break
        return current_range

    def _calc_globalscaler(self, current):
        ifs_rms = self._get_ifs()
        globalscaler = int(((current * 256.0 * 32) / (ifs_rms * (self.cs + 1))) + 0.5)
        if self.cap_global_scaler and globalscaler >= 256:
            return 0
        if globalscaler == 256:
            return 0
        if 1 <= globalscaler <= 31 or globalscaler > 256:
            current_range = self.fields.get_field("current_range")
            self.printer.invoke_shutdown(
                GLOBALSCALER_ERROR
                % (
                    self.type,
                    self.name,
                    globalscaler,
                    self.Rref,
                    self.cs,
                    f"{current_range:02b}",
                    f"{(KIFS[current_range] / 1000):.2f}",
                )
            )
        return globalscaler

    def _calc_current_bits(self, current, globalscaler):
        ifs_rms = self._get_ifs()
        if not globalscaler:
            globalscaler = 256
        bits = int((current * 256.0 * 32.0) / (globalscaler * ifs_rms) - 1.0 + 0.5)
        return max(0, min(31, bits))

    def _calc_current_from_field(self, field_name):
        ifs_rms = self._get_ifs()
        globalscaler = self.fields.get_field("globalscaler")
        if not globalscaler:
            globalscaler = 256
        bits = self.fields.get_field(field_name)
        return globalscaler * (bits + 1) * ifs_rms / (256.0 * 32.0)

    def get_current(self):
        ifs_rms = self._get_ifs()
        run_current = self._calc_current_from_field("irun")
        hold_current = self._calc_current_from_field("ihold")
        return (
            run_current,
            hold_current,
            (
                self.req_run_current
                if self.req_hold_current is None
                else self.req_hold_current
            ),
            ifs_rms,
            self.req_home_current,
        )

    def apply_current(self, print_time, recalculate_current_range=False):
        if recalculate_current_range:
            current_range = self._calc_current_range(self.actual_current)
            val = self.fields.set_field("current_range", current_range)
            self.mcu_tmc.set_register("DRV_CONF", val, print_time)
        gscaler = self._calc_globalscaler(self.actual_current)
        ihold = (
            self.cs
            if self.req_hold_current is None
            else self._calc_current_bits(
                min(self.actual_current, self.req_hold_current), gscaler
            )
        )
        val = self.fields.set_field("globalscaler", gscaler)
        self.mcu_tmc.set_register("GLOBALSCALER", val, print_time)
        self.fields.set_field("ihold", ihold)
        val = self.fields.set_field("irun", self.cs)
        self.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)


######################################################################
# TMC stepper overvoltage config helper
######################################################################


def TMC2240OvervoltageHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    ov_thres = config.getfloat(
        "overvoltage_threshold",
        37.735,  # 0xF25 by default
        maxval=40.0,
    )
    overvoltage_vth = int(ov_thres / 0.009732)
    fields.set_field("overvoltage_vth", overvoltage_vth)


######################################################################
# TMC stepper overtemperature warning config helper
######################################################################


def TMC2240OvertemperatureWarningHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    otw_thres = config.getfloat(
        "overtemperature_warning_threshold",
        120.0,  # 0xB92 by default
        maxval=120.0,
    )
    overtempprewarning_vth = int((otw_thres * 7.7) + 2038)
    fields.set_field("overtempprewarning_vth", overtempprewarning_vth)


######################################################################
# TMC2240 printer object
######################################################################


class TMC2240:
    def __init__(self, config):
        # Setup mcu communication
        self.fields = tmc.FieldHelper(Fields, SignedFields, FieldFormatters)
        if config.get("uart_pin", None) is not None:
            # use UART for communication
            self.mcu_tmc = tmc_uart.MCU_TMC_uart(
                config, Registers, self.fields, 3, TMC_FREQUENCY
            )
        else:
            # Use SPI bus for communication
            self.mcu_tmc = tmc2130.MCU_TMC_SPI(
                config, Registers, self.fields, TMC_FREQUENCY
            )
        # Allow virtual pins to be created
        tmc.TMCVirtualPinHelper(config, self.mcu_tmc)
        # Register commands
        current_helper = TMC2240CurrentHelper(config, self.mcu_tmc)
        TMC2240OvervoltageHelper(config, self.mcu_tmc)
        TMC2240OvertemperatureWarningHelper(config, self.mcu_tmc)
        cmdhelper = tmc.TMCCommandHelper(config, self.mcu_tmc, current_helper)
        cmdhelper.setup_register_dump(ReadRegisters)
        self.get_phase_offset = cmdhelper.get_phase_offset
        self.get_temperature = cmdhelper.get_temperature
        self.get_mcu = cmdhelper.get_mcu
        self.get_status = cmdhelper.get_status
        # Setup basic register values
        tmc.TMCWaveTableHelper(config, self.mcu_tmc)
        self.fields.set_config_field(config, "offset_sin90", 0)
        tmc.TMCStealthchopHelper(config, self.mcu_tmc, TMC_FREQUENCY)
        set_config_field = self.fields.set_config_field
        #   GCONF
        set_config_field(config, "multistep_filt", True)
        #   CHOPCONF
        set_config_field(config, "toff", 3)
        set_config_field(config, "hstrt", 5)
        set_config_field(config, "hend", 2)
        set_config_field(config, "fd3", 0)
        set_config_field(config, "disfdcc", 0)
        set_config_field(config, "chm", 0)
        set_config_field(config, "tbl", 2)
        set_config_field(config, "vhighfs", 0)
        set_config_field(config, "vhighchm", 0)
        set_config_field(config, "tpfd", 4)
        set_config_field(config, "diss2g", 0)
        set_config_field(config, "diss2vs", 0)
        #   COOLCONF
        set_config_field(config, "semin", 0)
        set_config_field(config, "seup", 0)
        set_config_field(config, "semax", 0)
        set_config_field(config, "sedn", 0)
        set_config_field(config, "seimin", 0)
        set_config_field(config, "sgt", 0)
        set_config_field(config, "sfilt", 0)
        #   IHOLDIRUN
        set_config_field(config, "iholddelay", 6)
        set_config_field(config, "irundelay", 4)
        #   PWMCONF
        set_config_field(config, "pwm_ofs", 29)
        set_config_field(config, "pwm_grad", 0)
        set_config_field(config, "pwm_freq", 0)
        set_config_field(config, "pwm_autoscale", True)
        set_config_field(config, "pwm_autograd", True)
        set_config_field(config, "freewheel", 0)
        set_config_field(config, "pwm_reg", 4)
        set_config_field(config, "pwm_lim", 12)
        #   TPOWERDOWN
        set_config_field(config, "tpowerdown", 10)
        #   SG4_THRS
        set_config_field(config, "sg4_angle_offset", 1)


def load_config_prefix(config):
    return TMC2240(config)
