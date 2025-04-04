# Kinematic input shaper to minimize motion vibrations in XY plane
#
# Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2020-2023  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections, inspect, logging, sys
from klippy import chelper
from . import extruder_smoother, shaper_defs


def parse_float_list(list_str):
    def parse_str(s):
        res = []
        for line in s.split("\n"):
            for coeff in line.split(","):
                res.append(float(coeff.strip()))
        return res

    try:
        return parse_str(list_str)
    except:
        return None


class TypedInputShaperParams:
    shapers = {s.name: s.init_func for s in shaper_defs.INPUT_SHAPERS}

    def __init__(self, axis, shaper_type, config):
        self.axis = axis
        self.shaper_type = shaper_type
        self.damping_ratio = shaper_defs.DEFAULT_DAMPING_RATIO
        self.shaper_freq = 0.0
        self.motor_damping = shaper_defs.DEFAULT_MOTOR_DAMPING_RATIO
        self.motor_freq = 0.0
        if config is not None:
            if shaper_type not in self.shapers:
                raise config.error(
                    "Unsupported shaper type: %s" % (shaper_type,)
                )
            global_damping_ratio = config.getfloat(
                "damping_ratio",
                self.damping_ratio,
                minval=0.0,
                maxval=1.0,
            )
            self.damping_ratio = config.getfloat(
                "damping_ratio_" + axis,
                global_damping_ratio,
                minval=0.0,
                maxval=1.0,
            )
            global_shaper_freq = config.getfloat(
                "shaper_freq", self.shaper_freq, minval=0.0
            )
            self.shaper_freq = config.getfloat(
                "shaper_freq_" + axis, global_shaper_freq, minval=0.0
            )

            global_motor_damping = config.getfloat(
                "motor_damping_ratio",
                self.motor_damping,
                minval=0.0,
                maxval=1.0,
            )
            self.motor_damping = config.getfloat(
                "motor_damping_ratio_" + axis,
                global_motor_damping,
                minval=0.0,
                maxval=1.0,
            )
            global_motor_freq = config.getfloat(
                "motor_freq", self.motor_freq, minval=0.0
            )
            self.motor_freq = config.getfloat(
                "motor_freq_" + axis, global_motor_freq, minval=0.0
            )

    def get_type(self):
        return self.shaper_type

    def get_axis(self):
        return self.axis

    def update(self, shaper_type, gcmd):
        if shaper_type not in self.shapers:
            raise gcmd.error("Unsupported shaper type: %s" % (shaper_type,))
        axis = self.axis.upper()
        self.damping_ratio = gcmd.get_float(
            "DAMPING_RATIO_" + axis, self.damping_ratio, minval=0.0, maxval=1.0
        )
        self.shaper_freq = gcmd.get_float(
            "SHAPER_FREQ_" + axis, self.shaper_freq, minval=0.0
        )
        self.motor_damping = gcmd.get_float(
            "MOTOR_DAMPING_" + axis, self.motor_damping, minval=0.0, maxval=1.0
        )
        self.motor_freq = gcmd.get_float(
            "MOTOR_FREQ_" + axis, self.motor_freq, minval=0.0
        )
        self.shaper_type = shaper_type

    def get_shaper(self):
        if not self.shaper_freq:
            A, T = shaper_defs.get_none_shaper()
        else:
            A, T = self.shapers[self.shaper_type](
                self.shaper_freq, self.damping_ratio
            )
        return len(A), A, T

    def get_motor_filter(self):
        if not self.motor_freq:
            A, T = shaper_defs.get_none_shaper()
        else:
            A, T = shaper_defs.get_mzv_shaper(
                self.motor_freq, self.motor_damping
            )
        return len(A), A, T

    def get_status(self):
        return collections.OrderedDict(
            [
                ("shaper_type", self.shaper_type),
                ("shaper_freq", "%.3f" % (self.shaper_freq,)),
                (
                    "damping_ratio",
                    "%.6f" % (self.damping_ratio,),
                ),
                (
                    "motor_damping",
                    "%.6f" % (self.motor_damping,),
                ),
                ("motor_freq", "%.3f" % (self.motor_freq,)),
            ]
        )


class CustomInputShaperParams:
    SHAPER_TYPE = "custom"

    def __init__(self, axis, config):
        self.axis = axis
        self.n, self.A, self.T = 0, [], []
        self.motor_filter_n, self.motor_filter_A, self.motor_filter_T = (
            0,
            [],
            [],
        )
        if config is not None:
            shaper_a_str = config.get("shaper_a_" + axis)
            shaper_t_str = config.get("shaper_t_" + axis)
            self.n, self.A, self.T = self._parse_custom_shaper(
                shaper_a_str, shaper_t_str, config.error
            )
            motor_filter_a_str = config.get("motor_filter_a_" + axis)
            motor_filter_t_str = config.get("motor_filter_t_" + axis)
            self.motor_filter_n, self.motor_filter_A, self.motor_filter_T = (
                self._parse_custom_shaper(
                    motor_filter_a_str, motor_filter_t_str, config.error
                )
            )

    def get_type(self):
        return self.SHAPER_TYPE

    def get_axis(self):
        return self.axis

    def update(self, shaper_type, gcmd):
        if shaper_type != self.SHAPER_TYPE:
            raise gcmd.error("Unsupported shaper type: %s" % (shaper_type,))
        axis = self.axis.upper()
        shaper_a_str = gcmd.get("SHAPER_A_" + axis, None)
        shaper_t_str = gcmd.get("SHAPER_T_" + axis, None)
        motor_filter_a_str = gcmd.get("MOTOR_FILTER_A_" + axis, None)
        motor_filter_t_str = gcmd.get("MOTOR_FILTER_T_" + axis, None)
        if (shaper_a_str is None) != (shaper_t_str is None):
            raise gcmd.error(
                "Both SHAPER_A_%s and SHAPER_T_%s parameters"
                " must be provided" % (axis, axis)
            )
        if (motor_filter_a_str is None) != (motor_filter_t_str is None):
            raise gcmd.error(
                "Both MOTOR_FILTER_A_%s and MOTOR_FILTER_T_%s parameters"
                " must be provided" % (axis, axis)
            )
        if shaper_a_str is not None:
            self.n, self.A, self.T = self._parse_custom_shaper(
                shaper_a_str, shaper_t_str, gcmd.error
            )
        if motor_filter_a_str is not None:
            self.motor_filter_n, self.motor_filter_A, self.motor_filter_T = (
                self._parse_custom_shaper(
                    motor_filter_a_str, motor_filter_t_str, gcmd.error
                )
            )

    def _parse_custom_shaper(self, custom_a_str, custom_t_str, parse_error):
        A = parse_float_list(custom_a_str)
        if A is None:
            raise parse_error("Invalid shaper A string: '%s'" % (custom_a_str,))
        if min([abs(a) for a in A]) < 0.001:
            raise parse_error("All shaper A coefficients must be non-zero")
        if sum(A) < 0.001:
            raise parse_error(
                "Shaper A parameter must sum up to a positive number"
            )
        T = parse_float_list(custom_t_str)
        if T is None:
            raise parse_error("Invalid shaper T string: '%s'" % (custom_t_str,))
        if T != sorted(T):
            raise parse_error("Shaper T parameter is not ordered: %s" % (T,))
        if len(A) != len(T):
            raise parse_error(
                "Shaper A and T parameters must have the same length:"
                " %d vs %d"
                % (
                    len(A),
                    len(T),
                )
            )
        dur = T[-1] - T[0]
        if len(T) > 1 and dur < 0.001:
            raise parse_error(
                "Shaper duration is too small (%.6f sec)" % (dur,)
            )
        if dur > 0.2:
            raise parse_error(
                "Shaper duration is too large (%.6f sec)" % (dur,)
            )
        return len(A), A, T

    def get_shaper(self):
        return self.n, self.A, self.T

    def get_motor_filter(self):
        return self.motor_filter_n, self.motor_filter_A, self.motor_filter_T

    def get_status(self):
        return collections.OrderedDict(
            [
                ("shaper_type", self.SHAPER_TYPE),
                ("shaper_a", ",".join(["%.6f" % (a,) for a in self.A])),
                ("shaper_t", ",".join(["%.6f" % (t,) for t in self.T])),
                (
                    "motor_filter_a",
                    ",".join(["%.6f" % (a,) for a in self.motor_filter_A]),
                ),
                (
                    "motor_filter_t",
                    ",".join(["%.6f" % (t,) for t in self.motor_filter_T]),
                ),
            ]
        )


class AxisInputShaper:
    def __init__(self, params):
        self.params = params
        self.axis_n, self.axis_A, self.axis_T = params.get_shaper()
        self.motor_n, self.motor_A, self.motor_T = params.get_motor_filter()
        self.n, self.A, self.T = self.determine_shaper()
        self.t_offs = shaper_defs.get_shaper_offset(self.A, self.T)
        self.saved_axis_shaper = None
        self.saved_motor_filter = None
        self.cached_axis_shaper = None
        self.cached_motor_filter = None

    def determine_shaper(self):
        if self.axis_n == 0 and self.motor_n == 0:
            A, T = shaper_defs.get_none_shaper()
            return 0, A, T
        if self.motor_n == 0:
            return self.axis_n, self.axis_A, self.axis_T
        if self.axis_n == 0:
            return self.motor_n, self.motor_A, self.motor_T
        A, T = shaper_defs.convolve(
            (self.axis_A, self.axis_T), (self.motor_A, self.motor_T)
        )
        return len(A), A, T

    def get_name(self):
        return "shaper_" + self.get_axis()

    def get_type(self):
        return self.params.get_type()

    def get_axis(self):
        return self.params.get_axis()

    def is_extruder_smoothing(self, exact_mode):
        return not exact_mode and self.A

    def is_enabled(self):
        return self.n > 0

    def update(self, shaper_type, gcmd):
        self.params.update(shaper_type, gcmd)
        self.axis_n, self.axis_A, self.axis_T = self.params.get_shaper()
        self.motor_n, self.motor_A, self.motor_T = (
            self.params.get_motor_filter()
        )
        self.n, self.A, self.T = self.determine_shaper()
        self.t_offs = shaper_defs.get_shaper_offset(self.A, self.T)

    def update_stepper_kinematics(self, sk):
        ffi_main, ffi_lib = chelper.get_ffi()
        axis = self.get_axis().encode()
        success = (
            ffi_lib.input_shaper_set_shaper_params(
                sk, axis, self.n, self.A, self.T
            )
            == 0
        )
        if not success:
            self.disable_shaping()
            ffi_lib.input_shaper_set_shaper_params(
                sk, axis, self.n, self.A, self.T
            )
        return success

    def update_extruder_kinematics(self, sk, exact_mode):
        ffi_main, ffi_lib = chelper.get_ffi()
        axis = self.get_axis().encode()
        if not self.is_extruder_smoothing(exact_mode):
            # Make sure to disable any active input smoothing
            coeffs, smooth_time = [], 0.0
            success = (
                ffi_lib.extruder_set_smoothing_params(
                    sk, axis, len(coeffs), coeffs, smooth_time, 0.0
                )
                == 0
            )
            success = (
                ffi_lib.extruder_set_shaper_params(
                    sk, axis, self.n, self.A, self.T
                )
                == 0
            )
        else:
            shaper_type = self.get_type()
            status = self.params.get_status()
            damping_ratio = float(
                status.get("damping_ratio", shaper_defs.DEFAULT_DAMPING_RATIO)
            )

            C_e, t_sm = extruder_smoother.get_extruder_smoother(
                shaper_type,
                self.T[-1] - self.T[0],
                damping_ratio,
                normalize_coeffs=False,
            )
            smoother_offset = self.t_offs - 0.5 * t_sm
            success = (
                ffi_lib.extruder_set_smoothing_params(
                    sk, axis, len(C_e), C_e, t_sm, smoother_offset
                )
                == 0
            )
        if not success:
            self.disable_shaping()
            ffi_lib.extruder_set_shaper_params(sk, axis, self.n, self.A, self.T)
        return success

    def disable_shaping(self, axis_shaper=True, motor_filter=True):
        was_enabled = False
        A, T = shaper_defs.get_none_shaper()
        if axis_shaper and self.saved_axis_shaper is None and self.axis_n:
            self.saved_axis_shaper = (self.axis_n, self.axis_A, self.axis_T)
            self.axis_n, self.axis_A, self.axis_T = len(A), A, T
            was_enabled = True
        if motor_filter and self.saved_motor_filter is None and self.motor_n:
            self.saved_motor_filter = (self.motor_n, self.motor_A, self.motor_T)
            self.motor_n, self.motor_A, self.motor_T = len(A), A, T
            was_enabled = True
        self.n, self.A, self.T = self.determine_shaper()
        return was_enabled

    def cache_shaping(self):
        was_enabled = False
        A, T = shaper_defs.get_none_shaper()
        if self.cached_axis_shaper is None and self.axis_n:
            self.cached_axis_shaper = (self.axis_n, self.axis_A, self.axis_T)
            self.axis_n, self.axis_A, self.axis_T = len(A), A, T
            was_enabled = True
        if self.cached_motor_filter is None and self.motor_n:
            self.cached_motor_filter = (
                self.motor_n,
                self.motor_A,
                self.motor_T,
            )
            self.motor_n, self.motor_A, self.motor_T = len(A), A, T
            was_enabled = True
        self.n, self.A, self.T = len(A), A, T
        return was_enabled

    def enable_shaping(self, axis_shaper=True, motor_filter=True):
        was_disabled = False
        if axis_shaper and self.saved_axis_shaper is not None:
            self.axis_n, self.axis_A, self.axis_T = self.saved_axis_shaper
            self.saved_axis_shaper = None
            was_disabled = True
        if motor_filter and self.saved_motor_filter is not None:
            self.motor_n, self.motor_A, self.motor_T = self.saved_motor_filter
            self.saved_motor_filter = None
            was_disabled = True

        self.n, self.A, self.T = self.determine_shaper()
        return was_disabled

    def restore_shaping(self):
        was_disabled = False
        if self.cached_axis_shaper is not None:
            self.axis_n, self.axis_A, self.axis_T = self.cached_axis_shaper
            self.cached_axis_shaper = None
            was_disabled = True
        if self.cached_motor_filter is not None:
            self.motor_n, self.motor_A, self.motor_T = self.cached_motor_filter
            self.cached_motor_filter = None
            was_disabled = True

        self.n, self.A, self.T = self.determine_shaper()
        return was_disabled

    def report(self, gcmd):
        info = " ".join(
            [
                "%s_%s:%s" % (key, self.get_axis(), value)
                for (key, value) in self.params.get_status().items()
            ]
        )
        gcmd.respond_info(info)


class TypedInputSmootherParams:
    smoothers = {s.name: s.init_func for s in shaper_defs.INPUT_SMOOTHERS}

    def __init__(self, axis, smoother_type, config):
        self.axis = axis
        self.smoother_type = smoother_type
        self.damping_ratio = shaper_defs.DEFAULT_DAMPING_RATIO
        self.smoother_freq = 0.0
        if config is not None:
            if smoother_type not in self.smoothers:
                raise config.error(
                    "Unsupported shaper type: %s" % (smoother_type,)
                )
            if self._supports_damping_ratio(smoother_type):
                self.damping_ratio = config.getfloat(
                    "damping_ratio_" + axis,
                    self.damping_ratio,
                    minval=0.0,
                    maxval=0.25,
                )
            self.smoother_freq = config.getfloat(
                "smoother_freq_" + axis, self.smoother_freq, minval=0.0
            )

    def _supports_damping_ratio(self, smoother_type):
        getargspec = (
            inspect.getfullargspec
            if sys.version_info.major >= 3
            else inspect.getargspec
        )
        return "damping_ratio" in getargspec(self.smoothers[smoother_type]).args

    def get_type(self):
        return self.smoother_type

    def get_axis(self):
        return self.axis

    def update(self, smoother_type, gcmd):
        if smoother_type not in self.smoothers:
            raise gcmd.error("Unsupported shaper type: %s" % (smoother_type,))
        axis = self.axis.upper()
        if self._supports_damping_ratio(smoother_type):
            self.damping_ratio = gcmd.get_float(
                "DAMPING_RATIO_" + axis,
                self.damping_ratio,
                minval=0.0,
                maxval=1.0,
            )
        else:
            self.damping_ratio = shaper_defs.DEFAULT_DAMPING_RATIO
        self.smoother_freq = gcmd.get_float(
            "SMOOTHER_FREQ_" + axis, self.smoother_freq, minval=0.0
        )
        self.smoother_type = smoother_type

    def get_smoother(self):
        if not self.smoother_freq:
            C, tsm = shaper_defs.get_none_smoother()
        else:
            C, tsm = self.smoothers[self.smoother_type](
                self.smoother_freq, self.damping_ratio, normalize_coeffs=False
            )
        return len(C), C, tsm

    def get_status(self):
        return collections.OrderedDict(
            [
                ("shaper_type", self.smoother_type),
                ("smoother_freq", "%.3f" % (self.smoother_freq,)),
                ("damping_ratio", "%.6f" % (self.damping_ratio,)),
            ]
        )


class CustomInputSmootherParams:
    SHAPER_TYPE = "smoother"

    def __init__(self, axis, config):
        self.axis = axis
        self.coeffs, self.smooth_time = shaper_defs.get_none_smoother()
        if config is not None:
            self.smooth_time = config.getfloat(
                "smooth_time_" + axis, self.smooth_time, minval=0.0
            )
            self.coeffs = list(
                reversed(config.getfloatlist("coeffs_" + axis, self.coeffs))
            )

    def get_type(self):
        return self.SHAPER_TYPE

    def get_axis(self):
        return self.axis

    def update(self, shaper_type, gcmd):
        if shaper_type != self.SHAPER_TYPE:
            raise gcmd.error("Unsupported shaper type: %s" % (shaper_type,))
        axis = self.axis.upper()
        self.smooth_time = gcmd.get_float(
            "SMOOTH_TIME_" + axis, self.smooth_time
        )
        coeffs_str = gcmd.get("COEFFS_" + axis, None)
        if coeffs_str is not None:
            try:
                coeffs = parse_float_list(coeffs_str)
                coeffs.reverse()
            except:
                raise gcmd.error("Invalid format for COEFFS parameter")
            self.coeffs = coeffs

    def get_smoother(self):
        return len(self.coeffs), self.coeffs, self.smooth_time

    def get_status(self):
        return collections.OrderedDict(
            [
                ("shaper_type", self.SHAPER_TYPE),
                (
                    "shaper_coeffs",
                    ",".join(["%.9e" % (a,) for a in reversed(self.coeffs)]),
                ),
                ("shaper_smooth_time", self.smooth_time),
            ]
        )


class AxisInputSmoother:
    def __init__(self, params):
        self.params = params
        self.n, self.coeffs, self.smooth_time = params.get_smoother()
        self.t_offs = shaper_defs.get_smoother_offset(
            self.coeffs, self.smooth_time, normalized=False
        )
        self.saved_smooth_time = 0.0

    def get_name(self):
        return "smoother_" + self.get_axis()

    def get_type(self):
        return self.params.get_type()

    def get_axis(self):
        return self.params.get_axis()

    def is_extruder_smoothing(self, exact_mode):
        return True

    def is_enabled(self):
        return self.smooth_time > 0.0

    def update(self, shaper_type, gcmd):
        self.params.update(shaper_type, gcmd)
        self.n, self.coeffs, self.smooth_time = self.params.get_smoother()
        self.t_offs = shaper_defs.get_smoother_offset(
            self.coeffs, self.smooth_time, normalized=False
        )

    def update_stepper_kinematics(self, sk):
        ffi_main, ffi_lib = chelper.get_ffi()
        axis = self.get_axis().encode()
        success = (
            ffi_lib.input_shaper_set_smoother_params(
                sk, axis, self.n, self.coeffs, self.smooth_time
            )
            == 0
        )
        if not success:
            self.disable_shaping()
            ffi_lib.input_shaper_set_smoother_params(
                sk, axis, self.n, self.coeffs, self.smooth_time
            )
        return success

    def update_extruder_kinematics(self, sk, exact_mode):
        ffi_main, ffi_lib = chelper.get_ffi()
        axis = self.get_axis().encode()
        # Make sure to disable any active input shaping
        A, T = shaper_defs.get_none_shaper()
        ffi_lib.extruder_set_shaper_params(sk, axis, len(A), A, T)
        if exact_mode:
            success = (
                ffi_lib.extruder_set_smoothing_params(
                    sk, axis, self.n, self.coeffs, self.smooth_time, self.t_offs
                )
                == 0
            )
        else:
            smoother_type = self.get_type()
            status = self.params.get_status()
            damping_ratio = float(
                status.get("damping_ratio", shaper_defs.DEFAULT_DAMPING_RATIO)
            )
            C_e, t_sm = extruder_smoother.get_extruder_smoother(
                smoother_type,
                self.smooth_time,
                damping_ratio,
                normalize_coeffs=False,
            )
            smoother_offset = self.t_offs + 0.5 * (self.smooth_time - t_sm)
            success = (
                ffi_lib.extruder_set_smoothing_params(
                    sk, axis, len(C_e), C_e, t_sm, smoother_offset
                )
                == 0
            )
        if not success:
            self.disable_shaping()
            ffi_lib.extruder_set_smoothing_params(
                sk, axis, self.n, self.coeffs, self.smooth_time, 0.0
            )
        return success

    def disable_shaping(self, axis_shaper=None, motor_filter=None):
        was_enabled = False
        if self.smooth_time:
            self.saved_smooth_time = self.smooth_time
            was_enabled = True
        self.smooth_time = 0.0
        return was_enabled

    def enable_shaping(self, axis_shaper=None, motor_filter=None):
        if not self.saved_smooth_time:
            # Input smoother was not disabled
            return False
        self.smooth_time = self.saved_smooth_time
        self.saved_smooth_time = 0.0
        return True

    def report(self, gcmd):
        info = " ".join(
            [
                "%s_%s:%s" % (key, self.get_axis(), value)
                for (key, value) in self.params.get_status().items()
            ]
        )
        gcmd.respond_info(info)


class ShaperFactory:
    def __init__(self):
        pass

    def _create_shaper(self, axis, type_name, config=None):
        if type_name == CustomInputSmootherParams.SHAPER_TYPE:
            return AxisInputSmoother(CustomInputSmootherParams(axis, config))
        if type_name == CustomInputShaperParams.SHAPER_TYPE:
            return AxisInputShaper(CustomInputShaperParams(axis, config))
        if type_name in TypedInputSmootherParams.smoothers:
            return AxisInputSmoother(
                TypedInputSmootherParams(axis, type_name, config)
            )
        if type_name in TypedInputShaperParams.shapers:
            return AxisInputShaper(
                TypedInputShaperParams(axis, type_name, config)
            )
        return None

    def create_shaper(self, axis, config):
        shaper_type = config.get("shaper_type", "mzv")
        shaper_type = config.get("shaper_type_" + axis, shaper_type).lower()
        shaper = self._create_shaper(axis, shaper_type, config)
        if shaper is None:
            raise config.error("Unsupported shaper type '%s'" % (shaper_type,))
        return shaper

    def update_shaper(self, shaper, gcmd):
        shaper_type = gcmd.get("SHAPER_TYPE", None)
        if shaper_type is None:
            shaper_type = gcmd.get(
                "SHAPER_TYPE_" + shaper.get_axis().upper(), shaper.get_type()
            )
        shaper_type = shaper_type.lower()
        try:
            shaper.update(shaper_type, gcmd)
            return shaper
        except gcmd.error:
            pass
        shaper = self._create_shaper(shaper.get_axis(), shaper_type)
        if shaper is None:
            raise gcmd.error("Unsupported shaper type '%s'" % (shaper_type,))
        shaper.update(shaper_type, gcmd)
        return shaper


class InputShaper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.toolhead = None
        self.extruders = []
        self.exact_mode = 0
        self.config_extruder_names = config.getlist("enabled_extruders", [])
        self.shaper_factory = ShaperFactory()
        self.shapers = [
            self.shaper_factory.create_shaper("x", config),
            self.shaper_factory.create_shaper("y", config),
        ]
        self.input_shaper_stepper_kinematics = []
        self.orig_stepper_kinematics = []
        # Register gcode commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SET_INPUT_SHAPER",
            self.cmd_SET_INPUT_SHAPER,
            desc=self.cmd_SET_INPUT_SHAPER_help,
        )
        gcode.register_command(
            "GET_INPUT_SHAPER",
            self.cmd_GET_INPUT_SHAPER,
            desc=self.cmd_GET_INPUT_SHAPER_help,
        )
        gcode.register_command(
            "ENABLE_INPUT_SHAPER",
            self.cmd_ENABLE_INPUT_SHAPER,
            desc=self.cmd_ENABLE_INPUT_SHAPER_help,
        )
        gcode.register_command(
            "DISABLE_INPUT_SHAPER",
            self.cmd_DISABLE_INPUT_SHAPER,
            desc=self.cmd_DISABLE_INPUT_SHAPER_help,
        )

    def get_shapers(self):
        return self.shapers

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        for en in self.config_extruder_names:
            extruder = self.printer.lookup_object(en)
            if not hasattr(extruder, "get_extruder_steppers"):
                raise self.printer.config_error(
                    "Invalid extruder '%s' in [input_shaper]" % (en,)
                )
            self.extruders.append(extruder)
        # Configure initial values
        self._update_input_shaping(error=self.printer.config_error)

    def _get_input_shaper_stepper_kinematics(self, stepper):
        # Lookup stepper kinematics
        sk = stepper.get_stepper_kinematics()
        if sk in self.orig_stepper_kinematics:
            # Already processed this stepper kinematics unsuccessfully
            return None
        if sk in self.input_shaper_stepper_kinematics:
            return sk
        self.orig_stepper_kinematics.append(sk)
        ffi_main, ffi_lib = chelper.get_ffi()
        is_sk = ffi_main.gc(ffi_lib.input_shaper_alloc(), ffi_lib.free)
        stepper.set_stepper_kinematics(is_sk)
        res = ffi_lib.input_shaper_set_sk(is_sk, sk)
        if res < 0:
            stepper.set_stepper_kinematics(sk)
            return None
        self.input_shaper_stepper_kinematics.append(is_sk)
        return is_sk

    def _update_input_shaping(self, error=None):
        self.toolhead.flush_step_generation()
        ffi_main, ffi_lib = chelper.get_ffi()
        kin = self.toolhead.get_kinematics()
        failed_shapers = []
        for s in kin.get_steppers():
            if s.get_trapq() is None:
                continue
            is_sk = self._get_input_shaper_stepper_kinematics(s)
            if is_sk is None:
                continue
            old_delay = ffi_lib.input_shaper_get_step_gen_window(is_sk)
            for shaper in self.shapers:
                if shaper in failed_shapers:
                    continue
                if not shaper.update_stepper_kinematics(is_sk):
                    failed_shapers.append(shaper)
            new_delay = ffi_lib.input_shaper_get_step_gen_window(is_sk)
            if old_delay != new_delay:
                self.toolhead.note_step_generation_scan_time(
                    new_delay, old_delay
                )
        for e in self.extruders:
            for es in e.get_extruder_steppers():
                failed_shapers.extend(
                    es.update_input_shaping(self.shapers, self.exact_mode)
                )
        if failed_shapers:
            error = error or self.printer.command_error
            raise error(
                "Failed to configure shaper(s) %s with given parameters"
                % (", ".join([s.get_name() for s in failed_shapers]))
            )

    def disable_shaping(self):
        for shaper in self.shapers:
            shaper.disable_shaping()
        self._update_input_shaping()

    def enable_shaping(self):
        for shaper in self.shapers:
            shaper.enable_shaping()
        self._update_input_shaping()

    def cache_shaping(self):
        for shaper in self.shapers:
            shaper.cache_shaping()
        self._update_input_shaping()

    def restore_shaping(self):
        for shaper in self.shapers:
            shaper.restore_shaping()
        self._update_input_shaping()

    cmd_SET_INPUT_SHAPER_help = "Set cartesian parameters for input shaper"

    def cmd_SET_INPUT_SHAPER(self, gcmd):
        verbose = gcmd.get_int("VERBOSE", 1, minval=0, maxval=1)
        if gcmd.get_command_parameters():
            self.shapers = [
                self.shaper_factory.update_shaper(shaper, gcmd)
                for shaper in self.shapers
            ]
            self._update_input_shaping()
        if verbose:
            for shaper in self.shapers:
                shaper.report(gcmd)

    cmd_GET_INPUT_SHAPER_help = "Report input shaper paramters"

    def cmd_GET_INPUT_SHAPER(self, gcmd):
        for shaper in self.shapers:
            shaper.report(gcmd)

    cmd_ENABLE_INPUT_SHAPER_help = "Enable input shaper for given objects"

    def cmd_ENABLE_INPUT_SHAPER(self, gcmd):
        self.toolhead.flush_step_generation()
        axes = gcmd.get("AXIS", "")
        axis_shaper = gcmd.get_int("AXIS_SHAPER", 1, minval=0, maxval=1)
        motor_filter = gcmd.get_int("MOTOR_FILTER", 1, minval=0, maxval=1)
        verbose = gcmd.get_int("VERBOSE", 1, minval=0, maxval=1)
        msg = ""
        for axis_str in axes.split(","):
            axis = axis_str.strip().lower()
            if not axis:
                continue
            shapers = [s for s in self.shapers if s.get_axis() == axis]
            if not shapers:
                raise gcmd.error("Invalid AXIS='%s'" % (axis_str,))
            for s in shapers:
                if s.enable_shaping(axis_shaper, motor_filter):
                    msg += "Enabled input shaper for AXIS='%s'\n" % (axis_str,)
                else:
                    msg += (
                        "Cannot enable input shaper for AXIS='%s': "
                        "was not disabled\n" % (axis_str,)
                    )
        extruders = gcmd.get("EXTRUDER", "")
        self.exact_mode = gcmd.get_int("EXACT", self.exact_mode)
        for en in extruders.split(","):
            extruder_name = en.strip()
            if not extruder_name:
                continue
            extruder = self.printer.lookup_object(extruder_name)
            if not hasattr(extruder, "get_extruder_steppers"):
                raise gcmd.error("Invalid EXTRUDER='%s'" % (en,))
            if extruder not in self.extruders:
                self.extruders.append(extruder)
                msg += "Enabled input shaper for '%s'\n" % (en,)
            else:
                msg += "Input shaper already enabled for '%s'\n" % (en,)
        self._update_input_shaping()
        if verbose:
            gcmd.respond_info(msg)

    cmd_DISABLE_INPUT_SHAPER_help = "Disable input shaper for given objects"

    def cmd_DISABLE_INPUT_SHAPER(self, gcmd):
        self.toolhead.flush_step_generation()
        axes = gcmd.get("AXIS", "")
        axis_shaper = gcmd.get_int("AXIS_SHAPER", 1, minval=0, maxval=1)
        motor_filter = gcmd.get_int("MOTOR_FILTER", 1, minval=0, maxval=1)
        verbose = gcmd.get_int("VERBOSE", 1, minval=0, maxval=1)
        msg = ""
        for axis_str in axes.split(","):
            axis = axis_str.strip().lower()
            if not axis:
                continue
            shapers = [s for s in self.shapers if s.get_axis() == axis]
            if not shapers:
                raise gcmd.error("Invalid AXIS='%s'" % (axis_str,))
            for s in shapers:
                if s.disable_shaping(axis_shaper, motor_filter):
                    msg += "Disabled input shaper for AXIS='%s'\n" % (axis_str,)
                else:
                    msg += (
                        "Cannot disable input shaper for AXIS='%s': not "
                        "enabled or was already disabled\n" % (axis_str,)
                    )
        extruders = gcmd.get("EXTRUDER", "")
        for en in extruders.split(","):
            extruder_name = en.strip()
            if not extruder_name:
                continue
            extruder = self.printer.lookup_object(extruder_name)
            if extruder in self.extruders:
                to_re_enable = [s for s in self.shapers if s.cache_shaping()]
                for es in extruder.get_extruder_steppers():
                    es.update_input_shaping(self.shapers, self.exact_mode)
                for shaper in to_re_enable:
                    shaper.restore_shaping()
                self.extruders.remove(extruder)
                msg += "Disabled input shaper for '%s'\n" % (en,)
            else:
                msg += "Input shaper not enabled for '%s'\n" % (en,)
        self._update_input_shaping()
        if verbose:
            gcmd.respond_info(msg)


def load_config(config):
    return InputShaper(config)
