import logging

class ProfileManager:
    def __init__(self, heater_instance):
        self.heater_instance = heater_instance
        self.profiles = {}
        self.incompatible_profiles = []
        # Fetch stored profiles from Config
        stored_profs = self.heater_instance.config.get_prefix_sections(
            "temperature_profile %s" % self.heater_instance.sensor_name
        )
        for profile in stored_profs:
            self._init_profile(profile, profile.get_name().split(" ", 2)[2])

    def _init_profile(self, config_section, name):
        temp_profile = self.heater_instance.build_profile_from_config(config_section, name)
        self.profiles[name] = temp_profile
        return temp_profile

    def _check_value_config(self, key, config_section, type, can_be_none):
        if type is int:
            value = config_section.getint(key, None)
        elif type is float:
            value = config_section.getfloat(key, None)
        else:
            value = config_section.get(key, None)
        if not can_be_none and value is None:
            raise self.heater_instance.gcode.error(
                "pid_profile: '%s' has to be "
                "specified in [pid_profile %s %s]."
                % (
                    key,
                    self.heater_instance.sensor_name,
                    config_section.get_name(),
                )
            )
        return value

    def _compute_section_name(self, profile_name):
        return (
            self.heater_instance.sensor_name
            if profile_name == "default"
            else (
                    "temperature_profile "
                    + self.heater_instance.sensor_name
                    + " "
                    + profile_name
            )
        )

    def _check_value_gcmd(
            self,
            name,
            default,
            gcmd,
            type,
            can_be_none,
            minval=None,
            maxval=None,
    ):
        if type is int:
            value = gcmd.get_int(
                name, default, minval=minval, maxval=maxval
            )
        elif type is float:
            value = gcmd.get_float(
                name, default, minval=minval, maxval=maxval
            )
        else:
            value = gcmd.get(name, default)
        if not can_be_none and value is None:
            raise self.heater_instance.gcode.error(
                "temperature_profile: '%s' has to be specified." % name
            )
        return value.lower() if type == "lower" else value

    def init_default_profile(self):
        return self._init_profile(
            self.heater_instance.sensor_config, "default"
        )

    def set_values(self, profile_name, gcmd, verbose):
        temp_profile = self.heater_instance.build_profile_from_gcmd(gcmd)
        keep_target = self._check_value_gcmd(
            "KEEP_TARGET", 0, gcmd, int, True, minval=0, maxval=1
        )
        load_clean = self._check_value_gcmd(
            "LOAD_CLEAN", 0, gcmd, int, True, minval=0, maxval=1
        )
        temp_control = self.heater_instance.lookup_control(
            temp_profile, load_clean
        )
        self.heater_instance.set_control(temp_control, keep_target)
        self.save_profile(profile_name=profile_name, verbose=True)

    def get_values(self, profile_name, gcmd, verbose):
        pass

    def save_profile(self, profile_name=None, gcmd=None, verbose=True):
        self.heater_instance.save_profile(profile_name, verbose)

    def load_profile(self, profile_name, gcmd, verbose):
        verbose = self._check_value_gcmd(
            "VERBOSE", "low", gcmd, "lower", True
        )
        load_clean = self._check_value_gcmd(
            "LOAD_CLEAN", 0, gcmd, int, True, minval=0, maxval=1
        )
        keep_target = self._check_value_gcmd(
            "KEEP_TARGET", 0, gcmd, int, True, minval=0, maxval=1
        )
        self.heater_instance.load_profile(profile_name, load_clean, verbose)
        if (
                profile_name
                == self.heater_instance.get_control().get_profile()["name"]
                and not load_clean
        ):
            if verbose == "high" or verbose == "low":
                self.heater_instance.gcode.respond_info(
                    "PID Profile [%s] already loaded for heater [%s]."
                    % (profile_name, self.heater_instance.sensor_name)
                )
            return
        profile = self.profiles.get(profile_name, None)
        defaulted = False
        default = gcmd.get("DEFAULT", None)
        if profile is None:
            if default is None:
                raise self.heater_instance.gcode.error(
                    "pid_profile: Unknown profile [%s] for heater [%s]."
                    % (profile_name, self.heater_instance.sensor_name)
                )
            profile = self.profiles.get(default, None)
            defaulted = True
            if profile is None:
                raise self.heater_instance.gcode.error(
                    "pid_profile: Unknown default "
                    "profile [%s] for heater [%s]."
                    % (default, self.heater_instance.sensor_name)
                )
        control = self.heater_instance.lookup_control(profile, load_clean)
        self.heater_instance.set_control(control, keep_target)

        if verbose != "high" and verbose != "low":
            return
        if defaulted:
            self.heater_instance.gcode.respond_info(
                "Couldn't find profile "
                "[%s] for heater [%s]"
                ", defaulted to [%s]."
                % (profile_name, self.heater_instance.sensor_name, default)
            )
        self.heater_instance.gcode.respond_info(
            "PID Profile [%s] loaded for heater [%s].\n"
            % (profile["name"], self.heater_instance.sensor_name)
        )
        if verbose == "high":
            smooth_time = (
                self.heater_instance.get_smooth_time()
                if profile["smooth_time"] is None
                else profile["smooth_time"]
            )
            smoothing_elements = (
                self.heater_instance.get_smoothing_elements()
                if profile["smoothing_elements"] is None
                else profile["smoothing_elements"]
            )
            msg = (
                    "Target: %.2f\n"
                    "Tolerance: %.4f\n"
                    "Control: %s\n"
                    % (
                        profile["pid_target"],
                        profile["pid_tolerance"],
                        profile["control"],
                    )
            )
            if smooth_time is not None:
                msg += "Smooth Time: %.3f\n" % smooth_time
            if smoothing_elements is not None:
                msg += "Smoothing Elements: %d\n" % smoothing_elements
            msg += (
                    "PID Parameters: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"
                    % (profile["pid_kp"], profile["pid_ki"], profile["pid_kd"])
            )
            self.heater_instance.gcode.respond_info(msg)

    def remove_profile(self, profile_name, gcmd, verbose):
        if profile_name in self.profiles:
            section_name = self._compute_section_name(profile_name)
            self.heater_instance.configfile.remove_section(section_name)
            profiles = dict(self.profiles)
            del profiles[profile_name]
            self.profiles = profiles
            self.heater_instance.gcode.respond_info(
                "Profile [%s] for heater [%s] "
                "removed from storage for this session.\n"
                "The SAVE_CONFIG command will update the printer\n"
                "configuration and restart the printer"
                % (profile_name, self.heater_instance.sensor_name)
            )
        else:
            self.heater_instance.gcode.respond_info(
                "No profile named [%s] to remove" % profile_name
            )

    cmd_PID_PROFILE_help = "PID Profile Persistent Storage management"

    def cmd_PID_PROFILE(self, gcmd):
        options = collections.OrderedDict(
            {
                "LOAD": self.load_profile,
                "SAVE": self.save_profile,
                "GET_VALUES": self.get_values,
                "SET_VALUES": self.set_values,
                "REMOVE": self.remove_profile,
            }
        )
        for key in options:
            profile_name = gcmd.get(key, None)
            if profile_name is not None:
                if not profile_name.strip():
                    raise self.heater_instance.gcode.error(
                        "pid_profile: Profile must be specified"
                    )
                options[key](profile_name, gcmd, True)
                return
        raise self.heater_instance.gcode.error(
            "pid_profile: Invalid syntax '%s'" % (gcmd.get_commandline(),)
        )