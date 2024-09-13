import collections


class ProfileManager:
    def __init__(self, heater, control_types):
        self.heater = heater
        self.control_types = control_types
        self.profiles = {}
        self.incompatible_profiles = []
        # Fetch stored profiles from Config
        stored_profs = self.heater.config.get_prefix_sections(
            "heater_profile %s" % self.heater.sensor_name
        )
        for profile in stored_profs:
            if len(self.heater.sensor_name.split(" ")) > 1:
                name = profile.get_name().split(" ", 3)[-1]
            else:
                name = profile.get_name().split(" ", 2)[-1]
            self._init_profile(profile, name)

    def _init_profile(self, config_section, name, force_control=None):
        control = self._check_value_config(
            "control", config_section, str, False, default=force_control
        )
        if control in self.control_types.keys():
            temp_profile = self.control_types[control].init_profile(
                config_section, name, self
            )
            if temp_profile is not None:
                temp_profile["name"] = name
                temp_profile["control"] = control
                self.profiles[name] = temp_profile
            return temp_profile
        else:
            raise self.heater.printer.config_error(
                "Unknown control type '%s' "
                "in [heater_profile %s %s]." % (control, self.heater.sensor_name, name)
            )

    def _check_value_config(
        self,
        key,
        config_section,
        type,
        can_be_none,
        default=None,
        above=None,
        minval=None,
    ):
        if type is int:
            value = config_section.getint(key, default=default, minval=minval)
        elif type is float:
            value = config_section.getfloat(
                key, default=default, minval=minval, above=above
            )
        elif type == "floatlist":
            value = config_section.getfloatlist(key, default=default)
        elif (
            isinstance(type, tuple)
            and len(type) == 4
            and isinstance(type[0], str)
            and type[0] == "lists"
        ):
            value = config_section.getlists(
                key,
                seps=type[1],
                parser=type[2],
                count=type[3],
                default=default,
            )
        else:
            value = config_section.get(key, default=default)
        if not can_be_none and value is None:
            raise self.heater.gcode.error(
                "heater_profile: '%s' has to be "
                "specified in [heater_profile %s %s]."
                % (
                    key,
                    self.heater.sensor_name,
                    config_section.get_name(),
                )
            )
        return value

    def _compute_section_name(self, profile_name):
        return (
            self.heater.sensor_name
            if profile_name == "default"
            else ("heater_profile " + self.heater.sensor_name + " " + profile_name)
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
            value = gcmd.get_int(name, default, minval=minval, maxval=maxval)
        elif type is float:
            value = gcmd.get_float(name, default, minval=minval, maxval=maxval)
        else:
            value = gcmd.get(name, default)
        if not can_be_none and value is None:
            raise self.heater.gcode.error(
                "heater_profile: '%s' has to be specified." % name
            )
        return value.lower() if type == "lower" else value

    def init_default_profile(self):
        return self._init_profile(self.heater.sensor_config, "default")

    def set_values(self, profile_name, gcmd, verbose=True):
        current_profile = self.heater.get_control().get_profile()
        control = self._check_value_gcmd(
            "CONTROL", current_profile["control"], gcmd, "lower", True
        )
        save_profile = self._check_value_gcmd(
            "SAVE_PROFILE", True, gcmd, int, True, minval=0, maxval=1
        )
        self.control_types[control].set_values(
            pmgr=self, gcmd=gcmd, control=control, profile_name=profile_name
        )
        if save_profile:
            self.save_profile(profile_name=profile_name, gcmd=gcmd, verbose=verbose)

    def get_values(self, profile_name, gcmd):
        temp_profile = self.heater.get_control().get_profile()
        target = temp_profile["pid_target"]
        tolerance = temp_profile["pid_tolerance"]
        control = temp_profile["control"]
        kp = temp_profile["pid_kp"]
        ki = temp_profile["pid_ki"]
        kd = temp_profile["pid_kd"]
        smooth_time = (
            self.heater.get_smooth_time()
            if temp_profile["smooth_time"] is None
            else temp_profile["smooth_time"]
        )
        smoothing_elements = (
            self.heater.get_smoothing_elements()
            if temp_profile["smoothing_elements"] is None
            else temp_profile["smoothing_elements"]
        )
        name = temp_profile["name"]
        self.heater.gcode.respond_info(
            "PID Parameters:\n"
            "Target: %.2f,\n"
            "Tolerance: %.4f\n"
            "Control: %s\n"
            "Smooth Time: %.3f\n"
            "Smoothing Elements: %d\n"
            "pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"
            "name: %s"
            % (
                target,
                tolerance,
                control,
                smooth_time,
                smoothing_elements,
                kp,
                ki,
                kd,
                name,
            )
        )

    def save_profile(self, profile_name=None, gcmd=None, verbose=True):
        temp_profile = self.heater.get_control().get_profile()
        self.control_types[temp_profile["control"]].save_profile(
            pmgr=self,
            temp_profile=temp_profile,
            profile_name=profile_name,
            gcmd=gcmd,
            verbose=verbose,
        )
        if profile_name is not None:
            self.heater.get_control().set_name(profile_name)

    def load_profile(self, profile_name, gcmd):
        verbose = self._check_value_gcmd("VERBOSE", "low", gcmd, "lower", True)
        load_clean = self._check_value_gcmd(
            "LOAD_CLEAN", 0, gcmd, int, True, minval=0, maxval=1
        )
        if (
            profile_name == self.heater.get_control().get_profile()["name"]
            and not load_clean
        ):
            if verbose == "high" or verbose == "low":
                self.heater.gcode.respond_info(
                    "Heater Profile [%s] already loaded for heater [%s]."
                    % (profile_name, self.heater.sensor_name)
                )
            return
        keep_target = self._check_value_gcmd(
            "KEEP_TARGET", 0, gcmd, int, True, minval=0, maxval=1
        )
        profile = self.profiles.get(profile_name, None)
        defaulted = False
        default = gcmd.get("DEFAULT", None)
        if profile is None:
            if default is None:
                raise self.heater.gcode.error(
                    "heater_profile: Unknown profile [%s] for heater [%s]."
                    % (profile_name, self.heater.sensor_name)
                )
            profile = self.profiles.get(default, None)
            defaulted = True
            if profile is None:
                raise self.heater.gcode.error(
                    "heater_profile: Unknown default "
                    "profile [%s] for heater [%s]." % (default, self.heater.sensor_name)
                )
        control = self.heater.lookup_control(profile, load_clean)
        self.heater.set_control(control, keep_target)

        if verbose != "high" and verbose != "low":
            return
        if defaulted:
            self.heater.gcode.respond_info(
                "Couldn't find profile "
                "[%s] for heater [%s]"
                ", defaulted to [%s]."
                % (profile_name, self.heater.sensor_name, default)
            )
        self.heater.gcode.respond_info(
            "Heater Profile [%s] loaded for heater [%s].\n"
            % (profile["name"], self.heater.sensor_name)
        )
        if verbose == "high":
            smooth_time = (
                self.heater.get_smooth_time()
                if profile["smooth_time"] is None
                else profile["smooth_time"]
            )
            smoothing_elements = (
                self.heater.get_smoothing_elements()
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
            msg += "PID Parameters: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n" % (
                profile["pid_kp"],
                profile["pid_ki"],
                profile["pid_kd"],
            )
            self.heater.gcode.respond_info(msg)

    def remove_profile(self, profile_name, gcmd):
        if profile_name in self.profiles:
            section_name = self._compute_section_name(profile_name)
            self.heater.configfile.remove_section(section_name)
            profiles = dict(self.profiles)
            del profiles[profile_name]
            self.profiles = profiles
            self.heater.gcode.respond_info(
                "Profile [%s] for heater [%s] "
                "removed from storage for this session.\n"
                "The SAVE_CONFIG command will update the printer\n"
                "configuration and restart the printer"
                % (profile_name, self.heater.sensor_name)
            )
        else:
            self.heater.gcode.respond_info(
                "No profile named [%s] to remove" % profile_name
            )

    cmd_HEATER_PROFILE_help = "Heater Profile Persistent Storage management"

    def cmd_HEATER_PROFILE(self, gcmd):
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
                    raise self.heater.gcode.error(
                        "heater_profile: Profile must be specified"
                    )
                options[key](profile_name, gcmd)
                return
        raise self.heater.gcode.error(
            "heater_profile: Invalid syntax '%s'" % (gcmd.get_commandline(),)
        )
