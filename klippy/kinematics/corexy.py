# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from klippy import stepper


AXIS_NAMES = {
    1: "x",
    2: "y",
    3: "z",
}


class CoreXYKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis rails
        self.rails = [
            stepper.LookupMultiRail(config.getsection("stepper_" + n))
            for n in "xyz"
        ]
        for s in self.rails[1].get_steppers():
            self.rails[0].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)
        self.rails[0].setup_itersolve("corexy_stepper_alloc", b"+")
        self.rails[1].setup_itersolve("corexy_stepper_alloc", b"-")
        self.rails[2].setup_itersolve("cartesian_stepper_alloc", b"z")
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler(
            "stepper_enable:motor_off", self._motor_off
        )

        self.printer.register_event_handler(
            "stepper_enable:disable_x", self._disable_xy
        )
        self.printer.register_event_handler(
            "stepper_enable:disable_y", self._disable_xy
        )
        self.printer.register_event_handler(
            "stepper_enable:disable_z", self._set_unhomed_z
        )

        self.printer.register_event_handler(
            "unhome:mark_as_unhomed_x", self._set_unhomed_x
        )
        self.printer.register_event_handler(
            "unhome:mark_as_unhomed_y", self._set_unhomed_y
        )
        self.printer.register_event_handler(
            "unhome:mark_as_unhomed_z", self._set_unhomed_z
        )

        self.printer.register_event_handler(
            "force_move:mark_as_homed_x", self._set_homed_x
        )
        self.printer.register_event_handler(
            "force_move:mark_as_homed_y", self._set_homed_y
        )
        self.printer.register_event_handler(
            "force_move:mark_as_homed_z", self._set_homed_z
        )
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            "max_z_velocity", max_velocity, above=0.0, maxval=max_velocity
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", max_accel, above=0.0, maxval=max_accel
        )
        self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.0)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.0)
        self.supports_dual_carriage = False

    def get_rails(self):
        return self.rails

    def get_connected_rails(self, axis):
        if axis == 0 or axis == 1:
            return [self.rails[0], self.rails[1]]
        elif axis == 2:
            return [self.rails[2]]
        raise IndexError("Rail does not exist")

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()

    def note_z_not_homed(self):
        self.clear_homing_state([2])

    def clear_homing_state(self, axes):
        for i, _ in enumerate(self.limits):
            if i in axes:
                self.limits[i] = (1.0, -1.0)

    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            rail = self.rails[axis]
            # Determine movement
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None, None]
            homepos[axis] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            axis_name = AXIS_NAMES[axis] if axis in AXIS_NAMES else None
            if axis_name is not None:
                self.printer.send_event(
                    "homing:homing_move_begin_%s" % axis_name
                )
            try:
                homing_state.home_rails([rail], forcepos, homepos)
            finally:
                if axis_name is not None:
                    self.printer.send_event(
                        "homing:homing_move_end_%s" % axis_name
                    )

    def _motor_off(self, print_time):
        self.clear_homing_state((0, 1, 2))

    def _set_unhomed_x(self, print_time):
        self.limits[0] = (1.0, -1.0)

    def _set_unhomed_y(self, print_time):
        self.limits[1] = (1.0, -1.0)

    def _set_unhomed_z(self, print_time):
        self.limits[2] = (1.0, -1.0)

    def _set_homed_x(self, print_time):
        self.limits[0] = self.rails[0].get_range()

    def _set_homed_y(self, print_time):
        self.limits[1] = self.rails[1].get_range()

    def _set_homed_z(self, print_time):
        self.limits[2] = self.rails[2].get_range()

    def _disable_xy(self, print_time):
        self.limits[0] = (1.0, -1.0)
        self.limits[1] = (1.0, -1.0)

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if move.axes_d[i] and (
                end_pos[i] < self.limits[i][0] or end_pos[i] > self.limits[i][1]
            ):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (
            xpos < limits[0][0]
            or xpos > limits[0][1]
            or ypos < limits[1][0]
            or ypos > limits[1][1]
        ):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio
        )

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            "kinematics": "corexy",
            "homed_axes": "".join(axes),
            "axis_minimum": self.axes_min,
            "axis_maximum": self.axes_max,
        }


def load_kinematics(toolhead, config):
    return CoreXYKinematics(toolhead, config)
