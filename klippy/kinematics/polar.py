# Code for handling the kinematics of polar robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
from klippy import stepper


class PolarKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis steppers
        stepper_bed = stepper.PrinterStepper(
            config.getsection("stepper_bed"), units_in_radians=True
        )
        rail_arm = stepper.PrinterRail(config.getsection("stepper_arm"))
        rail_z = stepper.LookupMultiRail(config.getsection("stepper_z"))
        stepper_bed.setup_itersolve("polar_stepper_alloc", b"a")
        rail_arm.setup_itersolve("polar_stepper_alloc", b"r")
        rail_z.setup_itersolve("cartesian_stepper_alloc", b"z")
        self.rails = [rail_arm, rail_z]
        self.steppers = [stepper_bed] + [
            s for r in self.rails for s in r.get_steppers()
        ]
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler(
            "stepper_enable:motor_off", self._motor_off
        )

        self.printer.register_event_handler(
            "stepper_enable:disable_bed", self._set_unhomed_xy
        )
        self.printer.register_event_handler(
            "stepper_enable:disable_arm", self._set_unhomed_xy
        )
        self.printer.register_event_handler(
            "stepper_enable:disable_z", self._set_unhomed_z
        )

        self.printer.register_event_handler(
            "unhome:mark_as_unhomed_x", self._set_unhomed_xy
        )
        self.printer.register_event_handler(
            "unhome:mark_as_unhomed_y", self._set_unhomed_xy
        )
        self.printer.register_event_handler(
            "unhome:mark_as_unhomed_z", self._set_unhomed_z
        )

        self.printer.register_event_handler(
            "force_move:mark_as_homed_x", self._set_homed_xy
        )
        self.printer.register_event_handler(
            "force_move:mark_as_homed_y", self._set_homed_xy
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
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.0
        max_xy = self.rails[0].get_range()[1]
        min_z, max_z = self.rails[1].get_range()
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, min_z, 0.0)
        self.axes_max = toolhead.Coord(max_xy, max_xy, max_z, 0.0)
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
        return list(self.steppers)

    def calc_position(self, stepper_positions):
        bed_angle = stepper_positions[self.steppers[0].get_name()]
        arm_pos = stepper_positions[self.rails[0].get_name()]
        z_pos = stepper_positions[self.rails[1].get_name()]
        return [
            math.cos(bed_angle) * arm_pos,
            math.sin(bed_angle) * arm_pos,
            z_pos,
        ]

    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
        if 2 in homing_axes:
            self.limit_z = self.rails[1].get_range()
        if 0 in homing_axes and 1 in homing_axes:
            self.limit_xy2 = self.rails[0].get_range()[1] ** 2

    def note_z_not_homed(self):
        self.clear_homing_state([2])

    def clear_homing_state(self, axes):
        if 0 in axes or 1 in axes:
            # X and Y cannot be cleared separately
            self.limit_xy2 = -1.0
        if 2 in axes:
            self.limit_z = (1.0, -1.0)

    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        if axis == 0:
            homepos[1] = 0.0
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= hi.position_endstop - position_min
        else:
            forcepos[axis] += position_max - hi.position_endstop
        # Perform homing
        axis_name = (
            "x"
            if axis == 0
            else "y"
            if axis == 1
            else "z"
            if axis == 2
            else None
        )
        if axis_name is not None:
            self.printer.send_event("homing:homing_move_begin_%s" % axis_name)
        try:
            homing_state.home_rails([rail], forcepos, homepos)
        finally:
            if axis_name is not None:
                self.printer.send_event("homing:homing_move_end_%s" % axis_name)

    def home(self, homing_state):
        # Always home XY together
        homing_axes = homing_state.get_axes()
        home_xy = 0 in homing_axes or 1 in homing_axes
        home_z = 2 in homing_axes
        updated_axes = []
        if home_xy:
            updated_axes = [0, 1]
        if home_z:
            updated_axes.append(2)
        homing_state.set_axes(updated_axes)
        # Do actual homing
        if home_xy:
            self._home_axis(homing_state, 0, self.rails[0])
        if home_z:
            self._home_axis(homing_state, 2, self.rails[1])

    def _motor_off(self, print_time):
        self.clear_homing_state((0, 1, 2))

    def _set_unhomed_xy(self, print_time):
        self.limit_xy2 = -1.0

    def _set_unhomed_z(self, print_time):
        self.limit_z = (1.0, -1.0)

    def _set_homed_xy(self, print_time):
        self.limit_xy2 = self.rails[0].get_range()[1] ** 2

    def _set_homed_z(self, print_time):
        self.limit_z = self.rails[1].get_range()

    def check_move(self, move):
        end_pos = move.end_pos
        xy2 = end_pos[0] ** 2 + end_pos[1] ** 2
        if xy2 > self.limit_xy2:
            if self.limit_xy2 < 0.0:
                raise move.move_error("Must home axis first")
            raise move.move_error()
        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
            # Move with Z - update velocity and accel for slower Z axis
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(
                self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio
            )

    def get_status(self, eventtime):
        xy_home = "xy" if self.limit_xy2 >= 0.0 else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            "kinematics": "polar",
            "homed_axes": xy_home + z_home,
            "axis_minimum": self.axes_min,
            "axis_maximum": self.axes_max,
        }


def load_kinematics(toolhead, config):
    return PolarKinematics(toolhead, config)
