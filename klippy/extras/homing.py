# Helper code for implementing homing operations
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import math
import logging
from .danger_options import get_danger_options

# HOMING_START_DELAY = 0.001
# ENDSTOP_SAMPLE_TIME = 0.000015
# ENDSTOP_SAMPLE_COUNT = 4
HOMING_START_DELAY = get_danger_options().homing_start_delay
ENDSTOP_SAMPLE_TIME = get_danger_options().endstop_sample_time
ENDSTOP_SAMPLE_COUNT = get_danger_options().endstop_sample_count


# Return a completion that completes when all completions in a list complete
def multi_complete(printer, completions):
    if len(completions) == 1:
        return completions[0]
    # Build completion that waits for all completions
    reactor = printer.get_reactor()
    cp = reactor.register_callback(lambda e: [c.wait() for c in completions])
    # If any completion indicates an error, then exit main completion early
    for c in completions:
        reactor.register_callback(
            lambda e, c=c: cp.complete(1) if c.wait() else 0
        )
    return cp


# Tracking of stepper positions during a homing/probing move
class StepperPosition:
    def __init__(self, stepper, endstop_name):
        self.stepper = stepper
        self.endstop_name = endstop_name
        self.stepper_name = stepper.get_name()
        self.start_pos = stepper.get_mcu_position()
        self.start_cmd_pos = stepper.mcu_to_commanded_position(self.start_pos)
        self.halt_pos = self.trig_pos = None

    def note_home_end(self, trigger_time):
        self.halt_pos = self.stepper.get_mcu_position()
        self.trig_pos = self.stepper.get_past_mcu_position(trigger_time)

    def verify_no_probe_skew(self, haltpos):
        new_start_pos = self.stepper.get_mcu_position(self.start_cmd_pos)
        if new_start_pos != self.start_pos:
            logging.warning(
                "Stepper '%s' position skew after probe: pos %d now %d",
                self.stepper.get_name(),
                self.start_pos,
                new_start_pos,
            )


# Implementation of homing/probing moves
class HomingMove:
    def __init__(self, printer, endstops, toolhead=None):
        self.printer = printer
        self.endstops = [es for es in endstops if es[0].get_steppers()]
        if toolhead is None:
            toolhead = printer.lookup_object("toolhead")
        self.toolhead = toolhead
        self.stepper_positions = []
        self.distance_elapsed = []

    def get_mcu_endstops(self):
        return [es for es, name in self.endstops]

    def _calc_endstop_rate(self, mcu_endstop, movepos, speed):
        startpos = self.toolhead.get_position()
        axes_d = [mp - sp for mp, sp in zip(movepos, startpos)]
        move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
        move_t = move_d / speed
        max_steps = max(
            [
                (
                    abs(
                        s.calc_position_from_coord(startpos)
                        - s.calc_position_from_coord(movepos)
                    )
                    / s.get_step_dist()
                )
                for s in mcu_endstop.get_steppers()
            ]
        )
        if max_steps <= 0.0:
            return 0.001
        return move_t / max_steps

    def calc_toolhead_pos(self, kin_spos, offsets):
        kin_spos = dict(kin_spos)
        kin = self.toolhead.get_kinematics()
        for stepper in kin.get_steppers():
            sname = stepper.get_name()
            kin_spos[sname] += offsets.get(sname, 0) * stepper.get_step_dist()
        thpos = self.toolhead.get_position()
        cpos = kin.calc_position(kin_spos)
        return [
            cp if cp is not None else tp for cp, tp in zip(cpos, thpos[:3])
        ] + thpos[3:]

    def homing_move(
        self,
        movepos,
        speed,
        probe_pos=False,
        triggered=True,
        check_triggered=True,
    ):
        # Notify start of homing/probing move
        self.printer.send_event("homing:homing_move_begin", self)
        # Note start location
        self.toolhead.flush_step_generation()
        kin = self.toolhead.get_kinematics()
        kin_spos = {
            s.get_name(): s.get_commanded_position() for s in kin.get_steppers()
        }
        self.stepper_positions = [
            StepperPosition(s, name)
            for es, name in self.endstops
            for s in es.get_steppers()
        ]
        # Start endstop checking
        print_time = self.toolhead.get_last_move_time()
        endstop_triggers = []
        for mcu_endstop, name in self.endstops:
            rest_time = self._calc_endstop_rate(mcu_endstop, movepos, speed)
            wait = mcu_endstop.home_start(
                print_time,
                ENDSTOP_SAMPLE_TIME,
                ENDSTOP_SAMPLE_COUNT,
                rest_time,
                triggered=triggered,
            )
            endstop_triggers.append(wait)
        all_endstop_trigger = multi_complete(self.printer, endstop_triggers)

        self.toolhead.dwell(HOMING_START_DELAY)
        # Issue move
        error = None
        try:
            self.toolhead.drip_move(movepos, speed, all_endstop_trigger)
        except self.printer.command_error as e:
            error = "Error during homing move: %s" % (str(e),)
        # Wait for endstops to trigger
        trigger_times = {}
        move_end_print_time = self.toolhead.get_last_move_time()
        for mcu_endstop, name in self.endstops:
            try:
                trigger_time = mcu_endstop.home_wait(move_end_print_time)
            except self.printer.command_error as e:
                if error is None:
                    error = "Error during homing %s: %s" % (name, str(e))
                continue
            if trigger_time > 0.0:
                trigger_times[name] = trigger_time
            elif check_triggered and error is None:
                error = "No trigger on %s after full movement" % (name,)
        # Determine stepper halt positions
        self.toolhead.flush_step_generation()
        for sp in self.stepper_positions:
            tt = trigger_times.get(sp.endstop_name, move_end_print_time)
            sp.note_home_end(tt)
        if probe_pos:
            halt_steps = {
                sp.stepper_name: sp.halt_pos - sp.start_pos
                for sp in self.stepper_positions
            }
            trig_steps = {
                sp.stepper_name: sp.trig_pos - sp.start_pos
                for sp in self.stepper_positions
            }
            haltpos = trigpos = self.calc_toolhead_pos(kin_spos, trig_steps)
            if trig_steps != halt_steps:
                haltpos = self.calc_toolhead_pos(kin_spos, halt_steps)
            self.toolhead.set_position(haltpos)
            for sp in self.stepper_positions:
                sp.verify_no_probe_skew(haltpos)
        else:
            haltpos = trigpos = movepos
            over_steps = {
                sp.stepper_name: sp.halt_pos - sp.trig_pos
                for sp in self.stepper_positions
            }
            steps_moved = {
                sp.stepper_name: (sp.halt_pos - sp.start_pos)
                * sp.stepper.get_step_dist()
                for sp in self.stepper_positions
            }
            filled_steps_moved = {
                sname: steps_moved.get(sname, 0)
                for sname in [s.get_name() for s in kin.get_steppers()]
            }
            self.distance_elapsed = kin.calc_position(filled_steps_moved)
            if any(over_steps.values()):
                self.toolhead.set_position(movepos)
                halt_kin_spos = {
                    s.get_name(): s.get_commanded_position()
                    for s in kin.get_steppers()
                }
                haltpos = self.calc_toolhead_pos(halt_kin_spos, over_steps)
            self.toolhead.set_position(haltpos)
        # Signal homing/probing move complete
        try:
            self.printer.send_event("homing:homing_move_end", self)
        except self.printer.command_error as e:
            if error is None:
                error = str(e)
        if error is not None:
            raise self.printer.command_error(error)
        return trigpos

    def check_no_movement(self):
        if self.printer.get_start_args().get("debuginput") is not None:
            return None
        for sp in self.stepper_positions:
            if sp.start_pos == sp.trig_pos:
                return sp.endstop_name
        return None

    def moved_less_than_dist(self, min_dist, homing_axes):
        homing_axis_distances = [
            dist
            for i, dist in enumerate(self.distance_elapsed)
            if i in homing_axes
        ]
        distance_tolerance = (
            get_danger_options().homing_elapsed_distance_tolerance
        )
        if any(
            [
                abs(dist) < min_dist
                and min_dist - abs(dist) >= distance_tolerance
                for dist in homing_axis_distances
            ]
        ):
            return True
        return False


# State tracking of homing requests
class Homing:
    def __init__(self, printer):
        self.printer = printer
        self.toolhead = printer.lookup_object("toolhead")
        self.gcode = self.printer.lookup_object("gcode")
        self.changed_axes = []
        self.trigger_mcu_pos = {}
        self.adjust_pos = {}

    def set_axes(self, axes):
        self.changed_axes = axes

    def get_axes(self):
        return self.changed_axes

    def get_trigger_position(self, stepper_name):
        return self.trigger_mcu_pos[stepper_name]

    def set_stepper_adjustment(self, stepper_name, adjustment):
        self.adjust_pos[stepper_name] = adjustment

    def _fill_coord(self, coord):
        # Fill in any None entries in 'coord' with current toolhead position
        thcoord = list(self.toolhead.get_position())
        for i in range(len(coord)):
            if coord[i] is not None:
                thcoord[i] = coord[i]
        return thcoord

    def set_homed_position(self, pos):
        self.toolhead.set_position(self._fill_coord(pos))

    def _set_homing_accel(self, accel, pre_homing):
        if accel is None:
            return
        if pre_homing:
            self.toolhead.set_accel(accel)
        else:
            self.toolhead.reset_accel()

    def _set_homing_current(self, homing_axes, pre_homing, perform_dwell=False):
        print_time = self.toolhead.get_last_move_time()
        affected_rails = set()
        for axis in homing_axes:
            axis_name = "xyz"[axis]  # only works for cartesian
            partial_rails = self.toolhead.get_active_rails_for_axis(axis_name)
            affected_rails = affected_rails | set(partial_rails)

        dwell_time = 0.0
        for rail in affected_rails:
            chs = rail.get_tmc_current_helpers()
            for ch in chs:
                if ch is not None:
                    current_dwell_time = ch.set_current_for_homing(
                        print_time, pre_homing, get_dwell_time=perform_dwell
                    )
                    dwell_time = max(dwell_time, current_dwell_time)

        if dwell_time:
            self.toolhead.dwell(dwell_time)

    def _reset_endstop_states(self, endstops):
        # re-querying a tmc endstop seems to reset the state
        # otherwise it triggers almost immediately upon second home
        # this seems to be an adequate substitute for a 2 second dwell.
        print_time = self.toolhead.get_last_move_time()
        for endstop in endstops:
            endstop[0].query_endstop(print_time)

    def _calc_mean(self, positions):
        return sum(positions) / float(len(positions))

    def _calc_median(self, positions):
        z_sorted = sorted(positions)
        middle = len(positions) // 2
        if (len(positions) & 1) == 1:
            # odd number of samples
            return z_sorted[middle]
        # even number of samples
        return self._calc_mean(z_sorted[middle - 1 : middle + 1])

    def init_homing(self, hi, homing_axes):
        pass

    def process_homing_info(self, hi):
        return hi

    def home_rails(self, rails, forcepos, movepos):
        # Notify of upcoming homing operation
        self.printer.send_event("homing:home_rails_begin", self, rails)
        # Alter kinematics class to think printer is at forcepos
        homing_axes = [axis for axis in range(3) if forcepos[axis] is not None]
        # Perform first home
        endstops = [es for rail in rails for es in rail.get_endstops()]
        hi = self.process_homing_info(rails[0].get_homing_info())
        self.init_homing(hi, homing_axes)
        needs_rehome = False
        retract_dist = hi.retract_dist
        hmove = HomingMove(self.printer, endstops)

        distances = []
        retries = 0
        first_home = True
        drop = hi.drop_first_result

        def _process_samples():
            nonlocal drop, first_home, distances, retries
            if hi.sample_count == 1:
                distances.append([0] * len(hmove.distance_elapsed))
                return
            if not drop:
                if first_home:
                    result = [0] * len(hmove.distance_elapsed)
                    first_home = False
                else:
                    result = [
                        abs(dist) - hi.sample_retract_dist
                        if i in homing_axes
                        else 0
                        for i, dist in enumerate(hmove.distance_elapsed)
                    ]
                distances.append(result)
                for i in homing_axes:
                    self.gcode.respond_info(
                        f"Homing sample for {'XYZ'[i]}: {result[i]}"
                    )

                if any(
                    [max(dist) > hi.samples_tolerance for dist in distances]
                ):
                    if retries >= hi.samples_retries:
                        raise self.printer.command_error(
                            "Homing samples exceed samples_tolerance"
                        )
                    self.gcode.respond_info(
                        "Homing samples exceed tolerance. Retrying..."
                    )
                    retries += 1
                    distances = []
            else:
                drop = False

            if len(distances) < hi.sample_count:
                sample_startpos = self._fill_coord(forcepos)
                sample_homepos = self._fill_coord(movepos)
                sample_axes_d = [
                    hp - sp for hp, sp in zip(sample_homepos, sample_startpos)
                ]
                sample_move_d = math.sqrt(
                    sum([d * d for d in sample_axes_d[:3]])
                )
                sample_retract_r = min(
                    1.0, hi.sample_retract_dist / sample_move_d
                )
                sample_retractpos = [
                    hp - ad * sample_retract_r
                    for hp, ad in zip(homepos, sample_axes_d)
                ]
                self.toolhead.move(sample_retractpos, hi.retract_speed)

        try:
            while len(distances) < hi.sample_count:
                startpos = self._fill_coord(forcepos)
                homepos = self._fill_coord(movepos)
                self.toolhead.set_position(startpos, homing_axes=homing_axes)
                hmove = HomingMove(self.printer, endstops)

                if (
                    drop
                    and hi.sample_count > 1
                    and hi.use_sensorless_homing
                    or not retract_dist
                ):
                    self.gcode.respond_info("Settling sample (ignored)...")
                try:
                    self._set_homing_accel(hi.accel, pre_homing=True)
                    self._set_homing_current(homing_axes, pre_homing=True)
                    self._reset_endstop_states(endstops)
                    hmove.homing_move(homepos, hi.speed)
                finally:
                    self._set_homing_accel(hi.accel, pre_homing=False)

                if hi.use_sensorless_homing and hmove.moved_less_than_dist(
                    hi.min_home_dist, homing_axes
                ):
                    needs_rehome = True
                    retract_dist = hi.min_home_dist
                    self.gcode.respond_info(
                        "Moved less than min_home_dist. Retrying..."
                    )
                    break
                if not hi.use_sensorless_homing and retract_dist:
                    break

                _process_samples()

            if (not hi.use_sensorless_homing or needs_rehome) and retract_dist:
                if needs_rehome:
                    logging.info(
                        "homing:needs rehome: %s",
                        [("X", "Y", "Z")[axis] for axis in homing_axes],
                    )
                # Retract
                startpos = self._fill_coord(forcepos)
                homepos = self._fill_coord(movepos)
                axes_d = [hp - sp for hp, sp in zip(homepos, startpos)]
                move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
                retract_r = min(1.0, retract_dist / move_d)
                retractpos = [
                    hp - ad * retract_r for hp, ad in zip(homepos, axes_d)
                ]
                self.toolhead.move(retractpos, hi.retract_speed)

                distances = []
                retries = 0
                first_home = True
                drop = hi.drop_first_result
                while len(distances) < hi.sample_count:
                    try:
                        if drop and hi.sample_count > 1:
                            self.gcode.respond_info(
                                "Settling sample (ignored)..."
                            )
                        # Home again
                        startpos = [
                            rp - ad * retract_r
                            for rp, ad in zip(retractpos, axes_d)
                        ]
                        self.toolhead.set_position(startpos)
                        self._set_homing_current(
                            homing_axes,
                            pre_homing=True,
                            perform_dwell=hi.use_sensorless_homing,
                        )
                        self._reset_endstop_states(endstops)
                        hmove = HomingMove(self.printer, endstops)
                        hmove.homing_move(homepos, hi.second_homing_speed)
                        if hmove.check_no_movement() is not None:
                            raise self.printer.command_error(
                                "Endstop %s still triggered after retract"
                                % (hmove.check_no_movement(),)
                            )
                        if (
                            hi.use_sensorless_homing
                            and needs_rehome
                            and hmove.moved_less_than_dist(
                                hi.min_home_dist, homing_axes
                            )
                        ):
                            raise self.printer.command_error(
                                "Early homing trigger on second home!"
                            )
                    finally:
                        self._set_homing_accel(hi.accel, pre_homing=False)

                    _process_samples()
        finally:
            self._set_homing_accel(hi.accel, pre_homing=False)
            self._set_homing_current(homing_axes, pre_homing=False)

        self.process_homing(distances, homing_axes)

        # Signal home operation complete
        self.toolhead.flush_step_generation()
        self.trigger_mcu_pos = {
            sp.stepper_name: sp.trig_pos for sp in hmove.stepper_positions
        }

        if len(distances) > 1:
            self.toolhead.wait_moves()
            pos = self.toolhead.get_position()
            home_pos = self.toolhead.get_position()
            calc_adjustment = (
                self._calc_median
                if hi.samples_result == "median"
                else self._calc_mean
            )
            for i in range(0, len(hmove.distance_elapsed)):
                pos[i] += (
                    calc_adjustment([dist[i] for dist in distances])
                    - distances[-1][i]
                )

            for i in homing_axes:
                self.gcode.respond_info(
                    f"Final homing position for {'XYZ'[i]}: {pos[i]}"
                )
            self.toolhead.set_position(pos)
            if hi.move_toolhead_after_adjusting:
                self.printer.lookup_object(
                    "gcode_move"
                ).last_position = home_pos
                self.toolhead.move(home_pos, hi.retract_speed)

        self.adjust_pos = {}
        self.printer.send_event("homing:home_rails_end", self, rails)
        if any(self.adjust_pos.values()):
            # Apply any homing offsets
            kin = self.toolhead.get_kinematics()
            homepos = self.toolhead.get_position()
            kin_spos = {
                s.get_name(): (
                    s.get_commanded_position()
                    + self.adjust_pos.get(s.get_name(), 0.0)
                )
                for s in kin.get_steppers()
            }
            newpos = kin.calc_position(kin_spos)
            for axis in homing_axes:
                if newpos[axis] is None:
                    raise self.printer.command_error(
                        "Cannot determine position of toolhead on "
                        "axis %s after homing" % "xyz"[axis]
                    )
                homepos[axis] = newpos[axis]
            self.toolhead.set_position(homepos)

        if hi.post_retract_dist:
            self.toolhead.wait_moves()
            startpos = self._fill_coord(forcepos)
            homepos = self.toolhead.get_position()
            axes_d = [hp - sp for hp, sp in zip(homepos, startpos)]
            move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
            retract_r = min(1.0, hi.post_retract_dist / move_d)
            retractpos = [
                hp - ad * retract_r for hp, ad in zip(homepos, axes_d)
            ]
            self.printer.lookup_object("gcode_move").last_position = retractpos
            self.toolhead.move(retractpos, hi.post_retract_speed)
        self.gcode.run_script_from_command("M400")

    def process_homing(self, distances, homing_axes):
        pass


class HomingAccuracy(Homing):
    def __init__(self, printer, gcmd):
        super().__init__(printer)
        self.gcmd = gcmd

    def init_homing(self, hi, homing_axes):
        axes = ["XYZ"[i] for i in homing_axes]
        for axis in axes:
            self.gcode.respond_info(
                "HOMING_ACCURACY for %s"
                " (samples=%d retract=%.3f"
                " speed=%.1f retract_speed=%.1f)\n"
                % (
                    axis,
                    hi.sample_count,
                    hi.sample_retract_dist,
                    hi.second_homing_speed,
                    hi.retract_speed,
                )
            )

    def process_homing_info(self, hi):
        drop_first_result = self.gcmd.get_int(
            "DROP_FIRST_RESULT", hi.drop_first_result
        )
        speed = self.gcmd.get_float("SPEED", hi.second_homing_speed, above=0.0)
        retract_speed = self.gcmd.get_float(
            "RETRACT_SPEED", hi.retract_speed, above=0.0
        )
        sample_count = self.gcmd.get_int("SAMPLES", 10, minval=1)
        sample_retract_dist = self.gcmd.get_float(
            "SAMPLE_RETRACT_DIST", hi.sample_retract_dist, above=0.0
        )
        homing_info = hi._replace(
            drop_first_result=drop_first_result,
            second_homing_speed=speed,
            retract_speed=retract_speed,
            sample_count=sample_count,
            sample_retract_dist=sample_retract_dist,
        )
        return homing_info

    def process_homing(self, distances, homing_axes):
        for i in homing_axes:
            dists = [dist[i] for dist in distances]
            min_value = min(dists)
            max_value = max(dists)
            avg_value = self._calc_mean(dists)
            median = self._calc_median(dists)
            range_value = max_value - min_value
            deviation_sum = 0
            for j in range(len(dists)):
                deviation_sum += pow(dists[j] - avg_value, 2.0)
            sigma = (deviation_sum / len(dists)) ** 0.5

            self.gcode.respond_info(
                "probe accuracy results: maximum %.6f, minimum %.6f, range %.6f, "
                "average %.6f, median %.6f, standard deviation %.6f"
                % (max_value, min_value, range_value, avg_value, median, sigma)
            )


class PrinterHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Register g-code commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("G28", self.cmd_G28)
        gcode.register_command(
            "HOMING_ACCURACY",
            self.cmd_HOMING_ACCURACY,
            desc=self.cmd_HOMING_ACCURACY_help,
        )

    def manual_home(
        self, toolhead, endstops, pos, speed, triggered, check_triggered
    ):
        hmove = HomingMove(self.printer, endstops, toolhead)
        try:
            hmove.homing_move(
                pos, speed, triggered=triggered, check_triggered=check_triggered
            )
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown"
                )
            raise

    def probing_move(self, mcu_probe, pos, speed):
        endstops = [(mcu_probe, "probe")]
        hmove = HomingMove(self.printer, endstops)
        try:
            epos = hmove.homing_move(pos, speed, probe_pos=True)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Probing failed due to printer shutdown"
                )
            raise
        if hmove.check_no_movement() is not None:
            raise self.printer.command_error(
                "Probe triggered prior to movement"
            )
        return epos

    cmd_HOMING_ACCURACY_help = "Check the accuracy of your endstops"

    def cmd_HOMING_ACCURACY(self, gcmd):
        axes = []
        for pos, axis in enumerate("XYZ"):
            if gcmd.get("AXIS") == axis:
                axes.append(pos)
        homing_state = HomingAccuracy(self.printer, gcmd)
        homing_state.set_axes(axes)
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        try:
            kin.home(homing_state)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown"
                )
            self.printer.lookup_object("stepper_enable").motor_off()
            raise

    def cmd_G28(self, gcmd):
        # Move to origin
        axes = []
        for pos, axis in enumerate("XYZ"):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
        if not axes:
            axes = [0, 1, 2]
        homing_state = Homing(self.printer)
        homing_state.set_axes(axes)
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        try:
            kin.home(homing_state)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown"
                )
            self.printer.lookup_object("stepper_enable").motor_off()
            raise


def load_config(config):
    return PrinterHoming(config)
