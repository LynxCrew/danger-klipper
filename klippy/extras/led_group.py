# Support for LED groups
#
# Copyright (C) 2022 Julian Schill <j.schill@web.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import led


def parse_chain(chain):
    chain = chain.strip()
    leds = []
    parms = [parameter.strip() for parameter in chain.split() if parameter.strip()]
    if parms:
        chainName = parms[0].replace(":", " ")
        ledIndices = "".join(parms[1:]).strip("()").split(",")
        for led in ledIndices:
            if led:
                if "-" in led:
                    start, stop = map(int, led.split("-"))
                    if stop == start:
                        ledList = [start - 1]
                    elif stop > start:
                        ledList = list(range(start - 1, stop))
                    else:
                        ledList = list(reversed(range(stop - 1, start)))
                    for i in ledList:
                        leds.append(int(i))
                else:
                    for i in led.split(","):
                        leds.append(int(i) - 1)

        return chainName, leds
    else:
        return None, None


class PrinterLEDGroup:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.config_leds = config.get("leds")
        self.config_chains = self.config_leds.split("\n")
        self.leds = []
        self.led_helpers = []
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        tcallbacks = []
        for chain in self.config_chains:
            chain_name, led_indices = parse_chain(chain)
            if chain_name and led_indices:
                led_helper = self.printer.lookup_object(chain_name).led_helper
                for led_index in led_indices:
                    self.leds.append((led_helper, led_index))
                    tcallbacks.append(led_helper.tcallbacks[led_index])
                if led_helper not in self.led_helpers:
                    self.led_helpers.append(led_helper)
        self.ledCount = len(self.leds)
        self.led_helper = led.LEDHelper(self.config, self.update_leds, self.ledCount)
        self.led_helper.tcallbacks = tcallbacks

    def update_leds(self, led_state, print_time):
        flush_callbacks = set()
        for i, (led_helper, index) in enumerate(self.leds):
            led_helper._set_color(index + 1, led_state[i])
            flush_callbacks.add(led_helper._check_transmit)
        for flush_callback in flush_callbacks:
            flush_callback(print_time)

    def get_status(self, eventtime=None):
        return self.led_helper.get_status(eventtime)


def load_config_prefix(config):
    return PrinterLEDGroup(config)
