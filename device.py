#!/usr/bin/env python3
# -*- mode: Python; coding: utf-8 -*-
# Time-stamp: "2018-09-27 11:06:04 user"

#  file       ueye.py
#  author     Hendrik v. Raven
#  created    2018-09-27 11:06:04
#
#  Copyright (C) 2018         Hendrik v. Raven
#
#  This file is part of qcontrol3 -- The MPQ-developed, python3-based
#  experiment control program.
#
#  License:
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#  Commentary:
#
#    <This file was auto-generated. Please replace this text and
#    explain in a few sentences what you are trying to achieve with
#    this file.>

from qcontrol3.server.writerdevice import WriterDevice  # saves data to their file structure
from qcontrol3.server import timingchannel
from qcontrol3.tools.units import u
from qcontrol3.tools import logsetup
log = logsetup.getlogger("driverlogger")

from .driver import UEyeCamera, UEyeTriggerMode


class UEyeChannelMixin(object):
    @property
    def attr(self):
        return getattr(self.parent._dev, self.attr_name)

    def set_value_raw(self, value):
        self.attr.set(value)

    def get_value_raw(self):
        return self.attr.get()


class ExposureChannel(UEyeChannelMixin,
        timingchannel.TimingChannelQuantity):
    attr_name = "exposure"


class GainChannel(UEyeChannelMixin,
        timingchannel.TimingChannelInt):
    attr_name = "gain"


class TriggerModeChannel(UEyeChannelMixin,
        timingchannel.TimingChannelSet):
    attr_name = "trigger_mode"


class BitDepthChannel(UEyeChannelMixin,
        timingchannel.TimingChannelSet):
    attr_name = "bit_depth"


class TriggerChannel(timingchannel.TimingChannelBool):
    def set_value_raw(self, value):
        if value:
            self.parent._dev.force_trigger()

    def get_value_raw(self):
        return False



class UEyeCameraDevice(WriterDevice):
    def __init__(self, name, description, serial):
        """
        Construct a camera device identified by its serial.
        """
        super().__init__(name, description)
        self._dev = UEyeCamera.from_serial(serial)
        # we need to be connected to the camera to query its capabilities
        self.connect()
        self.add_channels()
        self.images = 1

    def connect(self):
        self._dev.connect()
        log.debug(f"Connected to {self._dev.serial} ({self._dev.model})")

    def disconnect(self):
        log.debug(f"Disconnecting from {self._dev.serial} ({self._dev.model})")
        self._dev.disconnect()

    def on_system_about_to_startup(self):
        self.connect()

    def on_system_about_to_shutdown(self):
        self.disconnect()

    def add_channels(self):
        self.add_child(ExposureChannel(
            name="EXPOSURE",
            description="Camera exposure time",
            default_value=self._dev.exposure.get() * u.s,
            deadtime=0*u.s,
            unit=u.s,
            min_value=self._dev.exposure.range_min * u.s,
            max_value=self._dev.exposure.range_max * u.s,
        ))
        self.add_child(GainChannel(
            name="GAIN",
            description="Relative hardware gain (in percent)",
            default_value=self._dev.gain.default,
            deadtime=0*u.s,
            min_value=0,
            max_value=100
            ))
        self.add_child(TriggerModeChannel(
            name="TRIGGER_MODE",
            description="Camera trigger mode",
            deadtime=0 * u.s,
            default_value=UEyeTriggerMode.HI_LO,
            allowed_values=self._dev.trigger_mode.allowed_values,
            ))
        self.add_child(TriggerChannel(
            name="TRIGGER",
            description="Force a trigger via the software",
            deadtime=0 * u.s,
            default_value=False,
            ))

    def _initialize_capture(self):
        images = self.images
        self._dev.capture_start(
                callback=self._data_ready_callback,  # saves data
                images=images)

    def _finalize_capture(self):
        data = self._dev.capture_stop()
        self._data_ready_callback(data)  # saves data

# ueye.py ends here.
