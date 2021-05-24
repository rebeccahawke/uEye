#!/usr/bin/env python3
# -*- mode: Python; coding: utf-8 -*-
# Time-stamp: "2018-09-27 11:06:04 user"

#  file       driver.py
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
# Driver wrapper for the C style pyueye library provided by IDS


from functools import partial
import atexit
import ctypes
import enum
import numpy as np
import os
import pyueye
import threading


if os.name == "nt":
    import win32event


def CALL(fun, *args, wrap=True, **kwargs):
    ret = fun(*args, **kwargs)
    if wrap and ret != pyueye.ueye.IS_SUCCESS:
        raise UEyeDriverException(ret)
    return ret


class UEyeDriverWrapper(object):
    lib = pyueye.ueye

    def __getattr__(self, attr):
        # inject a wrapper into the ueye namespace which translates access to
        # FunctionName to is_FunctionName wrapped by the extra function CALL.
        # CALL cathes return codes and converts them into python exceptions.
        a = getattr(pyueye.ueye, attr, None)
        if a != None:
            return a

        fun_name = f"is_{attr}"
        a = getattr(pyueye.ueye, f"is_{attr}", None)
        if a != None:
            return partial(CALL, a)

        raise AttributeError(f"{attr} does not exist in pyueye")

    def __dir__(self):
        return dir(pyueye.ueye)


ueye = UEyeDriverWrapper()


class UEyeException(Exception):
    pass


class UEyeDriverException(Exception):
    def __init__(self, code):
        self.code = code

    def __str__(self):
        msg = self.error_codes.get(self.code, "unknown error code")
        return f"Received uEye driver error code {self.code} ({msg})"

    error_codes = {
        pyueye.ueye.IS_NO_SUCCESS: "General error message",
        pyueye.ueye.IS_SUCCESS: "Function executed successfully",
        pyueye.ueye.IS_INVALID_CAMERA_HANDLE: "Invalid camera handle. Most of the uEye SDK functions expect the camera handle as the first parameter.",
        pyueye.ueye.IS_IO_REQUEST_FAILED: "An IO request from the uEye driver failed. Possibly the versions of the ueye_api.dll (API) and the driver file (ueye_usb.sys or ueye_eth.sys) do not match.",
        pyueye.ueye.IS_CANT_OPEN_DEVICE: "An attempt to initialize or select the camera failed (no camera connected or initialization error).",
        pyueye.ueye.IS_CANT_OPEN_REGISTRY: "Error opening a Windows registry key",
        pyueye.ueye.IS_CANT_READ_REGISTRY: "Error reading settings from the Windows registry",
        pyueye.ueye.IS_NO_IMAGE_MEM_ALLOCATED: "The driver could not allocate memory.",
        pyueye.ueye.IS_CANT_CLEANUP_MEMORY: "The driver could not release the allocated memory.",
        pyueye.ueye.IS_CANT_COMMUNICATE_WITH_DRIVER: "Communication with the driver failed because no driver has been loaded.",
        pyueye.ueye.IS_FUNCTION_NOT_SUPPORTED_YET: "The function is not supported yet.",
        pyueye.ueye.IS_INVALID_IMAGE_SIZE: "Invalid image size",
        pyueye.ueye.IS_INVALID_CAPTURE_MODE: "The function can not be executed in the current camera operating mode (free run, trigger or standby).",
        pyueye.ueye.IS_INVALID_MEMORY_POINTER: "Invalid pointer or invalid memory ID",
        pyueye.ueye.IS_FILE_WRITE_OPEN_ERROR: "File cannot be opened for writing or reading.",
        pyueye.ueye.IS_FILE_READ_OPEN_ERROR: "The file cannot be opened.",
        pyueye.ueye.IS_FILE_READ_INVALID_BMP_ID: "The specified file is not a valid bitmap file.",
        pyueye.ueye.IS_FILE_READ_INVALID_BMP_SIZE: "The bitmap size is not correct (bitmap too large).",
        pyueye.ueye.IS_NO_ACTIVE_IMG_MEM: "No active image memory available. You must set the memory to active using the is_SetImageMem() function or create a sequence using the is_AddToSequence() function.",
        pyueye.ueye.IS_SEQUENCE_LIST_EMPTY: "The sequence list is empty and cannot be deleted.",
        pyueye.ueye.IS_CANT_ADD_TO_SEQUENCE: "The image memory is already included in the sequence and cannot be added again.",
        pyueye.ueye.IS_SEQUENCE_BUF_ALREADY_LOCKED: "The memory could not be locked. The pointer to the buffer is invalid.",
        pyueye.ueye.IS_INVALID_DEVICE_ID: "The device ID is invalid. Valid IDs start from 1 for USB cameras, and from 1001 for GigE cameras.",
        pyueye.ueye.IS_INVALID_BOARD_ID: "The board ID is invalid. Valid IDs range from 1 through 255.",
        pyueye.ueye.IS_ALL_DEVICES_BUSY: "All cameras are in use.",
        pyueye.ueye.IS_TIMED_OUT: "A timeout occurred. An image capturing process could not be terminated within the allowable period.",
        pyueye.ueye.IS_NULL_POINTER: "Invalid array",
        pyueye.ueye.IS_INVALID_PARAMETER: "One of the submitted parameters is outside the valid range or is not supported for this sensor or is not available in this mode.",
        pyueye.ueye.IS_OUT_OF_MEMORY: "No memory could be allocated.",
        pyueye.ueye.IS_ACCESS_VIOLATION: "An internal error has occurred.",
        pyueye.ueye.IS_NO_USB20: "The camera is connected to a port which does not support the USB 2.0 high-speed standard.  Cameras without a memory board cannot be operated on a USB 1.1 port.",
        pyueye.ueye.IS_CAPTURE_RUNNING: "A capturing operation is in progress and must be terminated first.",
        pyueye.ueye.IS_IMAGE_NOT_PRESENT: "The requested image is not available in the camera memory or is no longer valid.",
        pyueye.ueye.IS_TRIGGER_ACTIVATED: "The function cannot be used because the camera is waiting for a trigger signal.",
        pyueye.ueye.IS_CRC_ERROR: "A CRC error-correction problem occurred while reading the settings.",
        pyueye.ueye.IS_NOT_YET_RELEASED: "This function has not been enabled yet in this version.",
        pyueye.ueye.IS_NOT_CALIBRATED: "The camera does not contain any calibration data.",
        pyueye.ueye.IS_WAITING_FOR_KERNEL: "The system is waiting for the kernel driver to respond.",
        pyueye.ueye.IS_NOT_SUPPORTED: "The camera model used here does not support this function or setting.",
        pyueye.ueye.IS_TRIGGER_NOT_ACTIVATED: "The function is not possible as trigger is disabled.",
        pyueye.ueye.IS_OPERATION_ABORTED: "The dialog was canceled without a selection so that no file could be saved.",
        pyueye.ueye.IS_BAD_STRUCTURE_SIZE: "An internal structure has an incorrect size.",
        pyueye.ueye.IS_INVALID_BUFFER_SIZE: "The image memory has an inappropriate size to store the image in the desired format.",
        pyueye.ueye.IS_INVALID_PIXEL_CLOCK: "This setting is not available for the currently set pixel clock frequency.",
        pyueye.ueye.IS_INVALID_EXPOSURE_TIME: "This setting is not available for the currently set exposure time.",
        pyueye.ueye.IS_AUTO_EXPOSURE_RUNNING: "This setting cannot be changed while automatic exposure time control is enabled.",
        pyueye.ueye.IS_CANNOT_CREATE_BB_SURF: "The BackBuffer surface cannot be created.",
        pyueye.ueye.IS_CANNOT_CREATE_BB_MIX: "The BackBuffer mix surface cannot be created.",
        pyueye.ueye.IS_BB_OVLMEM_NULL: "The BackBuffer overlay memory cannot be locked.",
        pyueye.ueye.IS_CANNOT_CREATE_BB_OVL: "The BackBuffer overlay memory cannot be created.",
        pyueye.ueye.IS_NOT_SUPP_IN_OVL_SURF_MODE: "Not supported in BackBuffer Overlay mode.",
        pyueye.ueye.IS_INVALID_SURFACE: "Back buffer surface invalid.",
        pyueye.ueye.IS_SURFACE_LOST: "Back buffer surface not found.",
        pyueye.ueye.IS_RELEASE_BB_OVL_DC: "Error releasing the overlay device context.",
        pyueye.ueye.IS_BB_TIMER_NOT_CREATED: "The back buffer timer could not be created.",
        pyueye.ueye.IS_BB_OVL_NOT_EN: "The back buffer overlay was not enabled.",
        pyueye.ueye.IS_ONLY_IN_BB_MODE: "Only possible in BackBuffer mode.",
        pyueye.ueye.IS_INVALID_COLOR_FORMAT: "Invalid color format",
        pyueye.ueye.IS_INVALID_WB_BINNING_MODE: "Mono binning/mono sub-sampling do not support automatic white balance.",
        pyueye.ueye.IS_INVALID_I2C_DEVICE_ADDRESS: "Invalid I2C device address",
        pyueye.ueye.IS_COULD_NOT_CONVERT: "The current image could not be processed.",
        pyueye.ueye.IS_TRANSFER_ERROR: "Transfer error. Frequent transfer errors can mostly be avoided by reducing the pixel rate.",
        pyueye.ueye.IS_PARAMETER_SET_NOT_PRESENT: "Parameter set is not present.",
        pyueye.ueye.IS_INVALID_CAMERA_TYPE: "The camera type defined in the .ini file does not match the current camera model.",
        pyueye.ueye.IS_INVALID_HOST_IP_HIBYTE: "Invalid HIBYTE of host address",
        pyueye.ueye.IS_CM_NOT_SUPP_IN_CURR_DISPLAYMODE: "The color mode is not supported in the current display mode.",
        pyueye.ueye.IS_NO_IR_FILTER: "No IR filter available",
        pyueye.ueye.IS_STARTER_FW_UPLOAD_NEEDED: "The camera's starter firmware is not compatible with the driver and needs to be updated.",
        pyueye.ueye.IS_DR_LIBRARY_NOT_FOUND: "The DirectRenderer library could not be found.",
        pyueye.ueye.IS_DR_DEVICE_OUT_OF_MEMORY: "Not enough graphics memory available.",
        pyueye.ueye.IS_DR_CANNOT_CREATE_SURFACE: "The image surface or overlay surface could not be created.",
        pyueye.ueye.IS_DR_CANNOT_CREATE_VERTEX_BUFFER: "The vertex buffer could not be created.",
        pyueye.ueye.IS_DR_CANNOT_CREATE_TEXTURE: "The texture could not be created.",
        pyueye.ueye.IS_DR_CANNOT_LOCK_OVERLAY_SURFACE: "The overlay surface could not be locked.",
        pyueye.ueye.IS_DR_CANNOT_UNLOCK_OVERLAY_SURFACE: "The overlay surface could not be unlocked.",
        pyueye.ueye.IS_DR_CANNOT_GET_OVERLAY_DC: "Could not get the device context handle for the overlay.",
        pyueye.ueye.IS_DR_CANNOT_RELEASE_OVERLAY_DC: "Could not release the device context handle for the overlay.",
        pyueye.ueye.IS_DR_DEVICE_CAPS_INSUFFICIENT: "Function is not supported by the graphics hardware.",
        pyueye.ueye.IS_INCOMPATIBLE_SETTING: "Because of other incompatible settings the function is not possible.",
        pyueye.ueye.IS_DR_NOT_ALLOWED_WHILE_DC_IS_ACTIVE: "A device context handle is still open in the application.",
        pyueye.ueye.IS_DEVICE_ALREADY_PAIRED: "The device is already paired.",
        pyueye.ueye.IS_SUBNETMASK_MISMATCH: "The subnet mask of the camera and PC network card are different.",
        pyueye.ueye.IS_SUBNET_MISMATCH: "The subnet of the camera and PC network card are different.",
        pyueye.ueye.IS_INVALID_IP_CONFIGURATION: "The configuration of the IP address is invalid.",
        pyueye.ueye.IS_DEVICE_NOT_COMPATIBLE: "The device is not compatible to the drivers.",
        pyueye.ueye.IS_NETWORK_FRAME_SIZE_INCOMPATIBLE: "The settings for the image size of the camera are not compatible to the PC network card.",
        pyueye.ueye.IS_NETWORK_CONFIGURATION_INVALID: "The configuration of the network card is invalid.",
        pyueye.ueye.IS_ERROR_CPU_IDLE_STATES_CONFIGURATION: "The configuration of the CPU idle has failed.",
        pyueye.ueye.IS_DEVICE_BUSY: "The camera is busy and cannot transfer the requested image.",
        pyueye.ueye.IS_SENSOR_INITIALIZATION_FAILED: "The initialization of the sensor failed.",
        pyueye.ueye.IS_IMAGE_BUFFER_NOT_DWORD_ALIGNED: "The image buffer is not DWORD aligned.",
    }


class UEyeMem(object):
    def __init__(self, cam, width, height, bitspixel):
        self.cam = cam
        self.pid = ctypes.c_int()
        self.img_mem = ueye.c_mem_p()
        ueye.AllocImageMem(cam, width, height, bitspixel,
                self.img_mem, self.pid)
        atexit.register(self.clear)

    def clear(self):
        if self.img_mem:
            ueye.FreeImageMem(self.cam, self.img_mem, self.pid)


class UEyeMemSequence(object):
    def __init__(self, parent, length, width, height, bitspixel):
        self._ueye = ueye
        self._parent = parent
        self.width = width
        self.height = height
        self.bitspixel = bitspixel

        # register before initialising images to get the correct cleanup order
        atexit.register(self.clear)

        self._mem = []
        for _ in range(length):
            new_mem = UEyeMem(self._parent._cam, self.width, self.height,
                    self.bitspixel)
            self._mem.append(new_mem)
            ueye.AddToSequence(self._parent._cam, new_mem.img_mem,
                    len(self._mem))

    def clear(self):
        if self._mem:
            ueye.ClearSequence(self._parent._cam)


class UEyeAttr(object):
    def __init__(self, parent):
        self.parent = parent


class UEyeExposure(UEyeAttr):
    def _call(self, cmd, arg):
        ueye.Exposure(self.parent._cam, cmd, arg, ctypes.sizeof(arg))

    def get(self):
        param = ctypes.c_double(0)
        self._call(ueye.IS_EXPOSURE_CMD_GET_EXPOSURE, param)
        return param.value / 1000

    def set(self, value):
        param = ctypes.c_double(value * 1000)
        self._call(ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, param)
        return param.value

    @property
    def default(self):
        param = ctypes.c_double(0)
        self._call(ueye.IS_EXPOSURE_CMD_GET_EXPOSURE_DEFAULT, param)
        return param.value / 1000

    @property
    def range_min(self):
        param = ctypes.c_double(0)
        self._call(ueye.IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN, param)
        return param.value / 1000 # seconds

    @property
    def range_max(self):
        param = ctypes.c_double(0)
        self._call(ueye.IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX, param)
        return param.value / 1000 # seconds


class UEyeGain(UEyeAttr):
    def get(self):
        return ueye.SetHardwareGain(
            self.parent._cam,
            ueye.IS_GET_MASTER_GAIN,
            ueye.IS_IGNORE_PARAMETER,
            ueye.IS_IGNORE_PARAMETER,
            ueye.IS_IGNORE_PARAMETER,
        )

    def set(self, value):
        ueye.SetHardwareGain(
            self.parent._cam,
            value,
            ueye.IS_IGNORE_PARAMETER,
            ueye.IS_IGNORE_PARAMETER,
            ueye.IS_IGNORE_PARAMETER,
        )

    @property
    def default(self):
        return ueye.SetHardwareGain(
            self.parent._cam,
            ueye.IS_GET_DEFAULT_MASTER,
            ueye.IS_IGNORE_PARAMETER,
            ueye.IS_IGNORE_PARAMETER,
            ueye.IS_IGNORE_PARAMETER,
        )


class UEyeTriggerMode(enum.IntFlag):
    OFF = ueye.IS_SET_TRIGGER_OFF
    HI_LO = ueye.IS_SET_TRIGGER_HI_LO
    LO_HI = ueye.IS_SET_TRIGGER_LO_HI
    SOFTWARE = ueye.IS_SET_TRIGGER_SOFTWARE
    PRE_HI_LO = ueye.IS_SET_TRIGGER_PRE_HI_LO
    PRE_LO_HI = ueye.IS_SET_TRIGGER_PRE_LO_HI
    HI_LO_SYNC = ueye.IS_SET_TRIGGER_HI_LO_SYNC
    LO_HI_SYNC = ueye.IS_SET_TRIGGER_LO_HI_SYNC


class UEyeAttrTriggerMode(UEyeAttr):
    def get(self):
        ret = ueye.SetExternalTrigger(self.parent._cam, ueye.IS_GET_EXTERNALTRIGGER, wrap=False)
        return UEyeTriggerMode(ret)

    def set(self, value):
        ueye.SetExternalTrigger(self.parent._cam, value)

    @property
    def allowed_values(self):
        ret = ueye.SetExternalTrigger(
            self.parent._cam, ueye.IS_GET_SUPPORTED_TRIGGER_MODE,
            wrap=False
        )
        out = set()
        for mode in UEyeTriggerMode:
            if mode & ret:
                out.add(mode)
        return out


class UEyeBitDepth(enum.IntFlag):
    DEPTH_8_BIT = ueye.IS_SENSOR_BIT_DEPTH_8_BIT
    DEPTH_10_BIT = ueye.IS_SENSOR_BIT_DEPTH_10_BIT
    DEPTH_12_BIT = ueye.IS_SENSOR_BIT_DEPTH_12_BIT
    DEPTH_AUTO = ueye.IS_SENSOR_BIT_DEPTH_AUTO


class UEyeAttrBitDepth(UEyeAttr):
    def get(self):
        param = ctypes.c_uint(0)
        ret = ueye.DeviceFeature(
            self.parent._cam,
            ueye.IS_DEVICE_FEATURE_CMD_GET_SENSOR_BIT_DEPTH,
            param,
            ctypes.sizeof(param),
        )
        return UEyeBitDepth(param.value)

    def set(self, value):
        param = ctypes.c_uint(value)
        ueye.DeviceFeature(
            self.parent._cam,
            ueye.IS_DEVICE_FEATURE_CMD_SET_SENSOR_BIT_DEPTH,
            param,
            ctypes.sizeof(param),
        )

    @property
    def default(self):
        param = ctypes.c_uint(0)
        ret = ueye.DeviceFeature(
            self.parent._cam,
            ueye.IS_DEVICE_FEATURE_CMD_GET_SENSOR_BIT_DEPTH_DEFAULT,
            param,
            ctypes.sizeof(param),
        )
        return UEyeBitDepth(param.value)

    @property
    def allowed_values(self):
        param = ctypes.c_uint(0)
        ret = ueye.DeviceFeature(
            self.parent._cam,
            ueye.IS_DEVICE_FEATURE_CMD_GET_SUPPORTED_SENSOR_BIT_DEPTHS,
            param,
            ctypes.sizeof(param),
        )
        out = set()
        for mode in UEyeBitDepth:
            if mode & ret:
                out.add(mode)
        return out


class EventMonitorBase(threading.Thread):
    def __init__(self, handle, event, callback):
        super().__init__()
        self._cam = handle
        self._event = event
        self._callback = callback
        self._timeout = 500 # ms

    def join(self, timeout=None):
        self._running = False
        super().join(timeout)


class EventMonitorLinux(EventMonitorBase):
    def run(self):
        ueye.EnableEvent(self._cam, self._event)

        self._running = True
        while self._running:
            ret = ueye.is_WaitEvent(self._cam, self._event, self._timeout)
            if ret == ueye.IS_TIMED_OUT:
                continue
            elif ret == ueye.IS_SUCCESS:
                self._callback()
            else:
                raise UEyeDriverException(ret)

        ueye.DisableEvent(self._cam, self._event)


class EventMonitorWindows(EventMonitorBase):
    def run(self):
        pyhandle = win32event.CreateEvent(None, False, False, None)
        chandle = ctypes.c_long(pyhandle.handle)
        ueye.InitEvent(self._cam, chandle, self._event)
        ueye.EnableEvent(self._cam, self._event)

        self._running = True
        while self._running:
            ret = win32event.WaitForSingleObject(pyhandle, self._timeout)
            if ret == win32event.WAIT_TIMEOUT:
                continue
            elif ret == win32event.WAIT_OBJECT_0:
                self._callback()
            else:
                raise Exception(ret)

        ueye.DisableEvent(self._cam, self._event)
        ueye.ExitEvent(self._cam, self._event)
        pyhandle.close()

    def join(self, timeout=None):
        self._running = False
        super().join(timeout)


if os.name == "posix":
    EventMonitor = EventMonitorLinux
elif os.name == "nt":
    EventMonitor = EventMonitorWindows


class UEyeCamera(object):
    def __init__(self, camera_id=0):
        """
        Initialise the camera with the specified camera id. As the camera id
        can change when multiple cameras are in use it is recommended to use
        the from_serial helper. If you only have a single came
        """
        self._id = camera_id
        self._event_handlers = {}
        self._cam = None
        self.connect()
        self._init_camera_parameters()

    def _init_camera_parameters(self):
        pinfo = ueye.SENSORINFO()
        ueye.GetSensorInfo(self._cam, pinfo)
        self._size_x = pinfo.nMaxWidth.value
        self._size_y = pinfo.nMaxHeight.value
        self._roi_x = self._size_x
        self._roi_y = self._size_y
        self._bit_depth = 8 # TODO

    @property
    def serial(self):
        pinfo = ueye.CAMINFO()
        ueye.GetCameraInfo(self._cam, pinfo)
        return pinfo.SerNo.decode("ascii")

    @property
    def manufacturer(self):
        pinfo = ueye.CAMINFO()
        ueye.GetCameraInfo(self._cam, pinfo)
        return pinfo.ID.decode("ascii")

    @property
    def model(self):
        pinfo = ueye.SENSORINFO()
        ueye.GetSensorInfo(self._cam, pinfo)
        return pinfo.strSensorName.decode("ascii")

    @property
    def pixel_size_nm(self):
        pinfo = ueye.SENSORINFO()
        ueye.GetSensorInfo(self._cam, pinfo)
        return pinfo.wPixelSize.value * 10

    @staticmethod
    def from_serial(serial):
        cameras = ctypes.c_int()
        ueye.GetNumberOfCameras(cameras)
        pucl = ueye.UEYE_CAMERA_LIST(ueye.UEYE_CAMERA_INFO * cameras.value)
        pucl.dwCount = cameras.value
        ueye.GetCameraList(pucl)
        for entry in pucl.uci:
            if entry.SerNo.decode("ascii") == serial:
                return UEyeCamera(entry.dwDeviceID.value)
        raise UEyeException(f"Could not find camera with serial {serial}")

    def connect(self):
        self.disconnect()
        self._cam = ueye.HIDS(self._id | ueye.IS_USE_DEVICE_ID)
        ueye.InitCamera(self._cam, None)
        # store images into RAM instead of displaying them
        ueye.SetDisplayMode(self._cam, ueye.IS_SET_DM_DIB)
        atexit.register(self.disconnect)

    def disconnect(self):
        atexit.unregister(self.disconnect)
        if self._cam != None:
            ueye.ExitCamera(self._cam)
            self._cam = None

    properties = dict(
        exposure=UEyeExposure,
        gain=UEyeGain,
        trigger_mode=UEyeAttrTriggerMode,
        #bit_depth=UEyeAttrBitDepth,
    )

    def __getattr__(self, attr):
        prop = self.properties.get(attr)
        if prop:
            return prop(self)
        raise AttributeError(f"Could not find attribute \"{attr}\"")

    def __dir__(self):
        # return our custom properties added via getattr to the class dir.
        # This makes tab completion in ipython and similar work
        return super().__dir__().append(list(self.properties.keys()))

    def _alloc_images(self, images):
        self._mem = UEyeMemSequence(parent=self, length=images,
                width=self._roi_x, height=self._roi_y, bitspixel=self._bit_depth)

    def _retrieve_image(self):
        mem = ueye.c_mem_p()
        ueye.GetImageMem(self._cam, mem)
        ueye.LockSeqBuf(self._cam, ueye.IS_IGNORE_PARAMETER, mem)
        try:
            pitch = ctypes.c_int(0)
            ueye.GetImageMemPitch(self._cam, pitch)
            if pitch.value // self._size_x == 2:
                dtype = ctypes.c_uint16
            else:
                dtype = ctypes.c_uint8
            arr = np.ctypeslib.as_array(
                (dtype * self._size_x * self._size_y).from_address(mem.value)
            )
            return arr.copy().reshape((self._size_x, self._size_y))
        finally:
            ueye.UnlockSeqBuf(self._cam, ueye.IS_IGNORE_PARAMETER, mem)

    def _callback_wrapper(self, callback):
        def wrapper():
            img = self._retrieve_image()
            print("img", img)
            callback(img)
        return wrapper

    def _enable_event(self, event, callback):
        handler = self._event_handlers.get(event)
        if handler:
            # only update the callback
            handler._callback = callback
        else:
            handler = EventMonitor(self._cam, event, callback)
            self._event_handlers[event] = handler
            handler.start()
        atexit.register(handler.join, 1)

    def _disable_event(self, event):
        handler = self._event_handlers[event]
        del self._event_handlers[event]
        atexit.unregister(handler.join)
        handler.join(1)
        if handler.is_alive():
            print("Handler did not terminate within 5 seconds")

    def capture_start(self, callback, images=1):
        self._alloc_images(images)
        trigger = self.trigger_mode.get()
        if trigger == UEyeTriggerMode.OFF:
            raise NotImplementedError
        else:
            self._enable_event(ueye.IS_SET_EVENT_FRAME,
                    self._callback_wrapper(callback))
            ueye.CaptureVideo(self._cam, ueye.IS_DONT_WAIT)
            print("video captured", callback)

    def capture_stop(self):
        atexit.unregister(self.capture_stop)
        ueye.StopLiveVideo(self._cam, ueye.IS_FORCE_VIDEO_STOP)
        del self._mem
        self._mem = None
        self._disable_event(ueye.IS_SET_EVENT_FRAME)

    def capture_single(self):
        self.capture_start()
        return self._retrieve_image()

    def force_trigger(self):
        param = ctypes.c_ulong(ueye.IS_GET_STATUS)
        # call before to clear value
        ueye.CameraStatus(self._cam, ueye.IS_TRIGGER_MISSED, param, wrap=False)
        ueye.ForceTrigger(self._cam)
        return ueye.CameraStatus(self._cam, ueye.IS_TRIGGER_MISSED, param, wrap=False)


if __name__ == "__main__":
    cam = UEyeCamera(camera_id=1)
    for prop in UEyeCamera.properties.keys():
        print(prop + " " + str(getattr(cam, prop).get()))

    def callback(img):
        print(img)
        print(type(img))
        print(img.shape)

    print(cam.trigger_mode.allowed_values)
    print(cam.serial)

    cam.trigger_mode.set(UEyeTriggerMode.HI_LO)
    images = 2
    cam.capture_start(callback=callback, images=images)

    import time
    for i in range(images * 4):
        time.sleep(2)
        print(i)
        try:
            missed = cam.force_trigger()
            print(missed)
            if missed:
                print(f"missed {missed}!")
        except Exception as e:
            print(e)

    cam.capture_stop()
    print(images)

