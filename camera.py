from driver import UEyeCamera, UEyeTriggerMode

import logging

log = logging.Logger(name="log-it", level=logging.DEBUG)


class UEyeCameraDevice(object):
    def __init__(self, serial):
        """
        Construct a camera device identified by its serial.
        """
        self._dev = UEyeCamera.from_serial(serial)
        # we need to be connected to the camera to query its capabilities
        self.connect()

        self._dev.trigger_mode.set(UEyeTriggerMode.HI_LO)

        self.images = 10

    def connect(self):
        self._dev.connect()
        log.debug(f"Connected to {self._dev.serial} ({self._dev.model})")

    def disconnect(self):
        log.debug(f"Disconnecting from {self._dev.serial} ({self._dev.model})")
        self._dev.disconnect()

    def _initialize_capture(self):
        images = self.images
        self._dev.capture_start(
                callback=self._data_ready_callback,
                images=images)

    def _data_ready_callback(self, data):
        print(f'data ready from {self.serial}!: {data}')

    def _finalize_capture(self):
        data = self._dev.capture_stop()
        self._data_ready_callback(data)


if __name__ == "__main__":
    cm = UEyeCameraDevice(serial=4102961110)
    cam = cm._dev
    for prop in UEyeCamera.properties.keys():
        print(prop + " " + str(getattr(cam, prop).get()))

    def callback(img):
        print(img)
        print(type(img))
        print(img.shape)

    print(cam.trigger_mode.allowed_values)

    cam.trigger_mode.set(UEyeTriggerMode.HI_LO)
    images = 2
    cam.capture_start(callback=callback, images=images)

    import time
    for i in range(images * 4):
        time.sleep(2)
        try:
            missed = cam.force_trigger()
            if missed:
                print(f"missed {missed}!")
        except Exception as e:
            print(e)

    cam.capture_stop()