from pyueye import ueye
import numpy as np
import cv2
import sys
import ctypes
from time import time_ns

import matplotlib.pyplot as plt


class Camera(object):

    def __init__(self):
        # Variables
        self.hCam = ueye.HIDS(1)  # 0: first available camera;  1-254: The camera with the specified camera ID
        self.sInfo = ueye.SENSORINFO()
        self.cInfo = ueye.CAMINFO()
        self.pcImageMemory = ueye.c_mem_p()
        self.MemID = ueye.int()
        self.rectAOI = ueye.IS_RECT()
        self.pitch = ueye.INT()
        self.nBitsPerPixel = ueye.INT(8)  # 24: bits per pixel for color mode; take 8 bits per pixel for monochrome
        self.channels = 1  # 3 channels for color mode (RGB); take 1 channel for monochrome
        self.m_nColorMode = ueye.INT()  # Y8/RGB16/RGB24/REG32
        self.bytes_per_pixel = int(self.nBitsPerPixel / 8)

    def configure(self):
        self.connect()
        self.get_data()
        self.sensor_info()

        self.reset_camera()
        self.set_display_to_DIB()
        self.set_color_mode()
        self.set_full_auto()

    def connect(self):
        # Starts the driver and establishes the connection to the camera
        print("Connecting to camera")
        nRet = ueye.is_InitCamera(self.hCam, None)
        if nRet != ueye.IS_SUCCESS:
            print("is_InitCamera ERROR")
        else:
            print("Camera initialised")

    def get_data(self):
        # Reads out the data hard-coded in the non-volatile camera memory and writes it to the data structure that cInfo points to
        nRet = ueye.is_GetCameraInfo(self.hCam, self.cInfo)
        if nRet != ueye.IS_SUCCESS:
            print("is_GetCameraInfo ERROR")
        # else:
        #     print("GetCameraInfo complete")
            # print(self.cInfo)  # self.cInfo contains a lot of interesting device information
            # print()

    def sensor_info(self):
        # You can query additional information about the sensor type used in the camera
        nRet = ueye.is_GetSensorInfo(self.hCam, self.sInfo)
        if nRet != ueye.IS_SUCCESS:
            print("is_GetSensorInfo ERROR")
        else:
            # print("Get Sensor info complete")
            # print(self.sInfo)
            print("Camera model:\t\t", self.sInfo.strSensorName.decode('utf-8'))
            print("Camera serial no.:\t", self.cInfo.SerNo.decode('utf-8'))

        print(ueye.is_SetExternalTrigger(self.hCam, ueye.IS_GET_EXTERNALTRIGGER))
        print(ueye.is_SetExternalTrigger(self.hCam, ueye.IS_GET_SUPPORTED_TRIGGER_MODE))
        print(ueye.is_SetExternalTrigger(self.hCam, ueye.IS_GET_TRIGGER_STATUS))

    def reset_camera(self):
        nRet = ueye.is_ResetToDefault(self.hCam)
        if nRet != ueye.IS_SUCCESS:
            print("is_ResetToDefault ERROR")
        else:
            print("Camera reset complete")

    def set_display_to_DIB(self):
        # Set display mode to DIB
        nRet = ueye.is_SetDisplayMode(self.hCam, ueye.IS_SET_DM_DIB)
        if nRet != ueye.IS_SUCCESS:
            print("is_SetDisplayMode ERROR")
        else:
            print("Camera set to Device Independent Bitmap (DIB) display mode")

    def set_color_mode(self):
        # Set the right color mode
        if int.from_bytes(self.sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_BAYER:
            # setup the color depth to the current windows setting
            ueye.is_GetColorDepth(self.hCam, self.nBitsPerPixel, self.m_nColorMode)
            bytes_per_pixel = int(self.nBitsPerPixel / 8)
            print("IS_COLORMODE_BAYER: ", )
            print("\tm_nColorMode: \t\t", self.m_nColorMode)
            print("\tnBitsPerPixel: \t\t", self.nBitsPerPixel)
            print("\tbytes_per_pixel: \t\t", self.bytes_per_pixel)
            print()

        elif int.from_bytes(self.sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_CBYCRY:
            # for color camera models use RGB32 mode
            self.m_nColorMode = ueye.IS_CM_BGRA8_PACKED
            self.nBitsPerPixel = ueye.INT(32)
            self.bytes_per_pixel = int(self.nBitsPerPixel / 8)
            print("IS_COLORMODE_CBYCRY: ", )
            print("\tm_nColorMode: \t\t", self.m_nColorMode)
            print("\tnBitsPerPixel: \t\t", self.nBitsPerPixel)
            print("\tbytes_per_pixel: \t\t", self.bytes_per_pixel)
            print()

        elif int.from_bytes(self.sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_MONOCHROME:
            # for color camera models use RGB32 mode
            self.m_nColorMode = ueye.IS_CM_MONO8
            self.nBitsPerPixel = ueye.INT(8)
            self.bytes_per_pixel = int(self.nBitsPerPixel / 8)
            print("IS_COLORMODE_MONOCHROME: ", )
            print("\tm_nColorMode: \t\t", self.m_nColorMode)
            print("\tnBitsPerPixel: \t\t", self.nBitsPerPixel)
            print("\tbytes_per_pixel: \t\t", self.bytes_per_pixel)
            print()

        else:
            # for monochrome camera models use Y8 mode
            self.m_nColorMode = ueye.IS_CM_MONO8
            self.nBitsPerPixel = ueye.INT(8)
            self.bytes_per_pixel = int(self.nBitsPerPixel / 8)
            print("else")

    def det_area_of_interest(self, x=0, y=0, width=1280, height=1024):
        # set the size and position of an "area of interest"(AOI) within an image

        width = width or self.rectAOI.s32Width
        height = height or self.rectAOI.s32Height
        self.rectAOI.s32X = ueye.int(x)
        self.rectAOI.s32Y = ueye.int(y)
        self.rectAOI.s32Width = ueye.int(width)
        self.rectAOI.s32Height = ueye.int(height)

        nRet = ueye.is_AOI(self.hCam, ueye.IS_AOI_IMAGE_SET_AOI, self.rectAOI, ueye.sizeof(self.rectAOI))
        if nRet != ueye.IS_SUCCESS:
            print("is_AOI ERROR")

        print("Image width:\t", self.rectAOI.s32Width, width)
        print("Image height:\t", self.rectAOI.s32Height, height)

    def set_full_auto(self):
        print("full auto")
        disable = ueye.DOUBLE(0)
        enable = ueye.DOUBLE(1)
        zero = ueye.DOUBLE(0)
        ms = ueye.DOUBLE(20)
        rate = ueye.DOUBLE(50)
        newrate = ueye.DOUBLE()
        number = ueye.UINT()

        ret = ueye.is_SetAutoParameter(self.hCam, ueye.IS_SET_ENABLE_AUTO_GAIN, enable, zero)
        print('AG:',ret)
        ret = ueye.is_SetAutoParameter(self.hCam, ueye.IS_SET_ENABLE_AUTO_SHUTTER, enable, zero)
        print('A_SHUTTER:',ret)
        ret = ueye.is_SetFrameRate(self.hCam, rate, newrate)
        print('FR:',ret,newrate)
        ret = ueye.is_Exposure(self.hCam, ueye.IS_EXPOSURE_CMD_GET_EXPOSURE, ms, ueye.sizeof(ms))
        print('EXP:',ret,ms)

    def allocate_memory_for_image(self):
        # Allocates an image memory for an image having its dimensions defined by width and height and its color depth defined by nBitsPerPixel
        nRet = ueye.is_AllocImageMem(self.hCam, self.rectAOI.s32Width, self.rectAOI.s32Height, self.nBitsPerPixel, self.pcImageMemory, self.MemID)
        if nRet != ueye.IS_SUCCESS:
            print("is_AllocImageMem ERROR")
        else:
            # Makes the specified image memory the active memory
            nRet = ueye.is_SetImageMem(self.hCam, self.pcImageMemory, self.MemID)
            if nRet != ueye.IS_SUCCESS:
                print("is_SetImageMem ERROR")
            else:
                # Set the desired color mode
                nRet = ueye.is_SetColorMode(self.hCam, self.m_nColorMode)

    def set_external_trigger(self):
        # trigger mode hi_lo triggers on falling edge
        trigmode = ueye.IS_SET_TRIGGER_HI_LO
        nRet = ueye.is_SetExternalTrigger(self.hCam, trigmode)
        if nRet != ueye.IS_SUCCESS:
            print("SetExternalTrigger ERROR")
        print('External trigger mode set', ueye.is_SetExternalTrigger(self.hCam, ueye.IS_GET_EXTERNALTRIGGER), trigmode)

    def SetTriggerCounter(self, nValue):
        return ueye.is_SetTriggerCounter(self.hCam, nValue)

    def capture_video(self, wait=False):
        wait_param = ueye.IS_WAIT if wait else ueye.IS_DONT_WAIT
        return ueye.is_CaptureVideo(self.hCam, wait_param)

    def queue_mode(self):
        # Enables the queue mode for existing image memory sequences
        nRet = ueye.is_InquireImageMem(self.hCam, self.pcImageMemory, self.MemID, self.rectAOI.s32Width,
                                       self.rectAOI.s32Height, self.nBitsPerPixel, self.pitch)
        if nRet != ueye.IS_SUCCESS:
            print("is_InquireImageMem ERROR")

    def stop_video(self):
        return ueye.is_StopLiveVideo(self.hCam, ueye.IS_FORCE_VIDEO_STOP)

    def freeze_video(self, wait=False):
        wait_param = ueye.IS_WAIT if wait else ueye.IS_DONT_WAIT
        return ueye.is_FreezeVideo(self.hCam, wait_param)

    def take_image(self, display=True, threshold2=200):
        # In order to display the image in an OpenCV window we need to...
        # ...extract the data of our image memory
        array = ueye.get_data(self.pcImageMemory, self.rectAOI.s32Width, self.rectAOI.s32Height, self.nBitsPerPixel,
                              self.pitch, copy=False)
        t_s = time_ns() / 1e9
        # print('array',np.size(array))
        # bytes_per_pixel = int(nBitsPerPixel / 8)

        # ...reshape it in an numpy array...
        frame = np.reshape(array, (self.rectAOI.s32Height.value, self.rectAOI.s32Width.value, self.bytes_per_pixel))
        # ...resize the image by a half
        frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

        # Include image data processing here: in this case, use Canny edge detection to find the edge
        edges = cv2.Canny(frame, 0, threshold2)
        # here's one value for the edge, at height '10'
        # print(np.argmax(edges[10]))
        # here's an average over all horizontal slices.
        # print(time_ns(), np.mean([np.argmax(a) for a in edges]))
        # better still, take the median value
        dp = t_s, np.median(([np.argmax(a) for a in edges]))
        # ---------------------------------------------------------------------------------------------------------------------------------------
        # ...and finally display it
        if display:
            cv2.imshow("SimpleLive_Python_uEye_OpenCV", frame)
        cv2.imshow("Edges", edges)

        return dp

    def activate_live_video(self):
        # Activates the camera's live video mode (free run mode)
        nRet = self.capture_video(wait=False)
        if nRet != ueye.IS_SUCCESS:
            print("is_CaptureVideo ERROR")

        # Enables the queue mode for existing image memory sequences
        nRet = ueye.is_InquireImageMem(self.hCam, self.pcImageMemory, self.MemID, self.rectAOI.s32Width, self.rectAOI.s32Height, self.nBitsPerPixel, self.pitch)
        if nRet != ueye.IS_SUCCESS:
            print("is_InquireImageMem ERROR")
        else:
            print("Live video activated. Press q to quit")
            self.live_video_loop(nRet)
            self.release_memory()

    def live_video_loop(self, nRet):
        # Continuous image display
        while (nRet == ueye.IS_SUCCESS):

            self.take_image()

            # Press q if you want to end the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def monitor_edge(self, time_s, threshold2=200):
        # Activates the camera's live video mode (free run mode)
        nRet = self.capture_video(wait=False)
        if nRet != ueye.IS_SUCCESS:
            print("is_CaptureVideo ERROR")

        # Enables the queue mode for existing image memory sequences
        nRet = ueye.is_InquireImageMem(self.hCam, self.pcImageMemory, self.MemID, self.rectAOI.s32Width,
                                       self.rectAOI.s32Height, self.nBitsPerPixel, self.pitch)
        if nRet != ueye.IS_SUCCESS:
            print("is_InquireImageMem ERROR")
        else:
            print(f"Beginning edge monitoring for {time_s} seconds")
            times = []
            edges = []
            t0 = time_ns()
            while (nRet == ueye.IS_SUCCESS) and (time_ns() - t0 < time_s * 1e9):
                t, e = self.take_image(display=False, threshold2=threshold2)
                times.append(t)
                edges.append(e)
                if cv2.waitKey(1) & 0xFF == ord('q'):  # this line seems to help the image display correctly
                    break
            self.release_memory()

            return times, edges

    @staticmethod
    def save_edge_data(times, edges):
        savepath = r'C:\Users\r.hawke\PycharmProjects\uEye\data_files/edge-data_{}.csv'.format(times[0])
        with open(savepath, mode='w') as fp:
            fp.write("Timestamp,Edge (pixel)\n")
            for a, b in zip(times, edges):
                fp.write("{:.3f},{}\n".format(a - times[0], b))
            fp.close()
        print("Data saved to {}".format(savepath))

        plt.scatter(times, edges)
        plt.title("Plunger motion")
        plt.xlabel("Time (s)")
        plt.ylabel("Location (pixel)")
        plt.tight_layout()
        plt.savefig(savepath.strip("csv") + "png")
        plt.show()

    def triggered_video(self):
        self.set_external_trigger()
        ueye.is_CaptureVideo(self.hCam, ueye.int(2000))

        self.queue_mode()

        # Continuous image display
        while 0 == ueye.IS_SUCCESS:
            # ueye.is_Event(self.hCam, UINT
            # nCommand, void * pParam, UINT
            # nSizeOfParam)
            try:
                self.freeze_video()
                self.take_image()
                self.release_memory()
            except ValueError:
                continue

            # Press q if you want to end the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop_video()
                break

    def release_memory(self):
        # Release the image memory that was allocated using is_AllocImageMem() and remove it from the driver management
        ueye.is_FreeImageMem(self.hCam, self.pcImageMemory, self.MemID)

    def close_connection(self):
        # Destroys the OpenCv windows
        # cv2.destroyAllWindows()
        # Disables the hCam camera handle and releases the data structures and memory areas taken up by the uEye camera
        ueye.is_ExitCamera(self.hCam)
        print()
        print("END")


if __name__ == "__main__":
    cam = Camera()
    cam.configure()

    cam.det_area_of_interest(width=1000)

    cam.allocate_memory_for_image()

    cam.activate_live_video()

    cam.close_connection()