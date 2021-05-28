from uEye_class import Camera


cam = Camera()
cam.configure()

cam.det_area_of_interest(y=100, height=300)

cam.allocate_memory_for_image()

cam.activate_live_video()

cam.live_video_loop(savepath="dummy1.csv", start=0, stop=None)
cam.live_video_loop(savepath="dummy2.csv", start=0, stop=None)
# cam.activate_live_video(savepath='height_0_100.csv', start=0, stop=100)


cam.close_connection()

