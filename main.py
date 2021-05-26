from uEye_class import Camera


cam = Camera()
cam.configure()

cam.det_area_of_interest(y=100, height=300)

cam.allocate_memory_for_image()

# cam.activate_live_video()

cam.triggered_video()

cam.close_connection()

