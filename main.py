from uEye_class import Camera


cam = Camera()
cam.configure()
cam.det_area_of_interest(y=500, height=100)
cam.allocate_memory_for_image()
times, edges = cam.monitor_edge(20)
# cam.activate_live_video()
# cam.triggered_video()

cam.close_connection()

cam.save_edge_data(times, edges)
