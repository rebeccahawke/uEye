# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

from uEye_class import Camera

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    cam = Camera()
    cam.configure()

    cam.det_area_of_interest(y=100, height=300)

    cam.allocate_memory_for_image()

    # cam.activate_live_video()

    cam.triggered_video()

    cam.close_connection()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
