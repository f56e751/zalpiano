import cv2
import subprocess
import re
import time

class Camera():
    def __init__(self):
        camera_name = 'Arducam OV9782 USB Camera'
        camera_index = self.find_camera_index(camera_name)
        print(f"camera_index is {camera_index}")
        self.camera = cv2.VideoCapture(camera_index, cv2.CAP_V4L2) #################################### v4l2-ctl --list-devices

        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.camera.set(cv2.CAP_PROP_FPS, 120)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # Originally 1280
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)  # Originally 800
        if not self.camera.isOpened():
            self.get_logger().error('Camera could not be opened.')


    def setFps(self,fps):
        self.camera.set(cv2.CAP_PROP_FPS, fps)

    def setFrame(self, width, height):
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)  # Originally 1280
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)  # Originally 800

    
    def find_camera_index(self,camera_name):
        # Run v4l2-ctl to list devices
        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True)
        devices = result.stdout.split('\n\n')  # Devices are separated by a blank line

        # Parse output to find the desired camera
        for device in devices:
            if camera_name in device:
                # Find the line containing the video index
                match = re.search(r'/dev/video(\d+)', device)
                if match:
                    return int(match.group(1))
        return None  # Return None if no matching device is found
    
    def getCamera(self):
        return self.camera

if __name__ == "__main__":
    Camera = Camera()
    Camera.setFps(120)
    Camera.setFrame(640,400)
