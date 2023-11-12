# camera_multi.py

import cv2
from base_camera import BaseCamera


class Camera(BaseCamera):
    def __init__(self):
        super().__init__()

    # over-wride of BaseCamera class frames method
    @staticmethod
    def frames():
        camera = cv2.VideoCapture(0)
        if not camera.isOpened():
            raise RuntimeError('Could not start camera.')

        camera.set(3, 640)
        camera.set(4, 480)
        camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)

        while True:
            # read current frame
            _, img = camera.read()

            # encode as a jpeg image and return it
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
            yield cv2.imencode('.jpg', img, encode_param)[1].tobytes()
