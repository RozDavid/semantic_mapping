import numpy as np
from pyrender import IntrinsicsCamera
class CameraConfig(object):

    def __init__(self, yaml_config):

        self.K = yaml_config['camera_matrix']
        self.K = np.array(self.K).reshape((3, 3))
        self.D = yaml_config['distortion_coefficients']
        self.D = np.array(self.D)
        self.height = yaml_config['image_height']
        self.width = yaml_config['image_width']

        self.render_camera = IntrinsicsCamera(
            fx=self.K[0,0],
            fy=self.K[1,1],
            cx=self.K[0,2],
            cy=self.K[1,2],
            name='UndistortedCamera'
        )

    def image_shape(self):

        return self.height, self.width
