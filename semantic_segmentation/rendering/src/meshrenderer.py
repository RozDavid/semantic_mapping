

import numpy as np
import os
# os.environ['PYOPENGL_PLATFORM'] = 'egl'
import sys
import glob
import argparse
import yaml

import trimesh
import pyrender
import cv2

from utils import *
from src.CameraConfig import CameraConfig

class MeshRenderer(object):

    def __init__(self, args):

        self.args = args
        self.mesh_path = self.args.mesh_path
        self.dataset_path = remove_trailing(self.args.recordings_path)
        self.poses_path = self.dataset_path + '/poses/'
        self.images_path = self.dataset_path + '/imgs/'

        print('Loading mesh:', self.mesh_path)
        self.trimesh = trimesh.load(self.mesh_path)

        self.mesh = pyrender.Mesh.from_trimesh(self.trimesh)
        self.scene = pyrender.Scene(ambient_light=[0.5, 0.5, 0.5])
        self.scene.add(self.mesh)

        with open(self.args.camera_config) as file:
            camera_config_yml = yaml.load(file, Loader=yaml.FullLoader)
            self.camera_config = CameraConfig(camera_config_yml)

        self.render_camera = self.camera_config.render_camera
        self.renderer = pyrender.offscreen.OffscreenRenderer(self.camera_config.width,
                                                            self.camera_config.height)

        print('Finished scene initialization')

        trans = np.array([2.0, 1.0, 0.0])
        quat = np.array([0.707, 0.0, 0.0, 0.707])
        pose = ros_to_opengl(quat, trans)
        self.camera_node = self.scene.add(self.render_camera, pose=pose)

        if args.visualization:
            pyrender.Viewer(self.scene, viewport_size=self.camera_config.image_shape())

    def render_image(self, pose):

        self.scene.set_pose(self.camera_node, pose=pose)

        rendered_img, _ = self.renderer.render(self.scene, flags=pyrender.RenderFlags.FLAT)
        rendered_img = rendered_img[::-1]

        return rendered_img

    def render_depth(self, pose):

        self.scene.set_pose(self.camera_node, pose=pose)

        rendered_depth = self.renderer.render(self.scene, flags=pyrender.RenderFlags.DEPTH_ONLY)

        return rendered_depth

    def render_ros_transform(self, trans, rot, secs, depth = False):

        quat_np = [rot[3], rot[0], rot[1], rot[2]]  # [qw, qx, qy, qz]
        pose = ros_to_opengl(quat_np, trans)

        rendered_img = self.render_image(pose)
        rendered_img = cv2.cvtColor(rendered_img, cv2.COLOR_BGR2RGB)
        rendered_img = cv2.flip(rendered_img, 0)

        if depth:
            rendered_depth = self.render_depth(pose)
            print("Rendered RGB and D, with trans: {} and rot {} at {}".format(trans, rot, secs))
            return rendered_img, rendered_depth

        else:

            print("Rendered image, with trans: {} and rot {} at {}".format(trans, rot, secs))
            return rendered_img


    def process_files(self):

        for filename in glob.glob(self.poses_path + '*.yml'):

            filename_base = os.path.splitext(filename)[0]
            with open(self.args.camera_config) as file:
                pose_yaml = yaml.load(filename, Loader=yaml.FullLoader)

            trans = np.array(pose_yaml['trans'])
            quat_ros = pose_yaml['rot']  # [qx, qy, qz, qw]
            quat_np = [quat_ros[3], quat_ros[0], quat_ros[1], quat_ros[2]]  # [qw, qx, qy, qz]
            pose = ros_to_opengl(quat_np, trans)

            print("Processing file {}, with trans: {} and quat {}".format(filename_base, trans, quat))

            rendered_img = self.render_image(pose)
            rendered_img = cv2.cvtColor(rendered_img, cv2.COLOR_BGR2RGB)
            rendered_img = cv2.flip(rendered_img, 0)

            cv2.imwrite(self.dataset_path + '/' + filename_base + '.png', rendered_img)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    # Parse arguments for global params
    parser.add_argument('--mesh_path', type=str, default='../meshes/kinect_2cm_rgb.ply',
                        help='The semantic or RGB mesh, that provides the labels for the views')
    parser.add_argument('--recordings_path', type=str, default="./data",
                        help='Under this directory should be stored images and and poses'
                             ' in the \'imgs\' and \'poses\' folders accordingly')
    parser.add_argument('--camera_config', type=str, default='../config/calib_k4a.yml',
                        help='The camera parameters of the virtual camera that renders the image')
    parser.add_argument("--visualization", default=True, type=str2bool, nargs='?',
                        help="If we would like to open a viewer with the loaded model first")

    args = parser.parse_args()
    meshrenderer = MeshRenderer(args)

    trans = np.array([0.0, 0.0, 0.0])
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    pose = ros_to_opengl(quat, trans)

    rendered_img = meshrenderer.render_image(pose)

    trans = np.array([2.0, 1.0, 0.0])
    quat = np.array([0.707, 0.0, 0.0, 0.707])
    pose = ros_to_opengl(quat, trans)

    rendered_img2 = meshrenderer.render_image(pose)

    numpy_horizontal = np.hstack((rendered_img, rendered_img2))
    numpy_horizontal = cv2.resize(numpy_horizontal, (0, 0), fx=0.5, fy=0.5)
    numpy_horizontal = cv2.cvtColor(numpy_horizontal, cv2.COLOR_BGR2RGB)
    numpy_horizontal = cv2.flip(numpy_horizontal, 0)

    cv2.imshow('Rendered Images', numpy_horizontal)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    meshrenderer.process_files()

