#!/usr/bin/env python

# ros
import rospy
from unreal_cv_ros.msg import UeSensorRaw
from sensor_msgs.msg import PointCloud2, PointField

# Image conversion
import io

# Python
import sys
import math
import numpy as np
from struct import pack, unpack
import time


class SensorModel:

    def __init__(self):
        '''  Initialize ros node and read params '''

        # Read in params
        model_type_in = rospy.get_param('~model_type', 'ground_truth')
        camera_params_ns = rospy.get_param('~camera_params_ns', rospy.get_namespace()+"camera_params")
        self.maximum_distance = rospy.get_param('~maximum_distance', 0)  # Set to 0 to keep all points
        self.flatten_distance = rospy.get_param('~flatten_distance', 0)  # Set to 0 to ignore

        # Setup sensor type
        model_types = {'ground_truth': 'ground_truth'}      # Dictionary of implemented models
        selected = model_types.get(model_type_in, 'NotFound')
        if selected == 'NotFound':
            warning = "Unknown sensor model '" + model_type_in + "'. Implemented models are: " + \
                      "".join(["'" + m + "', " for m in model_types])
            rospy.logfatal(warning[:-2])
        else:
            self.model = selected

        # Initialize camera params from params or wait for unreal_ros_client to publish them
        if not rospy.has_param(camera_params_ns+'width'):
            rospy.loginfo("Waiting for unreal camera params at '%s' ...", camera_params_ns)
            while not rospy.has_param(camera_params_ns+'/width'):
                rospy.sleep(0.1)
        self.camera_params = [rospy.get_param(camera_params_ns+'/width'), rospy.get_param(camera_params_ns+'/height'),
                              rospy.get_param(camera_params_ns+'/focal_length')]

        # Initialize node
        self.pub = rospy.Publisher("ue_sensor_out", PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber("ue_sensor_raw", UeSensorRaw, self.callback, queue_size=10)

        rospy.loginfo("Sensor model setup cleanly.")

    def callback(self, ros_data):
        ''' Produce simulated sensor outputs from raw binary data '''
        # Read out images
        img_color = np.load(io.BytesIO(bytearray(ros_data.color_data)))
        img_depth = np.load(io.BytesIO(bytearray(ros_data.depth_data)))
        mask_depth = img_depth.reshape(-1)

        # Build 3D point cloud from depth
        if self.flatten_distance > 0:
            img_depth = np.clip(img_depth, 0, self.flatten_distance)
        (x, y, z) = self.depth_to_3d(img_depth)

        # Pack RGB image (for ros representation)
        rgb = self.rgb_to_float(img_color)

        # Remove invalid points
        if self.maximum_distance > 0:
            mask = mask_depth <= self.maximum_distance
            x = x[mask]
            y = y[mask]
            z = z[mask]
            rgb = rgb[mask]

        # Sensor model processing
        # if self.model == 'ground_truth':

        # Publish pointcloud
        data = np.transpose(np.vstack((x, y, z, rgb)))
        msg = PointCloud2()
        msg.header.stamp = ros_data.header.stamp
        msg.header.frame_id = 'camera'
        msg.width = data.shape[0]
        msg.height = 1
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.float32(data).tostring()
        self.pub.publish(msg)

    def depth_to_3d(self, img_depth):
        ''' Create point cloud from depth image and camera params. Returns a single array for x, y and z coords '''
        # read camera params and create image mesh
        height = self.camera_params[1]
        width = self.camera_params[0]
        center_x = width/2
        center_y = height/2
        f = self.camera_params[2]
        cols, rows = np.meshgrid(np.linspace(0, width - 1, num=width), np.linspace(0, height - 1, num=height))

        # Process depth image from ray length to camera axis depth
        distance = ((rows - center_y) ** 2 + (cols - center_x) ** 2) ** 0.5
        points_z = img_depth / (1 + (distance / f) ** 2) ** 0.5

        # Create x and y position
        points_x = points_z * (cols - center_x) / f
        points_y = points_z * (rows - center_y) / f

        return points_x.reshape(-1), points_y.reshape(-1), points_z.reshape(-1)

    @staticmethod
    def rgb_to_float(img_color):
        ''' Stack uint8 rgb image into a single float array (efficiently) for ros compatibility '''
        r = np.ravel(img_color[:, :, 0]).astype(int)
        g = np.ravel(img_color[:, :, 1]).astype(int)
        b = np.ravel(img_color[:, :, 2]).astype(int)
        color = np.left_shift(r, 16) + np.left_shift(g, 8) + b
        packed = pack('%di' % len(color), *color)
        unpacked = unpack('%df' % len(color), packed)
        return np.array(unpacked)


if __name__ == '__main__':
    rospy.init_node('sensor_model', anonymous=True)
    sm = SensorModel()
    rospy.spin()

