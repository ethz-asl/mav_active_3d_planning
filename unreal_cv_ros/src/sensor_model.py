#!/usr/bin/env python

# ros
import rospy
from unreal_cv_ros.msg import UeSensorRaw, UeSensorRawFast
from sensor_msgs.msg import PointCloud2, PointField
import tf

# Image conversion
from cv_bridge import CvBridge
import io

# Python
import sys
import math
import numpy as np
from struct import pack, unpack

class sensor_model:

    def __init__(self):
        '''  Initialize ros node and read params '''

        # Read in params
        model_type_in = rospy.get_param('~model_type', 'ground_truth')
        self.client_mode = rospy.get_param('~mode', 'standard')  # test, standard, fast, fast2

        # Setup sensor type
        model_types = {'ground_truth': 'ground_truth'}      # Dictionary of implemented models
        selected = model_types.get(model_type_in, 'NotFound')
        if selected == 'NotFound':
            warning = "Unknown sensor model '" + model_type_in + "'. Implemented models are: " + \
                      "".join(["'" + m + "', " for m in model_types])
            rospy.logfatal(warning[:-2])
        else:
            self.model = selected

        # Select client mode
        mode_types = {'standard': 'standard', 'fast': 'fast', 'test': 'test', 'fast2': 'fast2'}
        selected = mode_types.get(self.client_mode, 'NotFound')
        if selected == 'NotFound':
            warning = "Unknown client mode '" + self.mode + "'. Implemented modes are: " + \
                      "".join(["'" + m + "', " for m in mode_types])
            rospy.logfatal(warning[:-2])

        # Initialize node
        self.pub = rospy.Publisher("ue_sensor_out", PointCloud2, queue_size=1)
        if self.client_mode == 'standard' or self.client_mode == 'test':
            self.sub = rospy.Subscriber("ue_sensor_raw", UeSensorRaw, self.callback_standard, queue_size=1)
            self.bridge = CvBridge()
        else:
            self.sub = rospy.Subscriber("ue_sensor_raw", UeSensorRawFast, self.callback_fast, queue_size=1)
        self.tf_br = tf.TransformBroadcaster()

    def callback_standard(self, ros_data):
        ''' Produce simulated sensor outputs from raw unreal color+depth image '''
        if rospy.is_shutdown():
            return
        # Read out images
        img_color = self.bridge.imgmsg_to_cv2(ros_data.color_image, "8UC4")
        img_depth = self.bridge.imgmsg_to_cv2(ros_data.depth_image, "32FC1")
        self.callback_main(img_color, img_depth, ros_data)

    def callback_fast(self, ros_data):
        ''' Produce simulated sensor outputs from raw binary data '''
        if rospy.is_shutdown():
            return
        # Read out images
        z = list(str(ros_data.data))
        idx = int(len(z) / 2)   # Half of the bytes are the 4channel color img, half the depth
        img_color = np.load(io.BytesIO(bytearray(z[:idx])))
        img_depth = np.load(io.BytesIO(bytearray(z[idx:])))
        self.callback_main(img_color, img_depth, ros_data)

    def callback_main(self, img_color, img_depth, ros_data):
        ''' Produce pointcloud from color and depth image and apply sensor noise models'''
        # Process point cloud
        if self.model == 'ground_truth':
            pointcloud = self.process_ground_truth(img_depth, ros_data.camera_info)

        # Build colored pointcloud data
        rgb = np.ones((img_color.shape[0], img_color.shape[1]))
        for i in range(img_color.shape[0]):
            for j in range(img_color.shape[1]):
                color = int(img_color[i, j, 0]) << 16 | int(img_color[i, j, 1]) << 8 | int(img_color[i, j, 2])
                rgb[i, j] = unpack('f', pack('i', color))[0]
        data = np.dstack((pointcloud, rgb))

        # Publish pose
        position = ros_data.camera_link_pose.position
        orientation = ros_data.camera_link_pose.orientation
        self.tf_br.sendTransform((position.x, position.y, position.z),
                                 (orientation.x, orientation.y, orientation.z, orientation.w),
                                 ros_data.header.stamp, "camera_link", "world")

        # Publish pointcloud
        msg = PointCloud2()
        msg.header.stamp = ros_data.header.stamp
        msg.header.frame_id = 'camera'
        msg.width = pointcloud.shape[0]
        msg.height = pointcloud.shape[1]
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

    @staticmethod
    def process_ground_truth(img_depth, cam_info):
        ''' Create point cloud from depth image and camera params. Returns a width x height x 3 (XYZ) array '''
        # read camera params and create image mesh
        height = cam_info.height
        width = cam_info.width
        center_x = cam_info.K[2]
        center_y = cam_info.K[5]
        f = cam_info.K[0]
        cols, rows = np.meshgrid(np.linspace(0, width - 1, num=width), np.linspace(0, height - 1, num=height))

        # Process depth image from ray length to camera axis depth
        distance = ((rows - center_y) ** 2 + (cols - center_x) ** 2) ** 0.5
        points_z = img_depth / (1 + (distance / f) ** 2) ** 0.5

        # Set create x and y position
        points_x = points_z * (cols - center_x) / f
        points_y = points_z * (rows - center_y) / f

        # create pointcloud as XYZ image array
        return np.dstack([points_x, points_y, points_z])


if __name__ == '__main__':
    rospy.init_node('sensor_model', anonymous=True)
    sm = sensor_model()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down sensor_model")
