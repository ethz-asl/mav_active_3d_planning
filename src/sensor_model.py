#!/usr/bin/env python

# ros
import rospy
from unreal_cv_ros.msg import UeSensorRaw
from sensor_msgs.msg import PointCloud2, PointField

# Image conversion
from cv_bridge import CvBridge

# Python
import sys
import math
import numpy as np


class sensor_model:

    def __init__(self):
        '''  Initialize ros node and read params '''

        # Read in params
        model_type_in = rospy.get_param('~model_type', 'ground_truth')

        # Setup sensor type
        model_types = {'ground_truth': 'ground_truth'}      # Dictionary of implemented models
        selected = model_types.get(model_type_in, 'NotFound')
        if selected == 'NotFound':
            warning = "Unknown sensor model '" + model_type_in + "'. Implemented models are: " + \
                      "".join(["'" + m + "', " for m in model_types])
            rospy.logfatal(warning[:-2])
        else:
            self.model = selected

        # Initialize node
        self.pub = rospy.Publisher("ue_sensor_out", PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber("ue_sensor_raw", UeSensorRaw, self.callback, queue_size=1)
        self.bridge = CvBridge()

    def callback(self, data):
        ''' Produce simulated sensor outputs from raw unreal color+depth image '''
        if rospy.is_shutdown():
            return

        # Read out images
        img_color = self.bridge.imgmsg_to_cv2(data.color_image, "8UC4")
        img_depth = self.bridge.imgmsg_to_cv2(data.depth_image, "32FC1")

        # test
        # cv2.imshow("Color input", img_color[:, :, [2, 1, 0]])
        # cv2.waitKey(3)
        # cv2.imshow("Depth input", cv2.applyColorMap(np.uint8(img_depth), cv2.COLORMAP_JET))
        # cv2.imshow("Normalized depth", cv2.applyColorMap(np.uint8(255*(img_depth-np.amin(img_depth)/
        #                                                            (np.amax(img_depth)-np.amin(img_depth)))),
        #                                                           cv2.COLORMAP_JET))
        # cv2.waitKey(3)
        # print('image received')

        # Process point cloud
        if self.model == 'ground_truth':
            pointcloud = self.process_ground_truth(img_depth, data.camera_info)
            pointcloud = np.stack([pointcloud, img_color[:, :, [0, 1, 2]]], axis=2)

        # Publish pointcloud
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'camera'
        msg.width = pointcloud.shape[0]
        msg.height = pointcloud.shape[1]
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('r', 12, PointField.FLOAT32, 1),
            PointField('g', 16, PointField.FLOAT32, 1),
            PointField('b', 20, PointField.FLOAT32, 1)]
        # msg.fields = [
        #     PointField('x', 0, PointField.FLOAT32, 1),
        #     PointField('y', 4, PointField.FLOAT32, 1),
        #     PointField('z', 8, PointField.FLOAT32, 1),
        #     PointField('r', 12, PointField.UINT8, 1),
        #     PointField('g', 13, PointField.UINT8, 1),
        #     PointField('b', 14, PointField.UINT8, 1)]
        msg.is_bigendian = False
        msg.point_step = 24 #15
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        msg.data = np.float32(pointcloud).tostring()
        # data = np.dstack([np.float32(pointcloud), np.uint8(img_color[:, :, [0, 1, 2]])])
        # print(data.shape, data.dtype)
        # msg.data = data.tostring()

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
