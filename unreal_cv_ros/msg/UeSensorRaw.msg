Header header

# Color image of the scene
sensor_msgs/Image color_image

# Float depth (=ray distance) image of the scene
sensor_msgs/Image depth_image

# Camera parameters
sensor_msgs/CameraInfo camera_info

# Pose where image was taken (in odom frame)
geometry_msgs/Pose camera_link_pose
