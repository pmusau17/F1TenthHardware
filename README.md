# F1Tenth Hardware

This repository contains a host of ROS package for the F1Tenth Autonomous Racing Competition. Particularly it contains the ros implementation for the Traxxas Harware Platform

### Important Sensor Topics (In my humble opinion)

- /ackermann_cmd_mux/input/teleop
- /zed/zed_node/camera_odom: visual odometry provided by the camera
- /zed/zed_node/confidence/confidence_image: a confidence map that represents the confidence of a given measure (Z) for every pixel (X,Y) in the image. Confidence values vary between 0 and 100, representing the best confidence value possible
- /zed/zed_node/depth/depth_registered: camera depth cloud
- /zed/zed_node/path_odom: Sequence of camera odometry poses in Map frame
- /zed/zed_node/left/image_rect_color
- /zed/zed_node/right/image_rect_color
- /zed/zed_node/stereo/image_rect_color: stereo rectified pair images side-by-side

