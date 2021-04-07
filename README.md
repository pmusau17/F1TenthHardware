# F1Tenth Hardware

This repository contains a host of ROS package for the F1Tenth Autonomous Racing Competition and for the real-time reachability work described [here](https://github.com/pmusau17/rtreach_f1tenth).

### Real-time Reachability Experiments
![LEC_GIF](images/hardware_experiments.gif)

For those interesting in watching videos of our safety assurance implementation utilizing the real-time reachability algorithm, the videos can be found [here](images).

### Package List

- [Rtreach](src/rtreach): safety assurance regime using real-time reachability techniques. Details can be found [here](https://github.com/pmusau17/rtreach_f1tenth).
- [Computer_vision](src/computer_vision): implementation of end-to-end, object tracking, and other cv approaches
- [Map_server](src/map_server): custom map server amenable to the [google cartographer package](https://github.com/cartographer-project/cartographer)
- [Rl](src/rl): implementation of reinforcement learning techniques such as augmented random search, ppo, soft-actor critic.
- [Pure-pursuit](src/pure_pursuit): pure pursuit path tracking implementation for the F1/10 platform.
- [Particle_Filter](src/particle_filter): cuda enabled particle filter implementation by MIT
- [Racecar](src/racecar): main package containing a variety of nodes,launch files, and configuration files used to control the F1/10 car. 

### Important Sensor Topics (In my humble opinion)

- /ackermann_cmd_mux/input/teleop : topic used to issue control commands to the vehicle. Encoded as an AckermannStamped message. 
- /zed/zed_node/camera_odom: visual odometry provided by the camera
- /zed/zed_node/confidence/confidence_image: a confidence map that represents the confidence of a given measure (Z) for every pixel (X,Y) in the image. Confidence values vary between 0 and 100, representing the best confidence value possible
- /zed/zed_node/depth/depth_registered: camera depth cloud
- /zed/zed_node/path_odom: Sequence of camera odometry poses in Map frame
- /zed/zed_node/left/image_rect_color
- /zed/zed_node/right/image_rect_color
- /zed/zed_node/stereo/image_rect_color: stereo rectified pair images side-by-side


### If you have multiple cars 

- Change the namespace parameter in the launch file [src/racecar/launch/keyboard_teleop.launch]
- Change the link name to the corresponding name in this [urdf file](src/zed-ros-wrapper/zed_wrapper/urdf/zed.urdf) so that the static transforms are done correctly 
