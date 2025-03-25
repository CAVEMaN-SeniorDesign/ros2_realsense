# ros2_realsense

## Installing dependencies

1. In a directory of your choice (root will be fine), clone the following repository:

    `git clone https://github.com/CAVEMaN-SeniorDesign/RealSense_API.git`

2. Enter the repository:

    `cd RealSense_API`

3. Make the install_dependencies script available to run:

    `chmod +x ./tools/install_dependencies.sh`

4. Install the dependencies. This might take a while:

    `./tools/install_dependencies.sh`

## Cloning and building the ros2 package

1. Navigate to your `~/ros2_ws/src` folder, and clone the package:

    `git clone https://github.com/CAVEMaN-SeniorDesign/ros2_realsense.git`

2. Navigate to your `~/ros2_ws` folder, and build the package:

    `colcon build --packages-select ros2_realsense`

## Note on the two scripts

There are three scripts: 
1. One that will take a picture periodically (every x millisenconds), 
2. One that will take a picture every time it recieves a 1 from a selected input topic.
3. One that will take a picture every time it recieved a press from the x button on the xbox controller.
    
    All images should be published to `~/images_Color` and `~/images_Depth`.

## Run the periodic node

1. Run the node:

    `ros2 run ros2_realsense create_image_periodic --ros-args -p period_ms:=<period-in-milliseconds>`
    
    Example: `ros2 run ros2_realsense create_image_periodic --ros-args -p period_ms:=500 -p exposure_time:=15`

## Run the command-based node

1. Run the node:

    `ros2 run ros2_realsense create_image_command --ros-args -p topic:=<input-topic>`
    
    Example: `ros2 run ros2_realsense create_image_command --ros-args -p topic:=take_picture -p exposure_time:=15`

## Run the controller-based node

1. Run the node:

    `ros2 run ros2_realsense create_image_contol_cmd --ros-args -p topic:=<input-topic>`
    
    Example: `ros2 run ros2_realsense create_image_control_cmd --ros-args -p topic:=/joy -p exposure_time:=15`

## A note on the `exposure_time` argument

The `exposure_time` argument controls the number of photos that will be taken for autoexposure before the actaul photo is taken. These initial photos allow the camera to adjust to the lighting in the environment. Higher exposure allows for better color quality, but it takes longer time to take a photo. A recommended `exposure_time` to start with is 15.