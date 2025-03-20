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

    There are two scripts: one that will take a picture periodically (every x millisenconds), and one that will take a picture every time it recieves a "1" from a selected input topic.

## Run the periodic node

1. Create directories to store the images:

    `mkdir images_Color && mkdir images_Depth`

2. Run the node:

    `ros2 run ros2_realsense create_image_periodic --ros-args -p period_ms:=<period-in-milliseconds>`
    
    Example: `ros2 run ros2_realsense create_image_periodic --ros-args -p period_ms:=500`

## Run the command-based node

1. Create directories to store the images:

    `mkdir images_Color && mkdir images_Depth`

2. Run the node:

    `ros2 run ros2_realsense create_image_command --ros-args -p topic:=<input-topic>`
    
    Example: `ros2 run ros2_realsense create_image_command --ros-args -p topic:=take_picture`